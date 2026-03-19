"""
Non-temporal OD mapping with pre-generated routes for RouteAlphaCapacityModel
and RouteVehicleCapacityModel.
"""

export RouteODMap
export create_route_od_map


"""
    RouteODMap <: AbstractClusteringMap

Data mapping for RouteAlphaCapacityModel and RouteVehicleCapacityModel.
Extends the clustering OD structure with per-scenario route pools (no time index).

# Fields
- `station_id_to_array_idx::Dict{Int,Int}`: Station ID → array index
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID
- `scenarios::Vector{ScenarioData}`: Scenario reference
- `scenario_label_to_array_idx::Dict{String,Int}`: Scenario label → index
- `array_idx_to_scenario_label::Vector{String}`: Index → scenario label
- `Omega_s::Dict{Int,Vector{Tuple{Int,Int}}}`: scenario → OD pairs (aggregated)
- `Q_s::Dict{Int,Dict{Tuple{Int,Int},Int}}`: scenario → (o,d) → demand
- `valid_jk_pairs::Dict{Tuple{Int,Int},Vector{Tuple{Int,Int}}}`: (o,d) → valid (j_idx,k_idx)
- `max_walking_distance::Float64`: Walking limit used to compute valid_jk_pairs
- `routes_s::Dict{Int,Vector{NonTimedRouteData}}`: Per-scenario route pool
- `routes_by_jks::Dict{NTuple{3,Int},Vector{Tuple{Int,Int}}}`:
  `(s, j_idx, k_idx) → [(route_idx_within_s, α)]` for the covering constraint.
  α is actual passengers for RouteAlphaCapacityModel; C for RouteVehicleCapacityModel.
"""
struct RouteODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}

    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    max_walking_distance::Float64

    routes_s::Dict{Int, Vector{NonTimedRouteData}}
    routes_by_jks::Dict{NTuple{3,Int}, Vector{Tuple{Int,Int}}}
end


"""
    has_walking_distance_limit(mapping::RouteODMap) -> Bool
"""
has_walking_distance_limit(mapping::RouteODMap) = true


"""
    get_valid_jk_pairs(mapping::RouteODMap, o::Int, d::Int) -> Vector{Tuple{Int,Int}}

Return valid (j_idx, k_idx) pairs for OD pair (o_id, d_id).
"""
function get_valid_jk_pairs(mapping::RouteODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end


"""
    create_route_od_map(model, data) -> RouteODMap

Build the full OD mapping for RouteAlphaCapacityModel or RouteVehicleCapacityModel:
1. Station and scenario index mappings
2. Aggregated OD pairs and demand per scenario
3. Valid (j,k) pairs per OD pair (walking-filtered)
4. Per-scenario routes via non-temporal BFS
5. `routes_by_jks` index with α = actual passengers (Alpha) or C (Vehicle)
"""
function create_route_od_map(
    model::Union{RouteAlphaCapacityModel, RouteVehicleCapacityModel},
    data::StationSelectionData
)::RouteODMap

    # ── 1. Index mappings ──────────────────────────────────────────────────────
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    # ── 2. Aggregated OD pairs per scenario ────────────────────────────────────
    S = n_scenarios(data)
    Omega_s    = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s        = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for s in 1:S
        od_count = compute_scenario_od_count(data.scenarios[s])
        Omega_s[s] = collect(keys(od_count))
        Q_s[s]     = od_count
        union!(all_od_pairs, keys(od_count))
    end

    # ── 3. Valid (j,k) pairs per OD pair ──────────────────────────────────────
    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data,
        station_id_to_array_idx, array_idx_to_station_id,
        model.max_walking_distance
    )

    # ── 4. Non-temporal BFS routes ─────────────────────────────────────────────
    routes_s   = Dict{Int, Vector{NonTimedRouteData}}()
    routes_by_jks = Dict{NTuple{3,Int}, Vector{Tuple{Int,Int}}}()

    for s in 1:S
        println("  Scenario $s / $S: building non-timed orders...")
        flush(stdout)

        # Build _NonTimedOrder list for this scenario
        nontimed_orders = _NonTimedOrder[]
        for ((o_id, d_id), q) in Q_s[s]
            valid_pairs = get(valid_jk_pairs, (o_id, d_id), Tuple{Int,Int}[])
            isempty(valid_pairs) && continue
            vbs_id_pairs = Tuple{Int,Int}[
                (array_idx_to_station_id[j_idx], array_idx_to_station_id[k_idx])
                for (j_idx, k_idx) in valid_pairs
            ]
            push!(nontimed_orders, _NonTimedOrder(o_id, d_id, q, vbs_id_pairs))
        end

        if length(nontimed_orders) > 63
            @warn "Scenario $s has $(length(nontimed_orders)) orders for non-temporal BFS " *
                  "(limit 63); truncating to first 63. Consider reducing scenario duration."
            resize!(nontimed_orders, 63)
        end

        println("  Scenario $s / $S: running BFS with $(length(nontimed_orders)) orders...")
        flush(stdout)
        routes_s[s] = generate_routes_from_orders(
            nontimed_orders, data, station_id_to_array_idx;
            vehicle_capacity = model.vehicle_capacity,
            max_detour_time  = model.max_detour_time,
            max_detour_ratio = model.max_detour_ratio
        )

        println("  Scenario $s / $S: $(length(routes_s[s])) routes generated")
        flush(stdout)

        # Build (s, j_idx, k_idx) → [(route_idx_within_s, α)] index
        for (r_idx, ntr) in enumerate(routes_s[s])
            if model isa RouteAlphaCapacityModel
                # α = actual passengers on this leg
                for ((j_idx, k_idx), α) in ntr.alpha
                    key = (s, j_idx, k_idx)
                    push!(get!(routes_by_jks, key, Tuple{Int,Int}[]), (r_idx, α))
                end
            else
                # RouteVehicleCapacityModel: α = C (flat vehicle capacity)
                C = model.vehicle_capacity
                for (j_idx, k_idx) in keys(ntr.alpha)
                    key = (s, j_idx, k_idx)
                    push!(get!(routes_by_jks, key, Tuple{Int,Int}[]), (r_idx, C))
                end
            end
        end
    end

    return RouteODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        valid_jk_pairs,
        model.max_walking_distance,
        routes_s,
        routes_by_jks
    )
end
