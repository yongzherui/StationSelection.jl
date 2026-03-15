"""
OD mapping with time windows and pre-generated routes for TwoStageRouteModel.
"""

export TwoStageRouteODMap
export create_two_stage_route_od_map

"""
    TwoStageRouteODMap <: AbstractClusteringMap

Data mapping for TwoStageRouteModel. Extends the clustering OD structure with
time-indexed OD pairs and pre-generated vehicle routes.

# Fields (standard mode)
- `station_id_to_array_idx::Dict{Int,Int}`: Station ID → array index
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID
- `scenarios::Vector{ScenarioData}`: Scenario reference
- `scenario_label_to_array_idx::Dict{String,Int}`: Scenario label → index
- `array_idx_to_scenario_label::Vector{String}`: Index → scenario label
- `Omega_s_t::Dict{Int,Dict{Int,Vector{Tuple{Int,Int}}}}`: scenario → time_id → OD pairs
- `Q_s_t::Dict{Int,Dict{Int,Dict{Tuple{Int,Int},Int}}}`: scenario → time_id → (o,d) → demand
- `routes::Vector{RouteData}`: All generated routes (global pool, standard mode)
- `routes_by_jk::Dict{Tuple{Int,Int},Vector{Int}}`: (j_idx,k_idx) → route indices (1-based)
- `valid_jk_pairs::Dict{Tuple{Int,Int},Vector{Tuple{Int,Int}}}`: (o_id,d_id) → valid (j_idx,k_idx) pairs
- `max_walking_distance::Union{Float64,Nothing}`: Walking limit (nothing = no limit)
- `time_window_sec::Int`: Time window size in seconds

# Additional fields (temporal BFS mode, when `max_wait_time` is set)
- `routes_s::Union{Dict{Int,Vector{TimedRouteData}},Nothing}`:
  Per-scenario route pool from cross-window BFS. `routes_s[s]` = routes for scenario s.
- `routes_by_jkt_s::Union{Dict{NTuple{4,Int},Vector{Tuple{Int,Int}}},Nothing}`:
  `(s, t_id, j_idx, k_idx) → [(route_idx_within_s, α)]` for the covering constraint.
"""
struct TwoStageRouteODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}
    Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}

    routes::Vector{RouteData}
    routes_by_jk::Dict{Tuple{Int, Int}, Vector{Int}}

    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    max_walking_distance::Union{Float64, Nothing}
    time_window_sec::Int

    # Temporal BFS mode (nothing when standard mode)
    routes_s::Union{Dict{Int, Vector{TimedRouteData}}, Nothing}
    routes_by_jkt_s::Union{Dict{NTuple{4,Int}, Vector{Tuple{Int,Int}}}, Nothing}
end


"""
    has_walking_distance_limit(mapping::TwoStageRouteODMap) -> Bool
"""
has_walking_distance_limit(mapping::TwoStageRouteODMap) = !isnothing(mapping.max_walking_distance)


"""
    get_valid_jk_pairs(mapping::TwoStageRouteODMap, o::Int, d::Int) -> Vector{Tuple{Int,Int}}

Return valid (j_idx, k_idx) pairs for OD pair (o_id, d_id).
"""
function get_valid_jk_pairs(mapping::TwoStageRouteODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end


"""
    create_two_stage_route_od_map(model::TwoStageRouteModel, data::StationSelectionData)
    -> TwoStageRouteODMap

Build the full OD mapping for TwoStageRouteModel:
1. Station and scenario index mappings
2. Time-indexed OD pairs and demand per scenario
3. Route generation (standard or temporal BFS depending on `model.max_wait_time`)
4. Route-to-(j,k) index
5. Valid (j,k) pairs per OD pair (route-covered; optionally walking-filtered)

When `model.max_wait_time` is set, generates per-scenario routes via cross-window BFS
and populates `routes_s` / `routes_by_jkt_s` in addition to the standard fields.
"""
function create_two_stage_route_od_map(
    model::TwoStageRouteModel,
    data::StationSelectionData
)::TwoStageRouteODMap

    # ── 1. Index mappings ──────────────────────────────────────────────────────
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)
    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    # ── 2. Time-indexed OD pairs ───────────────────────────────────────────────
    S = n_scenarios(data)
    Omega_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    Q_s_t     = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for s in 1:S
        scenario = data.scenarios[s]
        time_to_od = compute_time_to_od_count_mapping(scenario, model.time_window_sec)

        Omega_s_t[s] = Dict{Int, Vector{Tuple{Int, Int}}}()
        Q_s_t[s]     = Dict{Int, Dict{Tuple{Int, Int}, Int}}()

        for (t_id, od_count) in time_to_od
            Omega_s_t[s][t_id] = collect(keys(od_count))
            Q_s_t[s][t_id]     = od_count
            union!(all_od_pairs, keys(od_count))
        end
    end

    # ── 3. Generate routes ─────────────────────────────────────────────────────
    # When detour limits are set AND walking is limited: pre-compute walking-feasible
    # VBS (j_id, k_id) pairs and pass those as od_pairs so the label-setting only
    # generates routes for station pairs that passengers can actually walk to.
    # Otherwise fall back to exhaustive station enumeration (generate_routes).
    use_order_based = (!isnothing(model.max_detour_time) || !isnothing(model.max_detour_ratio)) &&
                      model.use_walking_distance_limit

    local walking_valid  # may be computed here and reused in step 5

    if use_order_based
        walking_valid = compute_valid_jk_pairs(
            all_od_pairs, data,
            station_id_to_array_idx, array_idx_to_station_id,
            model.max_walking_distance
        )
        # Collect all unique VBS (j_id, k_id) pairs reachable by walking
        vbs_pairs_set = Set{Tuple{Int,Int}}()
        for pairs in values(walking_valid)
            for (j_idx, k_idx) in pairs
                push!(vbs_pairs_set, (array_idx_to_station_id[j_idx],
                                      array_idx_to_station_id[k_idx]))
            end
        end
        routes = generate_routes_from_orders(
            collect(vbs_pairs_set), data;
            vehicle_capacity      = model.vehicle_capacity,
            max_detour_time       = model.max_detour_time,
            max_detour_ratio      = model.max_detour_ratio,
            max_route_travel_time = model.max_route_travel_time
        )
    elseif !isnothing(model.max_detour_time) || !isnothing(model.max_detour_ratio)
        # Detour limits set but walking is unrestricted: all station pairs are feasible,
        # so pass the geographic demand pairs directly.
        routes = generate_routes_from_orders(
            collect(all_od_pairs), data;
            vehicle_capacity      = model.vehicle_capacity,
            max_detour_time       = model.max_detour_time,
            max_detour_ratio      = model.max_detour_ratio,
            max_route_travel_time = model.max_route_travel_time
        )
    else
        routes = generate_routes(
            data;
            vehicle_capacity       = model.vehicle_capacity,
            max_route_travel_time  = model.max_route_travel_time,
            max_intermediate_stops = model.max_intermediate_stops
        )
    end

    # ── 4. Build routes_by_jk ─────────────────────────────────────────────────
    # routes_by_jk[(j_idx, k_idx)] = list of route indices covering segment (j,k)
    routes_by_jk = Dict{Tuple{Int, Int}, Vector{Int}}()
    for (r_idx, route) in enumerate(routes)
        for (pickup_id, dropoff_id) in keys(route.od_capacity)
            j_idx = station_id_to_array_idx[pickup_id]
            k_idx = station_id_to_array_idx[dropoff_id]
            jk = (j_idx, k_idx)
            push!(get!(routes_by_jk, jk, Int[]), r_idx)
        end
    end

    # ── 5. Valid (j,k) pairs per OD pair ──────────────────────────────────────
    # Always route-covered; optionally intersected with walking-feasible pairs.
    covered_jk = Set{Tuple{Int, Int}}(keys(routes_by_jk))
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()

    if model.use_walking_distance_limit
        # Reuse walking_valid if already computed in step 3; otherwise compute now.
        if !@isdefined(walking_valid)
            walking_valid = compute_valid_jk_pairs(
                all_od_pairs, data,
                station_id_to_array_idx, array_idx_to_station_id,
                model.max_walking_distance
            )
        end
        for (o, d) in all_od_pairs
            walk_pairs = get(walking_valid, (o, d), Tuple{Int, Int}[])
            valid_jk_pairs[(o, d)] = filter(p -> p in covered_jk, walk_pairs)
        end
    else
        covered_jk_vec = collect(covered_jk)
        for (o, d) in all_od_pairs
            valid_jk_pairs[(o, d)] = covered_jk_vec
        end
    end

    max_walking_distance = model.use_walking_distance_limit ? model.max_walking_distance : nothing

    # ── 6. Temporal BFS routes (only when max_wait_time is set) ───────────────
    routes_s       = nothing
    routes_by_jkt_s = nothing

    if !isnothing(model.max_wait_time)
        routes_s        = Dict{Int, Vector{TimedRouteData}}()
        routes_by_jkt_s = Dict{NTuple{4,Int}, Vector{Tuple{Int,Int}}}()

        for s in 1:S
            # Build _TimedOrder list for this scenario
            timed_orders = _TimedOrder[]
            for (t_id, od_count) in Omega_s_t[s]
                for ((o_id, d_id), q) in od_count
                    valid_pairs = get(valid_jk_pairs, (o_id, d_id), Tuple{Int,Int}[])
                    isempty(valid_pairs) && continue
                    # Convert array-index pairs to station-ID pairs
                    vbs_id_pairs = Tuple{Int,Int}[
                        (array_idx_to_station_id[j_idx], array_idx_to_station_id[k_idx])
                        for (j_idx, k_idx) in valid_pairs
                    ]
                    push!(timed_orders, _TimedOrder(o_id, d_id, t_id, q, vbs_id_pairs))
                end
            end

            if length(timed_orders) > 63
                @warn "Scenario $s has $(length(timed_orders)) orders for temporal BFS " *
                      "(limit 63); truncating to first 63. Consider increasing time_window_sec."
                resize!(timed_orders, 63)
            end

            routes_s[s] = generate_routes_from_timed_orders(
                timed_orders, data, station_id_to_array_idx;
                vehicle_capacity = model.vehicle_capacity,
                max_wait_time    = model.max_wait_time,
                max_delay_time   = model.max_detour_time,
                max_delay_ratio  = model.max_detour_ratio,
                time_window_sec  = model.time_window_sec
            )

            # Build (s, t_id, j_idx, k_idx) → [(route_idx_within_s, α)] index
            for (r_idx, trd) in enumerate(routes_s[s])
                for ((t_id, (j_idx, k_idx)), α) in trd.alpha
                    key = (s, t_id, j_idx, k_idx)
                    push!(get!(routes_by_jkt_s, key, Tuple{Int,Int}[]), (r_idx, α))
                end
            end
        end
    end

    return TwoStageRouteODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s_t,
        Q_s_t,
        routes,
        routes_by_jk,
        valid_jk_pairs,
        max_walking_distance,
        model.time_window_sec,
        routes_s,
        routes_by_jkt_s
    )
end


"""
    is_temporal_mode(mapping::TwoStageRouteODMap) -> Bool

Return `true` when the mapping was built with cross-window temporal BFS routes.
"""
is_temporal_mode(mapping::TwoStageRouteODMap) = !isnothing(mapping.routes_s)
