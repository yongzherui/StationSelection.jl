"""
Non-temporal OD mapping for RouteVehicleCapacityModel (new formulation).

Routes are stored as plain RouteData (no alpha dict). The covering constraints
use explicit integer variables d_{jkts}, α^r_{jkts}, θ^r_{ts} at solve time.
"""

export VehicleCapacityODMap
export create_vehicle_capacity_od_map


"""
    VehicleCapacityODMap <: AbstractClusteringMap

Data mapping for RouteVehicleCapacityModel (new formulation).

Unlike RouteODMap, this struct stores plain RouteData (no per-leg alpha counts).
Route loading is made explicit via integer JuMP variables d/α/θ.

# Fields
- `station_id_to_array_idx::Dict{Int,Int}`: Station ID → array index
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID
- `scenarios::Vector{ScenarioData}`: Scenario reference
- `scenario_label_to_array_idx::Dict{String,Int}`: Scenario label → index
- `array_idx_to_scenario_label::Vector{String}`: Index → scenario label
- `Omega_s::Dict{Int,Vector{Tuple{Int,Int}}}`: scenario → OD pairs (aggregated, no time)
- `Q_s::Dict{Int,Dict{Tuple{Int,Int},Int}}`: scenario → (o,d) → demand (aggregated)
- `Omega_s_t::Dict{Int,Dict{Int,Vector{Tuple{Int,Int}}}}`: scenario → t_id → OD pairs
- `Q_s_t::Dict{Int,Dict{Int,Dict{Tuple{Int,Int},Int}}}`: scenario → t_id → (o,d) → demand
- `valid_jk_pairs::Dict{Tuple{Int,Int},Vector{Tuple{Int,Int}}}`: (o,d) → valid (j_idx,k_idx)
- `max_walking_distance::Float64`: Walking limit used to compute valid_jk_pairs
- `time_window_sec::Int`: Width of time bucket (seconds)
- `routes_s::Dict{Int,Vector{RouteData}}`: Per-scenario route pool (plain routes, no alpha)
"""
struct VehicleCapacityODMap <: AbstractClusteringMap
    station_id_to_array_idx     :: Dict{Int, Int}
    array_idx_to_station_id     :: Vector{Int}

    scenarios                   :: Vector{ScenarioData}
    scenario_label_to_array_idx :: Dict{String, Int}
    array_idx_to_scenario_label :: Vector{String}

    Omega_s   :: Dict{Int, Vector{Tuple{Int, Int}}}
    Q_s       :: Dict{Int, Dict{Tuple{Int, Int}, Int}}

    Omega_s_t :: Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}
    Q_s_t     :: Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}

    valid_jk_pairs       :: Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    max_walking_distance :: Float64
    time_window_sec      :: Int

    routes_s :: Dict{Int, Vector{RouteData}}
end


"""
    has_walking_distance_limit(mapping::VehicleCapacityODMap) -> Bool
"""
has_walking_distance_limit(mapping::VehicleCapacityODMap) = true


"""
    get_valid_jk_pairs(mapping::VehicleCapacityODMap, o::Int, d::Int) -> Vector{Tuple{Int,Int}}

Return valid (j_idx, k_idx) pairs for OD pair (o_id, d_id).
"""
function get_valid_jk_pairs(mapping::VehicleCapacityODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end


"""
    create_vehicle_capacity_od_map(model::RouteVehicleCapacityModel, data) -> VehicleCapacityODMap

Build the full OD mapping for RouteVehicleCapacityModel (new formulation):
1. Station and scenario index mappings
2. Aggregated OD pairs and demand per scenario (both time-aggregated and time-indexed)
3. Valid (j,k) pairs per OD pair (walking-filtered)
4. Per-scenario plain routes via non-temporal BFS (RouteData, no alpha dict)
"""
function create_vehicle_capacity_od_map(
    model      :: RouteVehicleCapacityModel,
    data       :: StationSelectionData;
    max_labels :: Int = 400_000
)::VehicleCapacityODMap

    # ── 1. Index mappings ──────────────────────────────────────────────────────
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    # ── 2. Aggregated OD pairs per scenario ────────────────────────────────────
    S = n_scenarios(data)
    Omega_s    = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s        = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    Omega_s_t  = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    Q_s_t      = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for s in 1:S
        scenario = data.scenarios[s]
        od_count = compute_scenario_od_count(scenario)
        Omega_s[s] = collect(keys(od_count))
        Q_s[s]     = od_count
        union!(all_od_pairs, keys(od_count))

        # Time-indexed demand (for time-period covering constraints)
        time_to_od = compute_time_to_od_count_mapping(scenario, model.time_window_sec)
        Omega_s_t[s] = Dict{Int, Vector{Tuple{Int, Int}}}()
        Q_s_t[s]     = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
        for (t_id, od_cnt) in time_to_od
            Omega_s_t[s][t_id] = collect(keys(od_cnt))
            Q_s_t[s][t_id]     = od_cnt
        end
    end

    # ── 3. Valid (j,k) pairs per OD pair ──────────────────────────────────────
    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data,
        station_id_to_array_idx, array_idx_to_station_id,
        model.max_walking_distance
    )

    # ── 4. Non-temporal BFS routes (plain RouteData, no alpha) ─────────────────
    routes_s = Dict{Int, Vector{RouteData}}()

    for s in 1:S
        println("  Scenario $s / $S: building non-timed orders (simple)...")
        flush(stdout)

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
        routes_s[s] = generate_simple_routes_from_orders(
            nontimed_orders, data, station_id_to_array_idx;
            vehicle_capacity     = model.vehicle_capacity,
            max_detour_time      = model.max_detour_time,
            max_detour_ratio     = model.max_detour_ratio,
            max_stations_visited = model.max_stations_visited,
            max_labels           = max_labels
        )

        println("  Scenario $s / $S: $(length(routes_s[s])) routes generated")
        flush(stdout)
    end

    return VehicleCapacityODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        Omega_s_t,
        Q_s_t,
        valid_jk_pairs,
        model.max_walking_distance,
        model.time_window_sec,
        routes_s
    )
end
