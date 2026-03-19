"""
OD mapping with time windows and pre-generated routes for TwoStageRouteWithTimeModel.
"""

export TwoStageRouteODMap
export create_two_stage_route_od_map

"""
    TwoStageRouteODMap <: AbstractClusteringMap

Data mapping for TwoStageRouteWithTimeModel. Extends the clustering OD structure with
time-indexed OD pairs and pre-generated vehicle routes.

# Fields
- `station_id_to_array_idx::Dict{Int,Int}`: Station ID → array index
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID
- `scenarios::Vector{ScenarioData}`: Scenario reference
- `scenario_label_to_array_idx::Dict{String,Int}`: Scenario label → index
- `array_idx_to_scenario_label::Vector{String}`: Index → scenario label
- `Omega_s_t::Dict{Int,Dict{Int,Vector{Tuple{Int,Int}}}}`: scenario → time_id → OD pairs
- `Q_s_t::Dict{Int,Dict{Int,Dict{Tuple{Int,Int},Int}}}`: scenario → time_id → (o,d) → demand
- `valid_jk_pairs::Dict{Tuple{Int,Int},Vector{Tuple{Int,Int}}}`: (o_id,d_id) → valid (j_idx,k_idx) pairs
- `max_walking_distance::Float64`: Walking limit
- `time_window_sec::Int`: Time window size in seconds
- `routes_s::Dict{Int,Vector{TimedRouteData}}`:
  Per-scenario route pool from cross-window BFS. `routes_s[s]` = routes for scenario s.
- `routes_by_jkt_s::Dict{NTuple{4,Int},Vector{Tuple{Int,Int}}}`:
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

    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    max_walking_distance::Float64
    time_window_sec::Int

    routes_s::Dict{Int, Vector{TimedRouteData}}
    routes_by_jkt_s::Dict{NTuple{4,Int}, Vector{Tuple{Int,Int}}}
end


"""
    has_walking_distance_limit(mapping::TwoStageRouteODMap) -> Bool
"""
has_walking_distance_limit(mapping::TwoStageRouteODMap) = true


"""
    get_valid_jk_pairs(mapping::TwoStageRouteODMap, o::Int, d::Int) -> Vector{Tuple{Int,Int}}

Return valid (j_idx, k_idx) pairs for OD pair (o_id, d_id).
"""
function get_valid_jk_pairs(mapping::TwoStageRouteODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end


"""
    create_two_stage_route_od_map(model::TwoStageRouteWithTimeModel, data::StationSelectionData)
    -> TwoStageRouteODMap

Build the full OD mapping for TwoStageRouteWithTimeModel:
1. Station and scenario index mappings
2. Time-indexed OD pairs and demand per scenario
3. Valid (j,k) pairs per OD pair (walking-filtered)
4. Per-scenario routes via cross-window temporal BFS
"""
function create_two_stage_route_od_map(
    model::TwoStageRouteWithTimeModel,
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
        # time_to_od is in format time_to_od[t][(o, d)] = count
        time_to_od = compute_time_to_od_count_mapping(scenario, model.time_window_sec)

        Omega_s_t[s] = Dict{Int, Vector{Tuple{Int, Int}}}()
        # thus, this is in Q_s_t[s][t][(o, d)] = count
        Q_s_t[s]     = Dict{Int, Dict{Tuple{Int, Int}, Int}}()

        for (t_id, od_count) in time_to_od
            Omega_s_t[s][t_id] = collect(keys(od_count))
            Q_s_t[s][t_id]     = od_count
            union!(all_od_pairs, keys(od_count))
        end
    end

    # ── 3. Valid (j,k) pairs per OD pair ──────────────────────────────────────
    # Walking-feasible pairs (no route-coverage filter needed since global pool is gone).
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    walking_valid = compute_valid_jk_pairs(
        all_od_pairs, data,
        station_id_to_array_idx, array_idx_to_station_id,
        model.max_walking_distance
    )
    for (o, d) in all_od_pairs
        valid_jk_pairs[(o, d)] = get(walking_valid, (o, d), Tuple{Int, Int}[])
    end

    max_walking_distance = model.max_walking_distance

    # ── 4. Temporal BFS routes ─────────────────────────────────────────────────
    routes_s        = Dict{Int, Vector{TimedRouteData}}()
    routes_by_jkt_s = Dict{NTuple{4,Int}, Vector{Tuple{Int,Int}}}()

    for s in 1:S
        println("  Scenario $s / $S: building timed orders...")
        flush(stdout)

        # Build _TimedOrder list for this scenario
        timed_orders = _TimedOrder[]
        for (t_id, od_count) in Q_s_t[s]
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

        println("  Scenario $s / $S: running BFS with $(length(timed_orders)) timed orders...")
        flush(stdout)
        routes_s[s] = generate_routes_from_timed_orders(
            timed_orders, data, station_id_to_array_idx;
            vehicle_capacity = model.vehicle_capacity,
            max_wait_time    = model.max_wait_time,
            max_delay_time   = model.max_detour_time,
            max_delay_ratio  = model.max_detour_ratio,
            time_window_sec  = model.time_window_sec
        )

        println("  Scenario $s / $S: $(length(routes_s[s])) routes generated")
        flush(stdout)

        # Build (s, t_id, j_idx, k_idx) → [(route_idx_within_s, α)] index
        for (r_idx, trd) in enumerate(routes_s[s])
            for ((t_id, (j_idx, k_idx)), α) in trd.alpha
                key = (s, t_id, j_idx, k_idx)
                push!(get!(routes_by_jkt_s, key, Tuple{Int,Int}[]), (r_idx, α))
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
        valid_jk_pairs,
        max_walking_distance,
        model.time_window_sec,
        routes_s,
        routes_by_jkt_s
    )
end

