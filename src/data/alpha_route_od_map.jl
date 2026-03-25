"""
OD mapping for AlphaRouteModel.

Routes are loaded from CSV via `load_routes_and_alpha`. Alpha capacity values are stored
as a fixed parameter dict keyed by (route_id, pickup_station_id, dropoff_station_id).
"""

export AlphaRouteODMap
export create_alpha_route_od_map


"""
    AlphaRouteODMap <: AbstractClusteringMap

Data mapping for AlphaRouteModel. Alpha values are fixed Float64 parameters, not
JuMP variables.

# Fields
- `station_id_to_array_idx`: Station ID → array index
- `array_idx_to_station_id`: Array index → station ID
- `scenarios`: Scenario reference
- `scenario_label_to_array_idx`: Scenario label → index
- `array_idx_to_scenario_label`: Index → scenario label
- `Omega_s`: scenario → OD pairs (aggregated)
- `Q_s`: scenario → (o,d) → demand (aggregated)
- `Omega_s_t`: scenario → t_id → OD pairs
- `Q_s_t`: scenario → t_id → (o,d) → demand
- `valid_jk_pairs`: (o,d) → valid (j_idx,k_idx) pairs
- `max_walking_distance`: Walking limit used to compute valid_jk_pairs
- `time_window_sec`: Time bucket width (seconds)
- `routes_s`: Per-(scenario, time-bucket) route pool; routes filtered from CSV
- `alpha_profile`: Fixed alpha parameters keyed (route_id, pickup_sid, dropoff_sid) → Float64
"""
struct AlphaRouteODMap <: AbstractClusteringMap
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

    routes_s      :: Dict{Int, Dict{Int, Vector{RouteData}}}
    alpha_profile :: Dict{NTuple{3, Int}, Float64}   # (route_id, pickup_sid, dropoff_sid)
end


"""
    has_walking_distance_limit(mapping::AlphaRouteODMap) -> Bool
"""
has_walking_distance_limit(mapping::AlphaRouteODMap) = true


"""
    get_valid_jk_pairs(mapping::AlphaRouteODMap, o::Int, d::Int) -> Vector{Tuple{Int,Int}}
"""
get_valid_jk_pairs(mapping::AlphaRouteODMap, o::Int, d::Int) =
    get(mapping.valid_jk_pairs, (o, d), Tuple{Int,Int}[])


"""
    create_alpha_route_od_map(model::AlphaRouteModel, data) -> AlphaRouteODMap

Build the OD mapping for AlphaRouteModel:
1. Station and scenario index mappings
2. Aggregated OD pairs and demand per scenario (both time-aggregated and time-indexed)
3. Valid (j,k) pairs per OD pair (walking-filtered)
4. Routes loaded from CSV and filtered per (scenario, time-bucket)
5. Alpha profile loaded from CSV
"""
function create_alpha_route_od_map(
    model :: AlphaRouteModel,
    data  :: StationSelectionData
)::AlphaRouteODMap

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
        Omega_s[s] = sort!(collect(keys(od_count)))
        Q_s[s]     = od_count
        union!(all_od_pairs, keys(od_count))

        time_to_od   = compute_time_to_od_count_mapping(scenario, model.time_window_sec)
        Omega_s_t[s] = Dict{Int, Vector{Tuple{Int, Int}}}()
        Q_s_t[s]     = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
        for (t_id, od_cnt) in time_to_od
            Omega_s_t[s][t_id] = sort!(collect(keys(od_cnt)))
            Q_s_t[s][t_id]     = od_cnt
        end
    end

    # ── 3. Valid (j,k) pairs per OD pair ──────────────────────────────────────
    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data,
        station_id_to_array_idx, array_idx_to_station_id,
        model.max_walking_distance
    )

    # ── 4. Load routes from CSV and filter per (scenario, time-bucket) ─────────
    println("  Loading routes from CSV: $(model.routes_file)")
    flush(stdout)
    rio = load_routes_and_alpha(
        model.routes_file,
        model.alpha_profile_file,
        data;
        max_detour_time  = model.max_detour_time,
        max_detour_ratio = model.max_detour_ratio
    )

    routes_s = Dict{Int, Dict{Int, Vector{RouteData}}}()
    for s in 1:S
        routes_s[s] = Dict{Int, Vector{RouteData}}()
        for t_id in sort(collect(keys(Q_s_t[s])))
            # Build (j_sid, k_sid) pairs for this bucket
            jk_sids = Set{Tuple{Int, Int}}()
            for (o, d) in Omega_s_t[s][t_id]
                for (j_idx, k_idx) in get(valid_jk_pairs, (o, d), Tuple{Int,Int}[])
                    push!(jk_sids, (array_idx_to_station_id[j_idx],
                                    array_idx_to_station_id[k_idx]))
                end
            end

            n_requests = sum(values(Q_s_t[s][t_id]); init=0)
            n_od       = length(Omega_s_t[s][t_id])
            print("  Scenario $s/$S, time bucket $t_id: $n_requests requests, $n_od OD pairs, $(length(jk_sids)) (j,k) pairs")
            flush(stdout)

            if isempty(jk_sids)
                routes_s[s][t_id] = RouteData[]
                println()
                flush(stdout)
                continue
            end

            bucket_routes = filter(rio.routes) do r
                any(leg in jk_sids for leg in r.detour_feasible_legs)
            end
            routes_s[s][t_id] = bucket_routes
            println(" → $(length(bucket_routes)) routes from CSV")
            flush(stdout)
        end
    end

    return AlphaRouteODMap(
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
        routes_s,
        rio.alpha_profile
    )
end
