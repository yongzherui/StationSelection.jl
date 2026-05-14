"""
OD mapping for AlphaRouteModel.

Routes and fixed alpha values can be injected from a prepared route pool or
constructed via the legacy one-shot initialization path.
"""

export AlphaRouteODMap
export create_alpha_route_od_map

struct AlphaRouteODMap <: AbstractClusteringMap
    station_id_to_array_idx     :: Dict{Int, Int}
    array_idx_to_station_id     :: Vector{Int}

    scenarios                   :: Vector{ScenarioData}
    scenario_label_to_array_idx :: Dict{String, Int}
    array_idx_to_scenario_label :: Vector{String}

    Q_s_t     :: Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}

    valid_jk_pairs       :: Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    max_walking_distance :: Float64
    time_window_sec      :: Int

    routes_s      :: Dict{Int, Dict{Int, Vector{RouteData}}}
    alpha_profile :: Dict{NTuple{3, Int}, Float64}
end

has_walking_distance_limit(mapping::AlphaRouteODMap) = true

get_valid_jk_pairs(mapping::AlphaRouteODMap, o::Int, d::Int) =
    get(mapping.valid_jk_pairs, (o, d), Tuple{Int,Int}[])

_time_ids(mapping::Union{VehicleCapacityODMap, AlphaRouteODMap}, s::Int) =
    sort!(collect(keys(mapping.Q_s_t[s])))

_time_od_pairs(mapping::Union{VehicleCapacityODMap, AlphaRouteODMap}, s::Int, t_id::Int) =
    sort!(collect(keys(mapping.Q_s_t[s][t_id])))

function _build_alpha_route_base(
    model :: AlphaRouteModel,
    data  :: StationSelectionData
)
    scenario_label_to_array_idx, array_idx_to_scenario_label =
        create_scenario_label_mappings(data.scenarios)

    S = n_scenarios(data)
    Q_s_t = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for s in 1:S
        scenario = data.scenarios[s]
        time_to_od = compute_time_to_od_count_mapping(scenario, model.time_window_sec)
        Q_s_t[s] = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
        for (t_id, od_cnt) in time_to_od
            Q_s_t[s][t_id] = od_cnt
            union!(all_od_pairs, keys(od_cnt))
        end
    end

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs, data,
        model.max_walking_distance
    )

    return (
        scenario_label_to_array_idx=scenario_label_to_array_idx,
        array_idx_to_scenario_label=array_idx_to_scenario_label,
        Q_s_t=Q_s_t,
        valid_jk_pairs=valid_jk_pairs,
        S=S,
    )
end

function _filter_alpha_route_pool_by_bucket(
    route_pool_state::RoutePoolState,
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}},
    S::Int
)::Dict{Int, Dict{Int, Vector{RouteData}}}
    route_alpha_jk = Dict{Int, Vector{Tuple{Int, Int}}}()
    for (route_id, j_idx, k_idx) in keys(route_pool_state.alpha_profile)
        push!(get!(route_alpha_jk, route_id, Tuple{Int, Int}[]), (j_idx, k_idx))
    end

    all_routes = _route_pool_sorted_routes(route_pool_state)
    routes_s = Dict{Int, Dict{Int, Vector{RouteData}}}()
    for s in 1:S
        routes_s[s] = Dict{Int, Vector{RouteData}}()
        for t_id in sort!(collect(keys(Q_s_t[s])))
            jk_set = Set{Tuple{Int, Int}}()
            od_pairs = sort!(collect(keys(Q_s_t[s][t_id])))
            for (o, d) in od_pairs
                for (j_idx, k_idx) in get(valid_jk_pairs, (o, d), Tuple{Int,Int}[])
                    push!(jk_set, (j_idx, k_idx))
                end
            end

            n_requests = sum(values(Q_s_t[s][t_id]); init=0)
            n_od = length(od_pairs)
            print("  Scenario $s/$S, time bucket $t_id: $n_requests requests, $n_od OD pairs, $(length(jk_set)) (j,k) pairs")
            flush(stdout)

            if isempty(jk_set)
                routes_s[s][t_id] = RouteData[]
                println()
                flush(stdout)
                continue
            end

            bucket_routes = RouteData[]
            for route in all_routes
                alpha_jk = get(route_alpha_jk, route.id, Tuple{Int, Int}[])
                min_covered = length(route.station_indices) == 2 ? 1 : 2
                count(jk -> jk ∈ jk_set, alpha_jk) >= min_covered &&
                    push!(bucket_routes, route)
            end
            routes_s[s][t_id] = bucket_routes
            println(" → $(length(bucket_routes)) routes")
            flush(stdout)
        end
    end
    return routes_s
end

function _legacy_alpha_route_init_spec(model::AlphaRouteModel)::RoutePoolInitSpec
    if model.generate_routes
        return RoutePoolInitSpec(
            :generated
        )
    end
    return RoutePoolInitSpec(
        :file,
        routes_file=model.routes_file,
        alpha_profile_file=model.alpha_profile_file
    )
end

function create_alpha_route_od_map(
    model :: AlphaRouteModel,
    data  :: StationSelectionData,
    route_pool_state::RoutePoolState
)::AlphaRouteODMap
    base = _build_alpha_route_base(model, data)
    routes_s = _filter_alpha_route_pool_by_bucket(
        route_pool_state,
        base.valid_jk_pairs,
        base.Q_s_t,
        base.S
    )

    return AlphaRouteODMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        base.scenario_label_to_array_idx,
        base.array_idx_to_scenario_label,
        base.Q_s_t,
        base.valid_jk_pairs,
        model.max_walking_distance,
        model.time_window_sec,
        routes_s,
        copy(route_pool_state.alpha_profile)
    )
end

function create_alpha_route_od_map(
    model :: AlphaRouteModel,
    data  :: StationSelectionData
)::AlphaRouteODMap
    route_pool_state = initialize_route_pool(
        _legacy_alpha_route_init_spec(model),
        data;
        vehicle_capacity=model.vehicle_capacity,
        max_walking_distance=model.max_walking_distance,
        max_detour_time=model.max_detour_time,
        max_detour_ratio=model.max_detour_ratio,
        stop_dwell_time=model.stop_dwell_time,
        initial_generated_max_route_length=model.generate_routes ? model.max_route_length : nothing
    )
    return create_alpha_route_od_map(model, data, route_pool_state)
end
