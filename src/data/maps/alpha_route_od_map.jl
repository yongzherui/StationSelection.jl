"""
OD mapping for AlphaRouteModel.

Routes and fixed alpha values are maintained per (scenario, time-bucket) pool.
"""

export AlphaRouteODMap
export create_alpha_route_od_map

struct AlphaRouteODMap <: AbstractClusteringMap
    station_id_to_array_idx     :: Dict{Int, Int}
    array_idx_to_station_id     :: Vector{Int}
    scenarios                   :: Vector{ScenarioData}
    scenario_label_to_array_idx :: Dict{String, Int}
    array_idx_to_scenario_label :: Vector{String}
    Q_s_t                       :: Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}
    valid_jk_pairs              :: Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    max_walking_distance        :: Float64
    time_window_sec             :: Int
    routes_s                    :: Dict{Int, Dict{Int, Vector{RouteData}}}
    alpha_profile               :: Dict{NTuple{3, Int}, Float64}
end

has_walking_distance_limit(mapping::AlphaRouteODMap) = true

get_valid_jk_pairs(mapping::AlphaRouteODMap, o::Int, d::Int) =
    get(mapping.valid_jk_pairs, (o, d), Tuple{Int,Int}[])

_time_ids(mapping::Union{VehicleCapacityODMap, AlphaRouteODMap}, s::Int) =
    sort!(collect(keys(mapping.Q_s_t[s])))

_time_od_pairs(mapping::Union{VehicleCapacityODMap, AlphaRouteODMap}, s::Int, t_id::Int) =
    sort!(collect(keys(mapping.Q_s_t[s][t_id])))

function _build_alpha_route_base(
    model::AlphaRouteModel,
    data::StationSelectionData
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

    valid_jk_pairs = compute_valid_jk_pairs(all_od_pairs, data, model.max_walking_distance)

    return (
        scenario_label_to_array_idx=scenario_label_to_array_idx,
        array_idx_to_scenario_label=array_idx_to_scenario_label,
        Q_s_t=Q_s_t,
        valid_jk_pairs=valid_jk_pairs,
        S=S,
    )
end

function _bucket_route_pool_to_mapping(
    bucket_state::RoutePoolState,
    n_requests::Int,
    n_od::Int
)::Vector{RouteData}
    routes = _route_pool_sorted_routes(bucket_state)
    @debug "_bucket_route_pool_to_mapping" scenario=bucket_state.scenario_idx time_id=bucket_state.time_id n_requests=n_requests n_od=n_od n_jk_pairs=length(bucket_state.valid_jk_pairs) n_routes=length(routes)
    return routes
end

function _collect_alpha_profile(global_state::AlphaRouteBucketPoolsState)
    alpha_profile = Dict{NTuple{3, Int}, Float64}()
    for bucket_state in values(global_state.bucket_states)
        merge!(alpha_profile, bucket_state.alpha_profile)
    end
    return alpha_profile
end

function _legacy_alpha_route_init_spec(model::AlphaRouteModel)::RoutePoolInitSpec
    if model.generate_routes
        return RoutePoolInitSpec(:generated)
    end
    return RoutePoolInitSpec(
        :file,
        routes_file=model.routes_file,
        alpha_profile_file=model.alpha_profile_file
    )
end

function create_alpha_route_od_map(
    model::AlphaRouteModel,
    data::StationSelectionData,
    bucket_pools::AlphaRouteBucketPoolsState
)::AlphaRouteODMap
    base = _build_alpha_route_base(model, data)
    routes_s = Dict{Int, Dict{Int, Vector{RouteData}}}()
    for s in 1:base.S
        routes_s[s] = Dict{Int, Vector{RouteData}}()
        for t_id in sort!(collect(keys(base.Q_s_t[s])))
            od_cnt = base.Q_s_t[s][t_id]
            n_requests = sum(values(od_cnt); init=0)
            n_od = length(od_cnt)
            bucket_state = get(bucket_pools.bucket_states, (s, t_id), nothing)
            routes_s[s][t_id] = isnothing(bucket_state) ? RouteData[] : _bucket_route_pool_to_mapping(bucket_state, n_requests, n_od)
        end
    end

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
        _collect_alpha_profile(bucket_pools)
    )
end

function create_alpha_route_od_map(
    model::AlphaRouteModel,
    data::StationSelectionData
)::AlphaRouteODMap
    base = _build_alpha_route_base(model, data)
    bucket_pools = initialize_route_pool(
        _legacy_alpha_route_init_spec(model),
        data,
        base.Q_s_t,
        base.valid_jk_pairs;
        vehicle_capacity=model.vehicle_capacity,
        route_generation_method=model.route_generation_method,
        iterative_config=model.iterative_route_generation_config,
        max_detour_time=model.max_detour_time,
        max_detour_ratio=model.max_detour_ratio,
        stop_dwell_time=model.stop_dwell_time,
        initial_generated_max_route_length=model.generate_routes ? model.max_route_length : nothing
    )
    return create_alpha_route_od_map(model, data, bucket_pools)
end
