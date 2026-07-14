export initialize_route_pool

function _route_alpha_signature(route::RouteData, alpha_profile::Dict{NTuple{3, Int}, Float64})::String
    legs = String[]
    for (rid, j_idx, k_idx) in sort!(collect(keys(alpha_profile)); by=x -> (x[2], x[3], x[1]))
        rid == route.id || continue
        push!(legs, "$(j_idx)>$(k_idx)=$(repr(alpha_profile[(rid, j_idx, k_idx)]))")
    end
    return join(route.station_indices, "|") * "#" * join(legs, ";")
end

function _empty_route_pool_state(
    scenario_idx::Int,
    time_id::Int,
    valid_jk_pairs::Set{Tuple{Int, Int}},
    x_candidate_count::Int
)::RoutePoolState
    return RoutePoolState(
        scenario_idx,
        time_id,
        copy(valid_jk_pairs),
        x_candidate_count,
        Dict{Int, RouteData}(),
        Dict{NTuple{3, Int}, Float64}(),
        Dict{String, Int}(),
        Dict{Int, Set{Symbol}}(),
        Set{Int}(),
        Set{Int}(),
        Set{Int}(),
        0,
    )
end

function _copy_route_with_id(route::RouteData, route_id::Int)::RouteData
    return RouteData(route_id, copy(route.station_indices), route.travel_time, copy(route.detour_feasible_legs))
end

function _allocate_route_id!(state::ExactDARPRouteBucketPoolsState)::Int
    route_id = state.next_global_route_id
    state.next_global_route_id += 1
    return route_id
end

function _insert_route_variant!(
    global_state::ExactDARPRouteBucketPoolsState,
    bucket_state::RoutePoolState,
    route::RouteData,
    source_alpha::Dict{NTuple{3, Int}, Float64},
    provenance::Symbol;
    protect_direct::Bool=false
)::Tuple{Int, Bool}
    signature = _route_alpha_signature(route, source_alpha)
    if haskey(bucket_state.signature_to_route_id, signature)
        route_id = bucket_state.signature_to_route_id[signature]
        push!(get!(bucket_state.provenance_by_route_id, route_id, Set{Symbol}()), provenance)
        if protect_direct
            push!(bucket_state.protected_route_ids, route_id)
            push!(bucket_state.direct_seed_route_ids, route_id)
        end
        for (rid, j_idx, k_idx) in keys(source_alpha)
            rid == route.id || continue
            new_key = (route_id, j_idx, k_idx)
            bucket_state.alpha_profile[new_key] = max(
                get(bucket_state.alpha_profile, new_key, 0.0),
                source_alpha[(rid, j_idx, k_idx)]
            )
        end
        return route_id, false
    end

    route_id = _allocate_route_id!(global_state)
    bucket_state.routes_by_id[route_id] = _copy_route_with_id(route, route_id)
    bucket_state.signature_to_route_id[signature] = route_id
    bucket_state.provenance_by_route_id[route_id] = Set([provenance])
    if protect_direct
        push!(bucket_state.protected_route_ids, route_id)
        push!(bucket_state.direct_seed_route_ids, route_id)
    end

    for (rid, j_idx, k_idx) in keys(source_alpha)
        rid == route.id || continue
        bucket_state.alpha_profile[(route_id, j_idx, k_idx)] = source_alpha[(rid, j_idx, k_idx)]
    end
    return route_id, true
end

function _merge_route_variants!(
    global_state::ExactDARPRouteBucketPoolsState,
    bucket_state::RoutePoolState,
    routes::Vector{RouteData},
    alpha_profile::Dict{NTuple{3, Int}, Float64},
    provenance::Symbol;
    protect_direct::Bool=false
)::Int
    added = 0
    for route in routes
        _, inserted = _insert_route_variant!(
            global_state,
            bucket_state,
            route,
            alpha_profile,
            provenance;
            protect_direct=protect_direct && length(route.station_indices) == 2
        )
        added += inserted ? 1 : 0
    end
    return added
end

function _build_direct_route_variants(
    jk_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData,
    vehicle_capacity::Int
)::Tuple{Vector{RouteData}, Dict{NTuple{3, Int}, Float64}}
    routes = RouteData[]
    alpha = Dict{NTuple{3, Int}, Float64}()
    next_id = 1
    for (j_idx, k_idx) in sort!(collect(jk_pairs))
        tt = get_routing_cost(data, j_idx, k_idx)
        route = RouteData(next_id, [j_idx, k_idx], tt, [(j_idx, k_idx)])
        push!(routes, route)
        alpha[(next_id, j_idx, k_idx)] = Float64(vehicle_capacity)
        next_id += 1
    end
    return routes, alpha
end

function _bucket_routes_from_route_io(
    rio::RouteIOData,
    jk_pairs::Set{Tuple{Int, Int}}
)::Tuple{Vector{RouteData}, Dict{NTuple{3, Int}, Float64}}
    routes = RouteData[]
    alpha = Dict{NTuple{3, Int}, Float64}()
    for route in rio.routes
        route_alpha = Tuple{Int, Int}[]
        local_alpha = Dict{NTuple{3, Int}, Float64}()
        for ((rid, j_idx, k_idx), value) in rio.alpha_profile
            rid == route.id || continue
            (j_idx, k_idx) in jk_pairs || continue
            local_alpha[(route.id, j_idx, k_idx)] = value
            push!(route_alpha, (j_idx, k_idx))
        end
        min_covered = length(route.station_indices) == 2 ? 1 : 2
        length(route_alpha) >= min_covered || continue
        push!(routes, route)
        merge!(alpha, local_alpha)
    end
    return routes, alpha
end

function _bucket_key_route_pools(
    Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}},
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
)::Dict{Tuple{Int, Int}, Tuple{Set{Tuple{Int, Int}}, Int}}
    bucket_info = Dict{Tuple{Int, Int}, Tuple{Set{Tuple{Int, Int}}, Int}}()
    for (s, by_t) in Q_s_t
        for (t_id, od_cnt) in by_t
            jk_set = Set{Tuple{Int, Int}}()
            x_candidate_count = 0
            for (o, d) in keys(od_cnt)
                valid = get(valid_jk_pairs, (o, d), Tuple{Int, Int}[])
                union!(jk_set, valid)
                x_candidate_count += length(valid)
            end
            bucket_info[(s, t_id)] = (jk_set, x_candidate_count)
        end
    end
    return bucket_info
end

function initialize_route_pool(
    init_spec::RoutePoolInitSpec,
    data::StationSelectionData,
    Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}},
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}};
    vehicle_capacity::Int,
    route_generation_method::Symbol=:dfs,
    iterative_config::Union{Nothing, IterativeRouteGenerationConfig}=nothing,
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
    initial_generated_max_route_length::Union{Int, Nothing}=nothing
)::ExactDARPRouteBucketPoolsState
    n_buckets = sum(length(by_t) for by_t in values(Q_s_t))
    @info "initialize_route_pool: starting" mode=init_spec.mode n_buckets=n_buckets vehicle_capacity=vehicle_capacity route_generation_method=route_generation_method initial_max_route_length=initial_generated_max_route_length

    global_state      = ExactDARPRouteBucketPoolsState(Dict{Tuple{Int, Int}, RoutePoolState}(), 1)
    bucket_info       = _bucket_key_route_pools(Q_s_t, valid_jk_pairs)
    n_generated_total = 0

    rio = nothing
    if init_spec.mode in (:file, :combined)
        rio = load_routes_and_alpha(
            init_spec.routes_file, init_spec.alpha_profile_file, data;
            max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio,
        )
        @info "initialize_route_pool: loaded routes from file" n_routes=length(rio.routes) n_alpha=length(rio.alpha_profile)
    end

    for ((s, t_id), (jk_set, x_candidate_count)) in sort!(collect(bucket_info); by=first)
        bucket_state   = _empty_route_pool_state(s, t_id, jk_set, x_candidate_count)
        direct_routes, direct_alpha = _build_direct_route_variants(jk_set, data, vehicle_capacity)
        n_direct = _merge_route_variants!(global_state, bucket_state, direct_routes, direct_alpha, :direct_seed; protect_direct=true)

        n_file = 0
        if init_spec.mode in (:file, :combined) && !isnothing(rio)
            file_routes, file_alpha = _bucket_routes_from_route_io(rio, jk_set)
            n_file = _merge_route_variants!(global_state, bucket_state, file_routes, file_alpha, :file)
        end

        n_generated = 0
        if init_spec.mode in (:generated, :combined)
            !isnothing(initial_generated_max_route_length) ||
                throw(ArgumentError("initial_generated_max_route_length is required for generated/combined route-pool initialization"))
            routes, alpha = generate_routes_and_alpha(
                data, jk_set;
                vehicle_capacity=vehicle_capacity,
                max_route_length=initial_generated_max_route_length,
                route_generation_method=route_generation_method,
                iterative_config=iterative_config,
                max_detour_time=max_detour_time,
                max_detour_ratio=max_detour_ratio,
                stop_dwell_time=stop_dwell_time,
            )
            n_generated = _merge_route_variants!(global_state, bucket_state, routes, alpha, :generated)
            bucket_state.current_generated_max_route_length = initial_generated_max_route_length
            n_generated_total += n_generated
        end

        if init_spec.mode == :direct_only
            bucket_state.current_generated_max_route_length = 2
        end

        @debug "initialize_route_pool: bucket" s=s t_id=t_id n_jk_pairs=length(jk_set) n_direct=n_direct n_file=n_file n_generated=n_generated total=length(bucket_state.routes_by_id)
        global_state.bucket_states[(s, t_id)] = bucket_state
    end

    n_routes_total = sum(length(b.routes_by_id) for b in values(global_state.bucket_states))
    n_alpha_total  = sum(length(b.alpha_profile) for b in values(global_state.bucket_states))
    @info "initialize_route_pool: done" n_buckets=length(global_state.bucket_states) n_routes_total=n_routes_total n_generated_total=n_generated_total n_alpha_entries_total=n_alpha_total
    return global_state
end

function _sorted_bucket_route_pool_keys(global_state::ExactDARPRouteBucketPoolsState)
    return sort!(collect(keys(global_state.bucket_states)))
end

function _route_pool_sorted_routes(state::RoutePoolState)::Vector{RouteData}
    return [state.routes_by_id[rid] for rid in sort!(collect(keys(state.routes_by_id)))]
end
