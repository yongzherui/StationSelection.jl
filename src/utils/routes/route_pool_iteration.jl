function _route_pool_alpha_for_route(state::RoutePoolState, route_id::Int)
    return Dict(
        (rid, j_idx, k_idx) => value
        for ((rid, j_idx, k_idx), value) in state.alpha_profile
        if rid == route_id
    )
end

function _route_usage_by_bucket(result::OptResult)::Dict{Tuple{Int, Int}, Dict{Int, Float64}}
    usage = Dict{Tuple{Int, Int}, Dict{Int, Float64}}()
    theta_r_ts = get(result.model.obj_dict, :theta_r_ts, Dict{NTuple{3, Int}, VariableRef}())
    for ((s, t_id, r_idx), theta_var) in theta_r_ts
        theta_val = JuMP.value(theta_var)
        theta_val > 0 || continue
        routes_t = get(result.mapping.routes_s[s], t_id, RouteData[])
        r_idx <= length(routes_t) || continue
        route_id = routes_t[r_idx].id
        bucket_usage = get!(usage, (s, t_id), Dict{Int, Float64}())
        bucket_usage[route_id] = get(bucket_usage, route_id, 0.0) + theta_val
    end
    return usage
end

function _effective_bucket_target_size(bucket_state::RoutePoolState, bucket_x_multiplier::Float64)::Int
    min_required = length(bucket_state.direct_seed_route_ids)
    proportional_cap = ceil(Int, bucket_x_multiplier * bucket_state.x_candidate_count)
    return max(min_required, proportional_cap)
end

function _mandatory_route_ids(
    state::RoutePoolState,
    usage_by_id::Dict{Int, Float64},
    min_theta_to_keep::Float64
)::Set{Int}
    mandatory_ids = Set{Int}()
    for route_id in keys(state.routes_by_id)
        if route_id in state.protected_route_ids ||
           get(usage_by_id, route_id, 0.0) > min_theta_to_keep
            push!(mandatory_ids, route_id)
        end
    end
    return mandatory_ids
end

function _prune_route_pool!(
    state::RoutePoolState,
    usage_by_id::Dict{Int, Float64},
    min_theta_to_keep::Float64,
    bucket_target_size::Int,
    retention_seed::Int
)::Int
    effective_target = max(length(state.direct_seed_route_ids), bucket_target_size)
    mandatory_ids = _mandatory_route_ids(state, usage_by_id, min_theta_to_keep)
    keep_ids = copy(mandatory_ids)
    if length(state.routes_by_id) > effective_target
        inactive_candidates = [
            route_id for route_id in sort!(collect(keys(state.routes_by_id)))
            if route_id ∉ mandatory_ids
        ]
        remaining_slots = max(effective_target - length(mandatory_ids), 0)
        if remaining_slots > 0 && !isempty(inactive_candidates)
            ordered = sort(inactive_candidates; by=route_id -> hash((retention_seed, state.scenario_idx, state.time_id, route_id)))
            for route_id in ordered[1:min(remaining_slots, length(ordered))]
                push!(keep_ids, route_id)
            end
        end
    else
        union!(keep_ids, keys(state.routes_by_id))
    end

    removed = 0
    for route_id in sort!(collect(setdiff(Set(keys(state.routes_by_id)), keep_ids)))
        route = state.routes_by_id[route_id]
        alpha_sig = _route_alpha_signature(route, _route_pool_alpha_for_route(state, route_id))
        delete!(state.routes_by_id, route_id)
        delete!(state.signature_to_route_id, alpha_sig)
        push!(state.removed_route_ids, route_id)
        delete!(state.provenance_by_route_id, route_id)
        for key in [key for key in keys(state.alpha_profile) if key[1] == route_id]
            delete!(state.alpha_profile, key)
        end
        removed += 1
    end
    return removed
end

function _enforce_global_total_route_cap!(
    global_state::AlphaRouteBucketPoolsState,
    usage_by_bucket::Dict{Tuple{Int, Int}, Dict{Int, Float64}},
    min_theta_to_keep::Float64,
    total_target_size::Int,
    retention_seed::Int
)::NamedTuple{(:removed, :buckets_touched), Tuple{Int, Int}}
    total_routes = sum(length(bucket.routes_by_id) for bucket in values(global_state.bucket_states))
    total_routes <= total_target_size && return (removed=0, buckets_touched=0)

    removable = Tuple{Int, Int, Int}[]
    mandatory_count = 0
    for (bucket_key, bucket_state) in global_state.bucket_states
        mandatory_ids = _mandatory_route_ids(bucket_state, get(usage_by_bucket, bucket_key, Dict{Int, Float64}()), min_theta_to_keep)
        mandatory_count += length(mandatory_ids)
        for route_id in keys(bucket_state.routes_by_id)
            route_id in mandatory_ids && continue
            push!(removable, (bucket_key[1], bucket_key[2], route_id))
        end
    end

    mandatory_count >= total_target_size && return (removed=0, buckets_touched=0)
    n_to_remove = min(total_routes - total_target_size, length(removable))
    n_to_remove <= 0 && return (removed=0, buckets_touched=0)

    ordered = sort(removable; by=x -> hash((retention_seed, x[1], x[2], x[3])))
    removed = 0
    touched_buckets = Set{Tuple{Int, Int}}()
    for (s, t_id, route_id) in ordered[1:n_to_remove]
        bucket_state = global_state.bucket_states[(s, t_id)]
        route = bucket_state.routes_by_id[route_id]
        alpha_sig = _route_alpha_signature(route, _route_pool_alpha_for_route(bucket_state, route_id))
        delete!(bucket_state.routes_by_id, route_id)
        delete!(bucket_state.signature_to_route_id, alpha_sig)
        push!(bucket_state.removed_route_ids, route_id)
        delete!(bucket_state.provenance_by_route_id, route_id)
        for key in [key for key in keys(bucket_state.alpha_profile) if key[1] == route_id]
            delete!(bucket_state.alpha_profile, key)
        end
        push!(touched_buckets, (s, t_id))
        removed += 1
    end
    return (removed=removed, buckets_touched=length(touched_buckets))
end

function _expand_route_pool!(
    global_state::AlphaRouteBucketPoolsState,
    bucket_state::RoutePoolState,
    data::StationSelectionData,
    target_max_route_length::Int;
    vehicle_capacity::Int,
    iterative_config::Union{Nothing, IterativeRouteGenerationConfig}=nothing,
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64
)
    target_max_route_length > bucket_state.current_generated_max_route_length ||
        return (added=0, n_iters=0, n_seeds=0, n_alpha=0, added_by_strategy=(geometry=0, coverage=0, interior=0, endpoint=0, reverse=0))

    seed_routes = collect(values(bucket_state.routes_by_id))
    exp_config  = isnothing(iterative_config) ?
        default_iterative_route_generation_config(target_max_route_length) :
        _with_max_route_length(iterative_config, target_max_route_length)

    ins_result = generate_routes_by_insertion(
        seed_routes,
        bucket_state.valid_jk_pairs,
        data;
        config=exp_config,
        max_detour_time=max_detour_time,
        max_detour_ratio=max_detour_ratio,
        stop_dwell_time=stop_dwell_time,
    )
    alpha           = derive_balanced_alpha(ins_result.routes, vehicle_capacity)
    alpha_before    = length(bucket_state.alpha_profile)
    added           = _merge_route_variants!(global_state, bucket_state, ins_result.routes, alpha, :expanded_insertion)
    n_alpha_added   = length(bucket_state.alpha_profile) - alpha_before
    bucket_state.current_generated_max_route_length = max(bucket_state.current_generated_max_route_length, target_max_route_length)
    return (added=added, n_iters=ins_result.n_iters, n_seeds=length(seed_routes), n_alpha=n_alpha_added, added_by_strategy=ins_result.added_by_strategy)
end
