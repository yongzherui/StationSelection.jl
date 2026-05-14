function _route_pool_sorted_routes(state::RoutePoolState)::Vector{RouteData}
    return [state.routes_by_id[rid] for rid in sort!(collect(keys(state.routes_by_id)))]
end

function _route_pool_alpha_for_route(state::RoutePoolState, route_id::Int)
    return Dict(
        (rid, j_idx, k_idx) => value
        for ((rid, j_idx, k_idx), value) in state.alpha_profile
        if rid == route_id
    )
end

function _route_usage_by_id(result::OptResult)::Dict{Int, Float64}
    usage = Dict{Int, Float64}()
    theta_r_ts = get(result.model.obj_dict, :theta_r_ts, Dict{NTuple{3, Int}, VariableRef}())
    for ((s, t_id, r_idx), theta_var) in theta_r_ts
        theta_val = JuMP.value(theta_var)
        theta_val > 0 || continue
        routes_t = get(result.mapping.routes_s[s], t_id, RouteData[])
        r_idx <= length(routes_t) || continue
        route_id = routes_t[r_idx].id
        usage[route_id] = get(usage, route_id, 0.0) + theta_val
    end
    return usage
end

function _prune_route_pool!(
    state::RoutePoolState,
    usage_by_id::Dict{Int, Float64},
    min_theta_to_keep::Float64,
    target_pool_size::Union{Int, Nothing},
    retention_seed::Int
)::Int
    isnothing(target_pool_size) && return 0

    mandatory_ids = Set{Int}()
    for route_id in keys(state.routes_by_id)
        route = state.routes_by_id[route_id]
        if route_id in state.protected_route_ids ||
           get(usage_by_id, route_id, 0.0) > min_theta_to_keep
            push!(mandatory_ids, route_id)
        end
    end

    keep_ids = copy(mandatory_ids)
    if length(state.routes_by_id) > target_pool_size
        inactive_candidates = [
            route_id for route_id in sort!(collect(keys(state.routes_by_id)))
            if route_id ∉ mandatory_ids
        ]
        remaining_slots = max(target_pool_size - length(mandatory_ids), 0)
        if remaining_slots > 0 && !isempty(inactive_candidates)
            ordered = sort(inactive_candidates; by=route_id -> hash((retention_seed, route_id)))
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

function _expand_route_pool!(
    state::RoutePoolState,
    data::StationSelectionData,
    target_max_route_length::Int;
    vehicle_capacity::Int,
    max_walking_distance::Float64,
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64
)::Int
    target_max_route_length > state.current_generated_max_route_length || return 0

    init_spec = RoutePoolInitSpec(
        :generated
    )
    generated_state = initialize_route_pool(
        init_spec,
        data;
        vehicle_capacity=vehicle_capacity,
        max_walking_distance=max_walking_distance,
        max_detour_time=max_detour_time,
        max_detour_ratio=max_detour_ratio,
        stop_dwell_time=stop_dwell_time,
        initial_generated_max_route_length=target_max_route_length
    )

    added = _merge_route_variants!(
        state,
        _route_pool_sorted_routes(generated_state),
        generated_state.alpha_profile,
        :generated_iterative
    )
    state.current_generated_max_route_length = max(state.current_generated_max_route_length, target_max_route_length)
    return added
end
