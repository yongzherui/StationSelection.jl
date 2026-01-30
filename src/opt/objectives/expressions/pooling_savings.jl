"""
Pooling savings expressions for objective functions.

These functions return AffExpr representing the savings from pooling
(same-source and same-destination detours).

Note: These return POSITIVE values representing savings, so they should
be SUBTRACTED from the total objective cost.
"""

using JuMP

export same_source_pooling_savings_expr
export same_dest_pooling_savings_expr


"""
    same_source_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}};
        routing_weight::Float64=1.0
    ) -> AffExpr

Compute the same-source pooling savings expression.

For each same-source triplet (j, k, l), pooling trips (j→l) and (k→l) saves:
    γ · r_{jl,kl} · u[s][t][local_idx]

Where:
- γ (routing_weight) = weight for routing costs
- r_{jl,kl} = c_{jl} - c_{kl} = routing savings from pooling
- u[s][t][local_idx] = 1 if pooling is used

When walking limits are enabled, uses mapping.feasible_same_source to get
feasible detour indices. Otherwise, all triplets are considered feasible.

Returns a POSITIVE AffExpr representing savings (subtract from total cost).
Only includes terms where r > 0 (actual savings exist).
"""
function same_source_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}};
        routing_weight::Float64=1.0
    )::AffExpr

    S = n_scenarios(data)
    u = m[:u]
    n_same_source = length(Xi_same_source)

    use_sparse = has_walking_distance_limit(mapping)

    # Precompute pooling savings for same-source triplets
    # r_{jl,kl} = c_{jl} - c_{kl}
    r_same_source = Float64[]
    for (j, k, l) in Xi_same_source
        c_jl = get_routing_cost(data, j, l)
        c_kl = get_routing_cost(data, k, l)
        push!(r_same_source, c_jl - c_kl)
    end

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # Only add pooling savings if there are multiple OD pairs
            if length(od_vector) <= 1
                continue
            end

            # Get feasible indices for this (s, t)
            if use_sparse
                feasible_indices = get(mapping.feasible_same_source[s], time_id, Int[])
            else
                feasible_indices = collect(1:n_same_source)
            end

            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                r = r_same_source[global_idx]
                if r > 0  # Only add if there's actual savings
                    add_to_expression!(expr, routing_weight * r, u[s][time_id][local_idx])
                end
            end
        end
    end

    return expr
end


"""
    same_source_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_source::Vector{Tuple{Int, Int, Int}};
        routing_weight::Float64=1.0
    ) -> AffExpr

Same-source pooling savings for TwoStageSingleDetourNoWalkingLimitModel.
"""
function same_source_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_source::Vector{Tuple{Int, Int, Int}};
        routing_weight::Float64=1.0
    )::AffExpr

    S = n_scenarios(data)
    u = m[:u]
    u_idx_map = m[:u_idx_map]

    r_same_source = Float64[]
    for (j, k, l) in Xi_same_source
        c_jl = get_routing_cost(data, j, l)
        c_kl = get_routing_cost(data, k, l)
        push!(r_same_source, c_jl - c_kl)
    end

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            if length(od_vector) <= 1
                continue
            end

            feasible_indices = get(u_idx_map[s], time_id, Int[])
            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                r = r_same_source[global_idx]
                if r > 0
                    add_to_expression!(expr, routing_weight * r, u[s][time_id][local_idx])
                end
            end
        end
    end

    return expr
end


"""
    same_dest_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    ) -> AffExpr

Compute the same-destination pooling savings expression.

For each same-dest quadruplet (j, k, l, t'), pooling trips (j→l) and (j→k) saves:
    γ · r_{jl,jk} · v[s][t][local_idx]

Where:
- γ (routing_weight) = weight for routing costs
- r_{jl,jk} = c_{jl} - c_{jk} = routing savings from pooling
- v[s][t][local_idx] = 1 if pooling is used

When walking limits are enabled, uses mapping.feasible_same_dest to get
feasible detour indices.

Returns a POSITIVE AffExpr representing savings (subtract from total cost).
Only includes terms where r > 0 (actual savings exist).
"""
function same_dest_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    )::AffExpr

    S = n_scenarios(data)
    v = m[:v]

    use_sparse = has_walking_distance_limit(mapping)

    # Precompute pooling savings for same-dest quadruplets
    # r_{jl,jk} = c_{jl} - c_{jk}
    r_same_dest = Float64[]
    for (j, k, l, _) in Xi_same_dest
        c_jl = get_routing_cost(data, j, l)
        c_jk = get_routing_cost(data, j, k)
        push!(r_same_dest, c_jl - c_jk)
    end

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, _) in mapping.Omega_s_t[s]
            # Get feasible indices
            if use_sparse
                feasible_indices = get(mapping.feasible_same_dest[s], time_id, Int[])
            else
                # Without walking limits, check time validity
                feasible_indices = Int[]
                for (idx, (_, _, _, time_delta)) in enumerate(Xi_same_dest)
                    future_time_id = time_id + time_delta
                    if haskey(mapping.Omega_s_t[s], future_time_id)
                        push!(feasible_indices, idx)
                    end
                end
            end

            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                r = r_same_dest[global_idx]
                if r > 0  # Only add if there's actual savings
                    add_to_expression!(expr, routing_weight * r, v[s][time_id][local_idx])
                end
            end
        end
    end

    return expr
end


"""
    same_dest_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    ) -> AffExpr

Same-destination pooling savings for TwoStageSingleDetourNoWalkingLimitModel.
"""
function same_dest_pooling_savings_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    )::AffExpr

    S = n_scenarios(data)
    v = m[:v]
    v_idx_map = m[:v_idx_map]

    r_same_dest = Float64[]
    for (j, k, l, _) in Xi_same_dest
        c_jl = get_routing_cost(data, j, l)
        c_jk = get_routing_cost(data, j, k)
        push!(r_same_dest, c_jl - c_jk)
    end

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, _) in mapping.Omega_s_t[s]
            feasible_indices = get(v_idx_map[s], time_id, Int[])
            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                r = r_same_dest[global_idx]
                if r > 0
                    add_to_expression!(expr, routing_weight * r, v[s][time_id][local_idx])
                end
            end
        end
    end

    return expr
end
