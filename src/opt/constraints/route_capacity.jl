"""
Route covering constraints for ExactDARPRouteModel.
"""

using JuMP

export add_route_capacity_constraints!
export add_route_capacity_lazy_constraints!

# ─────────────────────────────────────────────────────────────────────────────
# ExactDARPRouteODMap (ExactDARPRouteModel — fixed alpha parameters)
# ─────────────────────────────────────────────────────────────────────────────

"""
    add_route_capacity_constraints!(m, data, mapping::ExactDARPRouteODMap) -> Int

Add the alpha-parameterized capacity covering constraint (upfront):

    Σ_{od: (j,k) valid} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

Only added for (j,k,t,s) combinations where at least one route has a positive alpha value.
"""
function add_route_capacity_constraints!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: ExactDARPRouteODMap;
    store_refs::Bool=false
)::Int
    before           = _total_num_constraints(m)
    S                = n_scenarios(data)
    x                = m[:x]
    theta_r_ts       = m[:theta_r_ts]
    arm_alpha_params = m[:arm_alpha_params]
    arm_capacity_constraints = Dict{NTuple{4, Int}, ConstraintRef}()

    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            # Collect active (j,k) legs in this bucket
            jk_set = Set{Tuple{Int, Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end

            routes_t = get(mapping.routes_s[s], t_id, RouteData[])

            for (j_idx, k_idx) in jk_set
                # RHS: Σ_r α^r_{jk} · θ^r_{ts}  (only routes with positive alpha)
                rhs = AffExpr(0.0)
                for (r_idx, route) in enumerate(routes_t)
                    alpha_val = get(arm_alpha_params, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                    alpha_val > 0 || continue
                    theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                    theta_var === nothing && continue
                    add_to_expression!(rhs, alpha_val, theta_var)
                end
                isempty(rhs.terms) && continue   # no alpha coverage — skip

                # LHS: Σ_{od: (j,k) valid} x[s][t][od][pair]
                lhs = AffExpr(0.0)
                for (od_idx, (o, d)) in enumerate(od_pairs)
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                    pair_idx === nothing && continue
                    x_od = get(x[s][t_id], od_idx, VariableRef[])
                    isempty(x_od) && continue
                    add_to_expression!(lhs, 1.0, x_od[pair_idx])
                end

                con_ref = @constraint(m, lhs <= rhs)
                if store_refs
                    arm_capacity_constraints[(s, t_id, j_idx, k_idx)] = con_ref
                end
            end
        end
    end

    store_refs && (m[:arm_capacity_constraints] = arm_capacity_constraints)

    return _total_num_constraints(m) - before
end


"""
    add_route_capacity_lazy_constraints!(m, data, mapping::ExactDARPRouteODMap) -> Int

Register a lazy-constraint callback for the alpha-parameterized capacity constraint.

    Σ_{od: (j,k) valid} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

Returns 0 (no explicit constraints added upfront).
"""
function add_route_capacity_lazy_constraints!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: ExactDARPRouteODMap
)::Int
    before           = _total_num_constraints(m)
    S                = n_scenarios(data)
    x                = m[:x]
    theta_r_ts       = m[:theta_r_ts]
    arm_alpha_params = m[:arm_alpha_params]

    # Precompute active (j,k) set and OD index lookup per (s, t_id)
    jk_by_st     = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    od_idx_by_st = Dict{Tuple{Int, Int}, Dict{Tuple{Int, Int}, Int}}()

    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            jk_set    = Set{Tuple{Int, Int}}()
            od_lookup = Dict{Tuple{Int, Int}, Int}()
            for (od_idx, (o, d)) in enumerate(od_pairs)
                od_lookup[(o, d)] = od_idx
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end
            jk_by_st[(s, t_id)]     = collect(jk_set)
            od_idx_by_st[(s, t_id)] = od_lookup
        end
    end

    function lazy_cb(cb_data)
        callback_node_status(cb_data, m) == MOI.CALLBACK_NODE_STATUS_INTEGER || return

        for s in 1:S
            for t_id in _time_ids(mapping, s)
                od_pairs = _time_od_pairs(mapping, s, t_id)
                od_lookup = get(od_idx_by_st, (s, t_id), Dict{Tuple{Int,Int},Int}())
                routes_t  = get(mapping.routes_s[s], t_id, RouteData[])

                for (j_idx, k_idx) in get(jk_by_st, (s, t_id), Tuple{Int,Int}[])
                    # Evaluate RHS: Σ_r α^r_{jk} · θ^r_{ts}
                    rhs_val      = 0.0
                    has_coverage = false
                    for (r_idx, _route) in enumerate(routes_t)
                        alpha_val = get(arm_alpha_params, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                        alpha_val > 0 || continue
                        theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                        theta_var === nothing && continue
                        has_coverage = true
                        rhs_val += alpha_val * callback_value(cb_data, theta_var)
                    end
                    !has_coverage && continue   # no alpha coverage for this (j,k,t,s)

                    # Evaluate LHS: Σ_{od} x[s][t][od][pair]
                    lhs_val = 0.0
                    for (o, d) in od_pairs
                        valid_pairs = get_valid_jk_pairs(mapping, o, d)
                        pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                        pair_idx === nothing && continue
                        od_idx = get(od_lookup, (o, d), 0)
                        od_idx == 0 && continue
                        x_od = get(x[s][t_id], od_idx, VariableRef[])
                        isempty(x_od) && continue
                        lhs_val += callback_value(cb_data, x_od[pair_idx])
                    end

                    lhs_val <= rhs_val + 1e-6 && continue

                    # Violated — build and submit constraint
                    rhs_expr = AffExpr(0.0)
                    for (r_idx, _route) in enumerate(routes_t)
                        alpha_val = get(arm_alpha_params, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                        alpha_val > 0 || continue
                        theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                        theta_var === nothing && continue
                        add_to_expression!(rhs_expr, alpha_val, theta_var)
                    end
                    lhs_expr = AffExpr(0.0)
                    for (o, d) in od_pairs
                        valid_pairs = get_valid_jk_pairs(mapping, o, d)
                        pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                        pair_idx === nothing && continue
                        od_idx = get(od_lookup, (o, d), 0)
                        od_idx == 0 && continue
                        x_od = get(x[s][t_id], od_idx, VariableRef[])
                        isempty(x_od) && continue
                        add_to_expression!(lhs_expr, 1.0, x_od[pair_idx])
                    end
                    MOI.submit(m, MOI.LazyConstraint(cb_data),
                               @build_constraint(lhs_expr <= rhs_expr))
                end
            end
        end
    end

    MOI.set(m, MOI.LazyConstraintCallback(), lazy_cb)
    return _total_num_constraints(m) - before
end
