"""
Constraint functions for RobustTotalDemandCapModel.

Three constraint groups:
1. Assignment coverage:  Σ_{jk} x[s][od][pair] = 1
2. Assignment-to-active: x ≤ z[j,s], x ≤ z[k,s]
3. Robust dual:          alpha[s] + beta[s][od] ≥ Σ_{jk} cost_{odjk} · x[s][od][pair]

The per-OD assignment cost is an inline @expression rather than a separate
variable t, eliminating the recourse-cost equality constraint.
"""

using JuMP

export add_robust_assignment_constraints!
export add_robust_assignment_to_active_constraints!
export add_robust_dual_constraints!


"""
    add_robust_assignment_constraints!(m, data, mapping::RobustTotalDemandCapMap)

Σ_{j,k} x[s][od_idx][pair] = 1  ∀(od_idx, s).

The RHS is 1 (not demand count) because x is a recourse witness.
"""
function add_robust_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (od_idx, _) in enumerate(mapping.Omega_s[s])
            @constraint(m, sum(x[s][od_idx]) == 1.0)
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_robust_assignment_to_active_constraints!(m, data, mapping::RobustTotalDemandCapMap)

x[s][od_idx][pair] ≤ z[j,s]  and  x[s][od_idx][pair] ≤ z[k,s]  ∀(od, j, k, s).
"""
function add_robust_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (pair_idx, (j, k)) in enumerate(valid_pairs)
                @constraint(m, x[s][od_idx][pair_idx] <= z[j, s])
                @constraint(m, x[s][od_idx][pair_idx] <= z[k, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_robust_dual_constraints!(m, data, mapping::RobustTotalDemandCapMap;
                                 in_vehicle_time_weight=1.0)

alpha[s] + beta[s][od_idx] ≥ Σ_{j,k} cost_{odjk} · x[s][od_idx][pair]  ∀(od, s)

where cost_{odjk} = walk(o→j) + walk(k→d) + λ · route(j→k).

The assignment cost is an inline expression; no separate t variable is needed.
"""
function add_robust_dual_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap;
        in_vehicle_time_weight::Float64 = 1.0
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    alpha = m[:alpha]
    beta  = m[:beta]
    x     = m[:x]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            haskey(beta[s], od_idx) || continue
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            cost_expr = @expression(m,
                sum(
                    (get_walking_cost(data, o, j) +
                     get_walking_cost(data, k, d) +
                     in_vehicle_time_weight * get_routing_cost(data, j, k)) * x[s][od_idx][pair_idx]
                    for (pair_idx, (j, k)) in enumerate(valid_pairs)
                )
            )
            @constraint(m, alpha[s] + beta[s][od_idx] >= cost_expr)
        end
    end

    return _total_num_constraints(m) - before
end
