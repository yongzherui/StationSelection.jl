"""
Constraint functions for RobustTotalDemandCapModel.

Four constraint groups:
1. Assignment coverage:  Σ_{jk} x[s][od][pair] = 1
2. Assignment-to-active: x ≤ z[j,s], x ≤ z[k,s]
3. Recourse cost link:   t[s][od] = Σ_{jk} a_odjks · x[s][od][pair]
4. Robust dual:          alpha[s] + beta[s][od] ≥ t[s][od]
"""

using JuMP

export add_robust_assignment_constraints!
export add_robust_assignment_to_active_constraints!
export add_robust_recourse_cost_constraints!
export add_robust_dual_constraints!


"""
    add_robust_assignment_constraints!(m, data, mapping::RobustTotalDemandCapMap)

Σ_{j,k} x[s][od_idx][pair] = 1  ∀(od_idx, s) with valid pairs.

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
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            @constraint(m, sum(x_od) == 1.0)
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
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (pair_idx, (j, k)) in enumerate(valid_pairs)
                @constraint(m, x_od[pair_idx] <= z[j, s])
                @constraint(m, x_od[pair_idx] <= z[k, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_robust_recourse_cost_constraints!(m, data, mapping::RobustTotalDemandCapMap;
                                          in_vehicle_time_weight=1.0)

t[s][od_idx] = Σ_{j,k} a_odjks · x[s][od_idx][pair]  ∀(od, s)

where  a_odjks = walking_cost(o→j) + walking_cost(k→d) + λ · routing_cost(j→k).
"""
function add_robust_recourse_cost_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap;
        in_vehicle_time_weight::Float64 = 1.0
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]
    t = m[:t]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            valid_pairs = get_valid_jk_pairs(mapping, o, d)

            cost_expr = @expression(m,
                sum(
                    (
                        get_walking_cost(data, o, j) +
                        get_walking_cost(data, k, d) +
                        in_vehicle_time_weight * get_routing_cost(data, j, k)
                    ) * x_od[pair_idx]
                    for (pair_idx, (j, k)) in enumerate(valid_pairs)
                )
            )
            @constraint(m, t[s][od_idx] == cost_expr)
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_robust_dual_constraints!(m, data, mapping::RobustTotalDemandCapMap)

alpha[s] + beta[s][od_idx] ≥ t[s][od_idx]  ∀(od, s).
"""
function add_robust_dual_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    alpha = m[:alpha]
    beta  = m[:beta]
    t     = m[:t]

    for s in 1:S
        for (od_idx, _) in enumerate(mapping.Omega_s[s])
            haskey(t[s], od_idx) || continue
            @constraint(m, alpha[s] + beta[s][od_idx] >= t[s][od_idx])
        end
    end

    return _total_num_constraints(m) - before
end
