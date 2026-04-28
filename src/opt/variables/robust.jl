"""
Variable creation functions for RobustTotalDemandCapModel.
"""

using JuMP

export add_robust_assignment_variables!
export add_robust_dual_variables!

"""
    add_robust_assignment_variables!(m, data, mapping::RobustTotalDemandCapMap)

Add continuous recourse-witness assignment variables x[s][od_idx][pair_idx] ∈ [0,1].

Unlike the nominal model, x represents a fractional assignment probability (not a
demand count), so it is declared continuous.  OD pairs with no valid (j,k) pairs
receive an empty vector.
"""
function add_robust_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            n_pairs = length(valid_pairs)
            if n_pairs > 0
                x[s][od_idx] = @variable(m, [1:n_pairs], lower_bound=0.0, upper_bound=1.0)
            else
                x[s][od_idx] = VariableRef[]
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end


"""
    add_robust_dual_variables!(m, data, mapping::RobustTotalDemandCapMap)

Add dual and cost-witness variables for the robust counterpart:
- `t[s][od_idx] ≥ 0`: per-OD assignment cost witness (t_ods = Σ a·x)
- `alpha[s] ≥ 0`: dual variable for the budget constraint (one per scenario)
- `beta[s][od_idx] ≥ 0`: dual variable for per-OD upper-bound constraint
"""
function add_robust_dual_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    # alpha[s]: one per scenario
    alpha = @variable(m, [1:S], lower_bound=0.0)
    m[:alpha] = alpha

    # t[s][od_idx] and beta[s][od_idx]: one per active OD pair per scenario
    t    = [Dict{Int, VariableRef}() for _ in 1:S]
    beta = [Dict{Int, VariableRef}() for _ in 1:S]

    for s in 1:S
        for (od_idx, _) in enumerate(mapping.Omega_s[s])
            t[s][od_idx]    = @variable(m, lower_bound=0.0)
            beta[s][od_idx] = @variable(m, lower_bound=0.0)
        end
    end

    m[:t]    = t
    m[:beta] = beta

    return JuMP.num_variables(m) - before
end
