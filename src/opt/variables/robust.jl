"""
Variable creation functions for RobustTotalDemandCapModel.
"""

using JuMP

export add_robust_assignment_variables!
export add_robust_dual_variables!

"""
    add_robust_assignment_variables!(m, data, mapping::RobustTotalDemandCapMap)

Add continuous assignment variables x[s][od_idx][pair_idx] ∈ [0,1].

x[s][od_idx][pair_idx] = 1 selects station pair (j,k) for OD pair (o,d) in
scenario s.  The covering constraint Σ_{jk} x = 1 ensures exactly one pair is
chosen.  OD pairs with no valid (j,k) pairs receive an empty vector.

x is relaxed to [0,1] (not binary). The dual constraints share alpha[s] across
all OD pairs, but each constraint involves x for only one OD pair. For any fixed
alpha, the problem decomposes by OD: each subproblem minimises a linear objective
over a simplex, so x is integer at optimality. The optimal alpha creates no
incentive for fractional x (splitting x only raises cost, requiring higher beta).
Degeneracy (tied costs) can still produce fractional solutions with equal
objective value; the export warns if this occurs.
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
                x[s][od_idx] = @variable(m, [1:n_pairs], Bin)
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

Add dual variables for the robust counterpart:
- `alpha[s] ≥ 0`: dual variable for the total-demand budget constraint (one per scenario)
- `beta[s][od_idx] ≥ 0`: dual variable for per-OD upper-bound constraint

The per-OD assignment cost is expressed directly in the dual constraint rather than
as a separate variable t, keeping the model compact.
"""
function add_robust_dual_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    alpha = @variable(m, [1:S], lower_bound=0.0)
    m[:alpha] = alpha

    beta = [Dict{Int, VariableRef}() for _ in 1:S]
    for s in 1:S
        for (od_idx, _) in enumerate(mapping.Omega_s[s])
            x_od = get(m[:x][s], od_idx, VariableRef[])
            isempty(x_od) && continue
            beta[s][od_idx] = @variable(m, lower_bound=0.0)
        end
    end
    m[:beta] = beta

    return JuMP.num_variables(m) - before
end
