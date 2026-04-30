"""
Continuous assignment variables for NominalTwoStageODModel.
"""

using JuMP

export add_nominal_assignment_variables!

"""
    add_nominal_assignment_variables!(m, data, mapping::NominalTwoStageODMap)

Add continuous assignment variables x[s][od_idx][pair_idx] ∈ [0,1].

x = 1 selects station pair (j,k) for OD pair (o,d) in scenario s.  The mean
daily demand q_{ods} enters only through the objective coefficient; the variable
itself is a pure routing indicator.  OD pairs with no valid (j,k) pairs or zero
demand receive an empty vector and are skipped in constraints.

x is relaxed to [0,1] (not binary). For fixed binary z, each OD subproblem
minimises a linear objective over a simplex, so the LP optimum is always a
vertex (integer). Degeneracy (tied costs) can produce fractional solutions with
the same objective value; the export warns if this occurs.
"""
function add_nominal_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::NominalTwoStageODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            n_pairs = length(valid_pairs)
            demand = get(mapping.Q_s[s], (o, d), 0.0)
            if n_pairs > 0 && demand > 0
                x[s][od_idx] = @variable(m, [1:n_pairs], Bin)
            else
                x[s][od_idx] = VariableRef[]
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end
