"""
Assignment constraints for NominalTwoStageODModel.

Two constraint groups:
1. Coverage:        Σ_{jk} x[s][od_idx][pair_idx] = 1   ∀(od, s)
2. Active linking:  x[s][od_idx][pair_idx] ≤ z[j,s]
                    x[s][od_idx][pair_idx] ≤ z[k,s]

x is binary, so the big-M coefficient is 1.  Mean demand q_{ods} appears only
in the objective, not in these constraints.
"""

using JuMP

export add_nominal_assignment_constraints!
export add_nominal_assignment_to_active_constraints!

"""
    add_nominal_assignment_constraints!(m, data, mapping::NominalTwoStageODMap)

Σ_{jk} x[s][od_idx][pair_idx] = 1  ∀(od, s) with valid pairs.
"""
function add_nominal_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::NominalTwoStageODMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (od_idx, _) in enumerate(mapping.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            @constraint(m, sum(x_od) == 1)
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_nominal_assignment_to_active_constraints!(m, data, mapping::NominalTwoStageODMap)

x[s][od_idx][pair_idx] ≤ z[j,s]  and  x[s][od_idx][pair_idx] ≤ z[k,s]  ∀(od, j, k, s).
"""
function add_nominal_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::NominalTwoStageODMap
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
            for (idx, (j, k)) in enumerate(valid_pairs)
                @constraint(m, x_od[idx] <= z[j, s])
                @constraint(m, x_od[idx] <= z[k, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end
