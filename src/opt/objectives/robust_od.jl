"""
Objective function for RobustTotalDemandCapModel.
"""

using JuMP

export set_robust_total_demand_cap_objective!

"""
    set_robust_total_demand_cap_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )

Set the robust minimisation objective (dual of the demand uncertainty problem):

    min  Σ_s B_s · α_s  +  Σ_{s,od} q̂_ods · β_ods

α_s is the budget dual; β_ods is the per-OD upper-bound dual.
The assignment cost enters through the dual constraint α + β ≥ cost·x (no t variable).
"""
function set_robust_total_demand_cap_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    S     = n_scenarios(data)
    alpha = m[:alpha]
    beta  = m[:beta]

    @objective(m, Min,
        sum(mapping.B[s] * alpha[s] for s in 1:S)
        + sum(
            get(mapping.q_hat[s], (o, d), 0.0) * beta[s][od_idx]
            for s in 1:S
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            if haskey(beta[s], od_idx)
        )
    )

    return nothing
end
