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

Set the robust minimisation objective:

    min  Σ_{s,od} q̲_ods · t_ods
       + Σ_s      B_s   · α_s
       + Σ_{s,od} q̂_ods · β_ods
"""
function set_robust_total_demand_cap_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::RobustTotalDemandCapMap
    )
    S     = n_scenarios(data)
    t     = m[:t]
    alpha = m[:alpha]
    beta  = m[:beta]

    @objective(m, Min,
        # Lower-bound demand cost
        sum(
            get(mapping.q_low[s], (o, d), 0.0) * t[s][od_idx]
            for s in 1:S
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            if haskey(t[s], od_idx)
        )
        # Budget penalty
        + sum(mapping.B[s] * alpha[s] for s in 1:S)
        # Per-OD dual penalty
        + sum(
            get(mapping.q_hat[s], (o, d), 0.0) * beta[s][od_idx]
            for s in 1:S
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            if haskey(beta[s], od_idx)
        )
    )

    return nothing
end
