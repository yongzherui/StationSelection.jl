"""
Objective function for NominalTwoStageODModel.

Objective:
    min Σ_s Σ_{(o,d)∈Ω_s} q_{ods} · Σ_{j,k} (walk_{oj} + walk_{kd} + λ·route_{jk}) · x[s][od_idx][pair_idx]

x ∈ {0,1} is a binary routing indicator; the mean daily demand q_{ods} scales
the per-trip assignment cost so that high-demand corridors are prioritised.
"""

using JuMP

export set_nominal_od_objective!

"""
    set_nominal_od_objective!(m, data, mapping::NominalTwoStageODMap; in_vehicle_time_weight=1.0)

Set the minimization objective for NominalTwoStageODModel.
"""
function set_nominal_od_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::NominalTwoStageODMap;
        in_vehicle_time_weight::Float64=1.0
    )
    S = n_scenarios(data)
    x = m[:x]

    @objective(m, Min,
        sum(
            get(mapping.Q_s[s], (o, d), 0.0) * (
                get_walking_cost(data, o, j) +
                get_walking_cost(data, k, d) +
                in_vehicle_time_weight * get_routing_cost(data, j, k)
            ) * x[s][od_idx][idx]
            for s in 1:S
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            if !isempty(get(x[s], od_idx, VariableRef[]))
            for (idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
        )
    )

    return nothing
end
