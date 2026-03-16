"""
Objective function for TwoStageRouteModel.

Walking cost (no in-vehicle time) plus route activation penalty (temporal BFS mode).
"""

using JuMP

export set_route_od_objective!

"""
    set_route_od_objective!(m, data, mapping::TwoStageRouteODMap;
                             route_regularization_weight::Float64=1.0)

Set the minimization objective for TwoStageRouteModel.

    min Σ_s [ Σ_{(o,d,t)∈Ω_s} Σ_{(j,k)} q_{odts} (d_{oj} + d_{kd}) x_{odtjks}
            + μ Σ_r τ^r_s theta_s[s][r] ]

where r iterates over the per-scenario route pool from temporal BFS.
"""
function set_route_od_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::TwoStageRouteODMap;
    route_regularization_weight::Float64 = 1.0
)
    S   = n_scenarios(data)
    x   = m[:x]
    obj = AffExpr(0.0)

    # ── Walking cost (same for both modes) ────────────────────────────────────
    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            for (od_idx, (o, d)) in enumerate(od_pairs)
                x_od = get(x[s][t_id], od_idx, VariableRef[])
                isempty(x_od) && continue
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                demand = Float64(mapping.Q_s_t[s][t_id][(o, d)])
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    cost = demand * (
                        get_walking_cost(data, o, j_id) +
                        get_walking_cost(data, k_id, d)
                    )
                    add_to_expression!(obj, cost, x_od[pair_idx])
                end
            end
        end
    end

    # ── Route activation penalty ───────────────────────────────────────────────
    theta_s = m[:theta_s]
    for s in 1:S
        for (r_idx, trd) in enumerate(mapping.routes_s[s])
            tau_r = trd.route.travel_time
            add_to_expression!(obj,
                route_regularization_weight * tau_r, theta_s[s][r_idx])
        end
    end

    @objective(m, Min, obj)
    return nothing
end
