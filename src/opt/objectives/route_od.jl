"""
Objective function for RouteVehicleCapacityModel.

Walking cost plus route deployment penalty with repositioning time.
"""

using JuMP

export set_route_od_objective!

"""
    set_route_od_objective!(m, data, mapping::Union{VehicleCapacityODMap, AlphaRouteODMap};
                             route_regularization_weight::Float64=1.0,
                             repositioning_time::Float64=20.0)

Set the minimization objective for time-bucketed route models
(RouteVehicleCapacityModel and AlphaRouteModel).

    min Σ_s [ Σ_{(o,d,t)∈Ω_s} Σ_{(j,k)} (d_{oj} + d_{kd}) x_{odjkts}
            + μ Σ_{(s,t,r)} (τ^r + ρ) · θ^r_{ts} ]

x is time-indexed integer (counts passengers directly); no Q scaling needed.
Route penalty sums over all θ^r_{ts} deployments weighted by (τ^r + ρ), where ρ is
a constant repositioning time (seconds) added to each route deployment cost.
"""
function set_route_od_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::Union{VehicleCapacityODMap, AlphaRouteODMap};
    route_regularization_weight::Float64 = 1.0,
    repositioning_time::Float64 = 20.0
)
    S   = n_scenarios(data)
    x   = m[:x]
    obj = AffExpr(0.0)

    # ── Walking cost (x already counts passengers — no Q scaling) ─────────────
    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            for (od_idx, (o, d)) in enumerate(od_pairs)
                x_od = get(x[s][t_id], od_idx, VariableRef[])
                isempty(x_od) && continue
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    cost = get_walking_cost(data, o, j_id) + get_walking_cost(data, k_id, d)
                    add_to_expression!(obj, cost, x_od[pair_idx])
                end
            end
        end
    end

    # ── Route deployment penalty: μ Σ_{(s,t_id,r_idx)} (τ^r + ρ) * θ^r_{ts} ──
    theta_r_ts = m[:theta_r_ts]
    for ((s, t_id, r_idx), theta_var) in theta_r_ts
        tau_r = mapping.routes_s[s][t_id][r_idx].travel_time
        add_to_expression!(obj, route_regularization_weight * (tau_r + repositioning_time), theta_var)
    end

    @objective(m, Min, obj)
    return nothing
end
