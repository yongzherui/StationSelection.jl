"""
Objective function for RouteFleetLimitModel.

    min Σ_s [
        Σ_{p,j,k} c^walk_{pjk} x_{pjks}              [walking]
      + μ Σ_{r,t} (τ^r + ρ) θ^r_{ts}                 [deployment]
      + μ Σ_{r,j,k,t} d^r_{jk} α^r_{jkts}            [per-passenger delay]
      + λ Σ_{j,k,t} v_{jkts}                          [unmet demand]
    ]

d^r_{jk} (delay coefficient) is precomputed in FleetLimitODMap.delay_coeff.
"""

export set_fleet_limit_objective!


"""
    set_fleet_limit_objective!(m, data, mapping::FleetLimitODMap;
                               route_regularization_weight, repositioning_time,
                               unmet_demand_penalty)

Set the minimisation objective for RouteFleetLimitModel.

Four cost components:
1. Walking cost: identical to RouteVehicleCapacityModel.
2. Route deployment: μ × (τ^r + ρ) × θ^r_{ts} — identical to RouteVehicleCapacityModel.
3. Per-passenger delay: μ × d^r_{jk} × α^r_{jkts} (only for legs with d>0).
4. Unmet demand: λ × v_{jkts}.
"""
function set_fleet_limit_objective!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: FleetLimitODMap;
    route_regularization_weight :: Float64 = 1.0,
    repositioning_time          :: Float64 = 20.0,
    unmet_demand_penalty        :: Float64 = 10000.0
)
    S     = n_scenarios(data)
    inner = mapping.inner
    x     = m[:x]
    obj   = AffExpr(0.0)

    # ── 1. Walking cost ────────────────────────────────────────────────────────
    for s in 1:S
        for t_id in _time_ids(inner, s)
            od_pairs = _time_od_pairs(inner, s, t_id)
            for (od_idx, (o, d)) in enumerate(od_pairs)
                x_od = get(x[s][t_id], od_idx, VariableRef[])
                isempty(x_od) && continue
                valid_pairs = get_valid_jk_pairs(inner, o, d)
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    cost = get_walking_cost(data, o, j) + get_walking_cost(data, k, d)
                    add_to_expression!(obj, cost, x_od[pair_idx])
                end
            end
        end
    end

    # ── 2. Route deployment: μ Σ (τ^r + ρ) θ^r_{ts} ─────────────────────────
    for ((s, t_id, r_idx), theta_var) in m[:theta_r_ts]
        tau_r = inner.routes_s[s][t_id][r_idx].travel_time
        add_to_expression!(obj,
            route_regularization_weight * (tau_r + repositioning_time),
            theta_var)
    end

    # ── 3. Per-passenger delay: μ Σ d^r_{jk} α^r_{jkts} ─────────────────────
    for ((s, r_idx, j_idx, k_idx, t_id), alpha_var) in m[:alpha_r_jkts]
        d_coeff = get(mapping.delay_coeff, (s, t_id, r_idx, j_idx, k_idx), 0.0)
        d_coeff > 0 || continue
        add_to_expression!(obj, route_regularization_weight * d_coeff, alpha_var)
    end

    # ── 4. Unmet demand: λ Σ v_{jkts} ────────────────────────────────────────
    for (_, v_var) in m[:v_jkts]
        add_to_expression!(obj, unmet_demand_penalty, v_var)
    end

    @objective(m, Min, obj)
    return nothing
end
