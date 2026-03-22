"""
Objective function for TwoStageRouteWithTimeModel.

Walking cost (no in-vehicle time) plus route activation penalty (temporal BFS mode).
"""

using JuMP

export set_route_od_objective!

"""
    set_route_od_objective!(m, data, mapping::TwoStageRouteODMap;
                             route_regularization_weight::Float64=1.0)

Set the minimization objective for TwoStageRouteWithTimeModel.

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


"""
    set_route_od_objective!(m, data, mapping::RouteODMap;
                             route_regularization_weight::Float64=1.0,
                             route_var_name::Symbol=:theta_s)

Set the minimization objective for RouteAlphaCapacityModel / RouteVehicleCapacityModel.

    min Σ_s [ Σ_{(o,d)∈Ω_s} Σ_{(j,k)} q_{od,s} (d_{oj} + d_{kd}) x_{odjks}
            + μ Σ_r τ^r_s route_var[s][r] ]

where r iterates over the per-scenario route pool from non-temporal BFS.

`route_var_name` selects the route activation variable: `:theta_s` for
RouteAlphaCapacityModel, `:gamma_s` for RouteVehicleCapacityModel.
"""
function set_route_od_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::RouteODMap;
    route_regularization_weight::Float64 = 1.0,
    route_var_name::Symbol = :theta_s
)
    S   = n_scenarios(data)
    x   = m[:x]
    obj = AffExpr(0.0)

    # ── Walking cost ───────────────────────────────────────────────────────────
    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            demand = Float64(mapping.Q_s[s][(o, d)])
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

    # ── Route activation penalty ───────────────────────────────────────────────
    route_vars = m[route_var_name]
    for s in 1:S
        for (r_idx, ntr) in enumerate(mapping.routes_s[s])
            tau_r = ntr.route.travel_time
            add_to_expression!(obj,
                route_regularization_weight * tau_r, route_vars[s][r_idx])
        end
    end

    @objective(m, Min, obj)
    return nothing
end


"""
    set_route_od_objective!(m, data, mapping::VehicleCapacityODMap;
                             route_regularization_weight::Float64=1.0)

Set the minimization objective for RouteVehicleCapacityModel (new formulation).

    min Σ_s [ Σ_{(o,d)∈Ω_s} Σ_{(j,k)} q_{od,s} (d_{oj} + d_{kd}) x_{odjks}
            + μ Σ_{(s,t,r)} τ^r · θ^r_{ts} ]

Walking cost uses time-aggregated demand (Q_s); route penalty sums over all
θ^r_{ts} deployments weighted by route travel time τ^r.
"""
function set_route_od_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::VehicleCapacityODMap;
    route_regularization_weight::Float64 = 1.0
)
    S   = n_scenarios(data)
    x   = m[:x]
    obj = AffExpr(0.0)

    # ── Walking cost ───────────────────────────────────────────────────────────
    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            demand = Float64(mapping.Q_s[s][(o, d)])
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

    # ── Route deployment penalty: μ Σ_{(s,t_id,r_idx)} τ^r * θ^r_{ts} ────────
    theta_ts = m[:theta_ts]
    for ((s, t_id, r_idx), theta_var) in theta_ts
        tau_r = mapping.routes_s[s][r_idx].travel_time
        add_to_expression!(obj, route_regularization_weight * tau_r, theta_var)
    end

    @objective(m, Min, obj)
    return nothing
end
