"""
Route covering constraints for route-based models.
"""

using JuMP

export add_route_capacity_constraints!
export add_route_capacity_lazy_constraints!

"""
    add_route_capacity_constraints!(m, data, mapping::TwoStageRouteODMap) -> Int

Add route-covering constraints using actual per-leg passenger loads (α).

For each `(s, t_id, j_idx, k_idx)` entry in `mapping.routes_by_jkt_s`:

    Σ_{(r, α) ∈ routes_by_jkt_s[(s,t,j,k)]} α * theta_s[s][r]
      ≥  Σ_{(o,d): (j,k) valid} q * x[s][t][od][pair]

Returns the number of constraints added.
"""
function add_route_capacity_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::TwoStageRouteODMap
)::Int
    before  = _total_num_constraints(m)
    x       = m[:x]
    theta_s = m[:theta_s]

    for ((s, t_id, j_idx, k_idx), route_terms) in mapping.routes_by_jkt_s

        # LHS: Σ_{(r,α)} α * theta_s[s][r]
        lhs_expr = AffExpr(0.0)
        for (r_idx, α) in route_terms
            add_to_expression!(lhs_expr, Float64(α), theta_s[s][r_idx])
        end

        # RHS: Σ_{(o,d) with (j_idx,k_idx) valid} q * x[s][t_id][od_idx][pair_idx]
        rhs_expr = AffExpr(0.0)
        od_pairs = get(mapping.Omega_s_t[s], t_id, Tuple{Int,Int}[])
        for (od_idx, (o, d)) in enumerate(od_pairs)
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(valid_pairs) && continue
            x_od = get(x[s][t_id], od_idx, VariableRef[])
            isempty(x_od) && continue
            demand = Float64(mapping.Q_s_t[s][t_id][(o, d)])

            for (pair_idx, (j, k)) in enumerate(valid_pairs)
                (j == j_idx && k == k_idx) || continue
                add_to_expression!(rhs_expr, demand, x_od[pair_idx])
            end
        end

        @constraint(m, lhs_expr >= rhs_expr)
    end

    return _total_num_constraints(m) - before
end


"""
    add_route_capacity_constraints!(m, data, mapping::RouteODMap) -> Int

Add route-covering constraints for RouteAlphaCapacityModel / RouteVehicleCapacityModel
(legacy non-time-indexed dispatch, kept for backward compatibility).

For each `(s, j_idx, k_idx)` entry in `mapping.routes_by_jks`:

    Σ_{(r, α) ∈ routes_by_jks[(s,j,k)]} α * theta_s[s][r]
      ≥  Σ_{(o,d): (j,k) valid} q * x[s][od][pair]

Returns the number of constraints added.
"""
function add_route_capacity_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::RouteODMap
)::Int
    before  = _total_num_constraints(m)
    x       = m[:x]
    theta_s = m[:theta_s]

    for ((s, j_idx, k_idx), route_terms) in mapping.routes_by_jks

        # LHS: Σ_{(r,α)} α * theta_s[s][r]
        lhs_expr = AffExpr(0.0)
        for (r_idx, α) in route_terms
            add_to_expression!(lhs_expr, Float64(α), theta_s[s][r_idx])
        end

        # RHS: Σ_{(o,d) with (j_idx,k_idx) valid} q * x[s][od_idx][pair_idx]
        rhs_expr = AffExpr(0.0)
        od_pairs = mapping.Omega_s[s]
        for (od_idx, (o, d)) in enumerate(od_pairs)
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(valid_pairs) && continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            demand = Float64(mapping.Q_s[s][(o, d)])

            for (pair_idx, (j, k)) in enumerate(valid_pairs)
                (j == j_idx && k == k_idx) || continue
                add_to_expression!(rhs_expr, demand, x_od[pair_idx])
            end
        end

        @constraint(m, lhs_expr >= rhs_expr)
    end

    return _total_num_constraints(m) - before
end


"""
    add_route_capacity_constraints!(m, data, mapping::RouteODMap,
                                    model::RouteAlphaCapacityModel) -> Int

Add time-indexed route-covering constraints for RouteAlphaCapacityModel.

For each `(s, t_id, j_idx, k_idx)` with positive demand, enforces:

    Σ_r α^r_{jkt} · θ_s[s][r]  ≥  Σ_{(o,d): (j,k) valid} Q_s_t[s][t_id][(o,d)] · x[s][od][pair]

where α^r_{jkt} is computed via `compute_alpha_r_jkt` (placeholder — will error until
implemented).

Returns the number of constraints added.
"""
function add_route_capacity_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::RouteODMap,
    model::RouteAlphaCapacityModel
)::Int
    before  = _total_num_constraints(m)
    x       = m[:x]
    theta_s = m[:theta_s]
    S = n_scenarios(data)

    for s in 1:S
        for (t_id, od_pairs_t) in mapping.Omega_s_t[s]
            # Collect all (j_idx, k_idx) pairs that have demand in this time bucket
            jk_set = Set{Tuple{Int,Int}}()
            for (o, d) in od_pairs_t
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end

            for (j_idx, k_idx) in jk_set
                # LHS: Σ_r α^r_{jkt} * theta_s[s][r]
                lhs_expr = AffExpr(0.0)
                for (r_idx, ntr) in enumerate(mapping.routes_s[s])
                    # Only include routes that have this leg
                    haskey(ntr.alpha, (j_idx, k_idx)) || continue
                    α = compute_alpha_r_jkt(ntr, j_idx, k_idx, t_id)
                    add_to_expression!(lhs_expr, Float64(α), theta_s[s][r_idx])
                end

                # RHS: Σ_{(o,d): (j,k) valid} Q_s_t[s][t_id][(o,d)] * x[s][od][pair]
                rhs_expr = AffExpr(0.0)
                for (od_idx, (o, d)) in enumerate(od_pairs_t)
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    isempty(valid_pairs) && continue
                    x_od = get(x[s], od_idx, VariableRef[])
                    isempty(x_od) && continue
                    demand = Float64(mapping.Q_s_t[s][t_id][(o, d)])
                    demand == 0.0 && continue

                    for (pair_idx, (j, k)) in enumerate(valid_pairs)
                        (j == j_idx && k == k_idx) || continue
                        add_to_expression!(rhs_expr, demand, x_od[pair_idx])
                    end
                end

                @constraint(m, lhs_expr >= rhs_expr)
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_route_capacity_constraints!(m, data, mapping::VehicleCapacityODMap) -> Int

Add the three covering constraints for RouteVehicleCapacityModel (new formulation):

    (i)   d_{jkts} = Σ_{od: (j,k) valid} x[s][t][od][pair]
                                                                     ∀ j,k,t,s

    (ii)  d_{jkts} ≤ Σ_r α^r_{jkts}
                                                                     ∀ j,k,t,s

    (iii) Σ_{j,k: β^r_{jkl}=1} α^r_{jkts}  ≤  Cap_r · θ^r_{ts}
                                                                     ∀ t,r,l∈V(r),s

x is time-indexed integer; d is a direct sum of x (no Q scaling).

Returns the number of constraints added.
"""
function add_route_capacity_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::VehicleCapacityODMap
)::Int
    before   = _total_num_constraints(m)
    S        = n_scenarios(data)
    x        = m[:x]
    d_jkts   = m[:d_jkts]
    alpha_r_jkts     = m[:alpha_r_jkts]
    alpha_r_jkts_by_srt = m[:alpha_r_jkts_by_srt]
    theta_ts = m[:theta_ts]
    # Cap_r is set by build_model before calling this function
    Cap_r = Float64(m[:vehicle_capacity])

    # Precompute reverse OD index per scenario and time bucket:
    # (o,d) → od_idx in Omega_s_t[s][t_id]
    od_to_idx_t = Dict{Int, Dict{Int, Dict{Tuple{Int,Int}, Int}}}()
    for s in 1:S
        od_to_idx_t[s] = Dict{Int, Dict{Tuple{Int,Int}, Int}}()
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            od_to_idx_t[s][t_id] = Dict{Tuple{Int,Int}, Int}(
                od => idx for (idx, od) in enumerate(od_pairs)
            )
        end
    end

    # ── Constraint (i): d_{jkts} = Σ_{od: (j,k) valid} x[s][t][od][pair] ─────
    for ((s, j_idx, k_idx, t_id), d_var) in d_jkts
        rhs = AffExpr(0.0)
        od_pairs_t = get(mapping.Omega_s_t[s], t_id, Tuple{Int,Int}[])

        for (o, d) in od_pairs_t
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            pair_idx = findfirst(==((j_idx, k_idx)), valid_pairs)
            pair_idx === nothing && continue
            od_idx = get(get(od_to_idx_t[s], t_id, Dict{Tuple{Int,Int},Int}()), (o, d), 0)
            od_idx == 0 && continue
            x_od = get(get(x[s], t_id, Dict{Int, Vector{VariableRef}}()), od_idx, VariableRef[])
            isempty(x_od) && continue
            add_to_expression!(rhs, 1.0, x_od[pair_idx])
        end

        @constraint(m, d_var == rhs)
    end

    # ── Constraint (ii): d_{jkts} ≤ Σ_r α^r_{jkts} ───────────────────────────
    for ((s, j_idx, k_idx, t_id), d_var) in d_jkts
        rhs = AffExpr(0.0)
        routes_t = get(mapping.routes_s[s], t_id, RouteData[])
        for r_idx in 1:length(routes_t)
            key5 = (s, r_idx, j_idx, k_idx, t_id)
            alpha_var = get(alpha_r_jkts, key5, nothing)
            alpha_var === nothing && continue
            add_to_expression!(rhs, 1.0, alpha_var)
        end
        @constraint(m, d_var <= rhs)
    end

    # ── Constraint (iii): Σ_{j,k: β^r_{jkl}=1} α^r_{jkts} ≤ Cap_r * θ^r_{ts} ──
    for s in 1:S
        for (t_id, routes_t) in mapping.routes_s[s]
            for (r_idx, route) in enumerate(routes_t)
                n_segs = length(route.station_ids) - 1
                n_segs <= 0 && continue

                theta_var = get(theta_ts, (s, t_id, r_idx), nothing)
                theta_var === nothing && continue

                srt_key  = (s, r_idx, t_id)
                jk_list  = get(alpha_r_jkts_by_srt, srt_key, Tuple{Int,Int}[])
                isempty(jk_list) && continue

                for l in 1:n_segs
                    lhs = AffExpr(0.0)
                    for (j_idx, k_idx) in jk_list
                        compute_beta_r_jkl(
                            route, j_idx, k_idx, l,
                            mapping.array_idx_to_station_id
                        ) || continue
                        key5 = (s, r_idx, j_idx, k_idx, t_id)
                        alpha_var = get(alpha_r_jkts, key5, nothing)
                        alpha_var === nothing && continue
                        add_to_expression!(lhs, 1.0, alpha_var)
                    end
                    isempty(lhs.terms) && continue
                    @constraint(m, lhs <= Cap_r * theta_var)
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_route_capacity_lazy_constraints!(m, data, mapping::VehicleCapacityODMap) -> Int

Add constraint (i) explicitly and register a lazy-constraint callback for (ii) and (iii).

Constraint (i) is always explicit (it defines d as a direct sum of time-indexed integer x).
Constraints (ii) and (iii) are submitted only when violated at integer-feasible B&B nodes,
which can dramatically reduce solve time on large instances.

    (i)   d_{jkts} = Σ_{od: (j,k) valid} x[s][t][od][pair]                   (explicit)
    (ii)  d_{jkts} ≤ Σ_r α^r_{jkts}                                           (lazy)
    (iii) Σ_{j,k: β^r_{jkl}=1} α^r_{jkts} ≤ Cap_r · θ^r_{ts}                (lazy)

Returns the number of explicit constraints added (only constraint (i) counts).
"""
function add_route_capacity_lazy_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::VehicleCapacityODMap
)::Int
    before   = _total_num_constraints(m)
    S        = n_scenarios(data)
    x        = m[:x]
    d_jkts   = m[:d_jkts]
    alpha_r_jkts        = m[:alpha_r_jkts]
    alpha_r_jkts_by_srt = m[:alpha_r_jkts_by_srt]
    theta_ts = m[:theta_ts]
    Cap_r    = Float64(m[:vehicle_capacity])

    # Precompute reverse OD index per scenario and time bucket:
    # (o,d) → od_idx in Omega_s_t[s][t_id]
    od_to_idx_t = Dict{Int, Dict{Int, Dict{Tuple{Int,Int}, Int}}}()
    for s in 1:S
        od_to_idx_t[s] = Dict{Int, Dict{Tuple{Int,Int}, Int}}()
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            od_to_idx_t[s][t_id] = Dict{Tuple{Int,Int}, Int}(
                od => idx for (idx, od) in enumerate(od_pairs)
            )
        end
    end

    # ── Constraint (i): explicit, always ──────────────────────────────────────
    for ((s, j_idx, k_idx, t_id), d_var) in d_jkts
        rhs = AffExpr(0.0)
        od_pairs_t = get(mapping.Omega_s_t[s], t_id, Tuple{Int,Int}[])

        for (o, d) in od_pairs_t
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            pair_idx = findfirst(==((j_idx, k_idx)), valid_pairs)
            pair_idx === nothing && continue
            od_idx = get(get(od_to_idx_t[s], t_id, Dict{Tuple{Int,Int},Int}()), (o, d), 0)
            od_idx == 0 && continue
            x_od = get(get(x[s], t_id, Dict{Int, Vector{VariableRef}}()), od_idx, VariableRef[])
            isempty(x_od) && continue
            add_to_expression!(rhs, 1.0, x_od[pair_idx])
        end

        @constraint(m, d_var == rhs)
    end

    # ── Lazy callback for constraints (ii) and (iii) ──────────────────────────
    function lazy_cb(cb_data)
        callback_node_status(cb_data, m) == MOI.CALLBACK_NODE_STATUS_INTEGER || return

        # Constraint (ii): d_{jkts} ≤ Σ_r α^r_{jkts}
        for ((s, j_idx, k_idx, t_id), d_var) in d_jkts
            d_val = callback_value(cb_data, d_var)
            routes_t = get(mapping.routes_s[s], t_id, RouteData[])
            n_routes_t = length(routes_t)
            alpha_sum = 0.0
            for r_idx in 1:n_routes_t
                avar = get(alpha_r_jkts, (s, r_idx, j_idx, k_idx, t_id), nothing)
                avar === nothing && continue
                alpha_sum += callback_value(cb_data, avar)
            end
            d_val <= alpha_sum + 1e-6 && continue

            # Violated — submit constraint (ii)
            rhs = AffExpr(0.0)
            for r_idx in 1:n_routes_t
                avar = get(alpha_r_jkts, (s, r_idx, j_idx, k_idx, t_id), nothing)
                avar === nothing && continue
                add_to_expression!(rhs, 1.0, avar)
            end
            MOI.submit(m, MOI.LazyConstraint(cb_data),
                       @build_constraint(d_var <= rhs))
        end

        # Constraint (iii): Σ_{j,k: β=1} α^r_{jkts} ≤ Cap_r · θ^r_{ts}
        for s in 1:S
            for (t_id, routes_t) in mapping.routes_s[s]
                for (r_idx, route) in enumerate(routes_t)
                    n_segs = length(route.station_ids) - 1
                    n_segs <= 0 && continue

                    theta_var = get(theta_ts, (s, t_id, r_idx), nothing)
                    theta_var === nothing && continue
                    theta_val = callback_value(cb_data, theta_var)

                    jk_list = get(alpha_r_jkts_by_srt, (s, r_idx, t_id), Tuple{Int,Int}[])
                    isempty(jk_list) && continue

                    for l in 1:n_segs
                        lhs_val = 0.0
                        for (j_idx, k_idx) in jk_list
                            compute_beta_r_jkl(route, j_idx, k_idx, l,
                                               mapping.array_idx_to_station_id) || continue
                            avar = get(alpha_r_jkts, (s, r_idx, j_idx, k_idx, t_id), nothing)
                            avar === nothing && continue
                            lhs_val += callback_value(cb_data, avar)
                        end
                        lhs_val <= Cap_r * theta_val + 1e-6 && continue

                        # Violated — submit constraint (iii)
                        lhs = AffExpr(0.0)
                        for (j_idx, k_idx) in jk_list
                            compute_beta_r_jkl(route, j_idx, k_idx, l,
                                               mapping.array_idx_to_station_id) || continue
                            avar = get(alpha_r_jkts, (s, r_idx, j_idx, k_idx, t_id), nothing)
                            avar === nothing && continue
                            add_to_expression!(lhs, 1.0, avar)
                        end
                        MOI.submit(m, MOI.LazyConstraint(cb_data),
                                   @build_constraint(lhs <= Cap_r * theta_var))
                    end
                end
            end
        end
    end

    MOI.set(m, MOI.LazyConstraintCallback(), lazy_cb)

    return _total_num_constraints(m) - before
end
