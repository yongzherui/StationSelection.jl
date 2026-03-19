"""
Route covering constraints for TwoStageRouteWithTimeModel (temporal BFS mode).

For each (s, t_id, j_idx, k_idx) with demand and routes, enforces that the
total α-weighted activated routes cover all demand assigned to that VBS leg:

    Σ_{(r, α) ∈ routes_by_jkt_s[(s,t,j,k)]} α · θ_s[s][r]
      ≥  Σ_{(o,d): (j,k) valid} q_{od,s,t} · x[s][t][od][pair]

α is the actual passengers carried by route r on leg (j,k) in window t,
computed during BFS. Capacity feasibility (total passengers ≤ C) is
enforced implicitly: the BFS only generates routes where this holds.
"""

using JuMP

export add_route_capacity_constraints!

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

Add route-covering constraints for RouteAlphaCapacityModel / RouteVehicleCapacityModel.

For each `(s, j_idx, k_idx)` entry in `mapping.routes_by_jks`:

    Σ_{(r, α) ∈ routes_by_jks[(s,j,k)]} α * theta_s[s][r]
      ≥  Σ_{(o,d): (j,k) valid} q * x[s][od][pair]

α is actual passengers (RouteAlphaCapacityModel) or vehicle capacity C (RouteVehicleCapacityModel),
as encoded in `routes_by_jks` at map-construction time.

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
