"""
Constraints specific to RouteFleetLimitModel.

Two new constraint groups beyond those inherited from RouteVehicleCapacityModel:

  (eq)   Σ_{od:(j,k)} x[s][t][od][pair] = v_{jkts} + Σ_r α^r_{jkts}   ∀ j,k,t,s
  (fl)   Σ_r θ^r_{ts} ≤ F                                               ∀ t,s

The vehicle-capacity segment constraint (iii) is unchanged and is added by
calling `add_route_capacity_constraints!(m, data, mapping.inner)`.
"""

export add_fleet_limit_constraints!


"""
    add_fleet_limit_constraints!(m, data, mapping::FleetLimitODMap) -> Int

Add the two RouteFleetLimitModel-specific constraint groups:

(eq) Equality route-linking with unmet demand v:
     Σ x[s][t][od][pair] = v_{jkts} + Σ_r α^r_{jkts}    ∀ j,k,t,s

(fl) Fleet-size bound:
     Σ_r θ^r_{ts} ≤ F                                     ∀ t,s

Returns the number of constraints added.
"""
function add_fleet_limit_constraints!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: FleetLimitODMap
)::Int
    before = _total_num_constraints(m)
    S      = n_scenarios(data)
    inner  = mapping.inner

    x            = m[:x]
    alpha_r_jkts = m[:alpha_r_jkts]
    theta_r_ts   = m[:theta_r_ts]
    v_jkts       = m[:v_jkts]

    # ── (eq) Equality route-linking ────────────────────────────────────────────
    for s in 1:S
        for (t_id, od_pairs) in inner.Omega_s_t[s]
            # Collect all (j,k) legs active in this (s, t_id) bucket
            jk_set = Set{Tuple{Int, Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(inner, o, d)
                    push!(jk_set, (j, k))
                end
            end

            routes_t = get(inner.routes_s[s], t_id, RouteData[])

            for (j_idx, k_idx) in jk_set
                v_var = get(v_jkts, (s, j_idx, k_idx, t_id), nothing)
                v_var === nothing && continue   # no alpha coverage → skip

                # α sum: Σ_r α^r_{jkts}
                alpha_sum = AffExpr(0.0)
                for r_idx in 1:length(routes_t)
                    alpha_var = get(alpha_r_jkts, (s, r_idx, j_idx, k_idx, t_id), nothing)
                    alpha_var === nothing && continue
                    add_to_expression!(alpha_sum, 1.0, alpha_var)
                end

                # x sum: Σ_{od:(j,k)} x[s][t][od][pair]
                x_sum = AffExpr(0.0)
                for (od_idx, (o, d)) in enumerate(od_pairs)
                    valid_pairs = get_valid_jk_pairs(inner, o, d)
                    pair_idx    = findfirst(==((j_idx, k_idx)), valid_pairs)
                    pair_idx === nothing && continue
                    x_od = get(get(x[s], t_id, Dict{Int, Vector{VariableRef}}()), od_idx, VariableRef[])
                    isempty(x_od) && continue
                    add_to_expression!(x_sum, 1.0, x_od[pair_idx])
                end

                @constraint(m, x_sum == v_var + alpha_sum)
            end
        end
    end

    # ── (fl) Fleet-size bound: Σ_r θ^r_{ts} ≤ F   ∀ t, s ─────────────────────
    F = Float64(mapping.fleet_size)

    # Group theta variables by (s, t_id)
    st_to_thetas = Dict{Tuple{Int, Int}, Vector{VariableRef}}()
    for ((s, t_id, _r_idx), theta_var) in theta_r_ts
        push!(get!(st_to_thetas, (s, t_id), VariableRef[]), theta_var)
    end
    for ((s, t_id), thetas) in st_to_thetas
        isempty(thetas) && continue
        @constraint(m, sum(thetas) <= F)
    end

    return _total_num_constraints(m) - before
end
