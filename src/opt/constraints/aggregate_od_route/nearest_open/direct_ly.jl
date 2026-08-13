"""
`:direct_ly` nearest-open style: `(y, θ)`-only coverage via a per-request γ-chain, no
assignment variable `x` at all. Reuses `pair_chain.jl`'s ranking
(`_rank_aggregate_od_route_pairs_by_assignment_cost`) -- same deterministic cheapest-first
order `:pair_chain` uses, not an independently-invented one.
"""

export add_gamma_chain_nearest_open_coverage!

"""
    add_gamma_chain_nearest_open_coverage!(m, data, mapping) -> (Int, AffExpr)

`(y, θ)`-only nearest-open coverage for `NearestOpenAggregateODAssignmentPolicy(:direct_ly)`
— no assignment variable `x`, replaced by a real per-request `γ`-chain (kept as an actual
variable, not eliminated) directly modeled on `StationARC.jl`'s
`nearest_open_shared.jl:add_gamma_chain!`/`nearest_open.jl:build_nearest_open_master_lp`.

This package's own earlier attempt at this style eliminated *both* the assignment variable
and the per-endpoint selector down to a single raw affine row per pair
(`y_j+y_k-1-Σcheaper`); that was shown (empirically, at Zhuzhou n=20 scale, and by hand) to
have a catastrophically loose LP relaxation (`lp_bound=0.0`) because nothing forced the
"credit" mass on any side to actually sum to anything — `y` could be spread thin enough to
make every row's right-hand side `<=0` at once, at zero cost, regardless of how many route
columns were available. The γ-chain avoids this: `γ` is a real variable carrying its own
conservation law (`γ` climbs monotonically from `0` to a hard `γ_R=1`), so the same "spread
y to dodge every row" trick no longer has anywhere to hide — see git history / the design
discussion this function's introduction is tied to for the worked counterexample.

For each scenario `s`, each OD bucket `(o,d)` with demand, feasible pairs are ranked
*jointly* by combined walking cost (`_rank_aggregate_od_route_pairs_by_assignment_cost`,
the same ranking `:pair_chain` already uses — deterministic tie-break by `(j,k)` id, not an
independently-invented order). For rank `r` with pair `(j_r,k_r)` and `prev = γ_{r-1}`
(`0` at `r=1`):

```text
γ_r >= prev
γ_r >= y_{j_r} + y_{k_r} - 1
γ_r <= prev + y_{j_r}
γ_r <= prev + y_{k_r}
γ_R == 1                                          -- last rank only

Σ_{route covering (j_r,k_r) in s} θ_{route,s}  >=  γ_r - prev      -- route coverage, x folded in
```

`γ_r - prev` is exactly the indicator "pair `r` is the first (cheapest) ranked pair whose
both endpoints are open" — the standard tight sequential-OR encoding (verified exhaustively
against the true nearest-open indicator, all binary `y` patterns including ties, in
`test/opt/test_aggregate_od_route_direct_ly.jl`).

Requires `NearestOpenAggregateODAssignmentPolicy(:direct_ly)`'s caller to reject
`allow_walk_only` (no station-free walk-only sentinel wiring here, mirroring `:pair_chain`'s
restriction) — enforced in `_build_aggregate_od_route_core!`, not here.

Populates `m[:aggregate_od_route_coverage_constraints]`/`m[:aggregate_od_route_coverage_by_pair_s]`
in the identical shape `add_aggregate_od_route_coverage_constraints!` (joint_routing_assignment/)
uses (5-tuple key `(j, k, s, od_idx, pair_idx)`, `pair_idx` = index into that OD bucket's
`get_valid_jk_pairs` list, coinciding with the pair's rank here since every feasible pair gets
exactly one row), so `extract_aggregate_od_route_coverage_duals` and the rest of the
pricing/column-generation pipeline work completely unmodified — the `σ` reward aggregation
pricing needs is already implemented there; it now just sums duals of these new rows instead
of the old ones.

Returns the constraint count and an un-weighted walking-cost `AffExpr`
(`Σ_r cost(j_r,k_r) * (γ_r - prev)`) for the caller to fold into the objective at
`walk_cost_weight`, mirroring how ARC-LY's own objective attributes `assignment_cost` per
column — unlike the earlier per-pair-only attempt, walking cost has a clean linear
representation here (via `γ` differences), so `walk_cost_weight` is no longer forced to `0`.
"""
function add_gamma_chain_nearest_open_coverage!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Tuple{Int, AffExpr}
    before = _total_num_constraints(m)
    y = m[:y]
    theta = m[:theta_compat]
    relax_integrality = haskey(m, :aggregate_od_route_relax_integrality) &&
        Bool(m[:aggregate_od_route_relax_integrality])
    coverage = Dict{NTuple{5, Int}, ConstraintRef}()
    coverage_by_pair_s = Dict{NTuple{3, Int}, Vector{ConstraintRef}}()
    walking_cost_expr = AffExpr(0.0)

    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(pairs) && continue
            ranked_pair_idxs = _rank_aggregate_od_route_pairs_by_assignment_cost(data, o, d, pairs)
            R = length(ranked_pair_idxs)
            R > 0 || continue

            prev = AffExpr(0.0)
            for (rank, pair_idx) in enumerate(ranked_pair_idxs)
                j, k = pairs[pair_idx]
                g = relax_integrality ?
                    @variable(m, lower_bound = 0.0, upper_bound = 1.0) :
                    @variable(m, binary = true)
                @constraint(m, g >= prev)
                @constraint(m, g >= y[j] + y[k] - 1.0)
                @constraint(m, g <= prev + y[j])
                @constraint(m, g <= prev + y[k])
                rank == R && @constraint(m, g == 1.0)

                x_equiv = g - prev

                if !requires_no_vehicle_route((j, k))
                    expr = AffExpr(0.0)
                    for column_id in get(mapping.columns_by_pair, (j, k), Int[])
                        theta_var = get(theta, (column_id, s), nothing)
                        theta_var === nothing && continue
                        add_to_expression!(expr, 1.0, theta_var)
                    end
                    con = @constraint(m, expr - x_equiv >= 0.0)
                    coverage[(j, k, s, od_idx, pair_idx)] = con
                    push!(get!(coverage_by_pair_s, (j, k, s), ConstraintRef[]), con)
                end

                cost = _aggregate_od_route_assignment_pair_cost(data, o, d, (j, k))
                add_to_expression!(walking_cost_expr, cost, x_equiv)

                prev = 1.0 * g
            end
        end
    end
    m[:aggregate_od_route_coverage_constraints] = coverage
    m[:aggregate_od_route_coverage_by_pair_s] = coverage_by_pair_s
    return _total_num_constraints(m) - before, walking_cost_expr
end
