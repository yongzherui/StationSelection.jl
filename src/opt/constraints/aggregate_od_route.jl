"""
Constraints and dynamic column update helpers for AggregateODRouteModel.
"""

export add_assignment_to_selected_constraints!
export add_aggregate_od_route_coverage_constraints!
export add_aggregate_od_route_column!
export add_or_update_aggregate_od_route_column!
export aggregate_od_route_column_objective_coefficient
export add_nearest_open_assignment_constraints!
export validate_big_m_nearest_aggregate_od_route!
export add_fixed_open_station_constraints!
export assert_endpoint_chain_near_binary
export nearest_open_endpoint_diagnostics

const _AggregateODRouteEndpointChainKey = Tuple{Symbol, Tuple{Int, Vararg{Int}}, Tuple{Float64, Vararg{Float64}}}

_is_endpoint_nearest_style(style::Symbol)::Bool = style in (:big_m_nearest, :endpoint_chain)

function add_assignment_to_selected_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, pair) in enumerate(get_valid_jk_pairs(mapping, o, d))
                is_walk_only_pair(pair) && continue
                j, k = pair
                @constraint(m, x_od[pair_idx] <= y[j])
                @constraint(m, x_od[pair_idx] <= y[k])
            end
        end
    end
    return _total_num_constraints(m) - before
end

function _aggregate_od_route_assignment_pair_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pair::Tuple{Int, Int},
)::Float64
    j, k = pair
    return get_walking_cost(data, o, j) + get_walking_cost(data, k, d)
end

function _rank_aggregate_od_route_pairs_by_assignment_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pairs::AbstractVector{Tuple{Int, Int}},
)::Vector{Int}
    idxs = collect(eachindex(pairs))
    sort!(idxs, by=i -> (_aggregate_od_route_assignment_pair_cost(data, o, d, pairs[i]), pairs[i][1], pairs[i][2]))
    return idxs
end

function add_nearest_open_assignment_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            ranked_pair_idxs = _rank_aggregate_od_route_pairs_by_assignment_cost(data, o, d, pairs)
            for rank_pos in 2:length(ranked_pair_idxs)
                pair_idx = ranked_pair_idxs[rank_pos]
                for prior_rank_pos in 1:(rank_pos - 1)
                    prior_pair_idx = ranked_pair_idxs[prior_rank_pos]
                    prior_j, prior_k = pairs[prior_pair_idx]
                    @constraint(m, x_od[pair_idx] <= 2.0 - y[prior_j] - y[prior_k])
                end
            end
        end
    end
    return _total_num_constraints(m) - before
end

function _endpoint_chain_key(
    side::Symbol,
    endpoints::Vector{Int},
    costs::Vector{Float64},
)::_AggregateODRouteEndpointChainKey
    return (side, Tuple(endpoints), Tuple(round.(costs; digits=9)))
end

"""
    _endpoint_chain_variable!(m, y, side, endpoints, costs; binary=...)

Station-generic nearest-open-station chain for one physical endpoint (in one
directional role, `side ∈ (:pickup, :dropoff)`). Deduplicated by content
(`side`, sorted station set, sorted walking costs) via
`m[:nearest_endpoint_chain_cache]`, so every request whose origin/destination
has the same candidate-station/cost profile shares the identical `z`
variables — no per-request re-derivation, and no ranking over station pairs.

`binary` controls whether `z` is declared `Bin` (compact model / BendersXY
master, genuine MIPs) or continuous `[0,1]` (BendersY LP subproblem, needed
for valid simplex duals off the `y`-fixing constraints — `y` is already fixed
binary there, so the chain constraints still force `z` to resolve to exactly
0/1 at the optimum; verified by `assert_endpoint_chain_near_binary` rather
than merely assumed). Defaults to reading
`m[:aggregate_od_route_relax_integrality]` when present (compact-model build
path), else `true`. One shared implementation used by the compact-model
build, the BendersXY master, and the BendersY LP subproblem.

Given `sum(z) == 1` and `z[rank] <= y[station]` for every rank, `z` is already
forced onto the correct single (nearest-open) candidate at any binary `y` --
`selected_cost <= cost[idx] + M[idx]*(1-y[station])` collapses to
`selected_cost <= cost[idx]` for whichever candidate is open, and since
`selected_cost` is a weighted average of costs over the simplex `z`, that
bound can only be met by concentrating all weight on the (uniquely) cheapest
open candidate. No separate lower-bound row is needed.

Also emits, once per distinct chain, the endpoint-coverage constraint
`sum(y[j] for j in candidates) >= 1`. This is mathematically implied by
`z[rank] <= y[station]` + `sum(z) == 1` already (any feasible integer point
already satisfies it), so it's redundant-but-harmless for correctness; it
exists to tighten the LP relaxation and give a direct diagnostic instead of a
downstream `sum(z)==1` infeasibility with no obvious cause.
"""
function _endpoint_chain_variable!(
    m::Model,
    y,
    side::Symbol,
    endpoints::Vector{Int},
    costs::Vector{Float64};
    binary::Bool = !(haskey(m, :aggregate_od_route_relax_integrality) &&
                      Bool(m[:aggregate_od_route_relax_integrality])),
)
    cache = if haskey(m, :nearest_endpoint_chain_cache)
        m[:nearest_endpoint_chain_cache]
    else
        m[:nearest_endpoint_chain_cache] = Dict{_AggregateODRouteEndpointChainKey, Vector{VariableRef}}()
    end
    order = sortperm(collect(eachindex(endpoints)); by=i -> (costs[i], endpoints[i]))
    sorted_endpoints = endpoints[order]
    sorted_costs = costs[order]
    key = _endpoint_chain_key(side, sorted_endpoints, sorted_costs)
    return get!(cache, key) do
        z = binary ?
            @variable(m, [1:length(sorted_endpoints)], binary = true) :
            @variable(m, [1:length(sorted_endpoints)], lower_bound = 0.0, upper_bound = 1.0)
        @constraint(m, sum(z) == 1.0)
        for (rank, station) in enumerate(sorted_endpoints)
            @constraint(m, z[rank] <= y[station])
            for prior in 1:(rank - 1)
                @constraint(m, z[rank] <= 1.0 - y[sorted_endpoints[prior]])
            end
        end
        @constraint(m, sum(y[station] for station in sorted_endpoints) >= 1.0)
        z
    end, sorted_endpoints
end

"""
    _big_m_tie_break_costs(sorted_costs) -> tb_costs

The SINGLE canonical implementation of the `:big_m_nearest` tie-break perturbation --
`sorted_costs` (a chain's walking costs, already sorted ascending) plus a strictly
increasing per-rank offset, used everywhere a `:big_m_nearest` chain's Big-M row
(`selected_cost <= cost[idx] + M*(1-y)`) is built OR dualized. See
`_endpoint_big_m_variable!`'s docstring for why the perturbation itself is needed
(breaking ties between candidates at exactly equal true cost, which would otherwise
leave `z` free to split fractionally across them).

**Why this must be the only implementation, not one hand-copied at each call site**:
this exact perturbed cost array appears in three logically-independent places that all
need to describe the *same* primal LP -- the real primal builder
(`_endpoint_big_m_variable!`, below), the restricted-MW dual-completion derivation
(`_sorted_endpoint_chain` in `benders/y_mw_cut.jl`, which every BendersY/BendersYZ
completion LP is built from), and `BendersYZ`'s joint core-point construction
(`_yz_joint_core_point` in `benders/yz_mw_cut.jl`). Until 2026-07-24, only the first of
these three actually applied the perturbation -- `_sorted_endpoint_chain` used the raw,
un-perturbed sorted costs, so the dual-derivation code was silently building the dual of
a *slightly different* primal (off by up to `(rank-1) * tie_break_scale`, tiny in
absolute terms) than the one `_endpoint_big_m_variable!` actually builds. That tiny gap
lands exactly on the knife-edge the perturbation itself is designed to create -- nearly
(but not exactly) tied Big-M rows -- which is precisely where LP duals become most
sensitive to small input changes. On instances with many roughly-equidistant candidates
per endpoint (e.g. `max_walking_distance` wide enough that most/all stations are
candidates), this manifested as real, non-numerical-noise violations of the completion
LP's own z-dual constraints (confirmed by comparing genuine, Gurobi-certified duals of
the real primal against the completion LP's formulas: exactly zero violations on
instances with few candidates per chain, but large, systematic ones -- tracking
`nu_sum * tie_break_scale` -- once chains had many candidates), which in turn caused
spurious `completion_infeasible` failures in `_restricted_mw_optimality_cut`/
`_solve_restricted_mw_completion`. Fixed by centralizing the perturbation here: every
caller that needs a `:big_m_nearest` chain's cost values must call this function (or a
function that calls it) on the sorted raw costs, and must NOT reapply the perturbation
to an already-perturbed array (that would double-perturb, not fix anything) -- code that
already receives tie-broken costs (e.g. `chain.costs` after `_restricted_mw_chains`) is
already correct and should use them as-is.

**Sizing `tie_break_scale`** (fixed 2026-07-24, second fix on this function the same day --
see the n=15 method-compare sweep's `assert_endpoint_chain_near_binary` failures this
replaces): each Big-M row is `selected_cost <= tb_costs[idx] + big_m*(1 - y[station])` with
`big_m <= max_cost`. Gurobi only guarantees a `Bin` variable within `IntFeasTol` (default
1e-5) of its bound, so at a "clean", accepted MIP solution `(1 - y[station])` can still be
as large as `IntFeasTol` -- turning that row's slack into `big_m * IntFeasTol`, not 0. Since
`z` is a simplex-weighted average of `tb_costs` bounded by that slack, an open-but-near-tied
candidate can absorb up to `(big_m * IntFeasTol) / tie_break_scale` of `z`'s mass without
violating anything. The previous `1e-6` coefficient was smaller than `IntFeasTol` itself, so
it gave essentially no protection -- confirmed by reproducing a live failure
(`grid_n15_p8_s123`, BendersYZ) where `y[7] = 0.9999966` (well inside `IntFeasTol`) let the
Big-M row for a 3-way-tied dropoff chain absorb `z=0.897/0.103` between two candidates, with
every constraint satisfied to ~1e-9 -- a genuine alternate LP optimum, not solver noise.
Solving `tie_break_scale >= max_cost * IntFeasTol / target_drift` for a `target_drift` an
order of magnitude under `assert_endpoint_chain_near_binary`'s `atol=1e-3` (i.e.
`target_drift=1e-4`) gives `tie_break_scale >= max_cost * 0.1` -- hence the `0.1` coefficient
below. This is safe against mis-ordering candidates despite being a much larger perturbation:
`sorted_costs` is already sorted ascending before this function ever runs, and the offset
added per rank is strictly positive and increasing, so it can only widen the gap between
ranks, never reorder them. Its only real cost is a larger (but still small relative to
genuine routing costs) bias baked into the MW/zero-completion duals derived from these rows
-- a cut-tightness/performance concern, not a correctness one.
"""
function _big_m_tie_break_costs(sorted_costs::Vector{Float64})::Vector{Float64}
    tie_break_scale = max(1e-4, maximum(abs, sorted_costs; init=0.0) * 0.1)
    return [sorted_costs[idx] + tie_break_scale * (idx - 1) for idx in eachindex(sorted_costs)]
end

"""
    _endpoint_big_m_variable!(m, y, side, endpoints, costs; binary=...)

Big-M nearest-open selector for one physical endpoint. The selected endpoint
indicator is linked to open stations by `z[i] <= y[station_i]`; for every
candidate `j'`, the selected walking distance must be no larger than
`cost[j']` whenever `y[j'] == 1`, relaxed by `M[j'] * (1 - y[j'])` when
`j'` is closed.

Two candidates tied at exactly the same true cost make `selected_cost`'s own
definition flat across them, so a continuous `z` (`binary=false`, used when
`y` is integer-fixed/known and this LP's simplex duals are needed -- the
BendersY subproblem, and now the BendersXY/BendersYZ/BendersYZH masters with
`y` as the only genuinely binary variable) has a real 1-dimensional degenerate
face of optimal solutions where `z` splits fractionally between the tied,
open candidates: nothing here discriminates by station id the way
`_endpoint_chain_variable!`'s cascading domination rows already do. Fixed by
building `selected_cost` (and the big-M threshold) from the strictly increasing
`tb_costs` array `_big_m_tie_break_costs` produces -- see that function's
docstring for the exact perturbation and, critically, why it must be computed
there and nowhere else (every other place that needs these tie-broken costs,
including the dual-completion derivation this primal is dualized into, must
call it too, not recompute it locally).

As with `_endpoint_chain_variable!`, no separate lower-bound row is needed:
given `sum(z)==1`, `z<=y`, and the Big-M row above, `z` is already forced onto
the nearest open candidate at any binary `y` (the Big-M row for whichever
candidate is open collapses to `selected_cost <= cost[idx]`, which a simplex
weighted average can only satisfy by placing all weight on the cheapest open
one).
"""
function _endpoint_big_m_variable!(
    m::Model,
    y,
    side::Symbol,
    endpoints::Vector{Int},
    costs::Vector{Float64};
    binary::Bool = !(haskey(m, :aggregate_od_route_relax_integrality) &&
                      Bool(m[:aggregate_od_route_relax_integrality])),
)
    cache = if haskey(m, :nearest_endpoint_chain_cache)
        m[:nearest_endpoint_chain_cache]
    else
        m[:nearest_endpoint_chain_cache] = Dict{_AggregateODRouteEndpointChainKey, Vector{VariableRef}}()
    end
    order = sortperm(collect(eachindex(endpoints)); by=i -> (costs[i], endpoints[i]))
    sorted_endpoints = endpoints[order]
    sorted_costs = costs[order]
    key = _endpoint_chain_key(side, sorted_endpoints, sorted_costs)
    # DEBUG (temporary instrumentation, 2026-07-24): mirror the variable cache with a constraint
    # cache so debug scripts can extract genuine Gurobi duals for sum(z)==1/z<=y/Big-M rows.
    con_cache = if haskey(m, :nearest_endpoint_chain_constraint_cache)
        m[:nearest_endpoint_chain_constraint_cache]
    else
        m[:nearest_endpoint_chain_constraint_cache] = Dict{_AggregateODRouteEndpointChainKey, NamedTuple}()
    end
    return get!(cache, key) do
        z = binary ?
            @variable(m, [1:length(sorted_endpoints)], binary = true) :
            @variable(m, [1:length(sorted_endpoints)], lower_bound = 0.0, upper_bound = 1.0)
        sum_z_con = @constraint(m, sum(z) == 1.0)
        tb_costs = _big_m_tie_break_costs(sorted_costs)
        selected_cost = @expression(m, sum(tb_costs[idx] * z[idx] for idx in eachindex(sorted_endpoints)))
        max_cost = maximum(tb_costs)
        mu_cons = Vector{ConstraintRef}(undef, length(sorted_endpoints))
        nu_cons = Vector{ConstraintRef}(undef, length(sorted_endpoints))
        for (idx, station) in enumerate(sorted_endpoints)
            mu_cons[idx] = @constraint(m, z[idx] <= y[station])
            big_m = max_cost - tb_costs[idx]
            nu_cons[idx] = @constraint(m, selected_cost <= tb_costs[idx] + big_m * (1.0 - y[station]))
        end
        row5_con = @constraint(m, sum(y[station] for station in sorted_endpoints) >= 1.0)
        con_cache[key] = (sum_z=sum_z_con, mu=mu_cons, nu=nu_cons, row5=row5_con)
        z
    end, sorted_endpoints
end

function _endpoint_selector_variable!(
    m::Model,
    y,
    side::Symbol,
    endpoints::Vector{Int},
    costs::Vector{Float64};
    binary::Bool,
    selector_style::Symbol,
)
    if haskey(m, :nearest_endpoint_selector_style)
        m[:nearest_endpoint_selector_style] == selector_style || throw(ArgumentError(
            "cannot mix endpoint selector styles $(m[:nearest_endpoint_selector_style]) and $(selector_style) in one model"
        ))
    else
        m[:nearest_endpoint_selector_style] = selector_style
    end
    selector_style == :big_m_nearest && return _endpoint_big_m_variable!(
        m, y, side, endpoints, costs; binary=binary,
    )
    selector_style == :endpoint_chain && return _endpoint_chain_variable!(
        m, y, side, endpoints, costs; binary=binary,
    )
    throw(ArgumentError("unsupported endpoint selector style $(selector_style)"))
end

"""
    assert_endpoint_chain_near_binary(m::Model; atol=1e-3)

Runtime check that every `zp`/`zd` endpoint selector indicator variable in
`m` is within `atol` of 0 or 1 in the current solution. The shared endpoint
selector cache is stored under `m[:nearest_endpoint_chain_cache]`, so this
works generically against any solved model that used an endpoint-nearest
encoding -- a no-op if the model never built one (`:pair_chain` style, or a
model with no nearest-open constraints at all). Meant to be called right
after `optimize!(m)` on what should be an integer-valued solve (a genuine
MIP, or an LP with `y` fixed to an already-binary value) -- a fractional `z`
there means the "nearest open" endpoint ranking isn't actually resolved to
a single winner, undermining any cost/cut derived from it.

`atol=1e-3` is deliberately looser than Gurobi's own default `IntFeasTol`
(1e-5): on larger, more degenerate instances (e.g. real BendersYZH runs with
many near-tied endpoint candidates) `z` values like `0.9996092321175274` have
been observed -- still a clean, unambiguous "selected" indicator, but outside
even the 1e-5 `IntFeasTol`-matched tolerance this check used previously. Every
call site that reads a `z` value onward for further computation applies
`round.(value.(...))` (see benders/subproblem_api.jl and benders/yz.jl) rather
than using the raw float, so values this check passes are always snapped to
an exact 0.0/1.0 before they can propagate any of this slack into downstream
cost or dual computations.
"""
function assert_endpoint_chain_near_binary(m::Model; atol::Float64=1e-3)::Nothing
    haskey(m, :nearest_endpoint_chain_cache) || return nothing
    for (key, vars) in m[:nearest_endpoint_chain_cache]
        for (idx, var) in enumerate(vars)
            val = value(var)
            (val <= atol || val >= 1.0 - atol) || throw(ArgumentError(
                "endpoint-chain (z) indicator check failed: z[$(idx)] in chain $(key) has value " *
                "$(val), not within atol=$(atol) of 0 or 1"
            ))
        end
    end
    return nothing
end

"""
    nearest_open_endpoint_diagnostics(m, data, mapping) -> Dict{String, Any}

Reports sizing and integrality diagnostics for a solved (or built)
endpoint-nearest model: distinct endpoint selectors (`"endpoint_count"`, i.e.
distinct `z_{ij}` selectors, one per `(side, physical endpoint)`), the total
`z` variable count summed over chains, request-pair (`x`) vs direct-walking
(`w`, the `WALK_ONLY_PAIR` slot) variable counts, nearest-open selector
constraint count, and request-coupling constraint count (the `<=,<=,>=`
linearization rows). If `m` has been solved, also reports whether any
`z`/`x`/`w` value is fractional (`"has_fractional_solution"`,
`"fractional_variables"`) -- a no-op-safe `false`/empty when `m` hasn't been
solved or has no endpoint selectors at all (e.g. `:pair_chain`, or no
nearest-open constraints).
"""
function nearest_open_endpoint_diagnostics(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Dict{String, Any}
    diagnostics = Dict{String, Any}(
        "endpoint_count" => 0,
        "z_variable_count" => 0,
        "x_pair_variable_count" => 0,
        "w_walk_variable_count" => 0,
        "nearest_open_chain_constraint_count" => 0,
        "request_coupling_constraint_count" => 0,
        "endpoint_selector_style" => nothing,
        "has_fractional_solution" => false,
        "fractional_variables" => Tuple{Symbol, Any, Int}[],
    )
    if haskey(m, :nearest_endpoint_chain_cache)
        cache = m[:nearest_endpoint_chain_cache]
        diagnostics["endpoint_count"] = length(cache)
        diagnostics["z_variable_count"] = sum(length(z) for z in values(cache); init=0)
        selector_style = haskey(m, :nearest_endpoint_selector_style) ?
            m[:nearest_endpoint_selector_style] : :endpoint_chain
        diagnostics["endpoint_selector_style"] = string(selector_style)
        diagnostics["nearest_open_chain_constraint_count"] = if selector_style == :big_m_nearest
            # sum(z)==1, z<=y for each candidate, one Big-M row per candidate,
            # and the redundant endpoint coverage row.
            sum(2 + 2 * length(z) for z in values(cache); init=0)
        else
            # sum(z)==1, z<=y for each candidate, triangular prior-open rows,
            # and the redundant endpoint coverage row.
            sum(2 + length(z) + div(length(z) * (length(z) - 1), 2) for z in values(cache); init=0)
        end
        can_check_values = has_values(m)
        for (key, z) in cache
            for (idx, var) in enumerate(z)
                can_check_values || break
                val = value(var)
                (val > 1e-6 && val < 1.0 - 1e-6) &&
                    push!(diagnostics["fractional_variables"], (:z, key, idx))
            end
        end
    end
    x_pairs = 0
    w_pairs = 0
    coupling_constraints = 0
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(pairs) && continue
            real_count = count(!is_walk_only_pair, pairs)
            walk_count = count(is_walk_only_pair, pairs)
            x_pairs += real_count
            w_pairs += walk_count
            # <=, <=, >= per real pair, plus one >= per common-candidate walk row.
            coupling_constraints += 3 * real_count
        end
    end
    diagnostics["x_pair_variable_count"] = x_pairs
    diagnostics["w_walk_variable_count"] = w_pairs
    diagnostics["request_coupling_constraint_count"] = coupling_constraints
    diagnostics["has_fractional_solution"] = !isempty(diagnostics["fractional_variables"])
    return diagnostics
end

"""
    _nearest_open_endpoint_candidates(data, endpoint, max_walking_distance, side)

Stations within `max_walking_distance` of a physical demand `endpoint`, in
the walking-cost direction implied by `side` (`:pickup` queries
`get_walking_cost(data, endpoint, j)`, i.e. endpoint -> station; `:dropoff`
queries `get_walking_cost(data, j, endpoint)`, i.e. station -> endpoint —
walking costs are not assumed symmetric). This is `J_i` from the
station-generic nearest-open formulation, derived directly and independently
of any specific OD pair's off-diagonal (j != k) pair list — required so that
an endpoint's true candidate set is correct even in the degenerate case
where the *only* candidate on each side of an OD pair is the same station
(the off-diagonal pair list would then be empty, but the candidate sets are
each still the singleton `{that station}`).
"""
function _nearest_open_endpoint_candidates(
    data::StationSelectionData,
    endpoint::Int,
    max_walking_distance::Float64,
    side::Symbol,
)::Vector{Int}
    n = data.n_stations
    return [
        j for j in 1:n
        if (side == :pickup ? get_walking_cost(data, endpoint, j) : get_walking_cost(data, j, endpoint)) <=
           max_walking_distance
    ]
end

"""
    _check_big_m_nearest_pair_consistency!(data, o, d, real_pairs, pickups, dropoffs)

Defensive cross-check that `real_pairs` (the non-walk `(j,k)` pairs
`compute_valid_jk_pairs` produced for OD `(o,d)`) is *exactly* the Cartesian
product of the independently-derived candidate sets `pickups`/`dropoffs`
(`_nearest_open_endpoint_candidates`), either off-diagonal (same-station
pairs disabled, the default) or the full product (`allow_same_station=true`,
`is_same_station_pair`). Both are computed from the same underlying
walking-cost data and `max_walking_distance`, so any other mismatch
indicates a real bug, not a modeling choice. Accepting either shape here
(rather than requiring an explicit `allow_same_station` parameter threaded
through every caller) keeps this purely a defensive check, not a second
place that has to be kept in sync with the flag.
"""
function _check_big_m_nearest_pair_consistency!(
    data::StationSelectionData,
    o::Int,
    d::Int,
    real_pairs::Vector{Tuple{Int, Int}},
    pickups::Vector{Int},
    dropoffs::Vector{Int},
)::Nothing
    off_diagonal = Set((j, k) for j in pickups for k in dropoffs if j != k)
    full_product = Set((j, k) for j in pickups for k in dropoffs)
    real_pairs_set = Set(real_pairs)
    (real_pairs_set == off_diagonal || real_pairs_set == full_product) || throw(ArgumentError(
        ":big_m_nearest requires feasible pairs for OD $((o, d)) to be exactly the pickup/dropoff " *
        "Cartesian product (off-diagonal, or full if same-station pairs are enabled); got " *
        "$(sort(real_pairs)), expected $(sort(collect(off_diagonal))) or $(sort(collect(full_product)))"
    ))
    return nothing
end

function _assert_endpoint_has_candidates!(o::Int, d::Int, side::Symbol, candidates::Vector{Int})::Nothing
    isempty(candidates) && throw(ArgumentError(
        ":big_m_nearest requires at least one candidate station within max_walking_distance for OD " *
        "$((o, d))'s $(side) endpoint ($(side == :pickup ? o : d)); none found. Either widen " *
        "max_walking_distance or exclude this OD pair from the request data."
    ))
    return nothing
end

"""
    _assert_walk_collision_feasible!(data, o, d, pickups, dropoffs, pairs)

If some station is a candidate for *both* sides of OD `(o,d)` — i.e. under
some feasible `y` the nearest-open station could resolve to the same
physical station for both endpoints — then direct walking must be a
feasible fallback (`WALK_ONLY_PAIR in pairs`, i.e.
`dist(o,d) <= 2*max_walking_distance`). This is guaranteed by the triangle
inequality whenever walking costs form a true metric
(`dist(o,d) <= dist(o,j) + dist(j,d) <= D + D` for any common candidate
`j`); this assertion is the runtime check that the actual precomputed
walking-cost data honors that assumption instead of trusting it blindly
(real data can have asymmetric or disconnected pedestrian networks). Only
called when `allow_walk_only` was requested — without it, direct walking was
never offered as an option and this OD pair's behavior is unchanged from
before this feature existed.
"""
function _assert_walk_collision_feasible!(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pickups::Vector{Int},
    dropoffs::Vector{Int},
    pairs::Vector{Tuple{Int, Int}},
)::Nothing
    common = intersect(Set(pickups), Set(dropoffs))
    isempty(common) && return nothing
    any(is_walk_only_pair, pairs) || throw(ArgumentError(
        "OD $((o, d)) has station(s) $(sort(collect(common))) that could be the nearest-open " *
        "station for both endpoints simultaneously, but direct walking is not feasible " *
        "(dist(o,d)=$(get_walking_cost(data, o, d)) exceeds 2*max_walking_distance). This violates " *
        "the triangle-inequality assumption the nearest-open + direct-walking formulation relies on " *
        "— check for asymmetric or disconnected walking-cost data between $(o), $(d), and " *
        "$(sort(collect(common)))."
    ))
    return nothing
end

"""
    _nearest_open_endpoint_selectors!(m, data, y, o, d, pairs, max_walking_distance; binary, allow_walk_only, selector_style)

Candidate lookup, validation, and `zp`/`zd` chain construction for one
physical OD bucket `(o,d)` -- everything `_add_nearest_open_endpoint_linked_x!`
needs before it can link an assignment variable to the two sides. Extracted
so callers that need `zp`/`zd` (and their rank maps) without any `x`/`h`
linking -- e.g. the BendersYZ/BendersYZH master `z`-builder, which only needs
this function's cache side-effect on `m[:nearest_endpoint_chain_cache]` -- can
reuse the exact same candidate/validation/chain logic instead of duplicating
it.
"""
function _nearest_open_endpoint_selectors!(
    m::Model,
    data::StationSelectionData,
    y,
    o::Int,
    d::Int,
    pairs::Vector{Tuple{Int, Int}},
    max_walking_distance::Float64;
    binary::Bool,
    allow_walk_only::Bool,
    selector_style::Symbol,
)
    real_pairs = filter(!is_walk_only_pair, pairs)
    pickups = _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup)
    dropoffs = _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff)
    _assert_endpoint_has_candidates!(o, d, :pickup, pickups)
    _assert_endpoint_has_candidates!(o, d, :dropoff, dropoffs)
    _check_big_m_nearest_pair_consistency!(data, o, d, real_pairs, pickups, dropoffs)
    allow_walk_only && _assert_walk_collision_feasible!(data, o, d, pickups, dropoffs, pairs)

    pickup_costs = [get_walking_cost(data, o, j) for j in pickups]
    dropoff_costs = [get_walking_cost(data, k, d) for k in dropoffs]
    zp, sorted_pickups = _endpoint_selector_variable!(
        m, y, :pickup, pickups, pickup_costs; binary=binary, selector_style=selector_style,
    )
    zd, sorted_dropoffs = _endpoint_selector_variable!(
        m, y, :dropoff, dropoffs, dropoff_costs; binary=binary, selector_style=selector_style,
    )
    pickup_rank = Dict(station => idx for (idx, station) in enumerate(sorted_pickups))
    dropoff_rank = Dict(station => idx for (idx, station) in enumerate(sorted_dropoffs))

    return zp, zd, real_pairs, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs
end

"""
    _add_endpoint_x_linking!(m, real_pairs, pairs, assignment_by_pair, zp, zd, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs)

Links an assignment variable (`x` for the per-scenario master/subproblem
builders, `h` for BendersYZH's scenario-compressed master) to the `zp`/`zd`
endpoint selectors returned by [`_nearest_open_endpoint_selectors!`](@ref),
via the full product linearization `assignment <= zp, assignment <= zd,
assignment >= zp + zd - 1` for every real `(j,k)` pair, plus the walk-only
same-station coupling lower bound when `WALK_ONLY_PAIR` is present in
`pairs`. Body identical to the linking half of the pre-refactor
`_add_nearest_open_endpoint_linked_x!`; see that function's docstring for why
the lower bound alone is enough to make the outcome deterministic.
"""
function _add_endpoint_x_linking!(
    m::Model,
    real_pairs::Vector{Tuple{Int, Int}},
    pairs::Vector{Tuple{Int, Int}},
    assignment_by_pair::Dict{Tuple{Int, Int}, VariableRef},
    zp,
    zd,
    pickup_rank::Dict{Int, Int},
    dropoff_rank::Dict{Int, Int},
    sorted_pickups::Vector{Int},
    sorted_dropoffs::Vector{Int};
    debug_key_prefix=nothing,
)::Nothing
    # DEBUG (temporary instrumentation, 2026-07-24): capture x-linking constraint refs so debug
    # scripts can extract genuine Gurobi duals for the rhoO/rhoD/sigma rows.
    debug_out = if !isnothing(debug_key_prefix)
        haskey(m, :debug_x_linking_cons) || (m[:debug_x_linking_cons] = Dict{Any, NamedTuple}())
        m[:debug_x_linking_cons]
    else
        nothing
    end
    for (j, k) in real_pairs
        assignment = assignment_by_pair[(j, k)]
        le_zp = @constraint(m, assignment <= zp[pickup_rank[j]])
        le_zd = @constraint(m, assignment <= zd[dropoff_rank[k]])
        ge_sum = @constraint(m, assignment >= zp[pickup_rank[j]] + zd[dropoff_rank[k]] - 1.0)
        isnothing(debug_out) || (debug_out[(debug_key_prefix, (j, k))] = (le_zp=le_zp, le_zd=le_zd, ge_sum=ge_sum))
    end

    if any(is_walk_only_pair, pairs)
        assignment_walk = assignment_by_pair[WALK_ONLY_PAIR]
        for j in intersect(Set(sorted_pickups), Set(sorted_dropoffs))
            @constraint(m, assignment_walk >= zp[pickup_rank[j]] + zd[dropoff_rank[j]] - 1.0)
        end
    end
    return nothing
end

"""
    _add_nearest_open_endpoint_linked_x!(m, data, y, o, d, pairs, x_by_pair, max_walking_distance; binary, allow_walk_only, selector_style)

Station-generic nearest-open request coupling for one physical OD bucket
(call once per distinct `(o,d)` with positive demand — shared across every
request in that bucket and across scenarios, since `y` and the containing
model are shared). Builds/reuses the independent per-side `zp`/`zd` endpoint
selectors ([`_nearest_open_endpoint_selectors!`](@ref)) and links every real
`(j,k)` pair to them ([`_add_endpoint_x_linking!`](@ref)).
"""
function _add_nearest_open_endpoint_linked_x!(
    m::Model,
    data::StationSelectionData,
    y,
    o::Int,
    d::Int,
    pairs::Vector{Tuple{Int, Int}},
    x_by_pair::Dict{Tuple{Int, Int}, VariableRef},
    max_walking_distance::Float64;
    binary::Bool,
    allow_walk_only::Bool,
    selector_style::Symbol,
    debug_key_prefix=nothing,
)::Nothing
    zp, zd, real_pairs, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs =
        _nearest_open_endpoint_selectors!(
            m, data, y, o, d, pairs, max_walking_distance;
            binary=binary, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
    _add_endpoint_x_linking!(
        m, real_pairs, pairs, x_by_pair, zp, zd, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs;
        debug_key_prefix=debug_key_prefix,
    )
    return nothing
end

function add_nearest_open_endpoint_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    allow_walk_only::Bool,
    selector_style::Symbol,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    relax_integrality = haskey(m, :aggregate_od_route_relax_integrality) &&
        Bool(m[:aggregate_od_route_relax_integrality])
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            # Deliberately does NOT skip when there are zero feasible pairs
            # (unlike other aggregate-OD-route constraint builders): a
            # completely empty pair list is exactly the case
            # _assert_walk_collision_feasible!/_assert_endpoint_has_candidates!
            # need to see, to raise a clear diagnostic instead of silently
            # dropping this OD's demand from the model.
            x_od = get(x[s], od_idx, VariableRef[])
            pairs = get_valid_jk_pairs(mapping, o, d)
            x_by_pair = Dict(pair => x_od[idx] for (idx, pair) in enumerate(pairs))
            _add_nearest_open_endpoint_linked_x!(
                m, data, y, o, d, pairs, x_by_pair, mapping.max_walking_distance;
                binary=!relax_integrality, allow_walk_only=allow_walk_only, selector_style=selector_style,
            )
        end
    end
    return _total_num_constraints(m) - before
end

function validate_big_m_nearest_aggregate_od_route!(
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    allow_walk_only::Bool=false,
)::Nothing
    for s in 1:n_scenarios(data)
        for (o, d) in mapping.Omega_s[s]
            pairs = get_valid_jk_pairs(mapping, o, d)
            isempty(pairs) && continue
            real_pairs = filter(!is_walk_only_pair, pairs)
            pickups = _nearest_open_endpoint_candidates(data, o, mapping.max_walking_distance, :pickup)
            dropoffs = _nearest_open_endpoint_candidates(data, d, mapping.max_walking_distance, :dropoff)
            _assert_endpoint_has_candidates!(o, d, :pickup, pickups)
            _assert_endpoint_has_candidates!(o, d, :dropoff, dropoffs)
            _check_big_m_nearest_pair_consistency!(data, o, d, real_pairs, pickups, dropoffs)
            allow_walk_only && _assert_walk_collision_feasible!(data, o, d, pickups, dropoffs, pairs)
        end
    end
    return nothing
end

function add_fixed_open_station_constraints!(
    m::Model,
    data::StationSelectionData,
    model::RouteCoveringProblem,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    open_set = Set(model.open_stations)
    for station in 1:data.n_stations
        if station in open_set
            @constraint(m, y[station] == 1.0)
        else
            @constraint(m, y[station] == 0.0)
        end
    end
    return _total_num_constraints(m) - before
end

function add_aggregate_od_route_coverage_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    x = m[:x]
    theta = m[:theta_compat]
    coverage = Dict{NTuple{5, Int}, ConstraintRef}()
    coverage_by_pair_s = Dict{NTuple{3, Int}, Vector{ConstraintRef}}()
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, pair) in enumerate(get_valid_jk_pairs(mapping, o, d))
                # Walk-only and same-station assignments use no vehicle route, so
                # no route column needs to (or can) cover them — skip the coverage
                # constraint. Without this, a same-station x_{j,j} forced to 1 by
                # the endpoint-chain linking would be forced back to 0 here (no
                # route column covers a (j,j) "leg"), a direct infeasibility.
                requires_no_vehicle_route(pair) && continue
                j, k = pair
                expr = AffExpr(0.0)
                for column_id in get(mapping.columns_by_pair, (j, k), Int[])
                    theta_var = get(theta, (column_id, s), nothing)
                    theta_var === nothing && continue
                    add_to_expression!(expr, 1.0, theta_var)
                end
                # Set-covering semantics: a route covering this pair "for free" alongside
                # others it was selected for is harmless (>= 0, not == 0) -- routes routinely
                # overlap in which pairs they serve (that's the point of pooling), and forcing
                # exact equality would make otherwise-cheaper, perfectly valid combinations
                # infeasible whenever two selected routes happen to both cover the same pair.
                con = @constraint(m, expr - x_od[pair_idx] >= 0.0)
                coverage[(j, k, s, od_idx, pair_idx)] = con
                push!(get!(coverage_by_pair_s, (j, k, s), ConstraintRef[]), con)
            end
        end
    end
    m[:aggregate_od_route_coverage_constraints] = coverage
    m[:aggregate_od_route_coverage_by_pair_s] = coverage_by_pair_s
    return _total_num_constraints(m) - before
end

aggregate_od_route_column_objective_coefficient(
    route_regularization_weight::Real,
    repositioning_time::Real,
    column::AggregateODRouteColumn,
) = Float64(route_regularization_weight) * (column.tau + Float64(repositioning_time))

function add_aggregate_od_route_column!(
    m::Model,
    mapping::AggregateODRouteMap,
    column::AggregateODRouteColumn,
)::AggregateODRouteColumn
    _register_aggregate_od_route_column_metadata!(mapping, column)

    S = length(mapping.scenarios)
    relax_integrality = Bool(m[:aggregate_od_route_relax_integrality])
    mu = Float64(m[:aggregate_od_route_route_regularization_weight])
    rho = Float64(m[:aggregate_od_route_repositioning_time])
    theta = m[:theta_compat]
    coverage_by_pair_s = m[:aggregate_od_route_coverage_by_pair_s]

    obj_coef = aggregate_od_route_column_objective_coefficient(mu, rho, column)
    for s in 1:S
        theta_var = relax_integrality ?
            @variable(m, lower_bound = 0.0, upper_bound = 1.0) :
            @variable(m, binary = true)
        theta[(column.id, s)] = theta_var
        set_objective_coefficient(m, theta_var, obj_coef)

        for (j, k) in column.od_pairs
            for con in get(coverage_by_pair_s, (j, k, s), ConstraintRef[])
                set_normalized_coefficient(con, theta_var, 1.0)
            end
        end
    end
    return column
end

function _aggregate_od_route_column_signature_from_pairs(pairs)
    return Tuple(sort!(collect(pairs)))
end

_aggregate_od_route_column_signature_for_update(column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature_from_pairs(column.od_pairs)

function _replace_aggregate_od_route_column_metadata!(
    mapping::AggregateODRouteMap,
    existing_idx::Int,
    column::AggregateODRouteColumn,
)::AggregateODRouteColumn
    existing = mapping.columns[existing_idx]
    replacement = AggregateODRouteColumn(
        existing.id,
        existing.od_pairs,
        column.tau;
        metadata=column.metadata,
    )
    mapping.columns[existing_idx] = replacement
    return replacement
end

function add_or_update_aggregate_od_route_column!(
    m::Model,
    mapping::AggregateODRouteMap,
    column::AggregateODRouteColumn,
)::Tuple{Union{VariableRef, Nothing}, Symbol}
    signature = _aggregate_od_route_column_signature_for_update(column)
    existing_idx = findfirst(
        existing -> _aggregate_od_route_column_signature_for_update(existing) == signature,
        mapping.columns,
    )

    if !isnothing(existing_idx)
        existing = mapping.columns[existing_idx]
        theta = m[:theta_compat]
        if column.tau < existing.tau - 1e-9
            replacement = _replace_aggregate_od_route_column_metadata!(mapping, existing_idx, column)
            mu = Float64(m[:aggregate_od_route_route_regularization_weight])
            rho = Float64(m[:aggregate_od_route_repositioning_time])
            obj_coef = aggregate_od_route_column_objective_coefficient(mu, rho, replacement)
            for s in 1:length(mapping.scenarios)
                theta_var = get(theta, (replacement.id, s), nothing)
                theta_var === nothing && continue
                set_objective_coefficient(m, theta_var, obj_coef)
            end
            return get(theta, (replacement.id, 1), nothing), :replaced
        end
        return get(theta, (existing.id, 1), nothing), :skipped
    end

    added = add_aggregate_od_route_column!(m, mapping, column)
    return get(m[:theta_compat], (added.id, 1), nothing), :added
end

function add_or_update_aggregate_od_route_column!(
    build_result::BuildResult,
    column::AggregateODRouteColumn,
)::Tuple{Union{VariableRef, Nothing}, Symbol}
    return add_or_update_aggregate_od_route_column!(build_result.model, build_result.mapping, column)
end

function add_aggregate_od_route_column!(
    build_result::BuildResult,
    column::AggregateODRouteColumn,
)::AggregateODRouteColumn
    return add_aggregate_od_route_column!(build_result.model, build_result.mapping, column)
end
