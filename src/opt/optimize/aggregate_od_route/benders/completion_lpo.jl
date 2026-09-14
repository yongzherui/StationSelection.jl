"""
Locally Pareto-optimal dual selection for the ACTIVATED Benders subproblem
(`oracle = :column_generation_activated_lpo`).

Cummings, Jacquillat and Vaze's *Activated Benders Decomposition for Day-Ahead Paratransit
Itinerary Planning* supplies the principle: at a fixed incumbent `yhat` the recourse value
`z* = Q(yhat)` usually has MANY dual-optimal representations, an arbitrary one of them puts
arbitrary coefficients on the stations `yhat` does not build, and re-optimising over the
optimal dual FACE against a core point picks the representation that says the most about the
`y` the master might try next. Their procedure is: solve the activated subproblem, keep only
its VALUE, rebuild the activated dual, pin the value at `yhat` with an equality, maximise the
cut at a core point, reconstruct the omitted linking multipliers, emit the cut.

**This is that principle, not their procedure.** Two things differ and neither is cosmetic:

1. **Our second stage is column-generated.** Their dual constraint set is written down; ours
   has one row per route column and is exponential. Solving the Pareto LP once over the route
   pool left behind by the activated CG solve is **NOT sufficient** -- that pool certified the
   ORIGINAL duals, and the Pareto step moves them. `_benders_lpo_pareto!` therefore re-prices
   after every LP solve and turns each negative-reduced-cost route into a new dual row, until
   pricing certifies feasibility over the whole activated family. See "The Pareto pricing
   loop" below.
2. **Our omitted multipliers do not reconstruct coordinate-wise.** Theirs do. Ours are coupled
   through the pickup/dropoff reward `rho_pjk = alpha_p - gammaO_pj - gammaD_pk - c_walk`, so
   in place of a reconstruction we impose the conservative, analytically certifiable condition
   `rho_pjk <= 0` on every assignment touching an unbuilt station. That is OUR completion
   mechanism combined with THEIR local-Pareto principle; it is not equivalent to theirs and
   must not be described as such.

# Notation

`J+ = {j : yhat_j = 1}` (built), `J0 = {j : yhat_j = 0}` (unbuilt). `R+` is the full activated
route family; `C+ ⊆ R+` is the finite pool the activated CG solve left behind. `g_j` is the
cut's station coefficient, `g_j = sum_p gammaO_pj + sum_p gammaD_pj`, i.e. the `Gamma_j` the
rest of this package's Benders code calls it. A cut is
`Theta_s >= sum_p alpha_p - sum_j g_j y_j`, and `C(y)` denotes its right-hand side at `y`.

# Two methods, one completion rule

Both are reachable, selected by `BendersSubproblemConfig.lpo_completion`, and both use the
**same** validity rule -- which is the point: comparing a Pareto dual against a baseline that
was allowed a weaker validity rule would measure the rule, not the Pareto step.

## `:baseline` -- ordinary activated dual, damage-minimising completion

`alpha_hat` and the built-station `gamma_hat` are FIXED at whatever the activated CG solve
returned. Only the unbuilt-station multipliers are free, and they are chosen to do the least
damage to the cut at the core point:

    min_{g^0 >= 0}  sum_{j in J0} y^c_j g_j
    s.t.            rho_pjk <= 0   for every valid triple with j in J0 or k in J0

This needs **no pricing at all** -- see "Why the completion is globally valid", which shows
`rho <= 0` on the out-triples plus the activated certificate on the in-triples discharges
every route row in the FULL family `R`, and the activated certificate is exactly what the
completed activated CG solve already established at these same `alpha_hat, gamma_hat`.

It is a strict improvement on `_benders_activated_complete_duals!`'s closed form, which
satisfies the same rows by loading each one entirely onto one endpoint at its cheapest
partner; the LP is free to split the requirement and to reuse one station's multiplier across
many rows.

## `:pareto` -- optimal-face re-optimisation (the experiment)

Nothing is fixed except the VALUE. All of `alpha`, the built `gamma`, and the unbuilt `gamma`
are variables:

    max   sum_p alpha_p - sum_j y^c_j g_j                                     (the cut at y^c)
    s.t.  sum_p alpha_p - sum_{j in J+} g_j            =  z*                  (optimal face)
          alpha_p                                      <= w * demand_p * walk(o,d,WALK_ONLY)
                                                          for groups with a direct-walk option
          sum_{(p,j,k) in A^r} rho_pjk                 <= c_route(V^r)  for r in the pool
          rho_pjk                                      <= 0        for triples touching J0
          alpha, gamma                                 >= 0

The face equality is what makes this safe to run at all: it pins `C(yhat) = z*`, so the Pareto
step can never weaken the cut at the incumbent -- only redistribute mass away from it. The
`alpha <= c_walk` family is the dual constraint of the `x_walk` columns and is easy to forget:
without it `alpha` can exceed what direct walking costs and the cut becomes invalid at `y`
where the routes it credits do not exist.

# The Pareto pricing loop

`ParetoRMP` above carries route rows only for `C+`. The loop closes the gap to `R+`:

    repeat
        solve ParetoRMP over the current row set
        read (alpha, gammaO, gammaD); form rho
        price the EXISTING exact pricer at these rho over R+
        if min reduced cost >= -reduced_cost_tol: CERTIFIED, stop
        else: each returned route is a VIOLATED DUAL ROW -- add it, repeat

The primal/dual correspondence is worth stating in one line because the code reads as if it
were doing column generation and is not: **in the primal restricted master a priced route is a
new COLUMN; in this auxiliary dual LP the same route is a new CONSTRAINT.** The Pareto problem
is solved by constraint generation, and its separation oracle is the route pricer.

Two consequences the implementation depends on:

- **Optimality of `ParetoRMP` over the current rows is NOT a stopping condition.** Only
  `min_r rc_r >= -tol` over `R+`, established by an EXHAUSTED pricing round, is. A round
  stopped by its budget proves nothing, and a point it did not certify is installed nowhere:
  the loop tracks the last certified point and initialises it to the closed-form completion
  the caller already applied, which is certified. Truncation therefore degrades to today's
  cut and **cannot** produce an invalid one.
- **The priced routes are NOT added to the primal pool.** They cannot improve `Q_s(yhat)`
  (the activated CG solve already certified none exists at the original duals, and these
  price negative only against the moved duals), and adding them would invalidate the primal
  solution and duals that `_solve_one_joint_routing_assignment_benders_subproblem` reads after
  this function returns.

**The `rho <= 0` rows keep the pricing rounds ACTIVATED.** This is the reason to expect this
oracle to behave differently from the row-generation completion removed on 2026-09-10, which
recovered the cut strength and then spent it: lowering `gamma` on the unbuilt stations
re-admitted them to the pricer's `rho > 0` filter, so every separation round became a
full-universe search (6.8 s of built-only pricing against 501 s of row generation at n=30
s=3). Here the completion rows are IN the Pareto LP, so every candidate the LP can propose
already has `rho <= 0` on every triple through an unbuilt station and
`joint_routing_assignment_pricing_candidates` drops it -- each Pareto round is a built-only
search, the same one the activated oracle pays for anyway.

# Why the completion is globally valid (the `rho <= 0` argument)

A cut is valid iff the point it is read off is feasible for the dual of the FULL-universe
second stage; optimality at `yhat` only makes it tight. The full dual's route family is
`R`, and the pricing loop only certifies `R+`. The gap is closed by the completion rows plus
one structural property of this route family, which is verified below rather than assumed.

**The assignment-removal property.** For every feasible column `r = (V, A)` and any subset
`A_in ⊆ A`, there is a feasible column `r'' = (V'', A_in)` in the searched family with
`c_route(V'') <= c_route(V)`. Take `V''` to be `V` with every station that carries no
retained assignment deleted. Then:

- `tau_{V''} <= tau_V` -- the travel matrix is required to be metric package-wide (the
  pricer's age pruning asserts on violation), so deleting a stop cannot lengthen the route.
  `c_route = beta * (tau + repositioning_time)` is increasing in `tau`, so the cost drops.
- `r''` is FEASIBLE. Both route-feasibility conditions bound ELAPSED durations from above --
  the pickup window is `label.time <= max_wait_time`, the ride limit is
  `origin_age + travel <= detour_factor * routing_cost(j,k)` -- and deleting a stop only
  decreases each left-hand side. Both right-hand sides are unchanged: `max_wait_time` is a
  constant, and `routing_cost(j,k)` is computed from the ASSIGNMENT's pickup/dropoff stations
  before any route exists (`pricing_round.jl`), not from route positions. Nothing measures
  wait against a fixed request clock, so a retained passenger cannot wait longer.
- `r''` is in the ACTIVATED searched family when `A_in` is the built-only part of `A`: its
  nodes all carry built assignments, and the activated pricer's candidate generation proposes
  nodes only from surviving candidates' origins and destinations
  (`exact/seed.jl`, `exact/extend.jl`), which are exactly the built ones once `rho <= 0`
  drops the rest.

**The implication.** Split `A^r = A_in ∪ A_out` by whether both endpoints are built. Then

    sum_{A_out} rho  <= 0                      (the completion rows, one triple at a time)
    sum_{A_in}  rho  <= c_route(V'')           (certified over R+ -- by the loop, or, for
                                                `:baseline`, by the activated CG solve that
                                                did not move these multipliers)
    c_route(V'')     <= c_route(V)             (assignment-removal, above)
    ==> sum_{A^r} rho <= c_route(V)            for every r in R, not just R+.

which is precisely the dual row of `r`. With `alpha <= c_walk` (the `x_walk` rows) and
`alpha, gamma >= 0` also imposed, the point is feasible for the full dual, so the cut is
globally valid; and the face equality makes it tight at `yhat`.

**Which walking cost `rho` uses, and why the rows use the SMALLER one.** The dual row's walk
term is `w * demand_p * walk` (it comes from `joint_routing_assignment_column_cost`), while
the pricer's candidate reward uses `w * walk` with no demand factor
(`joint_routing_assignment_pricing_candidates`). The completion rows here are written with the
demand-free term, i.e.

    alpha_p - gammaO_pj - gammaD_pk <= w * walk(o, d, (j,k)),

which is the STRONGER requirement whenever `demand_p > 1`. That is deliberate and serves both
jobs at once: it implies the dual condition the validity argument needs, AND it matches the
pricer's filter exactly, which is what keeps the search activated. Writing the weaker
demand-weighted row instead would still be sound but would let a `demand_p > 1` triple through
an unbuilt station survive `rho > 0` and re-open the full-universe search.

# What is NOT claimed

- Not Cummings' completion. Theirs reconstructs the omitted multipliers; ours bounds them.
- Not Pareto-optimal over the whole dual polyhedron of the full family `R`: the `rho <= 0`
  rows are a strict tightening, so the optimum here can be worse than a true Magnanti-Wong
  point that enforces the real `R` rows directly. `benders_lpo_gold_standard.jl` measures
  that shortfall on tiny instances; production never enumerates `R`.
- `:pareto` is Pareto-optimal in the Magnanti-Wong sense only if `y^c` is in the RELATIVE
  INTERIOR of the master's feasible hull. `_benders_core_point` reports the max-min slack it
  achieved; a slack of 0 means some face is structurally tight and the claim degrades to
  "non-dominated among cuts that agree on that face".
"""

using JuMP
using Printf

# ---------------------------------------------------------------------- tolerances
#
# Defined here, once, and used by every check below and by the diagnostics that report them.
# `_LPO_FEAS_TOL` is what an LP-sized quantity is compared against (the face residual, the
# `rho <= 0` rows read back off a solution, a variable sitting on its safety bound);
# the PRICING tolerance is deliberately NOT one of these -- it is
# `BendersSubproblemConfig.reduced_cost_tol`, the same number the ordinary CG loop certifies
# against, because "no improving column" has to mean the same thing in both places.
const _LPO_FEAS_TOL = 1e-6
# Safety box on the auxiliary LP's variables, as a multiple of the largest incoming `alpha`.
# It exists only so an unbounded ray reports as a binding bound instead of aborting the
# completion: validity never depends on it (a certified point is certified whatever bounded
# it), only strength does, and `bound_binding` in the status says when strength was affected.
const _LPO_BOUND_MULTIPLE = 100.0

"""
    _benders_core_point(data, mapping, k; mode=:relative_interior)
        -> (Vector{Float64}, Float64)

A core point `y^c` of the master's feasible region, and the minimum slack it achieves.

The master's region here is `sum_j y_j == k`, `0 <= y_j <= 1`, and
`add_aggregate_od_route_endpoint_feasibility_constraints!`'s `sum_{j near pt} y_j >= 1` for
every location a demand group cannot walk from.

# Why this needs more care than it looks like it does

The Pareto objective is `C(y^c) = sum_p alpha_p - sum_j y^c_j g_j`. A coordinate with
`y^c_j == 0` therefore contributes NOTHING to it, so the auxiliary LP is free to put anything
it likes on `g_j` -- and, if `j` is built at the incumbent, `g_j` acquires an unbounded
improving ray, since the optimal-face equality pays `+1` per unit of built-station mass while
the objective charges `-y^c_j = 0` for it. The objective stops being a norm on the cut and
becomes a seminorm that ignores whole coordinates.

**MEASURED, 2026-09-10, and this is not a hypothetical**: with `mode = :max_min_slack` at
n=10 k=5 s=3 the LP returned `y^c` with coordinates at 0 and 1 (`min slack 0.0000`,
`y^c in [0.000, 1.000]`). The completion then put a mean `g_j` of **1.79e6** on the unbuilt
stations against the closed form's 7.1e3, the mean one-swap cut gap went from 7.0e3 to
1.74e6, and the fraction of neighbours where the cut says nothing at all went from 5.6% to
49.2%. Every cut was still VALID and still tight at its anchor -- the damage is entirely to
strength, which is exactly what a degenerate core point is supposed to cost.

The cause is not that a face was tight; it is that `max t` STOPS CARING once `t` is capped.
One structurally tight face (a required location whose only candidates must all be built)
forces `t* = 0`, and at `t = 0` every `t <= y_j <= 1 - t` row is slack, so the LP is free to
return a vertex. The old `:max_min_slack` mode is kept for exactly this comparison.

# `:relative_interior` (the default) -- built to have no zero coordinate

A point in the relative interior has strict slack in every constraint that is not tight at
EVERY feasible point. That is constructed here rather than optimised for, by averaging
witnesses:

- for each station `j`, a feasible point maximising `y_j`;
- for each required location, a feasible point maximising that endpoint row's slack;
- the `:max_min_slack` point, which handles the `y_j <= 1` side.

The average of feasible points is feasible (the region is convex), and it has
`y^c_j >= (1/m) max y_j` and row slack `>= (1/m) max slack`. So every coordinate and every
row that CAN be strictly interior is, which is the definition. A coordinate whose maximum is
0 is fixed at 0 in every feasible point, so it lies in the affine hull and the cut's
coefficient there is never read at any `y` the master can propose -- junk on it is harmless,
which is why "no zero coordinate" is the right target rather than "no zero coordinate at
all costs".

Cost is `n + |required| + 1` small LPs, once per run.

# The other two modes

`:max_min_slack` maximises the minimum slack (`max t` subject to `t <= y_j <= 1 - t` and
`sum_{near} y >= 1 + t`). Fine when `t* > 0`; degenerate as described above when it is not.

`:uniform` is Phase 3's natural point `y^c_j = k/|J|`. It satisfies the budget row strictly
inside `0 <= y_j <= 1`, but it sits exactly ON an endpoint row whenever a required location
has fewer than `|J|/k` candidate stations, and VIOLATES one when it has fewer still -- so it
is not in general even feasible, let alone relatively interior. Kept because it is the clean
version of the brief's Phase 3 and the comparison is worth having; the returned slack is its
real one, and `NaN` marks it infeasible.
"""
function _benders_core_point(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        k::Int;
        mode::Symbol = :relative_interior,
    )::Tuple{Vector{Float64}, Float64}
    n = data.n_stations
    # The locations the endpoint-feasibility rows are written for: an endpoint of a demand
    # group with no direct-walk fallback must have a station within walking distance.
    required = Set{Int}()
    for s in 1:n_scenarios(data)
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            mapping.Q_s[s][p] > 0 || continue
            any(is_walk_only_pair, get_valid_jk_pairs(mapping, o, d)) && continue
            push!(required, o)
            push!(required, d)
        end
    end
    rows = [(pt, [j for j in 1:n
                  if get_walking_cost(data, pt, j) <= mapping.max_walking_distance])
            for pt in sort!(collect(required))]
    filter!(r -> !isempty(r[2]), rows)

    """The point's true min-slack over every master row, or `NaN` if it is infeasible."""
    function achieved_slack(y::Vector{Float64})
        abs(sum(y) - k) <= 1e-6 || return NaN
        slack = minimum(min(yj, 1.0 - yj) for yj in y; init = Inf)
        for (_pt, cands) in rows
            slack = min(slack, sum(y[j] for j in cands) - 1.0)
        end
        return slack < -1e-9 ? NaN : max(0.0, slack)
    end

    if mode === :uniform
        y = fill(k / n, n)
        return y, achieved_slack(y)
    end

    """The master's own feasible region as an LP, plus the max-min-slack variable `t`."""
    function base_model()
        m = Model(() -> Gurobi.Optimizer())
        set_silent(m)
        @variable(m, 0.0 <= y[1:n] <= 1.0)
        @variable(m, t >= 0.0)
        @constraint(m, sum(y) == k)
        for (_pt, cands) in rows
            @constraint(m, sum(y[j] for j in cands) >= 1 + t)
        end
        for j in 1:n
            @constraint(m, y[j] >= t)
            @constraint(m, y[j] <= 1 - t)
        end
        return m, y, t
    end

    # The max-min-slack point, which both modes want: it is `:max_min_slack`'s answer and one
    # of `:relative_interior`'s witnesses (the only one that pushes coordinates off `y_j = 1`).
    m, y, t = base_model()
    @objective(m, Max, t)
    optimize!(m)
    if termination_status(m) != MOI.OPTIMAL
        # Nothing better to offer; report the uniform point's REAL slack, which may be NaN
        # (infeasible) and is the caller's signal that the Pareto claim has no basis here.
        yu = fill(k / n, n)
        return yu, achieved_slack(yu)
    end
    mms = Float64.(value.(y))
    mode === :max_min_slack && return mms, achieved_slack(mms)
    mode === :relative_interior || throw(ArgumentError(
        "unsupported core-point mode $(repr(mode)); expected :relative_interior, " *
        ":max_min_slack or :uniform"))

    witnesses = Vector{Float64}[mms]
    # One witness per coordinate, and one per endpoint row. Each is solved on a FRESH model:
    # re-objectiving one model would be cheaper, but these are `n + |required|` LPs of a few
    # dozen variables solved once per run, and a fresh model keeps the `t` row from silently
    # restricting the witnesses (a witness must range over the whole region, not the
    # max-min-slack sub-region).
    for j in 1:n
        wm, wy, _wt = base_model()
        @constraint(wm, _wt == 0.0)
        @objective(wm, Max, wy[j])
        optimize!(wm)
        termination_status(wm) == MOI.OPTIMAL && push!(witnesses, Float64.(value.(wy)))
    end
    for (_pt, cands) in rows
        wm, wy, _wt = base_model()
        @constraint(wm, _wt == 0.0)
        @objective(wm, Max, sum(wy[j] for j in cands))
        optimize!(wm)
        termination_status(wm) == MOI.OPTIMAL && push!(witnesses, Float64.(value.(wy)))
    end
    point = sum(witnesses) ./ length(witnesses)
    return point, achieved_slack(point)
end

"""
    _benders_lpo_core_point(sm) -> Vector{Float64}

The core point stashed on a subproblem model at build time (`build_subproblem.jl`), or the
uniform fallback if the build did not stash one. Computed ONCE per run on the MASTER, because
it depends only on the master's own feasible region and not on any incumbent -- and because it
must be the SAME point for every scenario, or two scenarios would pick completions that are
Pareto-optimal against different objectives and their sum would be optimal against neither.
"""
function _benders_lpo_core_point(sm::JuMP.Model)::Vector{Float64}
    haskey(sm.obj_dict, :benders_core_point) && return sm[:benders_core_point]::Vector{Float64}
    n = length(sm[:y])
    return fill(1.0 / max(1, n), n)
end

"""
    BendersLPOResult

What one call to `_benders_lpo_completion!` established, for the run's metadata and for the
per-iteration log the experiment reads.

`certified` is the bit that matters: `true` means the installed dual point is feasible for
the FULL-universe dual -- for `:baseline` by the `rho <= 0` argument in this file's header,
for `:pareto` by that argument plus an EXHAUSTED pricing round at the installed duals. When
it is `false` the installed point is the caller's closed-form completion, unchanged, which is
itself certified; a truncated Pareto loop therefore degrades to the ordinary activated cut
and never produces an invalid one.

`core_value_before`/`core_value_after` are `C(y^c)` for the incoming and installed points --
the objective the Pareto step maximises, and the number the two methods are compared on.
`face_residual` is `C(yhat) - z*`, which must be 0 to LP tolerance or the cut is not tight at
its own anchor. `worst_rho` is the largest `alpha_p - gammaO - gammaD - w*walk` over triples
touching an unbuilt station, i.e. the worst violation of the completion condition. `min_rc`
is the minimum reduced cost over the route POOL at the installed point -- always computable,
unlike the pricing round's own minimum, which is `Inf` on the round that certifies.
"""
struct BendersLPOResult
    method::Symbol
    status::String
    certified::Bool
    rounds::Int
    seed_rows::Int
    generated_rows::Int
    n_free::Int
    core_value_before::Float64
    core_value_after::Float64
    face_residual::Float64
    worst_rho::Float64
    min_rc::Float64
    lp_sec::Float64
    pricing_sec::Float64
    # How many auxiliary-LP variables sat on the safety box at the installed point, and what
    # that box was. A nonzero count means the reported point is the best WITHIN the box rather
    # than the Pareto optimum -- still valid (the certificate does not know about the box), but
    # the strength claim is capped. One or two variables is a degenerate ray the objective is
    # flat along; thousands means the LP is genuinely unbounded over the current row set and
    # the loop is being carried by the box instead of by the rows.
    n_at_bound::Int
    bound::Float64
    trace::Vector{NamedTuple}
end

_lpo_null_result(method::Symbol, status::String, core::Float64) = BendersLPOResult(
    method, status, true, 0, 0, 0, 0, core, core, 0.0, -Inf, Inf, 0.0, 0.0, 0, NaN,
    NamedTuple[])

# One term of an LP expression whose coefficient may multiply either a free variable or a
# value that this method holds FIXED. Having both shapes go through the same builder is what
# lets `:baseline` and `:pareto` share every row-construction routine below instead of
# maintaining two copies that could drift apart -- which would make the comparison between
# them meaningless.
_lpo_term!(expr::AffExpr, coef::Float64, x::VariableRef) = add_to_expression!(expr, coef, x)
_lpo_term!(expr::AffExpr, coef::Float64, x::Float64) = add_to_expression!(expr, coef * x)

"""
    _benders_lpo_completion!(alpha, gamma_o, gamma_d, incumbent, data, mapping, scenario,
                             build, config, core_point, z_star) -> BendersLPOResult

Replace the dual point the cut will be read off with a locally Pareto-optimal one, in place.

Callers must have applied `_benders_activated_complete_duals!` first: its closed-form point is
this function's certified fallback, the reference `core_value_before` is measured against, and
(for `:baseline`) the fixed `alpha_hat`/`gamma_hat` the completion is computed around.

`z_star` is the activated subproblem's optimal VALUE `Q_s(yhat)`. It is the only thing carried
over from the original dual solution under `:pareto` -- everything else is re-optimised.
"""
function _benders_lpo_completion!(
        alpha::Dict{Tuple{Int, Int}, Float64},
        gamma_o::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
        gamma_d::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
        incumbent::Vector{Float64},
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        scenario::Int,
        build::BuildResult,
        config::BendersSubproblemConfig,
        core_point::Vector{Float64},
        z_star::Float64,
    )::BendersLPOResult
    sm = build.model
    method = config.lpo_completion
    built(j) = incumbent[j] > 0.5
    w = Float64(sm[:joint_routing_assignment_walk_cost_weight])

    # ---- the instance's own index sets, read once -------------------------------------
    # `groups` mirrors the coverage rows; `pairs` mirrors the linking rows. Both are built
    # from `mapping` rather than from the model's constraint dicts so that a triple with no
    # pooled column still gets its completion row -- the whole point is to bound multipliers
    # on stations no current column uses.
    group_keys = Tuple{Int, Int}[]
    group_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    walk_bound = Dict{Tuple{Int, Int}, Float64}()
    pickup_keys = Tuple{Tuple{Int, Int}, Int}[]
    dropoff_keys = Tuple{Tuple{Int, Int}, Int}[]
    for (p, (o, d)) in enumerate(mapping.Omega_s[scenario])
        demand = mapping.Q_s[scenario][p]
        demand > 0 || continue
        key2 = (scenario, p)
        push!(group_keys, key2)
        pairs = Tuple{Int, Int}[]
        seen_o, seen_d = Set{Int}(), Set{Int}()
        for pair in get_valid_jk_pairs(mapping, o, d)
            if is_walk_only_pair(pair)
                # The `x_walk` column's dual row: `alpha_p <= w * demand_p * walk(o,d)`.
                # Only groups with a direct-walk option have it; for the rest nothing here
                # bounds `alpha` from above except the route rows.
                walk_bound[key2] =
                    w * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
                continue
            end
            j, k = pair
            push!(pairs, (j, k))
            j in seen_o || (push!(seen_o, j); push!(pickup_keys, (key2, j)))
            k in seen_d || (push!(seen_d, k); push!(dropoff_keys, (key2, k)))
        end
        group_pairs[key2] = pairs
    end

    """`C(y) = sum_p alpha_p - sum_j g_j y_j` for a dual point given as three dicts."""
    function cut_value(a, go, gd, y::Vector{Float64})
        v = sum(values(a); init = 0.0)
        for (key, g) in go; v -= y[key[2]] * g; end
        for (key, g) in gd; v -= y[key[2]] * g; end
        return v
    end
    """The largest `alpha_p - gammaO_pj - gammaD_pk - w*walk` over triples touching `J0`."""
    function worst_rho(a, go, gd)
        worst = -Inf
        for key2 in group_keys
            o, d = mapping.Omega_s[scenario][key2[2]]
            for (j, k) in group_pairs[key2]
                (built(j) && built(k)) && continue
                rho = get(a, key2, 0.0) - get(go, (key2, j), 0.0) - get(gd, (key2, k), 0.0) -
                    w * od_pair_walking_cost(data, o, d, (j, k))
                worst = max(worst, rho)
            end
        end
        return worst
    end
    """Minimum reduced cost over the pooled columns at a dual point -- the finite part of
    the dual constraint set, checkable without pricing."""
    function pool_min_rc(a, go, gd)
        worst = Inf
        for (_id, column) in sm[:joint_routing_assignment_columns]
            Int(column.metadata["scenario"]) == scenario || continue
            rc = joint_routing_assignment_column_cost(sm, data, mapping, column)
            for (p, j, k) in column.assignments
                key2 = (scenario, p)
                rc -= get(a, key2, 0.0)
                rc += get(go, (key2, j), 0.0) + get(gd, (key2, k), 0.0)
            end
            worst = min(worst, rc)
        end
        return worst
    end

    alpha_hat = Dict(key => get(alpha, key, 0.0) for key in group_keys)
    go_hat = Dict(key => get(gamma_o, key, 0.0) for key in pickup_keys)
    gd_hat = Dict(key => get(gamma_d, key, 0.0) for key in dropoff_keys)
    core_before = cut_value(alpha_hat, go_hat, gd_hat, core_point)

    free_o = [key for key in pickup_keys if !built(key[2])]
    free_d = [key for key in dropoff_keys if !built(key[2])]
    if method === :baseline && isempty(free_o) && isempty(free_d)
        # Every station is built: nothing to complete, and the incoming point is already the
        # ordinary activated dual.
        return _lpo_null_result(method, "nothing_free", core_before)
    end

    # ---- the auxiliary LP -------------------------------------------------------------
    lp = Model(() -> Gurobi.Optimizer())
    set_silent(lp)
    # Safety box. See `_LPO_BOUND_MULTIPLE`: it turns an unbounded ray into a reported
    # `bound_binding` rather than an aborted completion, and affects strength only.
    box = isnothing(config.lpo_variable_bound) ?
        _LPO_BOUND_MULTIPLE * max(1.0, maximum(values(alpha_hat); init = 1.0)) :
        config.lpo_variable_bound

    # `:baseline` FIXES alpha and the built-station gamma (Method A: the ordinary activated
    # dual, completed); `:pareto` frees all three families and pins only the VALUE (Method B).
    # Both share every row builder below, so the two methods differ in exactly this dict and
    # in the face row -- which is what makes the comparison isolate the Pareto step.
    pareto = method === :pareto
    A = Dict{Tuple{Int, Int}, Union{VariableRef, Float64}}()
    GO = Dict{Tuple{Tuple{Int, Int}, Int}, Union{VariableRef, Float64}}()
    GD = Dict{Tuple{Tuple{Int, Int}, Int}, Union{VariableRef, Float64}}()
    for key in group_keys
        A[key] = pareto ? @variable(lp, lower_bound = 0.0, upper_bound = box,
                                    base_name = "alpha[$(key[2])]") : alpha_hat[key]
    end
    for key in pickup_keys
        GO[key] = (pareto || !built(key[2])) ?
            @variable(lp, lower_bound = 0.0, upper_bound = box,
                      base_name = "gO[$(key[1][2]),$(key[2])]") : go_hat[key]
    end
    for key in dropoff_keys
        GD[key] = (pareto || !built(key[2])) ?
            @variable(lp, lower_bound = 0.0, upper_bound = box,
                      base_name = "gD[$(key[1][2]),$(key[2])]") : gd_hat[key]
    end
    n_free = count(v -> v isa VariableRef, values(A)) +
             count(v -> v isa VariableRef, values(GO)) +
             count(v -> v isa VariableRef, values(GD))

    # Objective: the cut evaluated at the core point, `C(y^c) = sum alpha - sum_j y^c_j g_j`.
    # Under `:baseline` the alpha terms and the built-station g terms are constants, so
    # maximising this is exactly minimising `sum_{j in J0} y^c_j g_j` -- Phase 2's objective,
    # written in the same expression as Phase 4's so the two are directly comparable.
    obj = AffExpr(0.0)
    for key in group_keys; _lpo_term!(obj, 1.0, A[key]); end
    for key in pickup_keys; _lpo_term!(obj, -core_point[key[2]], GO[key]); end
    for key in dropoff_keys; _lpo_term!(obj, -core_point[key[2]], GD[key]); end
    @objective(lp, Max, obj)

    if pareto
        # The optimal-face equality: `C(yhat) == z*`. This is what makes re-optimising the
        # WHOLE dual safe -- the Pareto step can redistribute mass away from the incumbent but
        # can never lower the cut at it, so the new cut is still tight at its anchor and the
        # loop's upper bound is unaffected. Written over the built stations only, which is the
        # binary-`yhat` form of `sum_p alpha_p - sum_j yhat_j g_j`.
        face = AffExpr(0.0)
        for key in group_keys; _lpo_term!(face, 1.0, A[key]); end
        for key in pickup_keys; built(key[2]) && _lpo_term!(face, -1.0, GO[key]); end
        for key in dropoff_keys; built(key[2]) && _lpo_term!(face, -1.0, GD[key]); end
        @constraint(lp, face == z_star)
        # The `x_walk` columns' dual rows. Easy to omit and fatal if omitted: without them
        # `alpha_p` may exceed what direct walking costs, and the cut then over-estimates
        # `Q_s` at any `y` whose routes do not serve `p`.
        for (key, bound) in walk_bound
            @constraint(lp, A[key] <= bound)
        end
    end

    # The completion rows: `rho_pjk <= 0` for every valid triple touching an unbuilt station,
    # written with the DEMAND-FREE walking term. See the header for why that stronger form is
    # the right one -- it implies the dual condition validity needs AND matches the pricer's
    # `rho > 0` filter, which is what keeps every pricing round below activated.
    n_completion_rows = 0
    for key2 in group_keys
        o, d = mapping.Omega_s[scenario][key2[2]]
        for (j, k) in group_pairs[key2]
            (built(j) && built(k)) && continue
            row = AffExpr(0.0)
            _lpo_term!(row, 1.0, A[key2])
            _lpo_term!(row, -1.0, GO[(key2, j)])
            _lpo_term!(row, -1.0, GD[(key2, k)])
            @constraint(lp, row <= w * od_pair_walking_cost(data, o, d, (j, k)))
            n_completion_rows += 1
        end
    end

    # ---- route rows -------------------------------------------------------------------
    # `sum_{(p,j,k) in A^r} rho_pjk <= c_route(V^r)`, written in the equivalent form the rest
    # of the package uses: `sum (alpha - gammaO - gammaD) <= f_r`, with `f_r` the column's
    # full objective coefficient (route cost AND demand-weighted walking).
    row_signatures = Set{Any}()
    function add_route_row!(column)::Bool
        Int(column.metadata["scenario"]) == scenario || return false
        signature = (_joint_routing_assignment_column_signature(column.assignments),
                     round(column.tau; digits = 9))
        signature in row_signatures && return false
        push!(row_signatures, signature)
        row = AffExpr(0.0)
        for (p, j, k) in column.assignments
            key2 = (scenario, p)
            haskey(A, key2) && _lpo_term!(row, 1.0, A[key2])
            haskey(GO, (key2, j)) && _lpo_term!(row, -1.0, GO[(key2, j)])
            haskey(GD, (key2, k)) && _lpo_term!(row, -1.0, GD[(key2, k)])
        end
        # A row with no free variable is a fact about constants; it is either already true
        # (it was, at the certified incoming point) or a bug, and JuMP cannot take it as a
        # constraint. Nothing to add either way.
        isempty(row.terms) && return false
        @constraint(lp, row <= joint_routing_assignment_column_cost(sm, data, mapping, column))
        return true
    end

    seed_rows = 0
    if pareto
        # Seed from `C+`, the pool the activated CG solve left behind. These rows are free --
        # no search -- and they are the ones most likely to bind, since they are exactly the
        # columns that were attractive at the original duals.
        for (_id, column) in sm[:joint_routing_assignment_columns]
            add_route_row!(column) && (seed_rows += 1)
        end
    end

    # ---- solve, verify, install --------------------------------------------------------
    val(x::VariableRef) = max(0.0, value(x))
    val(x::Float64) = x
    read_point() = (Dict(key => val(A[key]) for key in group_keys),
                    Dict(key => val(GO[key]) for key in pickup_keys),
                    Dict(key => val(GD[key]) for key in dropoff_keys))
    function install!(a, go, gd)
        for (key, v) in a; alpha[key] = v; end
        for (key, v) in go; gamma_o[key] = v; end
        for (key, v) in gd; gamma_d[key] = v; end
        return nothing
    end
    """Everything a candidate must satisfy before it may be installed, independent of how it
    was produced: tight at the anchor, and `rho <= 0` where the validity argument needs it.
    Checked on the numbers read back from the solver rather than assumed from the rows, so an
    LP that solved to a loose tolerance is caught here instead of in the cut."""
    function admissible(a, go, gd)
        residual = cut_value(a, go, gd, incumbent) - z_star
        rho = worst_rho(a, go, gd)
        scale = max(1.0, abs(z_star))
        return (abs(residual) <= _LPO_FEAS_TOL * scale && rho <= _LPO_FEAS_TOL,
                residual, rho)
    end

    trace = NamedTuple[]
    at_bound = 0
    lp_sec = 0.0
    pricing_sec = 0.0
    generated_rows = 0
    # The point that will be installed if nothing better is certified. The caller's
    # closed-form completion IS certified (`_benders_activated_complete_duals!`), so
    # truncation at any point below degrades to the ordinary activated cut.
    best = (alpha_hat, go_hat, gd_hat)
    status = "fallback"
    certified = true
    rounds = 0

    if !pareto
        # ---- Method A: one LP, no pricing. -------------------------------------------
        # Feasibility over the full route family follows from `rho <= 0` plus the activated
        # certificate at these SAME `alpha_hat`/`gamma_hat` -- see the header. The LP moves
        # only multipliers on unbuilt stations, which appear in no row the certificate used.
        t = time(); optimize!(lp); lp_sec += time() - t
        rounds = 1
        if termination_status(lp) != MOI.OPTIMAL
            status = "lp_$(termination_status(lp))"
        else
            a, go, gd = read_point()
            ok, residual, rho = admissible(a, go, gd)
            if !ok
                status = "rejected_inadmissible"
            else
                achieved = cut_value(a, go, gd, core_point)
                # The closed-form point satisfies every row of this LP (it is how those rows
                # are satisfiable at all), so the optimum can only match or beat it. A failure
                # here means the row set or the objective is wrong, not that the LP is weak.
                achieved >= core_before - _LPO_FEAS_TOL * max(1.0, abs(core_before)) || error(
                    "the baseline completion LP came out WORSE at the core point than the " *
                    "closed-form bound it relaxes ($(achieved) vs $(core_before)); the " *
                    "closed-form point is feasible for these rows, so this means the row " *
                    "set or the objective is wrong",
                )
                best = (a, go, gd)
                status = "certified_by_construction"
            end
        end
    else
        # ---- Method B: the Pareto pricing loop. ---------------------------------------
        settings = _benders_subproblem_cg_settings(config)
        pricing_formulation = sm[:joint_routing_assignment_pricing_formulation]
        # The certifying pricers prove "nothing improving remains" by exhausting a RELAXATION
        # rather than the route universe, and `_run_pricing_round` throws on them
        # (`pricing_round.jl`), so separation cannot be one code path. Either proof answers
        # separation's question: `certified` means no real route is over-credited, and a
        # `:column_found` attempt hands back the real columns its exhaustive subset searches
        # found -- which are exactly the violated rows this loop wants.
        certifying = sm[:joint_routing_assignment_pricing_mode]::Symbol in
            (:relaxed_cluster, :relaxed_cluster_two_tier)
        certified = false
        status = "round_limit"

        for round in 1:config.lpo_max_rounds
            rounds = round
            t = time(); optimize!(lp); lp_sec += time() - t
            lp_status = termination_status(lp)
            if lp_status != MOI.OPTIMAL
                # DUAL_INFEASIBLE cannot happen while the box is finite; INFEASIBLE would mean
                # the face equality and the completion rows are incompatible, which is the
                # failure mode worth naming loudly -- it would say the conservative completion
                # cannot represent `z*` at all.
                status = "lp_$(lp_status)"
                break
            end
            a, go, gd = read_point()
            pareto_obj = objective_value(lp)
            n_bound = count(v -> v isa VariableRef && value(v) >= box - _LPO_FEAS_TOL,
                            Iterators.flatten((values(A), values(GO), values(GD))))
            bound_hit = n_bound > 0

            # Install the candidate so the pricer reads it, then separate. The completion rows
            # keep every unbuilt-station triple at `rho <= 0`, so this search is ACTIVATED --
            # the same built-only search the oracle pays for anyway, not the full-universe
            # grind that sank the 2026-09-10 row-generation completion.
            install!(a, go, gd)
            t = time()
            exhausted = false
            columns = if certifying
                cert = cg_certification_round(
                    build, mapping, sm, (alpha, gamma_o, gamma_d), settings;
                    time_limit_sec = config.lpo_pricing_time_limit_sec,
                    iteration = round, only_scenarios = [scenario],
                )
                exhausted = cert.certified
                cert.certified ? Any[] : _cg_materialize_certification_columns(
                    build, mapping, sm, (alpha, gamma_o, gamma_d), cert.candidates,
                )
            else
                found = _run_pricing_round(
                    pricing_formulation, mapping, sm, (alpha, gamma_o, gamma_d), settings;
                    only_scenarios = [scenario],
                    time_limit = config.lpo_pricing_time_limit_sec,
                )
                # Empty alone proves nothing -- a budget-stopped search also returns nothing.
                # Only empty AND exhausted certifies.
                exhausted = isempty(found) && _cg_pricing_exhausted(sm)
                found
            end
            pricing_sec += time() - t
            priced_min_rc = isempty(columns) ? Inf :
                minimum(Float64(get(c.metadata, "reduced_cost", NaN)) for c in columns)

            stats = _lpo_station_stats(a, go, gd, incumbent)
            push!(trace, (pareto_iter = round, pareto_obj = pareto_obj,
                          incumbent_face_residual = cut_value(a, go, gd, incumbent) - z_star,
                          num_route_constraints = seed_rows + generated_rows,
                          pricing_min_rc = priced_min_rc, new_columns = length(columns),
                          mean_active_g = stats.mean_active, mean_inactive_g = stats.mean_inactive,
                          max_inactive_g = stats.max_inactive, bound_binding = bound_hit,
                          n_at_bound = n_bound, lp_sec = lp_sec, pricing_sec = pricing_sec))

            if exhausted && isempty(columns)
                ok, _residual, _rho = admissible(a, go, gd)
                if ok
                    best = (a, go, gd)
                    certified = true
                    at_bound = n_bound
                    status = bound_hit ? "certified_bound_binding" : "certified"
                else
                    status = "rejected_inadmissible"
                end
                break
            end

            added = 0
            for column in columns
                add_route_row!(column) && (added += 1)
            end
            generated_rows += added
            if added == 0
                # Either the search was truncated (proving nothing) or every violated row is
                # already present, which would mean the LP is not respecting a row it has.
                status = isempty(columns) ? "pricing_inconclusive" : "no_actionable_row"
                break
            end
        end
    end

    # Whatever is installed must be a point some argument certified; `best` only ever holds
    # one. Re-installing unconditionally matters: the loop above installs UNCERTIFIED
    # candidates into the caller's dicts so the pricer can read them, so leaving without this
    # would hand the cut builder the last rejected candidate.
    install!(best...)
    a, go, gd = best
    _ok, residual, rho = admissible(a, go, gd)
    return BendersLPOResult(
        method, status, certified, rounds, seed_rows, generated_rows, n_free,
        core_before, cut_value(a, go, gd, core_point), residual, rho,
        pool_min_rc(a, go, gd), lp_sec, pricing_sec, at_bound, box, trace,
    )
end

"""
    _lpo_station_stats(alpha, gamma_o, gamma_d, incumbent) -> NamedTuple

Per-station cut coefficients `g_j` split by whether `j` is built at the incumbent. The
inactive statistics are the hypothesis under test: an arbitrary dual can put large
coefficients on stations the incumbent does not build, and `C(y') = z* + g_a - g_b` for a
one-station swap `a -> b` says a big `g_b` is exactly what makes the cut uninformative there.
"""
function _lpo_station_stats(alpha, gamma_o, gamma_d, incumbent::Vector{Float64})
    g = Dict{Int, Float64}()
    for gammas in (gamma_o, gamma_d)
        for (key, v) in gammas
            v == 0.0 && continue
            g[key[2]] = get(g, key[2], 0.0) + v
        end
    end
    active = [get(g, j, 0.0) for j in eachindex(incumbent) if incumbent[j] > 0.5]
    inactive = [get(g, j, 0.0) for j in eachindex(incumbent) if incumbent[j] <= 0.5]
    mean_or(v) = isempty(v) ? 0.0 : sum(v) / length(v)
    median_or(v) = isempty(v) ? 0.0 : sort(v)[max(1, (length(v) + 1) ÷ 2)]
    return (mean_active = mean_or(active), mean_inactive = mean_or(inactive),
            median_inactive = median_or(inactive),
            max_inactive = isempty(inactive) ? 0.0 : maximum(inactive),
            max_active = isempty(active) ? 0.0 : maximum(active), g = g)
end

"""
    _accumulate_benders_lpo_stats!(m, results)

Accumulate the run's locally Pareto-optimal completion totals onto the MASTER model, under
`:benders_lpo_stats`, for `_benders_build_metadata` to report. A no-op unless the subproblem
results actually carry LPO outcomes, so a non-LPO run reports nothing rather than zeros.

`certified` against `calls` is the ratio to read first: an uncertified call installed the
closed-form completion instead, so a run with a low ratio is mostly the plain activated
oracle wearing a different name. `core_gain` is the sum of `C(y^c)` improvements over that
closed-form point -- the quantity the Pareto step exists to buy.
"""
function _accumulate_benders_lpo_stats!(m::JuMP.Model, results)
    any(r -> !isnothing(r.lpo), results) || return nothing
    stats = get!(m.obj_dict, :benders_lpo_stats) do
        Dict{String, Any}("calls" => 0, "certified" => 0, "rounds" => 0,
                          "seed_rows" => 0, "generated_rows" => 0,
                          "lp_sec" => 0.0, "pricing_sec" => 0.0, "core_gain" => 0.0,
                          "worst_face_residual" => 0.0, "worst_rho" => -Inf,
                          "min_pool_rc" => Inf, "at_bound_calls" => 0,
                          "max_at_bound" => 0, "statuses" => Dict{String, Int}())
    end
    for r in results
        isnothing(r.lpo) && continue
        lpo = r.lpo
        stats["calls"] += 1
        stats["certified"] += lpo.certified ? 1 : 0
        stats["rounds"] += lpo.rounds
        stats["seed_rows"] += lpo.seed_rows
        stats["generated_rows"] += lpo.generated_rows
        stats["lp_sec"] += lpo.lp_sec
        stats["pricing_sec"] += lpo.pricing_sec
        stats["core_gain"] += lpo.core_value_after - lpo.core_value_before
        stats["worst_face_residual"] =
            max(stats["worst_face_residual"], abs(lpo.face_residual))
        stats["worst_rho"] = max(stats["worst_rho"], lpo.worst_rho)
        stats["min_pool_rc"] = min(stats["min_pool_rc"], lpo.min_rc)
        stats["at_bound_calls"] += lpo.n_at_bound > 0 ? 1 : 0
        stats["max_at_bound"] = max(stats["max_at_bound"], lpo.n_at_bound)
        counts = stats["statuses"]::Dict{String, Int}
        counts[lpo.status] = get(counts, lpo.status, 0) + 1
    end
    return nothing
end

"""
    _benders_lpo_log(scenario, result)

One line per completion, plus one per Pareto round when there were any. The per-round fields
are the ones the experiment's hypothesis is stated in: the core-point objective it is
climbing, the face residual that must stay at zero for the cut to remain tight at its anchor,
how many route rows the loop has had to generate, and the inactive-station coefficients that
are supposed to be shrinking.
"""
function _benders_lpo_log(scenario::Int, result::BendersLPOResult)
    @printf("      [lpo s=%d] %s %s | rounds %d | rows %d+%d | free %d | C(y^c) %.4f -> %.4f (%+.4f) | face %+.2e | max rho %+.2e | pool min rc %+.2e | lp %.2fs price %.2fs\n",
            scenario, result.method, result.status, result.rounds, result.seed_rows,
            result.generated_rows, result.n_free, result.core_value_before,
            result.core_value_after, result.core_value_after - result.core_value_before,
            result.face_residual, result.worst_rho, result.min_rc,
            result.lp_sec, result.pricing_sec)
    result.n_at_bound == 0 || @printf(
        "        NOTE: %d variables sit on the safety box (%.4g) -- the point is the best WITHIN the box, not the Pareto optimum (still valid: the certificate does not know about the box)\n",
        result.n_at_bound, result.bound)
    for t in result.trace
        @printf("        iter %d | obj %.4f | face %+.2e | rows %d | min rc %s | new cols %d | g: act %.2f inact %.2f max %.2f%s\n",
                t.pareto_iter, t.pareto_obj, t.incumbent_face_residual,
                t.num_route_constraints,
                isfinite(t.pricing_min_rc) ? @sprintf("%+.3e", t.pricing_min_rc) : "none",
                t.new_columns, t.mean_active_g, t.mean_inactive_g, t.max_inactive_g,
                t.bound_binding ? " [$(t.n_at_bound) AT BOUND]" : "")
    end
    flush(stdout)
    return nothing
end
