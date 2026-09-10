"""
Solving the joint routing+assignment Benders subproblems at a master incumbent, and
turning their LP duals into cut data.

# The cut this derives

At a fixed `yhat`, scenario `s`'s subproblem is

    min  sum_p c_walk[s,p] x_walk[s,p] + sum_c f_c theta_c
    s.t. (alpha[s,p] >= 0)      x_walk[s,p] + sum_{c covers (s,p)} theta_c >= 1
         (gammaO[(s,p),j] >= 0) sum_{c: (s,p) picked up at j} theta_c <= yhat_j
         (gammaD[(s,p),k] >= 0) sum_{c: (s,p) dropped off at k} theta_c <= yhat_k
         theta, x_walk >= 0

with `alpha`/`gamma` exactly as `extract_joint_routing_assignment_duals` defines them
(`gamma` being the NEGATED duals of the `<=` rows, so both families are non-negative).
Its dual objective is

    sum_p alpha[s,p] - sum_j Gamma[s,j] yhat_j,     Gamma[s,j] = sum_p (gammaO[(s,p),j] + gammaD[(s,p),j])

and -- the whole point -- the dual's feasible region does not mention `yhat` at all. So
the same `(alpha, gamma)` is dual-feasible for **every** `y`, and

    Theta_s >= sum_p alpha[s,p] - sum_j Gamma[s,j] y_j

is a valid underestimator of scenario `s`'s second-stage cost everywhere, tight at the
`yhat` it was derived at. `Gamma[s,j]` is the same per-station dual aggregate the
station-fixing work calls `Gamma_j`.

# WHY THE CUT IS VALID -- the static argument, in full

Validity does not rest on any measurement. It rests on one structural property of the model
plus weak duality, and the property is checkable by reading the build.

Write the scenario-s second stage at a fixed `yhat` as `Q_s(yhat)` (the LP above). Its dual is

    Q_s(yhat) = max  sum_p alpha_p - sum_j Gamma_j yhat_j
                s.t. alpha_p <= c_walk[s,p]                                  for each p
                     sum_{p in c} alpha_p - sum_{(p,j) in c} gammaO_{pj}
                                          - sum_{(p,k) in c} gammaD_{pk} <= f_c   for each column c
                     alpha, gamma >= 0

**`yhat` appears in the dual OBJECTIVE and nowhere in the dual CONSTRAINTS.** Call the dual
feasible set `D_s`; it is determined by the column pool, the cost coefficients and the
incidence structure, and is the same set for every `y`. Therefore, for any
`(alpha, gamma)` in `D_s` and any `y` whatsoever,

    sum_p alpha_p - sum_j Gamma_j y_j  <=  Q_s(y)

because the left-hand side is the value of one particular *feasible* dual solution at `y`,
while `Q_s(y)` is the *maximum* over `D_s`. That single inequality is the whole validity
proof, and it holds globally -- not just near `yhat`.

Two further facts pin down what the cut actually is:

- **Tightness at the anchor.** At the `yhat` we derive from, `Q_s(yhat)` is finite (feasible
  by the master's endpoint rows, bounded below since all costs are non-negative), so strong
  duality gives an optimal `(alpha*, gamma*)` in `D_s` with
  `sum alpha* - sum Gamma* yhat = Q_s(yhat)`. The cut passes exactly through the value
  function at its anchor.
- **It is the strongest linear cut available at that anchor.** `Q_s` is a maximum of finitely
  many linear functions of `y` (one per vertex of `D_s`), hence convex and piecewise linear;
  the cut is the supporting hyperplane at `yhat` given by the optimal dual vertex, i.e. a
  subgradient cut. No valid linear inequality anchored at `yhat` dominates it.

## The four preconditions -- all statically checkable, one of them a future hazard

1. **`y` occurs only as the right-hand side of the linking rows, and with zero objective
   coefficient.** This is what puts `yhat` in the dual objective rather than the dual
   constraints, and it is the load-bearing assumption. Verifiable by reading
   `add_joint_routing_assignment_station_linking_constraints!` (writes `-y[j] <= 0`, theta
   coefficients patched in later) together with `set_joint_routing_assignment_objective!`
   (touches only `x_walk`) and `add_joint_routing_assignment_column!` (`set_objective_coefficient`
   on theta only). If `y` ever gained an objective term or appeared in a constraint
   coefficient, `D_s` would move with `y` and every cut here would become unsound.
2. **The subproblem is solved to optimality.** Only then is the extracted `(alpha, gamma)`
   actually in `D_s`. A truncated solve can return a vector outside it, and the "cut" built
   from that bounds nothing -- which is why `_solve_one_joint_routing_assignment_benders_subproblem`
   raises instead of proceeding.
3. **The dual signs match the convention.** `alpha = dual(>= row)`, `gamma = -dual(<= row)`.
   Checked numerically on EVERY solve by the strong-duality assertion below, which is exactly
   the identity from the tightness argument.
4. **The column pool must be FIXED for the whole run.** `D_s` depends on the pool: adding a
   column adds a dual constraint, which can only SHRINK `D_s` and therefore lower `Q_s`. A cut
   derived against a smaller pool bounds the LARGER `Q_s` of that pool, so once the pool grows
   the old cut may over-estimate the new `Q_s` -- i.e. become invalid, and prune the true
   optimum. Under `:direct_enumeration` the pool is enumerated once at build time and never
   changes, so this is free. **It is precisely what breaks under a `:column_generation`
   oracle**: cuts derived before a column is priced in are not valid afterwards. That oracle
   needs either a from-scratch cut rebuild after each pool change, a pool complete for the
   second stage before any cut is taken, or Lagrangian/optimality-cut machinery that accounts
   for it. Naming it here so it is not discovered by a wrong answer later.

# Two things that make this simpler than textbook Benders

**No feasibility cuts** -- but NOT because of `x_walk`, and the distinction matters.
`x_walk` exists only for groups whose `walking_cost(o, d) <= 2 * max_walking_distance`
(`compute_valid_jk_pairs`); a group beyond that has a coverage row `sum(theta) >= 1` with
no walk term at all, so it needs a route column whose two stations are BUILT. "Walk
everybody" is therefore not universally available and cannot be the guarantee.

What actually guarantees feasibility at every incumbent is the MASTER's
`add_aggregate_od_route_endpoint_feasibility_constraints!` rows
(`constraints/endpoint_feasibility.jl`, the same rows the CG master carries): for every
location required by a group with no walk fallback, some station within
`max_walking_distance` of it must be built. A `y` that fails this cannot be an incumbent,
because the master will not produce it.

**That condition is necessary, not sufficient**, so this is an empirical guarantee rather
than a theorem: the rows place a station near each endpoint, but do not by themselves
promise a physically feasible route column linking a built `(j, k)` pair for a given group.
MEASURED at n=10 seed 42, exhaustively over the master's whole feasible set: 0 of 86
endpoint-feasible station sets have an infeasible subproblem, at s=1 and s=3 alike
(`benchmarks/diagnostics/benders_brute_force_certificate.jl`, which checks exactly this).
Outside the master's feasible set infeasibility is easy to hit -- 166 of the 252 station
sets at that instance violate the endpoint rows -- which is why that script filters to the
master's domain and why `Q(y)` is only defined there.

`solve_subproblem` therefore keeps its `error()` on a non-optimal subproblem as a live
guard, not a can't-happen branch: an instance where some master-feasible `y` leaves a group
unserviceable would raise mid-loop rather than silently deriving a cut from a meaningless
dual. Adding genuine feasibility cuts (or strengthening the master with per-GROUP pickup/
dropoff rows, which are tighter than the per-location consolidation) is the fix if that ever
fires.

**No bound duals in the cut.** Neither `theta` nor `x_walk` carries an upper bound in the
relaxed build -- both are created with `lower_bound = 0.0` only, the coverage rows being
what hold them near 1 -- so the dual has no bound-multiplier term and the cut is exactly
the two-family expression above. If anyone ever adds `theta <= 1`, this derivation needs a
third term and silently under-cuts without it.

# The strong-duality check

Every solve verifies `sum(alpha) - sum(Gamma .* yhat) == objective` before the cut is
built. This is not a formality: it is one line that catches a wrong dual sign, a linking
row family missed in the aggregation, a `theta` bound sneaking in, or a subproblem whose
`mapping` disagrees with the master's -- each of which otherwise produces a run that
converges to a confidently wrong number with nothing anywhere raising.
"""

export JointRoutingAssignmentBendersScenarioResult
export JointRoutingAssignmentBendersSubproblemResult

"""
    JointRoutingAssignmentBendersScenarioResult

One scenario's second-stage evaluation: its exact cost at the incumbent, and the cut
derived from it (`cut_constant` = `sum_p alpha`, `y_coefficients[j]` = `Gamma[s,j]`,
sparse -- a station in no linking row of this scenario is simply absent).

`cg` is the inner CG outcome under the `:column_generation` oracle and `nothing` under
`:direct_enumeration`. It is kept rather than discarded because its `converged` bit is what
licensed the cut in the first place, and its iteration/column counts are the only way to see
whether pool accumulation across Benders iterations is doing its job.

`reduced_cost_mismatch` is a diagnostic, not a check: with `y` fixed, JuMP's
`reduced_cost(y[j])` is a second, independent route to `-Gamma[s,j]`, and this records the
largest disagreement between the two. It is reported rather than enforced because the
strong-duality identity (see this file's docstring) is the assertion that actually
establishes cut validity, and the sign convention of a fixed variable's reduced cost is a
solver-interface detail rather than a property of the decomposition.
"""
struct JointRoutingAssignmentBendersScenarioResult
    scenario::Int
    objective::Float64
    cut_constant::Float64
    y_coefficients::Dict{Int, Float64}
    reduced_cost_mismatch::Float64
    cg::Union{Nothing, BendersSubproblemCGResult}
    lpo::Any
end

"""
    JointRoutingAssignmentBendersSubproblemResult

Every scenario's evaluation at one incumbent. `total_objective` is the incumbent's exact
total second-stage cost, which -- the first stage being costless here -- is the whole
objective, i.e. the run's upper bound (`benders_upper_bound` reads this field).
"""
struct JointRoutingAssignmentBendersSubproblemResult
    scenarios::Vector{JointRoutingAssignmentBendersScenarioResult}
    total_objective::Float64
    solve_sec::Float64
end

"""
    _solve_joint_routing_assignment_benders_subproblems(m, incumbent, solver)
        -> JointRoutingAssignmentBendersSubproblemResult

Pin `y` to `incumbent` in every stashed per-scenario subproblem, solve each as an LP, and
read the cut data off the duals.

The models are the ones `build_master.jl` built once; only `JuMP.fix` values change
between iterations, so each solve warm-starts from the previous basis.
"""
function _solve_joint_routing_assignment_benders_subproblems(
        m::JuMP.Model,
        incumbent::Vector{Float64},
        solver::BendersSolver,
    )::JointRoutingAssignmentBendersSubproblemResult
    builds = m[:benders_subproblem_builds]::Vector{BuildResult}
    t0 = time()
    # Results are written BY INDEX, never pushed: the cut builder pairs `results[i]` with
    # `builds[i]`'s scenario, so completion order must not decide the ordering.
    results = Vector{JointRoutingAssignmentBendersScenarioResult}(undef, length(builds))

    # The scenarios are genuinely independent at a fixed `incumbent`: each holds its own
    # JuMP model with its own `Gurobi.Optimizer()` (hence its own environment), each fixes
    # `y` and grows the pool only in ITS model, and none of them touches the master `m` --
    # the stats accumulation that does is called after the loop, single-threaded. So this is
    # a wall-clock win with no shared state, and subproblems are ~99% of the loop's time.
    #
    # On by default: the pricer has no internal threading to collide with. Every
    # `Threads.@threads` in the package is a loop over scenarios guarded by
    # `length(scenarios) > 1`, and a subproblem model holds one scenario, so those loops are
    # inert here. See `BendersSolver.parallel_scenarios`.
    parallel = solver.parallel_scenarios && length(builds) > 1 && Threads.nthreads() > 1
    if parallel
        Threads.@threads for i in eachindex(builds)
            results[i] = _solve_one_joint_routing_assignment_benders_subproblem(
                builds[i], incumbent, solver,
            )
        end
    else
        for i in eachindex(builds)
            results[i] = _solve_one_joint_routing_assignment_benders_subproblem(
                builds[i], incumbent, solver,
            )
        end
    end

    total = sum(r.objective for r in results; init = 0.0)
    _accumulate_benders_cg_stats!(m, results)
    return JointRoutingAssignmentBendersSubproblemResult(collect(results), total,
                                                         time() - t0)
end

function _solve_one_joint_routing_assignment_benders_subproblem(
        build::BuildResult,
        incumbent::Vector{Float64},
        solver::BendersSolver,
    )::JointRoutingAssignmentBendersScenarioResult
    sm = build.model
    mapping = build.mapping
    scenario = Int(sm[:benders_subproblem_scenario])
    y = sm[:y]
    # `force=true` is required, not optional: `add_station_selection_variables!` gives the
    # relaxed `y` explicit [0,1] bounds, and `fix` refuses to override existing bounds
    # without it.
    for j in eachindex(y)
        JuMP.fix(y[j], incumbent[j]; force = true)
    end
    isnothing(solver.subproblem.time_limit_sec) ||
        set_time_limit_sec(sm, solver.subproblem.time_limit_sec)

    # Oracle split. `:direct_enumeration` solves the LP once over the complete pool.
    # `:column_generation` runs CG to EXHAUSTION first -- and its convergence is a hard
    # precondition for taking a cut, not a quality preference: duals from a non-exhausted
    # CG are feasible only for the restricted dual, so the cut can over-estimate the true
    # second-stage cost and prune the optimum (see subproblem_cg.jl).
    cg_result = nothing
    if solver.subproblem.oracle in (:column_generation, :column_generation_activated,
                                    :column_generation_activated_lpo,
                                    :column_generation_warm_start)
        cg_result = _solve_joint_routing_assignment_subproblem_by_cg!(
            build, solver.subproblem, incumbent,
        )
        cg_result.converged || error(
            "Benders subproblem for scenario $scenario did not converge under the " *
            ":column_generation oracle (stop_reason=$(cg_result.stop_reason), " *
            "$(cg_result.cg_iterations) CG iterations, $(cg_result.columns_added) columns " *
            "added). A cut may only be derived from an EXHAUSTED pricing round: duals from " *
            "a restricted pool are feasible for the restricted dual only, and the resulting " *
            "cut can exclude the true optimum. Raise cg_pricing_time_limit_sec or " *
            "max_cg_iterations; a 'dedup_stall' instead indicates the stale-tau column " *
            "livelock rather than a budget shortfall, and " *
            "'certification_inconclusive' means a relaxed-cluster attempt proved nothing -- " *
            "raise its budget, lower relaxed_cluster_count, or fall back to :exact.",
        )
    else
        optimize!(sm)
    end
    status = JuMP.termination_status(sm)
    status == MOI.OPTIMAL || error(
        "Benders subproblem for scenario $scenario returned $status, not OPTIMAL. A cut " *
        "may only be derived from an optimally solved subproblem -- a truncated or " *
        "infeasible one has no valid dual, and the resulting cut could exclude the true " *
        "optimum. (Infeasible would additionally contradict this formulation's " *
        "always-available direct-walk coverage; see subproblem.jl's docstring.)",
    )
    objective = JuMP.objective_value(sm)

    # One dual extraction, via the same function the CG master uses, so the sign convention
    # (`gamma = -dual` on the `<=` rows) is defined in exactly one place.
    alpha, gamma_o, gamma_d = extract_joint_routing_assignment_duals(sm)
    lpo_stats = nothing
    # NOT `:column_generation_warm_start`: its phase 2 exhausted the full universe
    # at the RAW duals, so those are already dual-feasible and completing them would only
    # inflate `Gamma` and weaken the cut for no reason.
    if solver.subproblem.oracle in (:column_generation_activated,
                                    :column_generation_activated_lpo)
        # Repairs the duals the restricted pricing left incomplete. Must run before the cut
        # is built, and (inside the CG loop) before each pricing round -- see the helper.
        _benders_activated_complete_duals!(
            alpha, gamma_o, gamma_d, incumbent,
            sm[:joint_routing_assignment_data], mapping, scenario,
            Float64(sm[:joint_routing_assignment_walk_cost_weight]),
        )
        if solver.subproblem.oracle === :column_generation_activated_lpo
            # Then trade the conservative closed-form bound for the strongest completion
            # that still passes separation. Runs only here, NOT inside the CG loop: during
            # pricing we WANT the aggressive restriction (it is what makes the search cheap);
            # for the cut we want the weakest valid penalties. The closed-form values just
            # written are this call's feasible fallback.
            lpo_stats = _benders_lpo_completion!(
                alpha, gamma_o, gamma_d, incumbent,
                sm[:joint_routing_assignment_data], mapping, scenario,
                build, solver.subproblem, _benders_lpo_core_point(sm),
            )
        end
    end
    alpha_sum = sum(values(alpha); init = 0.0)
    # Gamma[j], aggregated over both linking families and every demand group.
    coefficients = Dict{Int, Float64}()
    for gammas in (gamma_o, gamma_d)
        for (key, gamma) in gammas
            gamma == 0.0 && continue
            j = key[2]
            coefficients[j] = get(coefficients, j, 0.0) + gamma
        end
    end

    implied = alpha_sum
    for (j, gamma) in coefficients
        implied -= gamma * incumbent[j]
    end
    # Also a free check on the activated completion: it may only raise `gamma` on stations
    # with `incumbent[j] == 0`, whose terms drop out of `implied`. A completion that touched a
    # BUILT station would break this identity immediately.
    isapprox(implied, objective; rtol = 1e-6, atol = 1e-6) || error(
        "Benders subproblem scenario $scenario failed the strong-duality check: the dual " *
        "objective implied by the extracted duals is $implied but the primal optimum is " *
        "$objective. The cut derived from these duals would not be a valid " *
        "underestimator -- see subproblem.jl's docstring for the identity and what " *
        "breaking it usually means.",
    )

    # Independent cross-check on the same numbers (diagnostic only, see the result type).
    # Guarded, and NaN on failure: a fixed variable's reduced cost is a solver-interface
    # detail, and this must never be the thing that fails a run whose cut validity the
    # strong-duality check above has already established.
    mismatch = try
        worst = 0.0
        for j in eachindex(y)
            worst = max(worst, abs(JuMP.reduced_cost(y[j]) + get(coefficients, j, 0.0)))
        end
        worst
    catch
        NaN
    end

    return JointRoutingAssignmentBendersScenarioResult(
        scenario, objective, alpha_sum, coefficients, mismatch, cg_result, lpo_stats,
    )
end

"""
    _accumulate_benders_cg_stats!(m, results)

Accumulate the inner-CG totals for the whole run onto the MASTER model, under
`:benders_cg_stats`.

Lives here rather than in the generic loop because it is oracle-specific: the Benders loop
knows nothing about column generation, and `_benders_build_metadata` reads this key only if
it exists (a `:direct_enumeration` run never creates it). A no-op unless the subproblem
results actually carry CG outcomes.

`pool_final` is worth watching more than the totals: the whole bet of accumulating columns
across Benders iterations is that later iterations exhaust in one or two rounds, and
`iterations / rounds` per Benders iteration is what shows whether that is happening or
whether every iteration is re-pricing from cold.
"""
function _accumulate_benders_cg_stats!(m::JuMP.Model, results)
    any(r -> !isnothing(r.cg), results) || return nothing
    stats = get!(m.obj_dict, :benders_cg_stats) do
        Dict{String, Any}("iterations" => 0, "columns_added" => 0,
                          "pricing_sec" => 0.0, "restricted_pricing_sec" => 0.0,
                          "full_pricing_sec" => 0.0, "lp_sec" => 0.0, "rounds" => 0,
                          "certifications" => 0)
    end
    for r in results
        isnothing(r.cg) && continue
        stats["iterations"] += r.cg.cg_iterations
        stats["columns_added"] += r.cg.columns_added
        stats["pricing_sec"] += r.cg.pricing_sec
        # Split by searched universe -- see BendersSubproblemCGResult for why this is the
        # number that separates "shorter full search" from "smaller pool, same search".
        stats["restricted_pricing_sec"] += r.cg.restricted_pricing_sec
        stats["full_pricing_sec"] += r.cg.full_pricing_sec
        stats["lp_sec"] += r.cg.lp_sec
        stats["certifications"] += r.cg.certifications
        stats["rounds"] += 1
    end
    builds = m[:benders_subproblem_builds]::Vector{BuildResult}
    stats["pool_final"] = sum(length(b.model[:joint_routing_assignment_columns]) for b in builds)
    # LPO completion accounting, present only under the lpo oracle. `improved` is the
    # core-point-weighted reduction against the closed-form bound -- i.e. exactly how much
    # cut strength the Pareto selection bought, in the units it optimises.
    lpo = [r.lpo for r in results if !isnothing(r.lpo)]
    if !isempty(lpo)
        stats["lpo_rounds"] = get(stats, "lpo_rounds", 0) + sum(x.rounds for x in lpo)
        stats["lpo_rows"] = get(stats, "lpo_rows", 0) + sum(x.rows for x in lpo)
        stats["lpo_improved"] = get(stats, "lpo_improved", 0.0) + sum(x.improved for x in lpo)
        stats["lpo_optimal"] = get(stats, "lpo_optimal", 0) +
            count(x -> x.status == "optimal", lpo)
        stats["lpo_calls"] = get(stats, "lpo_calls", 0) + length(lpo)
        stats["lpo_pricing_sec"] = get(stats, "lpo_pricing_sec", 0.0) +
            sum(x.pricing_sec for x in lpo)
    end
    return nothing
end

"""
    _benders_activated_complete_duals!(alpha, gamma_o, gamma_d, incumbent, data, mapping,
                                       scenario, walk_cost_weight)

Repair the duals of an ACTIVATED subproblem so they are feasible for the full-universe dual:
raise `gamma` on every `(p, station)` linking row whose station is NOT built at `incumbent`,
to the smallest value that forces that triple's net reward non-positive.

What this function has to get exactly right is HOW MUCH to raise, since that is the cut's
strength. The validity proof is below, because it constrains the pricer this function reaches
into -- keep them together.

# Why this is valid

A column's dual constraint decomposes over its assignment triples, because linking is
per-assignment rather than per-route-node:

    sum_{(p,j,k) in c} [ alpha_p - gammaO_pj - gammaD_pk ]  <=  f_c

Setting `gammaO_pj := max(gammaO_pj, alpha_p)` for every unbuilt `j` (and likewise
`gammaD_pk`) does three things at once:

1. **It restricts the search for free.** `joint_routing_assignment_pricing_candidates`
   computes `rho = alpha_p - gammaO_pj - gammaD_pk - walk` and drops candidates with
   `rho <= 0`, so afterwards every candidate through an unbuilt station is dropped by the
   pricer's own filter. No change to any shared pricing code.
2. **It costs nothing at the anchor.** `yhat_j = 0` for those `j`, so the added terms vanish
   from the dual objective and the cut stays exactly TIGHT at `yhat`.
3. **It restores full-universe dual feasibility.** Any triple touching an unbuilt station now
   has `alpha_p - gamma - gamma <= 0`. For a column `c` mixing such triples with triples
   inside `S`, let `c''` be `c` with the out-of-`S` assignments dropped AND its unbuilt
   visited nodes removed. Then

       sum_{A_in} r <= f_{c''}                              (`c''` is in the searched universe)
       f_{c''} <= f_c - w * sum_{A_out} demand_p * walk_p    (`tau_{c''} <= tau_c`; walk >= 0)
       sum_{A_out} r <= w * sum_{A_out} walk_p              (the completion, per triple; demand >= 1)
       => sum_c r <= f_c.

**Removing the unbuilt nodes is not optional, and this is where the triangle inequality
enters.** The restriction is NOT "routes unrestricted, candidates shrink" -- candidate
generation is reward-driven, so the two are the same thing. `rho <= 0` drops the candidate
(`pricing_round.jl`), `create_joint_routing_assignment_pricing_data` builds
`assignments_by_origin`/`origin_layer_mask` from the survivors only, and both the seed
(`exact/seed.jl`, candidate origins only) and the extension (`exact/extend.jl`, nodes
proposed only from live origins and their opportunities' destinations) read exactly those.
So an unbuilt station leaves the searched ROUTE universe too, and the same-route `c'` is
never searched -- only the shortcut `c''` is. **`tau_{c''} <= tau_c` holds because the travel
matrix is required to be metric package-wide** (the pricer's age pruning already asserts on
violation), so this is a cross-cutting precondition, not a local assumption.

**`max_wait_time` is not a hazard for that shortcut**, though an earlier version of this
proof recorded it as one. Both route-feasibility conditions bound ELAPSED durations from
above -- the pickup window is `label.time <= max_wait_time`, the ride limit is
`origin_age + travel <= detour_factor * routing_cost(j,k)` -- and removing a stop only
decreases both. Nothing here measures wait against a fixed request clock, so a shortened
route cannot make a retained passenger wait longer. (`A -> U -> A -> B` with the pickup at
the second `A` is fine too: age is measured from the last visit to that station, so
collapsing the two visits leaves the in-vehicle time unchanged.)

For the same reason, "a column touching an unbuilt station is pinned to `theta = 0` by its
own linking row" -- true of a column with an unbuilt ASSIGNMENT -- does NOT cover one whose
route merely passes through an unbuilt node. Those are excluded from mattering by being
dominated by their own shortcut, not by being pinned.

**A free safety property**: the strong-duality assertion
`sum(alpha) - sum(Gamma .* yhat) == objective` still holds, because the completion only ever
touches stations with `yhat_j = 0`. So if it ever raised a `gamma` on a BUILT station, that
assertion fires immediately.

# The price is cut strength

Sound but weak, and the cause is which term gets credited rather than the bound being loose
in general: per unbuilt `(p,j)` the completion must cancel `alpha_p` and credits only the
walking term (0.2-1.2% of it), while ignoring the route travel it should charge for
(25-85%). With `walk_cost_weight = 0.1` against `route_regularization_weight = 10.0` that is
the wrong term by two orders of magnitude, so `gamma_pj` lands at essentially `alpha_p`.

The consequence is not "more iterations" but no convergence beyond n=10. Measured cut
strength against the exact value function, and the numbers behind both claims, are in
`notes/2026-09-10_activated_dual_completion_verified_and_why_weak.md` (job 22472084:
17/17 checks, 8 anchors, all 86 master-feasible station sets, `min rc = -1.8e-12`).
`:column_generation_activated_lpo` exists to buy that strength back -- see
`benders/completion_lpo.jl`.

# The bound, and why it credits the walking term

A triple's net reward is `alpha_p - gammaO_pj - gammaD_pk - w * walk(o, d, (j,k))`. The naive
completion sets `gamma := alpha_p`, which forces it non-positive using nothing but `alpha`.
But the walking term is already there and always non-negative, so it can be credited:

    gammaO_pj := max(0, alpha_p - w * min_k walk(o, d, (j,k)))

Then for every `k`, `net <= w * (walk_min - walk(o,d,(j,k))) <= 0`. Strictly smaller than
`alpha_p` whenever the walk cost is positive -- which it always is -- so a strictly stronger
cut at no cost. Clamping at zero is safe: if `alpha_p <= w * walk_min` the net is already
non-positive with no completion at all.

# Why NO demand factor, although `f_c` has one

The walk term enters the column cost as `w * demand_p * walk(...)`
(`joint_routing_assignment_column_cost`) but the pricer's candidate reward as
`w * walk(...)` with no demand (`joint_routing_assignment_pricing_candidates`). Those agree
only when `demand_p == 1`; the package guards the difference with a per-column assertion
(`_verify_joint_routing_assignment_master_reduced_cost`), which fires rather than drifting.

This function credits `w * walk_min` WITHOUT demand, which is the safe choice for both
requirements at once, because `demand_p >= 1`:

- against the DUAL CONSTRAINT (walk term `w * demand_p * walk`), crediting only
  `w * walk_min <= w * demand_p * walk_min` under-credits, so the resulting `gamma` is at
  least as large as required -- sound.
- against the PRICER's reward (walk term `w * walk`), it matches exactly, so the candidate is
  driven to `rho <= 0` and dropped by the `rho > 0` filter -- which is what makes the
  completion double as the search restriction.

Crediting `w * demand_p * walk_min` instead would still be sound for the dual but could leave
the pricer's `rho > 0` for a group with `demand_p > 1`, losing the restriction.

# Other properties

- `max` rather than assignment: the pool accumulates across Benders iterations, so it can
  already hold columns through currently-unbuilt stations whose rows carry a dual above this
  bound. Raising a `gamma` only relaxes an already-satisfied dual constraint, so `max` is safe
  and strictly tighter than overwriting.
- Only `(p, j)` pairs that appear in `get_valid_jk_pairs` are touched. A pair with no valid
  `(j,k)` has no linking row and no column can assign `p` there, so no dual constraint
  mentions it -- leaving it at zero is correct and a stronger cut.
- Never touches a station with `incumbent[j] == 1`, which keeps the cut tight at the anchor
  and is checked for free by the strong-duality assertion in
  `_solve_one_joint_routing_assignment_benders_subproblem`.
"""
function _benders_activated_complete_duals!(
        alpha::Dict{Tuple{Int, Int}, Float64},
        gamma_o::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
        gamma_d::Dict{Tuple{Tuple{Int, Int}, Int}, Float64},
        incumbent::Vector{Float64},
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        scenario::Int,
        walk_cost_weight::Float64,
    )
    built(j) = incumbent[j] > 0.5
    for (p, (o, d)) in enumerate(mapping.Omega_s[scenario])
        mapping.Q_s[scenario][p] > 0 || continue
        key2 = (scenario, p)
        a = get(alpha, key2, 0.0)
        a > 0.0 || continue

        # Per-station minimum walking cost over this group's valid pairs. Pickup side takes
        # the min over the partner dropoff, and vice versa -- each side must hold for EVERY
        # partner, so the binding case is the cheapest walk, not the average.
        walk_min_o = Dict{Int, Float64}()
        walk_min_d = Dict{Int, Float64}()
        for pair in get_valid_jk_pairs(mapping, o, d)
            is_walk_only_pair(pair) && continue
            j, k = pair
            cost = od_pair_walking_cost(data, o, d, pair)
            walk_min_o[j] = min(get(walk_min_o, j, Inf), cost)
            walk_min_d[k] = min(get(walk_min_d, k, Inf), cost)
        end

        for (j, wmin) in walk_min_o
            built(j) && continue
            needed = max(0.0, a - walk_cost_weight * wmin)
            gamma_o[(key2, j)] = max(get(gamma_o, (key2, j), 0.0), needed)
        end
        for (k, wmin) in walk_min_d
            built(k) && continue
            needed = max(0.0, a - walk_cost_weight * wmin)
            gamma_d[(key2, k)] = max(get(gamma_d, (key2, k), 0.0), needed)
        end
    end
    return nothing
end

"""
    _benders_lpo_core_point(sm) -> Vector{Float64}

The interior point the LPO completion optimises against, read off the model the MASTER build
stashed it on.

Each subproblem model is separate from the master, so the point has to be reachable from
here. It is stashed on the subproblem models too (the master build hands it over at
construction) rather than recomputed, because it depends only on the master's feasible
region and recomputing it per scenario per iteration would be pure waste -- and two
scenarios optimising against different interior points would select completions that are
Pareto-optimal with respect to different objectives, which is not what the cut wants.
"""
function _benders_lpo_core_point(sm::JuMP.Model)::Vector{Float64}
    haskey(sm.obj_dict, :benders_core_point) || error(
        "the :column_generation_activated_lpo oracle needs an interior point on the " *
        "subproblem model (:benders_core_point); the master build stashes it, so a model " *
        "without it was not built for this oracle",
    )
    return sm[:benders_core_point]::Vector{Float64}
end
