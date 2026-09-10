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
    results = JointRoutingAssignmentBendersScenarioResult[]
    total = 0.0
    for build in builds
        push!(results, _solve_one_joint_routing_assignment_benders_subproblem(
            build, incumbent, solver,
        ))
        total += results[end].objective
    end
    _accumulate_benders_cg_stats!(m, results)
    return JointRoutingAssignmentBendersSubproblemResult(results, total, time() - t0)
end

function _solve_one_joint_routing_assignment_benders_subproblem(
        build::BuildResult,
        incumbent::Vector{Float64},
        solver::BendersSolver,
    )::JointRoutingAssignmentBendersScenarioResult
    sm = build.model
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
    if solver.subproblem.oracle === :column_generation
        cg_result = _solve_joint_routing_assignment_subproblem_by_cg!(build, solver.subproblem)
        cg_result.converged || error(
            "Benders subproblem for scenario $scenario did not converge under the " *
            ":column_generation oracle (stop_reason=$(cg_result.stop_reason), " *
            "$(cg_result.cg_iterations) CG iterations, $(cg_result.columns_added) columns " *
            "added). A cut may only be derived from an EXHAUSTED pricing round: duals from " *
            "a restricted pool are feasible for the restricted dual only, and the resulting " *
            "cut can exclude the true optimum. Raise cg_pricing_time_limit_sec or " *
            "max_cg_iterations; a 'dedup_stall' instead indicates the stale-tau column " *
            "livelock rather than a budget shortfall.",
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

    alpha_sum = 0.0
    for (_key, con) in sm[:joint_routing_assignment_coverage]
        alpha_sum += dual(con)
    end
    # Gamma[j], aggregated over both linking families and every demand group. Same sign
    # convention as extract_joint_routing_assignment_duals: negate the `<=` rows' duals so
    # gamma >= 0.
    coefficients = Dict{Int, Float64}()
    for links in (sm[:joint_routing_assignment_pickup_link], sm[:joint_routing_assignment_dropoff_link])
        for (key, con) in links
            gamma = -dual(con)
            gamma == 0.0 && continue
            j = key[2]
            coefficients[j] = get(coefficients, j, 0.0) + gamma
        end
    end

    implied = alpha_sum
    for (j, gamma) in coefficients
        implied -= gamma * incumbent[j]
    end
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
        scenario, objective, alpha_sum, coefficients, mismatch, cg_result,
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
                          "pricing_sec" => 0.0, "lp_sec" => 0.0, "rounds" => 0)
    end
    for r in results
        isnothing(r.cg) && continue
        stats["iterations"] += r.cg.cg_iterations
        stats["columns_added"] += r.cg.columns_added
        stats["pricing_sec"] += r.cg.pricing_sec
        stats["lp_sec"] += r.cg.lp_sec
        stats["rounds"] += 1
    end
    builds = m[:benders_subproblem_builds]::Vector{BuildResult}
    stats["pool_final"] = sum(length(b.model[:joint_routing_assignment_columns]) for b in builds)
    return nothing
end
