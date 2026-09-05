export CGSolver

"""
    CGSolver <: AbstractSolver

Column-generation outer loop: repeatedly solve the restricted master (`build_result`'s
model), price new columns against its duals, and add any improving ones back into the
master, until pricing finds nothing improving or `max_iterations` is reached.

`pricing_time_limit_sec` is the wall-clock budget for **one whole pricing round**, across
every scenario. Under serial pricing `_run_pricing_round` divides it equally, so each
scenario's label search gets `pricing_time_limit_sec / n_scenarios`; under
`parallel_scenario_pricing` the searches overlap, so each scenario gets the *full* budget
and the round still fits the same wall. An empty result from a search that hits this
limit does not set `cg_converged=true`; only exhausted pricing can certify that the
restricted master needs no further improving columns.

The budget is per round rather than per scenario so that a round costs what it says it
costs -- no `n_scenarios` multiplier -- which is what makes `total_time_limit_sec` below
enforceable, and so that no scenario can starve another (a shared deadline spent in
scenario order would always favour the same scenarios, improving one scenario's coverage
round after round while the rest never advance).

## Two-tier pricing: regular vs. certifying rounds

Pricing runs at two budgets. Ordinary iterations use `pricing_time_limit_sec` (default
300 s), which is sized to *find* improving columns cheaply, not to prove none exist.
When a regular round comes back empty **without** having exhausted its frontier, the
empty result is inconclusive -- it may just have run out of time -- so the loop
immediately re-prices the same duals at `certifying_pricing_time_limit_sec` (default
3600 s). That second, longer pass is the *certifying* round: only it can turn an empty
result into `cg_converged=true`.

A regular round that comes back empty **and** exhausted needs no escalation -- it has
already proved no negative-reduced-cost column exists, and the loop stops certified.
Escalation is therefore paid at most once per iteration and only when it might change
the answer. `metadata["cg_certifying_rounds"]` counts how often it fired, and each
iteration log row carries `certifying_pricing` plus the `pricing_limit_sec` actually
used.

## Certify-first (`certification_pricing_mode`)

The two-tier escalation above is the expensive way to prove pricing is done: it re-prices
the identical duals under a much longer budget, and on hard instances that certifying
round dominates the whole solve. `certification_pricing_mode` (default `nothing`) adds a
cheap way to *try* proving it first. Two modes today, both relaxed-cluster:

- `:relaxed_cluster` -- the one-shot form. Gives up the moment the relaxation finds any
  improving cluster route, which at a converged master it essentially always does (0/31
  measured), so it is kept for comparison rather than for use.
- `:relaxed_cluster_nogood` -- the no-good-cut loop, and the mode that actually certifies.
  When the relaxation names an improving cluster route, it searches that route's cluster
  support exhaustively with the real pricer; a barren support becomes a cut and the
  relaxation is asked again. `certification_max_rounds` (default 32) caps the cuts per
  scenario per round.

When set, every iteration -- before the real pricing round -- runs a **relaxation** of the
pricing problem under `certification_time_limit_sec` (default 300 s). The relaxation is
built so that its minimum reduced cost lower-bounds the real one, so if it exhausts
without finding anything below `-reduced_cost_tol`, no real improving column exists
either: the loop stops with `cg_converged=true` and
`cg_stop_reason="converged_by_certification"`, having skipped both the regular and the
certifying round. If it fails -- either it found an improving *relaxed* solution (the
relaxation is too loose, or an improving column genuinely exists) or it ran out of time --
it has proved nothing and the iteration proceeds to `price_columns` exactly as if the
feature were off.

Cost differs sharply between the two modes. A failed `:relaxed_cluster` attempt is nearly
free: the relaxed search early-exits at the first improving solution, which is the common
case in every iteration but the last. `:relaxed_cluster_nogood` instead pays one
exhaustive real-pricer search per cut it adds, so a failing attempt can cost as much as a
pricing round -- budget `certification_time_limit_sec` accordingly.

The certificate covers the **full route universe**, because the relaxation bounds every
real route rather than only the ones the active pricer searches. So a certified run
reports `cg_optimality_scope="full_route_universe"` even when its column-finding pricer is
`:station_simple`, and `cg_certified_by_relaxation=true` records that the certificate came
from the relaxation rather than from exhausted pricing.
`metadata["cg_certification_rounds"]`/`["cg_certification_sec"]` cost it, and
`["cg_certification_refuted_rounds"]`/`["cg_certification_inconclusive_rounds"]` split the
failures into the two kinds that call for opposite fixes -- *refuted* versus
*inconclusive* (the attempt ran out of `certification_time_limit_sec`, or hit the round or
cut cap). What *refuted* means depends on the mode: under `:relaxed_cluster` an improving
relaxed solution existed, so the relaxation is too loose; under
`:relaxed_cluster_nogood` an exhaustive real search found a genuinely improving column, so
it is a true negative and says nothing against the relaxation. Each iteration log row
carries `certification_sec`, `certification_certified` and `certification_outcome`.

Requires a formulation implementing `cg_certification_supported`/`cg_certification_round`;
a mode that nothing supports is rejected up front, never silently ignored. For both
relaxed-cluster modes that means building
`AggregateODRouteJointRoutingAssignmentFormulation(relaxed_cluster_count = K)`, whose
station partition is fixed at build time -- see
`label_setting/joint_routing_assignment/relaxed_cluster/`.

## Total budget (`total_time_limit_sec`)

A strict wall-clock cap on the CG loop (default `Inf`). It is checked before every
iteration and additionally *clamps* each pricing round's budget to the remaining budget.
Because that budget is per round (see above), the clamp is exact: a round cannot spend
more than it was granted, so the loop cannot overrun the cap by more than one label
search's clock-check granularity. When the budget runs out the
loop stops and the run reports `cg_converged=false` / `cg_pricing_exhausted=false` with
`cg_stop_reason="total_budget"` and `cg_total_budget_exhausted=true`: the incumbent is
feasible but its optimality is **not** certified, and the LP value is *not* a valid
bound on the unrestricted optimum. The point is that a budget-bound run still returns a
usable result instead of being killed by the scheduler with nothing written.

`parallel_scenario_pricing` (default `false`) prices scenarios concurrently with
`Threads.@threads` when more than one thread is available. Both settings obey the same
round wall budget, so the comparison is like for like on time; what differs is how much
search fits inside it. Serial splits the round `n_scenarios` ways, parallel gives every
scenario the whole round, so parallel performs up to `n_scenarios` x more label search per
round and can therefore certify instances a serial run cannot. The round's wall bound holds
only while `Threads.nthreads() >= n_scenarios`; with fewer threads the searches run in
waves and a round can take up to `ceil(n_scenarios / nthreads)` x the budget.

Note the cap bounds the **loop**. When `recover_integer_solution=true` the recovery MIP
runs afterwards under its own `config.time_limit_sec`, so the whole solve can exceed
`total_time_limit_sec` by at most that one solve -- size the SLURM walltime with room
for both (e.g. a 4 h budget and a 300 s recovery limit fit comfortably in a 6 h job).

Relies on three formulation-specific hooks -- implemented per `AbstractFormulation`
(or per `AbstractProblem`), not here:

    extract_duals(build_result, mapping, m) -> duals
    price_columns(build_result, mapping, m, duals, solver::CGSolver) -> Union{Nothing, AbstractVector}
    add_columns!(build_result, mapping, m, columns) -> Int

`mapping` (`build_result.mapping`) is passed as its own positional argument, not just
read off `build_result`, so that formulation-specific methods can dispatch on its
concrete type (e.g. `mapping::AggregateODRouteMap`). Dispatching on `build_result::BuildResult`
alone can't distinguish formulations -- every formulation's hook would share the exact
same `(BuildResult, JuMP.Model, ...)` signature as this file's generic fallback, which
Julia treats as a redefinition (not a new method) and module precompilation then rejects
outright as illegal method overwriting.

`price_columns` returns `nothing` (or an empty collection) when no improving column
exists -- that's the convergence signal this loop watches for. `add_columns!` mutates
the restricted master in place and returns how many columns it added.

## Integer recovery (`recover_integer_solution`)

The loop above solves an LP relaxation throughout -- the master's first-stage/column
variables (e.g. `y`, `θ`) are continuous so their duals are valid for pricing. That LP
optimum is generally fractional. When `recover_integer_solution=true`, once the loop
above exits (converged or `max_iterations` reached) with an `OPTIMAL` LP on hand, a
fourth hook

    integer_recovery_build(build_result, mapping, m) -> BuildResult

is called to *rebuild* the master from scratch in its true (binary/integer) domain, over
the exact column pool CG has generated so far -- no further pricing happens. This is a
real `build_model`-shaped rebuild, not an in-place mutation of `m`: see
`_build_joint_routing_assignment_model`/`integer_recovery_build`
(`optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`) for
why sharing the actual construction code with `build_model` (parameterized by
`relax_integrality`/seed columns) is safer than duplicating it as a set of post-hoc
`set_binary` calls. The returned `BuildResult` replaces this call's `build_result`/`m`,
which is then re-optimized once as a genuine MIP. This is the standard "restricted master
heuristic": the resulting integer solution is feasible for the real problem and its
objective is a valid upper bound, but -- because pricing only ever ran against LP duals
-- it is not guaranteed globally optimal for the original (unrestricted) column set. The
pre-recovery LP objective is preserved in `OptResult.metadata` under
`"cg_lp_objective_value"` as a lower bound for judging that gap; `"cg_converged"` records
whether pricing actually exhausted (vs. hit `max_iterations`), since only the converged
case makes that LP value a valid bound on the true (unrestricted) optimum.

## Per-iteration log (`metadata["cg_iteration_log"]`)

One `NamedTuple` per CG iteration, in order:

| field | meaning |
| --- | --- |
| `iteration` | 1-based iteration index |
| `master_sec` | wall time in this iteration's master `optimize!` |
| `pricing_sec` | wall time in `price_columns` |
| `add_columns_sec` | wall time in `add_columns!` |
| `columns_added` | how many columns pricing returned this iteration |
| `columns_accepted` | how many of those actually entered the master (`add_columns!`'s return; the rest were de-duplicated away) |
| `cumulative_columns_added` | running total of `columns_accepted`, excluding seed columns |
| `master_objective` | master LP objective, or `missing` if not `OPTIMAL` |
| `master_status` | this iteration's master termination status |
| `pricing_limit_sec` | the time limit this iteration's pricing actually ran under (regular, certifying, or whatever the total budget clamped it to) |
| `certifying_pricing` | `true` if this iteration escalated to a certifying round |
| `certification_sec` | wall time in this iteration's relaxation certification attempt (`0.0` when the feature is off) |
| `certification_certified` | `true` on the single iteration whose relaxation certified, ending the loop |
| `certification_outcome` | `"certified"` / `"refuted"` (an improving relaxed solution existed -- the relaxation is too loose) / `"inconclusive"` (the attempt ran out of budget) / `"none"` (no attempt this iteration) |

The final iteration is always logged, including the one that breaks the loop (on
convergence, on a non-`OPTIMAL` master, or on the last `max_iterations` pass), so
`length(log) == metadata["cg_iterations"]`.

Splitting the wall time three ways is what makes a master-bound run distinguishable
from a pricing-bound one without re-running under a profiler. Note this is separate
from `"cg_pricing_stats"`, which is a flat per-(iteration x scenario) list of *label
search* counters carrying no iteration index, and whose own `t_*_sec` timers are only
populated when the label-setting round is called with `profile=true`.

`"cg_lp_loop_sec"` is the CG loop alone and `"cg_integer_recovery_sec"` the recovery
solve (`0.0` when recovery is off), so the two can be reported separately even though
`OptResult.runtime_sec` covers both.
"""
struct CGSolver <: AbstractSolver
    config::SolverOptions
    max_iterations::Int
    reduced_cost_tol::Float64
    pricing_time_limit_sec::Float64
    certifying_pricing_time_limit_sec::Float64
    total_time_limit_sec::Float64
    parallel_scenario_pricing::Bool
    initial_columns::Union{Nothing, AbstractVector}
    recover_integer_solution::Bool
    warm_start_pricing_mode::Union{Nothing, Symbol}
    certification_pricing_mode::Union{Nothing, Symbol}
    certification_time_limit_sec::Float64
    certification_max_rounds::Int

    function CGSolver(;
            config::SolverOptions=SolverOptions(),
            max_iterations::Int=1_000,
            reduced_cost_tol::Number=1e-6,
            pricing_time_limit_sec::Number=300.0,
            certifying_pricing_time_limit_sec::Number=3600.0,
            total_time_limit_sec::Number=Inf,
            parallel_scenario_pricing::Bool=false,
            initial_columns::Union{Nothing, AbstractVector}=nothing,
            recover_integer_solution::Bool=false,
            warm_start_pricing_mode::Union{Nothing, Symbol}=nothing,
            certification_pricing_mode::Union{Nothing, Symbol}=nothing,
            certification_time_limit_sec::Number=300.0,
            certification_max_rounds::Int=32,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        reduced_cost_tol >= 0 || throw(ArgumentError("reduced_cost_tol must be non-negative"))
        pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
        certifying_pricing_time_limit_sec > 0 ||
            throw(ArgumentError("certifying_pricing_time_limit_sec must be positive"))
        certifying_pricing_time_limit_sec >= pricing_time_limit_sec || throw(ArgumentError(
            "certifying_pricing_time_limit_sec ($certifying_pricing_time_limit_sec) must be >= " *
            "pricing_time_limit_sec ($pricing_time_limit_sec): the certifying round exists to give " *
            "an inconclusive regular round MORE time, never less",
        ))
        total_time_limit_sec > 0 || throw(ArgumentError("total_time_limit_sec must be positive"))
        certification_time_limit_sec > 0 ||
            throw(ArgumentError("certification_time_limit_sec must be positive"))
        isnothing(certification_pricing_mode) ||
            certification_pricing_mode in (:relaxed_cluster, :relaxed_cluster_nogood) ||
            throw(ArgumentError(
                "certification_pricing_mode must be :relaxed_cluster, " *
                ":relaxed_cluster_nogood, or nothing, got " *
                "$(repr(certification_pricing_mode))",
            ))
        certification_max_rounds >= 1 || throw(ArgumentError(
            "certification_max_rounds must be >= 1, got $(certification_max_rounds)",
        ))
        new(
            config, max_iterations, Float64(reduced_cost_tol),
            Float64(pricing_time_limit_sec), Float64(certifying_pricing_time_limit_sec),
            Float64(total_time_limit_sec), parallel_scenario_pricing,
            initial_columns, recover_integer_solution, warm_start_pricing_mode,
            certification_pricing_mode, Float64(certification_time_limit_sec),
            certification_max_rounds,
        )
    end
end

function optimize_model(build_result::BuildResult, solver::CGSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)
    mapping = build_result.mapping

    start_time = time()
    # Strict wall budget for the loop. `remaining_budget()` is what every pricing call is
    # clamped to, so no single label search can run past it (see the docstring: the
    # recovery MIP afterwards is bounded separately by `config.time_limit_sec`).
    remaining_budget() = solver.total_time_limit_sec - (time() - start_time)

    iterations_run = 0
    converged = false
    budget_exhausted = false
    certifying_rounds = 0
    stop_reason = "max_iterations"

    # Warm-start phasing. `active_pricing_mode` is the pricer in force right now; the loop
    # runs phase 1 in `warm_start_pricing_mode` until that universe is exhausted, then
    # hands off to the formulation's own pricer for phase 2, which is the only phase that
    # can certify. Both phases share one column pool and one master -- the handoff changes
    # what pricing searches, nothing else, so every column phase 1 harvested stays.
    final_pricing_mode = cg_pricing_mode(build_result, mapping, m)
    warm_start_mode = solver.warm_start_pricing_mode
    warm_start_active = false
    warm_start_iterations = 0
    # Wall seconds spent in phase 1, i.e. how long the restricted universe took to exhaust.
    # Recorded here rather than summed from `cg_iteration_log` afterwards because the log
    # captures only master/pricing/add-columns time, so a sum of it silently omits
    # everything else in the iteration and understates the phase.
    warm_start_sec = 0.0
    if !isnothing(warm_start_mode)
        isnothing(final_pricing_mode) && throw(ArgumentError(
            "warm_start_pricing_mode=$(repr(warm_start_mode)) was requested, but this " *
            "formulation has no selectable pricer (no `cg_pricing_mode` method), so there " *
            "is nothing to warm-start from and nothing to hand off to",
        ))
        warm_start_mode === final_pricing_mode && throw(ArgumentError(
            "warm_start_pricing_mode=$(repr(warm_start_mode)) equals the formulation's own " *
            "pricing_mode: a warm-start phase that hands off to itself is a no-op, and " *
            "would silently halve max_iterations",
        ))
        set_cg_pricing_mode!(build_result, mapping, m, warm_start_mode)
        warm_start_active = true
    end
    active_pricing_mode = warm_start_active ? warm_start_mode : final_pricing_mode
    _mode_label() = string(something(active_pricing_mode, :default))

    # Relaxation-based certification. When enabled, every iteration asks the cheap relaxed
    # pricer "can an improving column still exist?" BEFORE paying for the real one; a `no`
    # is a full-route-universe optimality certificate and ends the solve on the spot, a
    # `yes` proves nothing and the loop falls through to the real pricer unchanged. See
    # `label_setting/joint_routing_assignment/relaxed_cluster/types.jl` for the bound.
    certification_mode = solver.certification_pricing_mode
    if !isnothing(certification_mode)
        cg_certification_supported(build_result, mapping, m, certification_mode) || throw(ArgumentError(
            "certification_pricing_mode=$(repr(certification_mode)) was requested, but this " *
            "model has no such certification pricer available -- for :relaxed_cluster, build " *
            "the formulation with `relaxed_cluster_count = K`",
        ))
    end
    certification_rounds = 0
    # Why the failed attempts failed, which is the only way to read a run that never
    # certifies: "refuted" means an improving relaxed solution existed, so the relaxation
    # is too loose as configured -- a between-runs observation, since a certification
    # pricer's tightness is fixed before the solve starts and nothing here can react to it;
    # "inconclusive" means the search ran out of `certification_time_limit_sec` without
    # settling either way, which IS a knob on this solver. The bare attempt count cannot
    # tell the two apart, and they point at different places.
    certification_refuted_rounds = 0
    certification_inconclusive_rounds = 0
    certification_sec = 0.0
    # Columns recovered from FAILED certification attempts (see the harvest branch below).
    # Reported so the feature's cost can be read net of what it gave back.
    certification_harvested_columns = 0
    certified_by_relaxation = false
    # One row per CG iteration, exposed as metadata["cg_iteration_log"]. Wall time is
    # split into the master LP solve, the pricing call, and column insertion, so a run
    # that is master-bound can be told apart from one that is pricing-bound without
    # re-running under a profiler. `columns_added` is what pricing actually returned
    # this iteration (the pool grows by that much); `cumulative_columns_added` excludes
    # any seed columns the model was built with.
    iteration_log = NamedTuple[]
    cumulative_columns_added = 0
    for iteration in 1:solver.max_iterations
        if remaining_budget() <= 0
            budget_exhausted = true
            stop_reason = "total_budget"
            break
        end
        iterations_run = iteration
        # Declared before the master solve because the non-OPTIMAL early-exit below logs
        # them too (as a zero-cost, uncertified round -- certification never got to run).
        iteration_certification_sec = 0.0
        certification_candidates = Any[]
        iteration_certified = false
        iteration_certification_outcome = "none"

        t0 = time()
        optimize!(m)
        master_sec = time() - t0
        status = JuMP.termination_status(m)

        if status != MOI.OPTIMAL
            push!(iteration_log, (
                iteration=iteration, master_sec=master_sec, pricing_sec=0.0,
                add_columns_sec=0.0, columns_added=0, columns_accepted=0,
                cumulative_columns_added=cumulative_columns_added,
                master_objective=missing, master_status=string(status),
                pricing_limit_sec=0.0, certifying_pricing=false,
                pricing_mode=_mode_label(),
                certification_sec=iteration_certification_sec,
                certification_certified=false,
                certification_outcome=iteration_certification_outcome,
            ))
            stop_reason = "master_not_optimal"
            break
        end
        master_objective = JuMP.objective_value(m)

        duals = extract_duals(build_result, mapping, m)

        # Certify-first: a successful relaxed round makes the real pricing round
        # unnecessary, so it runs before it rather than as a fallback after it. It is
        # bounded by its own (short) budget, and a round that is going to fail early-exits
        # at the first improving relaxed route, so the cost of a failed attempt is small.
        if !isnothing(certification_mode)
            certification_limit = min(solver.certification_time_limit_sec, remaining_budget())
            if certification_limit > 0
                t_cert = time()
                certification_rounds += 1
                certification = cg_certification_round(
                    build_result, mapping, m, duals, solver, certification_mode;
                    time_limit_sec=certification_limit,
                )
                iteration_certification_sec = time() - t_cert
                certification_sec += iteration_certification_sec
                iteration_certified = certification.certified
                certification_candidates = certification.candidates
                iteration_certification_outcome = if certification.certified
                    "certified"
                elseif certification.improving_found
                    certification_refuted_rounds += 1
                    "refuted"
                else
                    certification_inconclusive_rounds += 1
                    "inconclusive"
                end
            end
        end
        if iteration_certified
            # A relaxation certificate covers the FULL route universe (it bounds every real
            # route, not just the ones the active pricer would search), so unlike the
            # warm-start phase boundary above there is nothing left to hand off to: the
            # solve is genuinely done, whichever pricer was in force.
            converged = true
            certified_by_relaxation = true
            stop_reason = "converged_by_certification"
            push!(iteration_log, (
                iteration=iteration, master_sec=master_sec, pricing_sec=0.0,
                add_columns_sec=0.0, columns_added=0, columns_accepted=0,
                cumulative_columns_added=cumulative_columns_added,
                master_objective=master_objective, master_status=string(status),
                pricing_limit_sec=0.0, certifying_pricing=false,
                pricing_mode=_mode_label(),
                certification_sec=iteration_certification_sec, certification_certified=true,
                certification_outcome=iteration_certification_outcome,
            ))
            break
        end

        # A FAILED certification attempt is not wasted work. `:relaxed_cluster_nogood`
        # refutes the relaxation by running the real exact pricer over a station subset,
        # and hands back the improving columns that search found. Taking them as this
        # iteration's pricing result skips the regular round entirely -- the expensive
        # full-station search -- for the price of an attempt that had to run anyway.
        #
        # Soundness: these columns are ordinary priced columns (same materialization, same
        # `_pricing_verify_column` cross-check), but the subset they came from is a
        # RESTRICTED route universe, so their absence would prove nothing. That is why this
        # branch only ever *skips* a pricing round when it has columns to show for it --
        # convergence is still declared only by a full-universe certificate above, or by a
        # full-universe pricing round exhausting below. `cg_pricing_exhausted` is left
        # untouched here for the same reason: this round proved no exhaustion of anything.
        harvested_columns = _cg_materialize_certification_columns(
            build_result, mapping, m, duals, certification_candidates,
        )
        if !isempty(harvested_columns)
            t_add = time()
            accepted = add_columns!(build_result, mapping, m, harvested_columns)
            add_sec = time() - t_add
            cumulative_columns_added += accepted
            certification_harvested_columns += accepted
            push!(iteration_log, (
                iteration=iteration, master_sec=master_sec, pricing_sec=0.0,
                add_columns_sec=add_sec, columns_added=length(harvested_columns),
                columns_accepted=accepted,
                cumulative_columns_added=cumulative_columns_added,
                master_objective=master_objective, master_status=string(status),
                pricing_limit_sec=0.0, certifying_pricing=false,
                pricing_mode=_mode_label(),
                certification_sec=iteration_certification_sec, certification_certified=false,
                certification_outcome=iteration_certification_outcome,
            ))
            accepted == 0 && break   # nothing entered the master: no progress is possible
            continue
        end

        t0 = time()
        pricing_limit = min(solver.pricing_time_limit_sec, remaining_budget())
        new_columns = price_columns(build_result, mapping, m, duals, solver;
                                    time_limit_sec=pricing_limit)
        certifying = false

        # A regular round that comes back empty is only conclusive if its label searches
        # actually exhausted. If it merely ran out of its (short) budget, re-price the
        # same duals at the certifying budget before concluding anything -- otherwise a
        # cheap pricing timeout would be indistinguishable from a real optimality proof.
        if (isnothing(new_columns) || isempty(new_columns)) && !_cg_pricing_exhausted(m)
            certifying_limit = min(solver.certifying_pricing_time_limit_sec, remaining_budget())
            if certifying_limit > pricing_limit
                certifying = true
                certifying_rounds += 1
                pricing_limit = certifying_limit
                new_columns = price_columns(build_result, mapping, m, duals, solver;
                                            time_limit_sec=certifying_limit)
            end
        end
        pricing_sec = time() - t0

        if isnothing(new_columns) || isempty(new_columns)
            # An empty result certifies convergence only when every underlying
            # label search exhausted its frontier. A time-limited pricing pass
            # can also return no columns; treating that as exhaustion silently
            # turns a pricing timeout into a false optimality certificate.
            converged = _cg_pricing_exhausted(m)

            # Phase 1 exhausting its own universe is a phase boundary, not the end of the
            # solve. Hand off to the formulation's real pricer and keep going against the
            # same pool: `converged` is deliberately reset to false, because exhausting the
            # warm-start universe proves nothing about the full one, and leaving it true
            # here is exactly how a restricted search would masquerade as a certificate.
            if converged && warm_start_active
                push!(iteration_log, (
                    iteration=iteration, master_sec=master_sec, pricing_sec=pricing_sec,
                    add_columns_sec=0.0, columns_added=0, columns_accepted=0,
                    cumulative_columns_added=cumulative_columns_added,
                    master_objective=master_objective, master_status=string(status),
                    pricing_limit_sec=pricing_limit, certifying_pricing=certifying,
                    pricing_mode=_mode_label(),
                    certification_sec=iteration_certification_sec,
                    certification_certified=false,
                    certification_outcome=iteration_certification_outcome,
                ))
                set_cg_pricing_mode!(build_result, mapping, m, final_pricing_mode)
                warm_start_active = false
                warm_start_iterations = iteration
                warm_start_sec = time() - start_time
                active_pricing_mode = final_pricing_mode
                converged = false
                stop_reason = "max_iterations"
                continue
            end

            if converged
                stop_reason = "converged"
            elseif remaining_budget() <= 0
                budget_exhausted = true
                stop_reason = "total_budget"
            else
                stop_reason = "pricing_inconclusive"
            end
            push!(iteration_log, (
                iteration=iteration, master_sec=master_sec, pricing_sec=pricing_sec,
                add_columns_sec=0.0, columns_added=0, columns_accepted=0,
                cumulative_columns_added=cumulative_columns_added,
                master_objective=master_objective, master_status=string(status),
                pricing_limit_sec=pricing_limit, certifying_pricing=certifying,
                pricing_mode=_mode_label(),
                certification_sec=iteration_certification_sec,
                certification_certified=false,
                certification_outcome=iteration_certification_outcome,
            ))
            break
        end

        t0 = time()
        columns_accepted = add_columns!(build_result, mapping, m, new_columns)
        add_columns_sec = time() - t0
        cumulative_columns_added += columns_accepted
        push!(iteration_log, (
            iteration=iteration, master_sec=master_sec, pricing_sec=pricing_sec,
            add_columns_sec=add_columns_sec, columns_added=length(new_columns),
            columns_accepted=columns_accepted,
            cumulative_columns_added=cumulative_columns_added,
            master_objective=master_objective, master_status=string(status),
            pricing_limit_sec=pricing_limit, certifying_pricing=certifying,
            pricing_mode=_mode_label(),
            certification_sec=iteration_certification_sec,
            certification_certified=false,
            certification_outcome=iteration_certification_outcome,
        ))

        # Pricing returned improving columns but every one was de-duplicated away, so the
        # master is unchanged and the next iteration would extract identical duals and
        # re-find the identical columns -- a livelock that otherwise burns every remaining
        # iteration (see notes/2026-08-25_study6_cg_livelock_stale_tau_columns.md).
        # `converged` deliberately stays false: pricing DID find a negative-reduced-cost
        # column, so this is a failure to make progress, never an optimality certificate.
        if columns_accepted == 0
            stop_reason = "no_columns_accepted"
            break
        end
    end
    # The loop can exit having just added columns but not yet re-solved the master --
    # budget expiry, the iteration cap, or a de-dup stall all do this. Adding variables
    # invalidates JuMP's solution, so the model reports OPTIMIZE_NOT_CALLED and the run
    # would return no incumbent and no objective at all: a censored job would write an
    # empty row, which is exactly what the total budget exists to avoid. Re-solve the
    # master once (bounded by config.time_limit_sec) so a stopped-early run still yields
    # a usable LP solution and a pool for integer recovery. Only OPTIMIZE_NOT_CALLED is
    # retried -- a genuine INFEASIBLE/UNBOUNDED status is a real answer, not staleness.
    final_master_resolved = false
    if JuMP.termination_status(m) == MOI.OPTIMIZE_NOT_CALLED
        final_master_resolved = true
        optimize!(m)
    end
    lp_loop_sec = time() - start_time

    metadata = Dict{String, Any}(
        "cg_iterations" => iterations_run,
        "cg_converged" => converged,
        "cg_pricing_exhausted" => converged,
        # Why the loop stopped: "converged", "converged_by_certification", "total_budget",
        # "max_iterations", "master_not_optimal", "no_columns_accepted", or
        # "pricing_inconclusive". Only the two "converged*" reasons make
        # cg_lp_objective_value a valid bound on the true optimum.
        "cg_stop_reason" => stop_reason,
        "cg_total_budget_exhausted" => budget_exhausted,
        "cg_total_time_limit_sec" => solver.total_time_limit_sec,
        "cg_certifying_rounds" => certifying_rounds,
        # Which pricer each phase used, and how much of the run was the warm-start phase.
        # `cg_pricing_universe_restricted` is what makes a run's OPTIMAL claim trustworthy:
        # true means pricing finished in a restricted universe, so no certificate is possible.
        "cg_warm_start_pricing_mode" => warm_start_mode,
        "cg_warm_start_iterations" => warm_start_iterations,
        # Elapsed wall at the phase-1 -> phase-2 handoff. Zero when no warm start ran, and
        # ALSO zero when a warm start was requested but its phase never exhausted (the run
        # stopped inside phase 1) -- `cg_warm_start_iterations` distinguishes those two.
        "cg_warm_start_sec" => warm_start_sec,
        "cg_final_pricing_mode" => active_pricing_mode,
        # Which certification pricer ran (if any), how many rounds it cost, and whether it
        # is what ended the solve. A relaxation certificate bounds EVERY real route, so it
        # is a full-route-universe certificate regardless of which pricer was finding
        # columns -- which is why it overrides the two scope keys below.
        "cg_certification_pricing_mode" => certification_mode,
        "cg_certification_rounds" => certification_rounds,
        "cg_certification_refuted_rounds" => certification_refuted_rounds,
        "cg_certification_inconclusive_rounds" => certification_inconclusive_rounds,
        # Columns recovered from failed certification attempts. Read against
        # `cg_certification_sec` to judge the feature's NET cost: a failed attempt that
        # hands back columns replaced a pricing round rather than adding to one.
        "cg_certification_harvested_columns" => certification_harvested_columns,
        "cg_certification_sec" => certification_sec,
        "cg_certified_by_relaxation" => certified_by_relaxation,
        "cg_pricing_universe_restricted" => !certified_by_relaxation &&
            _cg_pricing_universe_is_restricted(active_pricing_mode),
        # What an OPTIMAL status on THIS result actually asserts. "elementary_routes_only"
        # means pricing never considered a revisiting column, so the optimum is optimal
        # within that restriction and may be beaten outside it.
        "cg_optimality_scope" => certified_by_relaxation ? "full_route_universe" :
            _cg_optimality_scope(active_pricing_mode),
        "cg_final_master_resolved" => final_master_resolved,
        "cg_integer_recovery" => solver.recover_integer_solution,
        "cg_iteration_log" => iteration_log,
        # Excludes integer recovery, unlike OptResult.runtime_sec.
        "cg_lp_loop_sec" => lp_loop_sec,
        "cg_integer_recovery_sec" => 0.0,
        "cg_pricing_stats" => copy(get(JuMP.object_dictionary(m),
            :label_setting_pricing_stats, Any[])),
        # Copied off the model HERE, before integer recovery below replaces `m` with a
        # freshly built one -- the rebuilt model carries an empty stats vector, so anything
        # left only on the model is invisible to exactly the runs that use recovery.
        "cg_relaxed_cluster_guide_stats" => copy(get(JuMP.object_dictionary(m),
            :relaxed_cluster_guide_stats, Any[])),
    )
    if solver.recover_integer_solution && JuMP.termination_status(m) == MOI.OPTIMAL
        metadata["cg_lp_objective_value"] = JuMP.objective_value(m)
        t0 = time()
        build_result = integer_recovery_build(build_result, mapping, m)
        m = build_result.model
        _apply_solver_config!(m, solver.config)
        optimize!(m)
        metadata["cg_integer_recovery_sec"] = time() - t0
    end
    runtime_sec = time() - start_time

    # `converged` is the ONLY thing that makes this master's optimum the problem's
    # optimum: it means pricing exhausted, i.e. no negative-reduced-cost column remains
    # outside the pool. Every other exit (total budget, iteration cap, de-dup stall,
    # inconclusive pricing) leaves the pool possibly incomplete, so the incumbent is a
    # valid upper bound and nothing more -- `_solve_status` reports SOLVE_FEASIBLE there
    # instead of the SOLVE_OPTIMAL the restricted master would otherwise claim.
    # `converged` means pricing exhausted the universe it was searching, and that is what
    # `certified` reports -- including for a restricted pricer such as `:station_simple`,
    # whose OPTIMAL is an optimum over elementary routes only. The scope of the claim is
    # carried alongside it in `cg_optimality_scope`/`cg_pricing_universe_restricted`
    # rather than folded into the status, so a caller reading the status gets the same
    # meaning it always had ("pricing exhausted") and a caller who needs to know which
    # universe was exhausted can ask.
    return _package_result(build_result, m, runtime_sec;
                           metadata=metadata, certified=converged)
end

# ── hooks (implemented per Problem/Formulation, dispatching on `mapping`'s concrete
# type) ──────────────────────────────────────────────────────────────────────────────

function extract_duals(build_result::BuildResult, mapping, m::JuMP.Model)
    throw(MethodError(extract_duals, (build_result, mapping, m)))
end

"""
    cg_pricing_mode(build_result, mapping, m) -> Union{Nothing, Symbol}
    set_cg_pricing_mode!(build_result, mapping, m, mode::Symbol)

Optional hook pair supporting `CGSolver.warm_start_pricing_mode`. A formulation whose
pricer is selectable implements both: the getter returns the mode currently in force, the
setter switches it mid-solve. The defaults make the feature inert -- `cg_pricing_mode`
returns `nothing`, meaning "this formulation has no selectable pricer", and the loop then
refuses a `warm_start_pricing_mode` rather than silently ignoring it.

The switch has to reach the *model*, not the formulation object: hooks only ever receive
`build_result`/`mapping`/`m`, and the pricer is chosen per pricing call from state stashed
on `m` at build time.
"""
cg_pricing_mode(build_result::BuildResult, mapping, m::JuMP.Model) = nothing

function set_cg_pricing_mode!(build_result::BuildResult, mapping, m::JuMP.Model, mode::Symbol)
    throw(MethodError(set_cg_pricing_mode!, (build_result, mapping, m, mode)))
end

"""
    _cg_pricing_universe_is_restricted(mode) -> Bool
    _cg_optimality_scope(mode) -> String

Whether pricing in `mode` searches a strict subset of the formulation's route universe,
and a label for the scope of any optimality claim made in it.

A run finishing in a restricted mode still reports `SOLVE_OPTIMAL` when its pricing
exhausted -- the status keeps its usual meaning, "no improving column remains in the
universe that was searched". What changes is the *scope* of that statement:
`:station_simple` exhausts elementary routes only, so its optimum can be beaten by a
column that revisits a station (`o->d->o` serving both `o->d` and `d->o` is the minimal
case). These two functions exist so that scope travels with every result instead of being
something the reader has to infer from the formulation, and `cg_optimality_scope` is the
key to grep for when auditing whether a certified number is a full-universe optimum.
"""
_cg_pricing_universe_is_restricted(mode::Union{Nothing, Symbol}) =
    mode === :station_simple || mode === :relaxed_cluster_guided

function _cg_optimality_scope(mode::Union{Nothing, Symbol})
    mode === :station_simple && return "elementary_routes_only"
    # The relaxation-guided pricer searches only the stations inside the clusters its
    # relaxed optimum visited, and that subset is re-derived from the duals every round --
    # so exhausting it says nothing about the stations left out, and says it about a
    # different subset each iteration.
    mode === :relaxed_cluster_guided && return "relaxed_cluster_station_subset_only"
    return "full_route_universe"
end

function price_columns(build_result::BuildResult, mapping, m::JuMP.Model, duals, solver::CGSolver;
        time_limit_sec::Real=solver.pricing_time_limit_sec)
    throw(MethodError(price_columns, (build_result, mapping, m, duals, solver)))
end

"""
    cg_certification_supported(build_result, mapping, m, mode) -> Bool
    cg_certification_round(build_result, mapping, m, duals, solver, mode; time_limit_sec)
        -> (; certified::Bool, ...)

Optional hook pair supporting `CGSolver.certification_pricing_mode`: a *relaxation*
pricer, whose only job is to answer "can an improving column still exist for these duals?"

This is a different contract from `price_columns`, not a variant of it. A certification
pricer searches a relaxed problem whose solutions need not correspond to real columns, so
it returns no columns at all -- only a bit. What makes that bit worth having is the
direction of the relaxation: it must lower-bound the real pricing problem's minimum
reduced cost, so

    no relaxed solution below -reduced_cost_tol  =>  no real column below it either

and the loop can stop *certified* without ever running the expensive exhaustive search
that would otherwise be needed to prove the same thing. A `false` proves nothing (the
relaxation may simply be loose), and the loop falls through to `price_columns` unchanged.

The result must have a `certified::Bool` field; anything else on it is the pricer's own
diagnostics. The defaults make the feature inert: `cg_certification_supported` returns
`false`, so the loop refuses a `certification_pricing_mode` rather than silently ignoring
it. Only `AggregateODRouteJointRoutingAssignmentFormulation` implements the pair today,
for `:relaxed_cluster`
(`label_setting/joint_routing_assignment/relaxed_cluster/certify.jl`).
"""
cg_certification_supported(build_result::BuildResult, mapping, m::JuMP.Model, mode::Symbol) = false

function cg_certification_round(build_result::BuildResult, mapping, m::JuMP.Model, duals,
        solver::CGSolver, mode::Symbol; time_limit_sec::Real)
    throw(MethodError(cg_certification_round, (build_result, mapping, m, duals, solver, mode)))
end

"""
    _cg_materialize_certification_columns(build_result, mapping, m, duals, candidates)

Turn the candidates a failed certification attempt harvested into real columns, via the
same path a pricing round uses (`_materialize_pricing_columns` -- same id allocation, same
`_pricing_verify_column` cross-check against the master's own duals), so a harvested column
is indistinguishable from a priced one.

Empty in, empty out: the plain `:relaxed_cluster` round never harvests, and neither does a
successful attempt, so this is a no-op unless `:relaxed_cluster_nogood` actually refuted
something. Generic over formulations rather than dispatched, because the candidates already
carry the search context that knows how to build their columns.
"""
function _cg_materialize_certification_columns(
    build_result::BuildResult, mapping, m::JuMP.Model, duals, candidates,
)
    isempty(candidates) && return Any[]
    return _materialize_pricing_columns(
        m[:aggregate_od_route_formulation], mapping, m, duals, candidates,
    )
end

"""
    _cg_pricing_exhausted(m) -> Bool

Did the last pricing round prove no negative-reduced-cost column remains? Formulations
record this on the model as `:label_setting_pricing_exhausted`; a model that never sets
it is treated as exhausted, preserving the pre-two-tier behaviour for pricers with no
notion of a time limit.
"""
function _cg_pricing_exhausted(m::JuMP.Model)
    key = :label_setting_pricing_exhausted
    return !haskey(JuMP.object_dictionary(m), key) || Bool(m[key])
end

function add_columns!(build_result::BuildResult, mapping, m::JuMP.Model, columns)::Int
    throw(MethodError(add_columns!, (build_result, mapping, m, columns)))
end

function integer_recovery_build(build_result::BuildResult, mapping, m::JuMP.Model)::BuildResult
    throw(MethodError(integer_recovery_build, (build_result, mapping, m)))
end
