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
        new(
            config, max_iterations, Float64(reduced_cost_tol),
            Float64(pricing_time_limit_sec), Float64(certifying_pricing_time_limit_sec),
            Float64(total_time_limit_sec), parallel_scenario_pricing,
            initial_columns, recover_integer_solution,
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
            ))
            stop_reason = "master_not_optimal"
            break
        end
        master_objective = JuMP.objective_value(m)

        duals = extract_duals(build_result, mapping, m)
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
        # Why the loop stopped: "converged", "total_budget", "max_iterations",
        # "master_not_optimal", "no_columns_accepted", or "pricing_inconclusive".
        # Only "converged" makes cg_lp_objective_value a valid bound on the true optimum.
        "cg_stop_reason" => stop_reason,
        "cg_total_budget_exhausted" => budget_exhausted,
        "cg_total_time_limit_sec" => solver.total_time_limit_sec,
        "cg_certifying_rounds" => certifying_rounds,
        "cg_final_master_resolved" => final_master_resolved,
        "cg_integer_recovery" => solver.recover_integer_solution,
        "cg_iteration_log" => iteration_log,
        # Excludes integer recovery, unlike OptResult.runtime_sec.
        "cg_lp_loop_sec" => lp_loop_sec,
        "cg_integer_recovery_sec" => 0.0,
        "cg_pricing_stats" => copy(get(JuMP.object_dictionary(m),
            :label_setting_pricing_stats, Any[])),
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

    return _package_result(build_result, m, runtime_sec; metadata=metadata)
end

# ── hooks (implemented per Problem/Formulation, dispatching on `mapping`'s concrete
# type) ──────────────────────────────────────────────────────────────────────────────

function extract_duals(build_result::BuildResult, mapping, m::JuMP.Model)
    throw(MethodError(extract_duals, (build_result, mapping, m)))
end

function price_columns(build_result::BuildResult, mapping, m::JuMP.Model, duals, solver::CGSolver;
        time_limit_sec::Real=solver.pricing_time_limit_sec)
    throw(MethodError(price_columns, (build_result, mapping, m, duals, solver)))
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
