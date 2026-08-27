export CGSolver

"""
    CGSolver <: AbstractSolver

Column-generation outer loop: repeatedly solve the restricted master (`build_result`'s
model), price new columns against its duals, and add any improving ones back into the
master, until pricing finds nothing improving or `max_iterations` is reached.

`pricing_time_limit_sec` is the wall-clock budget for each scenario's label
search in one pricing round. An empty result from a search that hits this
limit does not set `cg_converged=true`; only exhausted pricing can certify
that the restricted master needs no further improving columns.

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
    initial_columns::Union{Nothing, AbstractVector}
    recover_integer_solution::Bool

    function CGSolver(;
            config::SolverOptions=SolverOptions(),
            max_iterations::Int=1_000,
            reduced_cost_tol::Number=1e-6,
            pricing_time_limit_sec::Number=30.0,
            initial_columns::Union{Nothing, AbstractVector}=nothing,
            recover_integer_solution::Bool=false,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        reduced_cost_tol >= 0 || throw(ArgumentError("reduced_cost_tol must be non-negative"))
        pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
        new(
            config, max_iterations, Float64(reduced_cost_tol),
            Float64(pricing_time_limit_sec), initial_columns, recover_integer_solution,
        )
    end
end

function optimize_model(build_result::BuildResult, solver::CGSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)
    mapping = build_result.mapping

    start_time = time()
    iterations_run = 0
    converged = false
    # One row per CG iteration, exposed as metadata["cg_iteration_log"]. Wall time is
    # split into the master LP solve, the pricing call, and column insertion, so a run
    # that is master-bound can be told apart from one that is pricing-bound without
    # re-running under a profiler. `columns_added` is what pricing actually returned
    # this iteration (the pool grows by that much); `cumulative_columns_added` excludes
    # any seed columns the model was built with.
    iteration_log = NamedTuple[]
    cumulative_columns_added = 0
    for iteration in 1:solver.max_iterations
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
            ))
            break
        end
        master_objective = JuMP.objective_value(m)

        duals = extract_duals(build_result, mapping, m)
        t0 = time()
        new_columns = price_columns(build_result, mapping, m, duals, solver)
        pricing_sec = time() - t0

        if isnothing(new_columns) || isempty(new_columns)
            # An empty result certifies convergence only when every underlying
            # label search exhausted its frontier. A time-limited pricing pass
            # can also return no columns; treating that as exhaustion silently
            # turns a pricing timeout into a false optimality certificate.
            key = :label_setting_pricing_exhausted
            converged = !haskey(JuMP.object_dictionary(m), key) || Bool(m[key])
            push!(iteration_log, (
                iteration=iteration, master_sec=master_sec, pricing_sec=pricing_sec,
                add_columns_sec=0.0, columns_added=0, columns_accepted=0,
                cumulative_columns_added=cumulative_columns_added,
                master_objective=master_objective, master_status=string(status),
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
        ))

        # Pricing returned improving columns but every one was de-duplicated away, so the
        # master is unchanged and the next iteration would extract identical duals and
        # re-find the identical columns -- a livelock that otherwise burns every remaining
        # iteration (see notes/2026-08-25_study6_cg_livelock_stale_tau_columns.md).
        # `converged` deliberately stays false: pricing DID find a negative-reduced-cost
        # column, so this is a failure to make progress, never an optimality certificate.
        columns_accepted == 0 && break
    end
    lp_loop_sec = time() - start_time

    metadata = Dict{String, Any}(
        "cg_iterations" => iterations_run,
        "cg_converged" => converged,
        "cg_pricing_exhausted" => converged,
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

function price_columns(build_result::BuildResult, mapping, m::JuMP.Model, duals, solver::CGSolver)
    throw(MethodError(price_columns, (build_result, mapping, m, duals, solver)))
end

function add_columns!(build_result::BuildResult, mapping, m::JuMP.Model, columns)::Int
    throw(MethodError(add_columns!, (build_result, mapping, m, columns)))
end

function integer_recovery_build(build_result::BuildResult, mapping, m::JuMP.Model)::BuildResult
    throw(MethodError(integer_recovery_build, (build_result, mapping, m)))
end
