export CGSolver

"""
    CGSolver <: AbstractSolver

Column-generation outer loop: repeatedly solve the restricted master (`build_result`'s
model), price new columns against its duals, and add any improving ones back into the
master, until pricing finds nothing improving or `max_iterations` is reached.

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
"""
struct CGSolver <: AbstractSolver
    config::SolverOptions
    max_iterations::Int
    reduced_cost_tol::Float64
    initial_columns::Union{Nothing, AbstractVector}
    recover_integer_solution::Bool

    function CGSolver(;
            config::SolverOptions=SolverOptions(),
            max_iterations::Int=1_000,
            reduced_cost_tol::Number=1e-6,
            initial_columns::Union{Nothing, AbstractVector}=nothing,
            recover_integer_solution::Bool=false,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        reduced_cost_tol >= 0 || throw(ArgumentError("reduced_cost_tol must be non-negative"))
        new(config, max_iterations, Float64(reduced_cost_tol), initial_columns, recover_integer_solution)
    end
end

function optimize_model(build_result::BuildResult, solver::CGSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)
    mapping = build_result.mapping

    start_time = time()
    iterations_run = 0
    converged = false
    for iteration in 1:solver.max_iterations
        iterations_run = iteration
        optimize!(m)

        JuMP.termination_status(m) == MOI.OPTIMAL || break

        duals = extract_duals(build_result, mapping, m)
        new_columns = price_columns(build_result, mapping, m, duals, solver)
        if isnothing(new_columns) || isempty(new_columns)
            converged = true
            break
        end

        add_columns!(build_result, mapping, m, new_columns)
    end

    metadata = Dict{String, Any}(
        "cg_iterations" => iterations_run,
        "cg_converged" => converged,
        "cg_integer_recovery" => solver.recover_integer_solution,
    )
    if solver.recover_integer_solution && JuMP.termination_status(m) == MOI.OPTIMAL
        metadata["cg_lp_objective_value"] = JuMP.objective_value(m)
        build_result = integer_recovery_build(build_result, mapping, m)
        m = build_result.model
        _apply_solver_config!(m, solver.config)
        optimize!(m)
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
