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
"""
struct CGSolver <: AbstractSolver
    config::SolverOptions
    max_iterations::Int
    reduced_cost_tol::Float64
    initial_columns::Union{Nothing, AbstractVector}

    function CGSolver(;
            config::SolverOptions=SolverOptions(),
            max_iterations::Int=1_000,
            reduced_cost_tol::Number=1e-6,
            initial_columns::Union{Nothing, AbstractVector}=nothing,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        reduced_cost_tol >= 0 || throw(ArgumentError("reduced_cost_tol must be non-negative"))
        new(config, max_iterations, Float64(reduced_cost_tol), initial_columns)
    end
end

function optimize_model(build_result::BuildResult, solver::CGSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)

    start_time = time()
    iterations_run = 0
    for iteration in 1:solver.max_iterations
        iterations_run = iteration
        optimize!(m)

        JuMP.termination_status(m) == MOI.OPTIMAL || break

        mapping = build_result.mapping
        duals = extract_duals(build_result, mapping, m)
        new_columns = price_columns(build_result, mapping, m, duals, solver)
        (isnothing(new_columns) || isempty(new_columns)) && break

        add_columns!(build_result, mapping, m, new_columns)
    end
    runtime_sec = time() - start_time

    return _package_result(
        build_result, m, runtime_sec;
        metadata=Dict{String, Any}("cg_iterations" => iterations_run),
    )
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
