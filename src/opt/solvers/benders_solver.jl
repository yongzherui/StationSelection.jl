export BendersSolver

"""
    BendersSolver <: AbstractSolver

Benders decomposition outer loop: repeatedly solve the master (`build_result`'s
model), solve a subproblem at the master's current incumbent first-stage decision, and
add a cut derived from that subproblem back into the master, until the subproblem
certifies convergence or `max_iterations` is reached.

Relies on four formulation-specific hooks -- implemented per `AbstractFormulation` (or
per `AbstractProblem`), not here:

    extract_incumbent(build_result, mapping, m) -> incumbent
    solve_subproblem(build_result, mapping, m, incumbent, solver::BendersSolver) -> subproblem_result
    benders_converged(build_result, mapping, m, subproblem_result, solver::BendersSolver) -> Bool
    add_benders_cut!(build_result, mapping, m, subproblem_result, solver::BendersSolver)

`mapping` (`build_result.mapping`) is passed as its own positional argument, not just
read off `build_result`, so that formulation-specific methods can dispatch on its
concrete type (e.g. `mapping::AggregateODRouteMap`) -- mirrors `CGSolver`'s identical
hook-dispatch pattern (`opt/solvers/cg_solver.jl`), see that file's docstring for why
dispatching on `build_result::BuildResult` alone can't distinguish formulations.

`extract_incumbent` reads whatever the master's first-stage decision is (e.g. `y`
values) off the just-solved master. `benders_converged` decides whether the current
master/subproblem pair already proves optimality (or infeasibility resolution);
`add_benders_cut!` mutates the master in place otherwise.
"""
struct BendersSolver <: AbstractSolver
    config::SolverOptions
    max_iterations::Int
    optimality_tol::Float64

    function BendersSolver(;
            config::SolverOptions=SolverOptions(),
            max_iterations::Int=1_000,
            optimality_tol::Number=1e-6,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        optimality_tol >= 0 || throw(ArgumentError("optimality_tol must be non-negative"))
        new(config, max_iterations, Float64(optimality_tol))
    end
end

function optimize_model(build_result::BuildResult, solver::BendersSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)
    mapping = build_result.mapping

    start_time = time()
    iterations_run = 0
    for iteration in 1:solver.max_iterations
        iterations_run = iteration
        optimize!(m)

        JuMP.termination_status(m) == MOI.OPTIMAL || break

        incumbent = extract_incumbent(build_result, mapping, m)
        subproblem_result = solve_subproblem(build_result, mapping, m, incumbent, solver)

        benders_converged(build_result, mapping, m, subproblem_result, solver) && break

        add_benders_cut!(build_result, mapping, m, subproblem_result, solver)
    end
    runtime_sec = time() - start_time

    return _package_result(
        build_result, m, runtime_sec;
        metadata=Dict{String, Any}("benders_iterations" => iterations_run),
    )
end

# ── hooks (implemented per Problem/Formulation, dispatching on `mapping`'s concrete
# type) ──────────────────────────────────────────────────────────────────────────────

function extract_incumbent(build_result::BuildResult, mapping, m::JuMP.Model)
    throw(MethodError(extract_incumbent, (build_result, mapping, m)))
end

function solve_subproblem(build_result::BuildResult, mapping, m::JuMP.Model, incumbent, solver::BendersSolver)
    throw(MethodError(solve_subproblem, (build_result, mapping, m, incumbent, solver)))
end

function benders_converged(build_result::BuildResult, mapping, m::JuMP.Model, subproblem_result, solver::BendersSolver)::Bool
    throw(MethodError(benders_converged, (build_result, mapping, m, subproblem_result, solver)))
end

function add_benders_cut!(build_result::BuildResult, mapping, m::JuMP.Model, subproblem_result, solver::BendersSolver)
    throw(MethodError(add_benders_cut!, (build_result, mapping, m, subproblem_result, solver)))
end
