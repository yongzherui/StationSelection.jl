export DirectMIPSolver

"""
    DirectMIPSolver <: AbstractSolver

Solve the built model as-is with a single `optimize!` call -- no outer loop, no
formulation-specific hooks. The simplest solver: whatever `build_model(problem,
formulation, solver)` produced is handed straight to the MIP/LP solver.
"""
struct DirectMIPSolver <: AbstractSolver
    config::SolverOptions
end

DirectMIPSolver(; config::SolverOptions=SolverOptions()) = DirectMIPSolver(config)

function optimize_model(build_result::BuildResult, solver::DirectMIPSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)

    start_time = time()
    optimize!(m)
    runtime_sec = time() - start_time

    return _package_result(build_result, m, runtime_sec)
end
