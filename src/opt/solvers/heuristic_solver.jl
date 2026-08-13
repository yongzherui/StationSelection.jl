export HeuristicDispatchSolver

"""
    HeuristicDispatchSolver <: AbstractSolver

Thin dispatch shell for heuristic solve strategies: delegates entirely to
`run_heuristic!`, a formulation-specific hook -- implemented per `AbstractFormulation`
(or per `AbstractProblem`), not here -- that drives `build_result`'s model however that
particular heuristic works (iterative pool pruning/expansion, warm-start-then-fix,
...). `method` is an open-ended tag a formulation's `run_heuristic!` method can
dispatch on when one formulation supports more than one heuristic strategy.
"""
struct HeuristicDispatchSolver <: AbstractSolver
    config::SolverOptions
    method::Symbol
end

HeuristicDispatchSolver(method::Symbol=:default; config::SolverOptions=SolverOptions()) =
    HeuristicDispatchSolver(config, method)

function run_heuristic!(build_result::BuildResult, m::JuMP.Model, solver::HeuristicDispatchSolver)
    throw(MethodError(run_heuristic!, (build_result, m, solver)))
end

function optimize_model(build_result::BuildResult, solver::HeuristicDispatchSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)

    start_time = time()
    run_heuristic!(build_result, m, solver)
    runtime_sec = time() - start_time

    return _package_result(build_result, m, runtime_sec)
end
