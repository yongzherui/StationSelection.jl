"""
Single dispatch choke point for `ColumnGenerationSolver` over `AggregateODRouteProblem`/
`RouteCoveringProblem`, mirroring `benders/dispatch.jl`. Resolves `solver.algorithm` (falling back
to `_default_cg_algorithm(formulation)`, which reproduces today's implicit
`formulation.assignment_policy`-based fork) and routes to the matching CG driver.
"""

"""
    _default_cg_algorithm(formulation) -> AbstractColumnGenerationAlgorithm

`FreeAggregateODAssignmentPolicy` models route to `JointRoutingAssignmentCG`; every other
`AggregateODRouteProblem` and every `RouteCoveringProblem` (whose assignments are already fixed, so
passenger-level free assignment never applies) route to `AggregateODRouteCG`.
"""
_default_cg_algorithm(formulation::AggregateODRouteProblem) =
    formulation.assignment_policy isa FreeAggregateODAssignmentPolicy ?
        JointRoutingAssignmentCG() : AggregateODRouteCG()
_default_cg_algorithm(::RouteCoveringProblem) = AggregateODRouteCG()

"""
    _resolved_cg_algorithm(solver, formulation) -> AbstractColumnGenerationAlgorithm

`solver.algorithm` if explicitly set, else `_default_cg_algorithm(formulation)`.
"""
_resolved_cg_algorithm(solver::ColumnGenerationSolver, formulation) =
    isnothing(solver.algorithm) ? _default_cg_algorithm(formulation) : solver.algorithm

function run_opt(
    instance::StationSelectionData,
    formulation::AggregateODRouteProblem,
    solver::ColumnGenerationSolver,
)
    algorithm = _resolved_cg_algorithm(solver, formulation)
    if algorithm isa JointRoutingAssignmentCG
        formulation.assignment_policy isa FreeAggregateODAssignmentPolicy || throw(ArgumentError(
            "JointRoutingAssignmentCG requires FreeAggregateODAssignmentPolicy; got " *
            "$(typeof(formulation.assignment_policy))"
        ))
        cfg = solver.config
        result = run_joint_routing_assignment_column_generation(
            formulation,
            instance;
            optimizer_env=cfg.optimizer_env,
            max_cg_iters=solver.max_iterations,
            n_candidates=solver.n_candidates,
            max_new_columns=solver.max_columns_per_iteration,
            reduced_cost_tol=solver.reduced_cost_tol,
            pricing_time_limit_sec=solver.pricing_time_limit_sec,
            ip_time_limit_sec=solver.final_ip_time_limit_sec,
            mip_gap=cfg.mip_gap,
            iteration_log_path=_aggregate_od_route_cg_log_path(
                solver, "joint_routing_assignment_cg_iterations.csv",
            ),
            column_log_path=_aggregate_od_route_cg_log_path(
                solver, "joint_routing_assignment_cg_columns.csv",
            ),
            verbose=!cfg.silent,
            silent=cfg.silent,
        )
        return result.final_result
    elseif algorithm isa AggregateODRouteCG
        formulation.assignment_policy isa FreeAggregateODAssignmentPolicy && throw(ArgumentError(
            "AggregateODRouteCG does not support FreeAggregateODAssignmentPolicy; use " *
            "JointRoutingAssignmentCG() instead"
        ))
        return _run_aggregate_od_route_column_generation_opt(instance, formulation, solver)
    end
    throw(ArgumentError("unsupported column-generation algorithm $(typeof(algorithm))"))
end

function run_opt(
    instance::StationSelectionData,
    formulation::RouteCoveringProblem,
    solver::ColumnGenerationSolver,
)
    algorithm = _resolved_cg_algorithm(solver, formulation)
    algorithm isa AggregateODRouteCG || throw(ArgumentError(
        "RouteCoveringProblem only supports AggregateODRouteCG; got $(typeof(algorithm))"
    ))
    cfg = solver.config
    result = run_aggregate_od_route_column_generation(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        verbose=!cfg.silent,
        cg_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_iterations.csv"),
        column_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_columns.csv"),
        dual_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_duals.csv"),
        max_cg_iters=solver.max_iterations,
        max_new_columns=solver.max_columns_per_iteration,
        n_candidates=solver.n_candidates,
        reduced_cost_tol=solver.reduced_cost_tol,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        ip_time_limit_sec=solver.final_ip_time_limit_sec,
        mip_gap=cfg.mip_gap,
        silent=cfg.silent,
    )
    return result.final_result
end
