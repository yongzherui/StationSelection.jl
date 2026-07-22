function _safe_jump_metric(f, m)
    try
        value = f(m)
        if value isa Number
            isfinite(Float64(value)) || return nothing
        end
        return value
    catch
        return nothing
    end
end

function _safe_moi_metric(m, attr)
    try
        return MOI.get(JuMP.backend(m), attr)
    catch
        return nothing
    end
end

function _solve_metrics(m::Model, objective_value)
    objective_bound = _safe_jump_metric(JuMP.objective_bound, m)
    relative_gap = _safe_jump_metric(JuMP.relative_gap, m)
    gap_from_bound = if objective_value isa Number && objective_bound isa Number && !iszero(Float64(objective_value))
        abs(Float64(objective_value) - Float64(objective_bound)) / abs(Float64(objective_value))
    else
        nothing
    end
    return Dict{String, Any}(
        "objective_bound" => objective_bound,
        "relative_gap" => relative_gap,
        "gap_from_objective_bound" => gap_from_bound,
        "simplex_iterations" => _safe_moi_metric(m, MOI.SimplexIterations()),
        "barrier_iterations" => _safe_moi_metric(m, MOI.BarrierIterations()),
        "node_count" => _safe_moi_metric(m, MOI.NodeCount()),
    )
end

function _run_opt_alpha_single_impl(
    model::ExactDARPRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    route_pool_state::Union{ExactDARPRouteBucketPoolsState, Nothing}=nothing,
    restricted_master::Bool=false
)
    start_time = now()

    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    build_start = now()
    if restricted_master && isnothing(route_pool_state)
        throw(ArgumentError("route_pool_state is required for restricted_master=true"))
    end

    build_result = restricted_master ?
        build_exact_darp_route_restricted_master(
            model,
            data,
            route_pool_state;
            optimizer_env=optimizer_env
        ) :
        build_model(
            model,
            data;
            optimizer_env=optimizer_env,
            route_pool_state=route_pool_state
        )
    m = build_result.model
    build_time_sec = Dates.value(now() - build_start) / 1000

    if show_counts
        if !isempty(build_result.counts.variables)
            _print_counts("Variables", build_result.counts.variables)
        end
        if !isempty(build_result.counts.constraints)
            _print_counts("Constraints", build_result.counts.constraints)
        end
        if !isempty(build_result.counts.extras)
            _print_counts("Extras", build_result.counts.extras)
        end
    end

    silent && set_silent(m)
    !isnothing(mip_gap) && set_optimizer_attribute(m, "MIPGap", mip_gap)

    warm_start_solution = nothing
    warm_start_time_sec = nothing
    if warm_start
        ws_start = now()
        warm_start_solution = get_exact_darp_route_warm_start_solution(
            model, data, build_result;
            optimizer_env=optimizer_env, silent=false,
            check_feasibility=check_feasibility
        )
        warm_start_time_sec = Dates.value(now() - ws_start) / 1000
        if !isnothing(warm_start_solution)
            _apply_warm_start!(m, warm_start_solution)
            if check_feasibility
                _verify_start_completeness(m)
                _check_warm_start_feasibility(m, warm_start_solution)
            end
        end
    end

    solve_time_sec = nothing
    if do_optimize
        solve_start = now()
        optimize!(m)
        solve_time_sec = Dates.value(now() - solve_start) / 1000
    end

    term_status = do_optimize ? JuMP.termination_status(m) : MOI.OPTIMIZE_NOT_CALLED
    obj = nothing
    solution = nothing
    if term_status == MOI.OPTIMAL
        obj = JuMP.objective_value(m)
        x_val = _value_recursive(m[:x])
        y_val = _value_recursive(m[:y])
        solution = (x_val, y_val)
    end
    solver_metrics = _solve_metrics(m, obj)

    runtime_sec = Dates.value(now() - start_time) / 1000
    if restricted_master
        build_result.metadata["restricted_master"] = true
        build_result.metadata["relax_integrality"] = true
        build_result.metadata["lp_objective_value"] = obj
        build_result.metadata["lp_objective_bound"] = get(solver_metrics, "objective_bound", nothing)
    end
    return OptResult(
        term_status,
        obj,
        solution,
        runtime_sec,
        m,
        build_result.mapping,
        build_result.detour_combos,
        build_result.counts,
        warm_start_solution,
        Dict{String, Any}(
            "build_time_sec" => build_time_sec,
            "warm_start_time_sec" => warm_start_time_sec,
            "solve_time_sec" => solve_time_sec,
            "solver" => solver_metrics,
        )
    )
end

function run_exact_darp_route_iterative(
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    config::ExactDARPRouteRunnerConfig;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    output_dir::Union{String, Nothing}=nothing
)::ExactDARPRouteRunnerResult
    iterative_result = run_iterative_solve(
        ExactDARPRouteIterativeStrategy(config),
        model,
        data;
        optimizer_env=optimizer_env,
        silent=silent,
        show_counts=show_counts,
        do_optimize=do_optimize,
        warm_start=warm_start,
        check_feasibility=check_feasibility,
        mip_gap=mip_gap,
        output_dir=output_dir,
    )

    alpha_iterations = [
        ExactDARPRouteIterationSummary(
            it.iteration,
            it.objective_value,
            it.state_size_before,
            it.state_size_after,
            get(it.metadata, "active_route_count", 0),
            it.added_count,
            it.removed_count,
            it.state_change_ratio,
            it.objective_improvement,
            it.objective_delta,
            it.relative_objective_improvement,
            get(it.metadata, "build_time_sec", nothing),
            get(it.metadata, "warm_start_time_sec", nothing),
            get(it.metadata, "solve_time_sec", nothing),
            get(it.metadata, "runtime_sec", nothing),
        ) for it in iterative_result.iterations
    ]

    return ExactDARPRouteRunnerResult(
        iterative_result.final_result,
        alpha_iterations,
        iterative_result.convergence_reason,
        iterative_result.final_state,
    )
end

function _exact_darp_route_runner_config(solver::HeuristicSolver)::ExactDARPRouteRunnerConfig
    return ExactDARPRouteRunnerConfig(
        solver.init_spec;
        iterative=true,
        max_iterations=solver.max_iterations,
        route_length_schedule=solver.route_length_schedule,
        prune_enabled=solver.prune_enabled,
        expand_enabled=solver.expand_enabled,
        min_theta_to_keep=solver.min_active_value_to_keep,
        route_pool_target_size=solver.pool_target_size,
        route_pool_bucket_x_multiplier=solver.bucket_multiplier,
        random_retention_seed=solver.random_retention_seed,
        objective_improvement_tol=solver.objective_improvement_tol,
        route_pool_change_tol=solver.pool_change_tol,
        export_iteration_artifacts=solver.export_iteration_artifacts,
        enrichment=solver.enrichment,
    )
end

function _exact_darp_route_column_generation_config(
    solver::ColumnGenerationSolver,
)::ExactDARPRouteColumnGenerationConfig
    return ExactDARPRouteColumnGenerationConfig(
        max_iterations=solver.max_iterations,
        rc_tolerance=-abs(solver.reduced_cost_tol),
        max_columns_per_iteration=solver.max_columns_per_iteration,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        export_iteration_artifacts=false,
    )
end

function run_opt(
    instance::StationSelectionData,
    formulation::ExactDARPRouteModel,
    solver::DirectSolver,
)
    cfg = solver.config
    if cfg.do_optimize
        feasibility_issue = check_model_feasibility(formulation, instance)
        if !isnothing(feasibility_issue)
            return OptResult(
                MOI.INFEASIBLE,
                nothing,
                nothing,
                0.0,
                JuMP.Model(),
                EmptyStationSelectionMap(),
                nothing,
                nothing,
                nothing,
                Dict{String, Any}("feasibility_issue" => feasibility_issue),
            )
        end
    end

    return _run_opt_alpha_single_impl(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        silent=cfg.silent,
        show_counts=cfg.show_counts,
        do_optimize=cfg.do_optimize,
        warm_start=cfg.warm_start,
        check_feasibility=cfg.check_feasibility,
        mip_gap=cfg.mip_gap,
    )
end

function run_opt(
    instance::StationSelectionData,
    formulation::ExactDARPRouteModel,
    solver::ColumnGenerationSolver,
)
    cfg = solver.config
    return run_exact_darp_route_column_generation(
        formulation,
        instance,
        _exact_darp_route_column_generation_config(solver);
        optimizer_env=cfg.optimizer_env,
        silent=cfg.silent,
        show_counts=cfg.show_counts,
        do_optimize=cfg.do_optimize,
        warm_start=cfg.warm_start,
        check_feasibility=cfg.check_feasibility,
        mip_gap=cfg.mip_gap,
        output_dir=cfg.output_dir,
    ).final_result
end

function run_opt(
    instance::StationSelectionData,
    formulation::ExactDARPRouteModel,
    solver::HeuristicSolver,
)
    cfg = solver.config
    return run_exact_darp_route_iterative(
        formulation,
        instance,
        _exact_darp_route_runner_config(solver);
        optimizer_env=cfg.optimizer_env,
        silent=cfg.silent,
        show_counts=cfg.show_counts,
        do_optimize=cfg.do_optimize,
        warm_start=cfg.warm_start,
        check_feasibility=cfg.check_feasibility,
        mip_gap=cfg.mip_gap,
        output_dir=cfg.output_dir,
    ).final_result
end
