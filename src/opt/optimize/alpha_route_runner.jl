function _run_opt_alpha_single_impl(
    model::AlphaRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    route_pool_state::Union{AlphaRouteBucketPoolsState, Nothing}=nothing,
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
        build_alpha_route_restricted_master(
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
        warm_start_solution = get_warm_start_solution(
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

    runtime_sec = Dates.value(now() - start_time) / 1000
    if restricted_master
        build_result.metadata["restricted_master"] = true
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
            "solve_time_sec" => solve_time_sec
        )
    )
end

function run_alpha_route_iterative(
    model::AlphaRouteModel,
    data::StationSelectionData,
    config::AlphaRouteRunnerConfig;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    output_dir::Union{String, Nothing}=nothing
)::AlphaRouteRunnerResult
    iterative_result = run_iterative_solve(
        AlphaRouteIterativeStrategy(config),
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
        AlphaRouteIterationSummary(
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

    return AlphaRouteRunnerResult(
        iterative_result.final_result,
        alpha_iterations,
        iterative_result.convergence_reason,
        iterative_result.final_state,
    )
end

function run_opt(
    model::AlphaRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    route_pool_state::Union{AlphaRouteBucketPoolsState, Nothing}=nothing,
    solve_strategy::Union{AbstractSolveStrategy, Nothing}=nothing,
    output_dir::Union{String, Nothing}=nothing
)
    if solve_strategy isa AbstractIterativeSolveStrategy
        isnothing(route_pool_state) || throw(ArgumentError("route_pool_state and solve_strategy cannot both be provided to AlphaRouteModel run_opt"))
        return run_iterative_solve(
            solve_strategy,
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
        ).final_result
    end

    if do_optimize
        feasibility_issue = check_model_feasibility(model, data)
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
                Dict{String, Any}("feasibility_issue" => feasibility_issue)
            )
        end
    end

    return _run_opt_alpha_single_impl(
        model,
        data;
        optimizer_env=optimizer_env,
        silent=silent,
        show_counts=show_counts,
        do_optimize=do_optimize,
        warm_start=warm_start,
        check_feasibility=check_feasibility,
        mip_gap=mip_gap,
        route_pool_state=route_pool_state
    )
end
