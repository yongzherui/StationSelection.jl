export run_exact_darp_route_column_generation

function _lp_gap_to_integer(lp_objective, integer_objective)
    (lp_objective isa Number && integer_objective isa Number) || return nothing
    denom = max(abs(Float64(integer_objective)), 1.0)
    return (Float64(integer_objective) - Float64(lp_objective)) / denom
end

function run_exact_darp_route_column_generation(
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    config::ExactDARPRouteColumnGenerationConfig;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    output_dir::Union{String, Nothing}=nothing,
)::ExactDARPRouteColumnGenerationRunnerResult
    iterative_result = run_iterative_solve(
        ExactDARPRouteColumnGenerationStrategy(config),
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

    lp_final_result = iterative_result.final_result
    final_result = lp_final_result
    if do_optimize && lp_final_result.termination_status == MOI.OPTIMAL
        @info "exact_darp_route_cg: solving final integer master" route_count=sum(length(bucket.routes_by_id) for bucket in values(iterative_result.final_state.route_pool.bucket_states))
        integer_result = _run_opt_alpha_single_impl(
            model,
            data;
            optimizer_env=optimizer_env,
            silent=silent,
            show_counts=show_counts,
            do_optimize=do_optimize,
            warm_start=warm_start,
            check_feasibility=check_feasibility,
            mip_gap=mip_gap,
            route_pool_state=iterative_result.final_state.route_pool,
            restricted_master=false,
        )

        cg_metadata = get(lp_final_result.metadata, "exact_darp_route_column_generation", Dict{String, Any}())
        lp_objective = lp_final_result.objective_value
        integer_objective = integer_result.objective_value
        cg_metadata["lp_relaxation_objective_value"] = lp_objective
        cg_metadata["integer_objective_value"] = integer_objective
        cg_metadata["lp_relaxation_gap"] = _lp_gap_to_integer(lp_objective, integer_objective)
        cg_metadata["final_integer_solve"] = Dict{String, Any}(
            "termination_status" => string(integer_result.termination_status),
            "objective_value" => integer_objective,
            "runtime_sec" => integer_result.runtime_sec,
            "build_time_sec" => get(integer_result.metadata, "build_time_sec", nothing),
            "solve_time_sec" => get(integer_result.metadata, "solve_time_sec", nothing),
            "solver" => get(integer_result.metadata, "solver", Dict{String, Any}()),
        )
        integer_result.metadata["exact_darp_route_column_generation"] = cg_metadata
        integer_result.metadata["lp_relaxation"] = Dict{String, Any}(
            "objective_value" => lp_objective,
            "solver" => get(lp_final_result.metadata, "solver", Dict{String, Any}()),
            "gap_to_integer" => cg_metadata["lp_relaxation_gap"],
        )
        final_result = integer_result
        @info "exact_darp_route_cg: final integer master complete" termination_status=integer_result.termination_status lp_objective=lp_objective integer_objective=integer_objective lp_gap=cg_metadata["lp_relaxation_gap"]
    end

    return ExactDARPRouteColumnGenerationRunnerResult(
        final_result,
        iterative_result.iterations,
        iterative_result.convergence_reason,
        iterative_result.final_state,
    )
end
