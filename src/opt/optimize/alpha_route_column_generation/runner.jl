export run_alpha_route_column_generation

function run_alpha_route_column_generation(
    model::AlphaRouteModel,
    data::StationSelectionData,
    config::AlphaRouteColumnGenerationConfig;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    output_dir::Union{String, Nothing}=nothing,
)::AlphaRouteColumnGenerationRunnerResult
    iterative_result = run_iterative_solve(
        AlphaRouteColumnGenerationStrategy(config),
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

    return AlphaRouteColumnGenerationRunnerResult(
        iterative_result.final_result,
        iterative_result.iterations,
        iterative_result.convergence_reason,
        iterative_result.final_state,
    )
end
