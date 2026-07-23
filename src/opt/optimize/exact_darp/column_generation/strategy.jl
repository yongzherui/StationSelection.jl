export ExactDARPRouteColumnGenerationStrategy

struct ExactDARPRouteColumnGenerationStrategy <: AbstractIterativeSolveStrategy
    config::ExactDARPRouteColumnGenerationConfig
end

function initialize_iteration_state(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData
)::ExactDARPRouteColumnGenerationState
    base = _build_exact_darp_route_base(model, data)
    route_pool = initialize_route_pool(
        strategy.config.init_spec,
        data,
        base.Q_s_t,
        base.valid_jk_pairs;
        vehicle_capacity=model.vehicle_capacity,
        route_generation_method=model.route_generation_method,
        iterative_config=model.iterative_route_generation_config,
        max_detour_time=model.max_detour_time,
        max_detour_ratio=model.max_detour_ratio,
        stop_dwell_time=model.stop_dwell_time,
        initial_generated_max_route_length=nothing,
    )
    return ExactDARPRouteColumnGenerationState(route_pool, nothing, nothing)
end

function _apply_priced_columns!(
    route_pool::ExactDARPRouteBucketPoolsState,
    columns::Vector{ExactDARPRoutePricedColumn},
)::Int
    added = 0
    for column in columns
        bucket_state = get(route_pool.bucket_states, (column.scenario_idx, column.time_id), nothing)
        isnothing(bucket_state) && continue
        _, inserted = _insert_route_variant!(
            route_pool,
            bucket_state,
            column.route,
            column.alpha_profile,
            :column_generation_exact;
            protect_direct=false,
        )
        bucket_state.current_generated_max_route_length = max(
            bucket_state.current_generated_max_route_length,
            length(column.route.station_indices),
        )
        added += inserted ? 1 : 0
    end
    return added
end

function run_iteration_subproblem(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
)
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
        route_pool_state=state.route_pool,
        restricted_master=true,
    )
end

iteration_state_size(strategy::ExactDARPRouteColumnGenerationStrategy, state::ExactDARPRouteColumnGenerationState)::Int =
    sum(length(bucket.routes_by_id) for bucket in values(state.route_pool.bucket_states))

function update_iteration_state!(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState,
    result::OptResult,
    iteration::Int
)
    duals = extract_exact_darp_route_cg_duals(result.model)
    positive_duals = [v for v in values(duals.route_capacity) if v > 0.0]
    raw_duals = collect(values(duals.raw_route_capacity))
    @info "exact_darp_route_cg: extracted restricted-master duals" iteration=iteration dual_count=length(duals.route_capacity) positive_dual_count=length(positive_duals) max_price=(isempty(positive_duals) ? 0.0 : maximum(positive_duals)) min_raw_dual=(isempty(raw_duals) ? nothing : minimum(raw_duals)) max_raw_dual=(isempty(raw_duals) ? nothing : maximum(raw_duals))
    _cg_log(
        "duals_extracted";
        iteration=iteration,
        dual_count=length(duals.route_capacity),
        positive_dual_count=length(positive_duals),
        max_price=isempty(positive_duals) ? 0.0 : round(maximum(positive_duals); digits=6),
        min_raw_dual=isempty(raw_duals) ? nothing : round(minimum(raw_duals); digits=6),
        max_raw_dual=isempty(raw_duals) ? nothing : round(maximum(raw_duals); digits=6),
    )
    pricing_result = solve_exact_darp_route_pricing(
        model,
        data,
        state,
        duals;
        rc_tolerance=strategy.config.rc_tolerance,
        max_columns=strategy.config.max_columns_per_iteration,
        time_limit_sec=strategy.config.pricing_time_limit_sec,
    )
    state.last_duals = duals
    state.last_pricing_result = pricing_result
    inserted_count = _apply_priced_columns!(state.route_pool, pricing_result.columns)
    pricing_result.metadata["inserted_count"] = inserted_count
    @info "exact_darp_route_cg: pricing iteration complete" iteration=iteration pricing_status=pricing_result.status inserted_count=inserted_count total_negative_columns=get(pricing_result.metadata, "total_negative_columns", 0) novel_negative_columns=get(pricing_result.metadata, "novel_negative_columns", 0) message=pricing_result.message
    _cg_log(
        "iteration_pricing_done";
        iteration=iteration,
        pricing_status=pricing_result.status,
        inserted_count=inserted_count,
        total_negative_columns=get(pricing_result.metadata, "total_negative_columns", 0),
        novel_negative_columns=get(pricing_result.metadata, "novel_negative_columns", 0),
    )

    return (
        added_count=inserted_count,
        removed_count=0,
        dual_count=length(duals.route_capacity),
        positive_dual_count=length(positive_duals),
        max_dual_price=isempty(positive_duals) ? 0.0 : maximum(positive_duals),
        min_raw_dual=isempty(raw_duals) ? nothing : minimum(raw_duals),
        max_raw_dual=isempty(raw_duals) ? nothing : maximum(raw_duals),
        pricing_status=pricing_result.status,
    )
end

function build_iteration_metadata(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState,
    result::OptResult,
    update_info,
    iteration::Int
)::Dict{String, Any}
    pricing_result = state.last_pricing_result
    return Dict{String, Any}(
        "dual_count" => get(update_info, :dual_count, 0),
        "positive_dual_count" => get(update_info, :positive_dual_count, 0),
        "max_dual_price" => get(update_info, :max_dual_price, 0.0),
        "min_raw_dual" => get(update_info, :min_raw_dual, nothing),
        "max_raw_dual" => get(update_info, :max_raw_dual, nothing),
        "pricing_status" => String(get(update_info, :pricing_status, :unknown)),
        "new_columns" => get(update_info, :added_count, 0),
        "pricing_message" => isnothing(pricing_result) ? "" : pricing_result.message,
        "pricing_metadata" => isnothing(pricing_result) ? Dict{String, Any}() : pricing_result.metadata,
        "restricted_master" => get(result.metadata, "restricted_master", false),
        "lp_objective_value" => get(result.metadata, "lp_objective_value", result.objective_value),
        "lp_objective_bound" => get(result.metadata, "lp_objective_bound", nothing),
        "solver" => get(result.metadata, "solver", Dict{String, Any}()),
    )
end

function should_stop_iteration(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState,
    result::OptResult,
    history::Vector{IterativeSolveIterationSummary},
    iteration::Int
)::Union{Nothing, String}
    pricing_result = state.last_pricing_result
    if !isnothing(pricing_result) && pricing_result.status == :time_limit
        return "pricing_time_limit"
    end
    if !isnothing(pricing_result) && get(pricing_result.metadata, "inserted_count", 0) == 0
        return "no_negative_reduced_cost_column"
    end
    if iteration >= strategy.config.max_iterations
        return "max_iterations"
    end
    return nothing
end

function finalize_iterative_result!(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    final_result::OptResult,
    history::Vector{IterativeSolveIterationSummary},
    final_state::ExactDARPRouteColumnGenerationState,
    convergence_reason::String
)
    final_result.metadata["exact_darp_route_column_generation"] = Dict(
        "convergence_reason" => convergence_reason,
        "iteration_count" => length(history),
        "seed_route_count" => isempty(history) ? iteration_state_size(strategy, final_state) : history[1].state_size_before,
        "final_route_count" => iteration_state_size(strategy, final_state),
        "pricing_status" => isnothing(final_state.last_pricing_result) ? "not_run" : String(final_state.last_pricing_result.status),
        "new_columns_last_iter" => isnothing(final_state.last_pricing_result) ? 0 : get(final_state.last_pricing_result.metadata, "inserted_count", 0),
        "total_negative_columns_last_iter" => isnothing(final_state.last_pricing_result) ? 0 : get(final_state.last_pricing_result.metadata, "total_negative_columns", 0),
        "novel_negative_columns_last_iter" => isnothing(final_state.last_pricing_result) ? 0 : get(final_state.last_pricing_result.metadata, "novel_negative_columns", 0),
        "last_pricing_metadata" => isnothing(final_state.last_pricing_result) ? Dict{String, Any}() : final_state.last_pricing_result.metadata,
        "iterations" => [
            Dict{String, Any}(
                "iteration" => it.iteration,
                "lp_objective_value" => it.objective_value,
                "state_size_before" => it.state_size_before,
                "state_size_after" => it.state_size_after,
                "added_count" => it.added_count,
                "removed_count" => it.removed_count,
                "state_change_ratio" => it.state_change_ratio,
                "objective_improvement" => it.objective_improvement,
                "objective_delta" => it.objective_delta,
                "relative_objective_improvement" => it.relative_objective_improvement,
                "metadata" => it.metadata,
            ) for it in history
        ],
    )
    return nothing
end

function export_initial_iteration_state(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState,
    output_dir::String
)
    export_exact_darp_route_bucket_pools_state(
        state.route_pool,
        joinpath(output_dir, "cg_iteration_00_initial");
        array_idx_to_station_id=data.array_idx_to_station_id
    )
    return nothing
end

function export_iteration_state(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState,
    output_dir::String,
    iteration::Int
)
    strategy.config.export_iteration_artifacts || return nothing
    export_exact_darp_route_bucket_pools_state(
        state.route_pool,
        joinpath(output_dir, "cg_iteration_" * lpad(string(iteration), 2, '0'));
        array_idx_to_station_id=data.array_idx_to_station_id
    )
    return nothing
end

function export_final_iteration_state(
    strategy::ExactDARPRouteColumnGenerationStrategy,
    model::ExactDARPRouteModel,
    data::StationSelectionData,
    state::ExactDARPRouteColumnGenerationState,
    output_dir::String
)
    export_exact_darp_route_bucket_pools_state(
        state.route_pool,
        joinpath(output_dir, "cg_route_pool_final");
        array_idx_to_station_id=data.array_idx_to_station_id
    )
    return nothing
end
