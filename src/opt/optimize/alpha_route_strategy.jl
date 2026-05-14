export AlphaRouteIterativeStrategy

struct AlphaRouteIterativeStrategy <: AbstractIterativeSolveStrategy
    config::AlphaRouteRunnerConfig
end

function _alpha_route_pool_target_length(strategy::AlphaRouteIterativeStrategy)::Union{Int, Nothing}
    return isempty(strategy.config.route_length_schedule) ? nothing : first(strategy.config.route_length_schedule)
end

function initialize_iteration_state(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData
)::AlphaRouteBucketPoolsState
    base = _build_alpha_route_base(model, data)
    state = initialize_route_pool(
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
        initial_generated_max_route_length=_alpha_route_pool_target_length(strategy)
    )

    for bucket_state in values(state.bucket_states)
        min_required_pool_size = length(bucket_state.direct_seed_route_ids)
        if _effective_bucket_target_size(bucket_state, strategy.config.route_pool_bucket_x_multiplier) < min_required_pool_size
            throw(ArgumentError(
                "route_pool_bucket_x_multiplier=$(strategy.config.route_pool_bucket_x_multiplier) induces a bucket cap smaller than the direct-route feasibility floor " *
                "($min_required_pool_size valid (j,k) pairs) for bucket (s=$(bucket_state.scenario_idx), t=$(bucket_state.time_id))"
            ))
        end
    end
    total_direct_routes = sum(length(bucket.direct_seed_route_ids) for bucket in values(state.bucket_states))
    if strategy.config.route_pool_target_size < total_direct_routes
        throw(ArgumentError(
            "route_pool_target_size=$(strategy.config.route_pool_target_size) is smaller than the minimum feasible total direct-route pool size " *
            "($total_direct_routes across all buckets)"
        ))
    end
    return state
end

function run_iteration_subproblem(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
)
    return run_opt(
        model,
        data;
        optimizer_env=optimizer_env,
        silent=silent,
        show_counts=show_counts,
        do_optimize=do_optimize,
        warm_start=warm_start,
        check_feasibility=check_feasibility,
        mip_gap=mip_gap,
        route_pool_state=state,
        solve_strategy=nothing,
        output_dir=nothing,
    )
end

iteration_state_size(strategy::AlphaRouteIterativeStrategy, state::AlphaRouteBucketPoolsState)::Int =
    sum(length(bucket.routes_by_id) for bucket in values(state.bucket_states))

function _bucket_count(state::AlphaRouteBucketPoolsState)::Int
    return length(state.bucket_states)
end

function update_iteration_state!(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState,
    result::OptResult,
    iteration::Int
)
    usage_by_bucket = _route_usage_by_bucket(result)
    total_active = 0
    total_removed = 0
    total_added = 0

    for bucket_key in _sorted_bucket_route_pool_keys(state)
        bucket_state = state.bucket_states[bucket_key]
        bucket_usage = get(usage_by_bucket, bucket_key, Dict{Int, Float64}())
        total_active += count(v -> v > strategy.config.min_theta_to_keep, values(bucket_usage))
        bucket_target_size = _effective_bucket_target_size(bucket_state, strategy.config.route_pool_bucket_x_multiplier)

        total_removed += strategy.config.prune_enabled ?
            _prune_route_pool!(
                bucket_state,
                bucket_usage,
                strategy.config.min_theta_to_keep,
                bucket_target_size,
                strategy.config.random_retention_seed
            ) : 0

        if strategy.config.expand_enabled && !isempty(strategy.config.route_length_schedule)
            schedule_idx = min(iteration + 1, length(strategy.config.route_length_schedule))
            target_length = strategy.config.route_length_schedule[schedule_idx]
            total_added += _expand_route_pool!(
                state,
                bucket_state,
                data,
                target_length;
                vehicle_capacity=model.vehicle_capacity,
                route_generation_method=model.route_generation_method,
                iterative_config=model.iterative_route_generation_config,
                max_detour_time=model.max_detour_time,
                max_detour_ratio=model.max_detour_ratio,
                stop_dwell_time=model.stop_dwell_time
            )
        end
    end

    total_removed += strategy.config.prune_enabled ?
        _enforce_global_total_route_cap!(
            state,
            usage_by_bucket,
            strategy.config.min_theta_to_keep,
            strategy.config.route_pool_target_size,
            strategy.config.random_retention_seed
        ) : 0

    return (
        added_count=total_added,
        removed_count=total_removed,
        active_route_count=total_active,
        bucket_count=_bucket_count(state),
    )
end

function build_iteration_metadata(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState,
    result::OptResult,
    update_info,
    iteration::Int
)::Dict{String, Any}
    return Dict{String, Any}(
        "active_route_count" => get(update_info, :active_route_count, 0),
        "bucket_count" => get(update_info, :bucket_count, _bucket_count(state)),
    )
end

function should_stop_iteration(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState,
    result::OptResult,
    history::Vector{IterativeSolveIterationSummary},
    iteration::Int
)::Union{Nothing, String}
    if !strategy.config.iterative
        return "single_iteration"
    end
    if iteration >= strategy.config.max_iterations
        return "max_iterations"
    end
    latest = history[end]
    if !isnothing(latest.objective_improvement) &&
       latest.objective_improvement <= strategy.config.objective_improvement_tol &&
       latest.state_change_ratio <= strategy.config.route_pool_change_tol
        return "hybrid_convergence"
    end
    return nothing
end

function finalize_iterative_result!(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    final_result::OptResult,
    history::Vector{IterativeSolveIterationSummary},
    final_state::AlphaRouteBucketPoolsState,
    convergence_reason::String
)
    final_result.metadata["iterative_solve"] = Dict(
        "convergence_reason" => convergence_reason,
        "iteration_count" => length(history),
        "bucket_count" => _bucket_count(final_state),
    )
    final_result.metadata["alpha_route_runner"] = Dict(
        "iterations" => [
            Dict(
                "iteration" => it.iteration,
                "route_count_before" => it.state_size_before,
                "route_count_after" => it.state_size_after,
                "objective_value" => it.objective_value,
                "active_route_count" => get(it.metadata, "active_route_count", 0),
                "bucket_count" => get(it.metadata, "bucket_count", 0),
                "added_route_count" => it.added_count,
                "removed_route_count" => it.removed_count,
                "pool_change_ratio" => it.state_change_ratio,
                "objective_improvement" => it.objective_improvement,
            ) for it in history
        ],
        "convergence_reason" => convergence_reason,
        "final_route_count" => iteration_state_size(strategy, final_state),
        "final_bucket_count" => _bucket_count(final_state),
        "route_length_schedule" => strategy.config.route_length_schedule,
        "route_pool_target_size" => strategy.config.route_pool_target_size,
        "route_pool_bucket_x_multiplier" => strategy.config.route_pool_bucket_x_multiplier,
        "random_retention_seed" => strategy.config.random_retention_seed,
    )
    return nothing
end

function export_initial_iteration_state(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState,
    output_dir::String
)
    export_alpha_route_bucket_pools_state(
        state,
        joinpath(output_dir, "iteration_00_initial");
        array_idx_to_station_id=data.array_idx_to_station_id
    )
    return nothing
end

function export_iteration_state(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState,
    output_dir::String,
    iteration::Int
)
    strategy.config.export_iteration_artifacts || return nothing
    export_alpha_route_bucket_pools_state(
        state,
        joinpath(output_dir, "iteration_" * lpad(string(iteration), 2, '0'));
        array_idx_to_station_id=data.array_idx_to_station_id
    )
    return nothing
end

function export_final_iteration_state(
    strategy::AlphaRouteIterativeStrategy,
    model::AlphaRouteModel,
    data::StationSelectionData,
    state::AlphaRouteBucketPoolsState,
    output_dir::String
)
    export_alpha_route_bucket_pools_state(
        state,
        joinpath(output_dir, "route_pool_final");
        array_idx_to_station_id=data.array_idx_to_station_id
    )
    return nothing
end
