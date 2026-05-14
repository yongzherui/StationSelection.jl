export run_iterative_solve

function initialize_iteration_state(strategy::AbstractIterativeSolveStrategy, model, data)
    throw(MethodError(initialize_iteration_state, (strategy, model, data)))
end

function run_iteration_subproblem(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state;
    kwargs...
)
    throw(MethodError(run_iteration_subproblem, (strategy, model, data, state)))
end

function update_iteration_state!(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state,
    result::OptResult,
    iteration::Int
)
    throw(MethodError(update_iteration_state!, (strategy, model, data, state, result, iteration)))
end

function iteration_state_size(strategy::AbstractIterativeSolveStrategy, state)::Int
    throw(MethodError(iteration_state_size, (strategy, state)))
end

function build_iteration_metadata(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state,
    result::OptResult,
    update_info,
    iteration::Int
)::Dict{String, Any}
    return Dict{String, Any}()
end

function should_stop_iteration(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state,
    result::OptResult,
    history::Vector{IterativeSolveIterationSummary},
    iteration::Int
)::Union{Nothing, String}
    return nothing
end

function finalize_iterative_result!(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    final_result::OptResult,
    history::Vector{IterativeSolveIterationSummary},
    final_state,
    convergence_reason::String
)
    return nothing
end

function export_initial_iteration_state(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state,
    output_dir::String
)
    return nothing
end

function export_iteration_state(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state,
    output_dir::String,
    iteration::Int
)
    return nothing
end

function export_final_iteration_state(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data,
    state,
    output_dir::String
)
    return nothing
end

function run_iterative_solve(
    strategy::AbstractIterativeSolveStrategy,
    model,
    data;
    optimizer_env=nothing,
    silent::Bool=false,
    show_counts::Bool=false,
    do_optimize::Bool=true,
    warm_start::Bool=false,
    check_feasibility::Bool=true,
    mip_gap::Union{Float64, Nothing}=nothing,
    output_dir::Union{String, Nothing}=nothing
)::IterativeSolveResult
    state = initialize_iteration_state(strategy, model, data)
    history = IterativeSolveIterationSummary[]
    final_result = nothing
    convergence_reason = "max_iterations"

    if !isnothing(output_dir)
        mkpath(output_dir)
        export_initial_iteration_state(strategy, model, data, state, output_dir)
    end

    iteration = 1
    while true
        state_size_before = iteration_state_size(strategy, state)
        result = run_iteration_subproblem(
            strategy,
            model,
            data,
            state;
            optimizer_env=optimizer_env,
            silent=silent,
            show_counts=show_counts,
            do_optimize=do_optimize,
            warm_start=warm_start,
            check_feasibility=check_feasibility,
            mip_gap=mip_gap,
        )
        final_result = result

        if result.termination_status != MOI.OPTIMAL
            convergence_reason = "termination_$(result.termination_status)"
            break
        end

        prev_objective = isempty(history) ? nothing : history[end].objective_value
        update_info = update_iteration_state!(strategy, model, data, state, result, iteration)
        state_size_after = iteration_state_size(strategy, state)
        objective_improvement = isnothing(prev_objective) ? nothing : abs(prev_objective - result.objective_value)
        state_change_ratio = (get(update_info, :added_count, 0) + get(update_info, :removed_count, 0)) / max(state_size_before, 1)
        metadata = build_iteration_metadata(strategy, model, data, state, result, update_info, iteration)

        push!(history, IterativeSolveIterationSummary(
            iteration,
            something(result.objective_value, NaN),
            state_size_before,
            state_size_after,
            get(update_info, :added_count, 0),
            get(update_info, :removed_count, 0),
            state_change_ratio,
            objective_improvement,
            metadata,
        ))

        if !isnothing(output_dir)
            export_iteration_state(strategy, model, data, state, output_dir, iteration)
        end

        stop_reason = should_stop_iteration(strategy, model, data, state, result, history, iteration)
        if !isnothing(stop_reason)
            convergence_reason = stop_reason
            break
        end
        iteration += 1
    end

    finalize_iterative_result!(strategy, model, data, final_result, history, state, convergence_reason)

    if !isnothing(output_dir)
        export_final_iteration_state(strategy, model, data, state, output_dir)
    end

    return IterativeSolveResult(
        final_result,
        history,
        convergence_reason,
        state,
        Dict{String, Any}(),
    )
end
