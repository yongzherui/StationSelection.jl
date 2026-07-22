"""
Column-generation logging for AggregateODRouteModel: the logger struct,
per-iteration/termination log records, and CSV output.
"""

export AggregateODRouteCGLogger
export AggregateODRouteCGIterationLog
export AggregateODRouteCGTerminationLog

mutable struct AggregateODRouteCGLogger
    verbose::Bool
    cg_log_path::Union{Nothing, String}
    iteration_rows::Vector{NamedTuple}
end

struct AggregateODRouteCGIterationLog
    iteration::Int
    columns_before::Int
    columns_after::Int
    lp_status::Symbol
    lp_objective::Union{Nothing, Float64}
    lp_solve_seconds::Float64
    pricing_seconds::Union{Nothing, Float64}
    iteration_seconds::Float64
    new_columns_returned::Int
    columns_added::Int
    columns_replaced::Int
    best_reduced_cost::Union{Nothing, Float64}
    pricing_exhausted::Bool
    stop_reason::Symbol
    dual_min::Union{Nothing, Float64}
    dual_max::Union{Nothing, Float64}
    dual_mean::Union{Nothing, Float64}
    dual_std::Union{Nothing, Float64}
    labels_generated::Union{Nothing, Int}
    labels_rejected_by_dominance::Union{Nothing, Int}
    labels_removed_by_dominance::Union{Nothing, Int}
    stale_pops::Union{Nothing, Int}
    max_frontier_size::Union{Nothing, Int}
    max_live_labels::Union{Nothing, Int}
    t_queue_sec::Union{Nothing, Float64}
    t_candidates_sec::Union{Nothing, Float64}
    t_extension_sec::Union{Nothing, Float64}
    t_dominance_sec::Union{Nothing, Float64}
end

struct AggregateODRouteCGTerminationLog
    reason::Symbol
    iteration::Int
    final_pool_size::Int
end

function _write_aggregate_od_route_cg_log_csv(path::AbstractString, rows; headers::Union{Nothing, Vector{Symbol}}=nothing)
    dir = dirname(path)
    isempty(dir) || mkpath(dir)
    if isempty(rows)
        isnothing(headers) && return path
        open(path, "w") do io
            println(io, join(string.(headers), ","))
        end
        return path
    end
    headers = isnothing(headers) ? collect(keys(first(rows))) : headers
    open(path, "w") do io
        println(io, join(string.(headers), ","))
        for row in rows
            values = [
                begin
                    value = getproperty(row, header)
                    if value === nothing
                        ""
                    elseif value isa AbstractString
                        "\"" * replace(value, "\"" => "\"\"") * "\""
                    elseif value isa Symbol
                        string(value)
                    elseif value isa Bool
                        value ? "true" : "false"
                    else
                        string(value)
                    end
                end
                for header in headers
            ]
            println(io, join(values, ","))
        end
    end
    return path
end

_create_aggregate_od_route_cg_logger(; verbose::Bool, cg_log_path::Union{Nothing, AbstractString}) =
    AggregateODRouteCGLogger(verbose, isnothing(cg_log_path) ? nothing : String(cg_log_path), NamedTuple[])

function _aggregate_od_route_log_header!(
    logger::AggregateODRouteCGLogger,
    n_active_pairs::Int,
    initial_pool_size::Int,
    max_cg_iters::Int,
    pricing_time_limit_sec::Float64,
    max_new_columns::Int,
)
    logger.verbose || return nothing
    println("=" ^ 60)
    println("AggregateODRouteModel — Column Generation")
    println("=" ^ 60)
    @printf("  Active station OD pairs : %d\n", n_active_pairs)
    @printf("  Initial RMP cols        : %d\n", initial_pool_size)
    @printf("  Max CG iterations       : %d\n", max_cg_iters)
    @printf("  Pricing time limit      : %.2f sec\n", pricing_time_limit_sec)
    @printf("  Max new columns/iter    : %d\n", max_new_columns)
    println("=" ^ 60)
    return nothing
end

function _to_named_tuple(log::AggregateODRouteCGIterationLog)
    return (
        iteration=log.iteration,
        columns_before=log.columns_before,
        columns_after=log.columns_after,
        lp_status=string(log.lp_status),
        lp_objective=log.lp_objective,
        lp_solve_seconds=log.lp_solve_seconds,
        pricing_seconds=log.pricing_seconds,
        iteration_seconds=log.iteration_seconds,
        new_columns_returned=log.new_columns_returned,
        columns_added=log.columns_added,
        columns_replaced=log.columns_replaced,
        best_reduced_cost=log.best_reduced_cost,
        pricing_exhausted=log.pricing_exhausted,
        stop_reason=string(log.stop_reason),
        dual_min=log.dual_min,
        dual_max=log.dual_max,
        dual_mean=log.dual_mean,
        dual_std=log.dual_std,
        labels_generated=log.labels_generated,
        labels_rejected_by_dominance=log.labels_rejected_by_dominance,
        labels_removed_by_dominance=log.labels_removed_by_dominance,
        stale_pops=log.stale_pops,
        max_frontier_size=log.max_frontier_size,
        max_live_labels=log.max_live_labels,
        t_queue_sec=log.t_queue_sec,
        t_candidates_sec=log.t_candidates_sec,
        t_extension_sec=log.t_extension_sec,
        t_dominance_sec=log.t_dominance_sec,
    )
end

function _record_aggregate_od_route_cg_iteration!(
    logger::AggregateODRouteCGLogger,
    log::AggregateODRouteCGIterationLog,
)
    push!(logger.iteration_rows, _to_named_tuple(log))
    logger.verbose || return nothing
    println()
    @printf("CG iteration %d\n", log.iteration)
    @printf("  RMP cols before pricing : %d\n", log.columns_before)
    @printf("  LP status               : %s\n", log.lp_status)
    isnothing(log.lp_objective) ? println("  LP objective            : unavailable") :
        @printf("  LP objective            : %.6f\n", log.lp_objective)
    @printf("  LP runtime              : %.3f sec\n", log.lp_solve_seconds)
    isnothing(log.pricing_seconds) ? println("  Pricing runtime         : unavailable") :
        @printf("  Pricing runtime         : %.3f sec\n", log.pricing_seconds)
    @printf("  Iteration runtime       : %.3f sec\n", log.iteration_seconds)
    @printf("  New columns returned    : %d\n", log.new_columns_returned)
    @printf("  Columns added           : %d\n", log.columns_added)
    @printf("  Columns replaced        : %d\n", log.columns_replaced)
    isnothing(log.best_reduced_cost) ? println("  Best reduced cost       : n/a") :
        @printf("  Best reduced cost       : %.6f\n", log.best_reduced_cost)
    @printf("  Pricing exhausted       : %s\n", log.pricing_exhausted)
    @printf("  RMP cols after pricing  : %d\n", log.columns_after)
    if !isnothing(log.dual_min)
        @printf("  Duals [min/max/mean/std]: %.4f / %.4f / %.4f / %.4f\n",
            log.dual_min, log.dual_max, log.dual_mean, log.dual_std)
    end
    if !isnothing(log.labels_generated)
        @printf("  Labels generated        : %d  (rejected=%d  removed=%d  stale=%d)\n",
            log.labels_generated, log.labels_rejected_by_dominance,
            log.labels_removed_by_dominance, log.stale_pops)
        @printf("  Max frontier / live     : %d / %d\n",
            log.max_frontier_size, log.max_live_labels)
    end
    if !isnothing(log.t_queue_sec)
        total_accounted = log.t_queue_sec + log.t_candidates_sec + log.t_extension_sec + log.t_dominance_sec
        @printf("  Phase timing (s)        : queue=%.2f  candidates=%.2f  extension=%.2f  dominance=%.2f  (total=%.2f)\n",
            log.t_queue_sec, log.t_candidates_sec, log.t_extension_sec, log.t_dominance_sec, total_accounted)
    end
    return nothing
end

function _record_aggregate_od_route_cg_termination!(
    logger::AggregateODRouteCGLogger,
    log::AggregateODRouteCGTerminationLog,
)
    logger.verbose || return nothing
    println()
    println("=" ^ 60)
    println("Aggregate OD Route Column Generation Terminated")
    println("=" ^ 60)
    @printf("  Iterations completed : %d\n", log.iteration)
    @printf("  Final RMP cols       : %d\n", log.final_pool_size)
    @printf("  Reason               : %s\n", log.reason)
    println("=" ^ 60)
    return nothing
end

function _flush_aggregate_od_route_cg_log!(logger::AggregateODRouteCGLogger)
    isnothing(logger.cg_log_path) && return nothing
    _write_aggregate_od_route_cg_log_csv(logger.cg_log_path, logger.iteration_rows)
    return nothing
end

function _merge_pricing_stats(stats)
    isempty(stats) && return (
        labels_generated=0,
        labels_rejected_by_dominance=0,
        labels_removed_by_dominance=0,
        stale_pops=0,
        max_frontier_size=0,
        max_live_labels=0,
        t_queue_sec=0.0,
        t_candidates_sec=0.0,
        t_extension_sec=0.0,
        t_dominance_sec=0.0,
    )
    return (
        labels_generated=sum(s.labels_generated for s in stats; init=0),
        labels_rejected_by_dominance=sum(s.labels_rejected_by_dominance for s in stats; init=0),
        labels_removed_by_dominance=sum(s.labels_removed_by_dominance for s in stats; init=0),
        stale_pops=sum(s.stale_pops for s in stats; init=0),
        max_frontier_size=maximum(s.max_frontier_size for s in stats; init=0),
        max_live_labels=maximum(s.max_live_labels for s in stats; init=0),
        t_queue_sec=sum(s.t_queue_sec for s in stats; init=0.0),
        t_candidates_sec=sum(s.t_candidates_sec for s in stats; init=0.0),
        t_extension_sec=sum(s.t_extension_sec for s in stats; init=0.0),
        t_dominance_sec=sum(s.t_dominance_sec for s in stats; init=0.0),
    )
end

function _aggregate_od_route_cg_log_path(solver::ColumnGenerationSolver, filename::String)
    isnothing(solver.log_dir) && return nothing
    return joinpath(solver.log_dir, filename)
end

function _aggregate_od_route_cg_log_path(solver::BendersSolver, filename::String)
    isnothing(solver.log_dir) && return nothing
    return joinpath(solver.log_dir, filename)
end
