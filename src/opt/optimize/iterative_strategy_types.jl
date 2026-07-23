export AbstractStationSelectionSolver
export SolverConfig
export DirectSolver
export ColumnGenerationSolver
export AbstractSolveStrategy
export AbstractIterativeSolveStrategy
export IterativeSolveIterationSummary
export IterativeSolveResult

abstract type AbstractStationSelectionSolver end

struct SolverConfig
    optimizer_env::Any
    silent::Bool
    show_counts::Bool
    do_optimize::Bool
    warm_start::Bool
    check_feasibility::Bool
    mip_gap::Union{Float64, Nothing}
    output_dir::Union{String, Nothing}

    function SolverConfig(;
        optimizer_env=nothing,
        silent::Bool=false,
        show_counts::Bool=false,
        do_optimize::Bool=true,
        warm_start::Bool=false,
        check_feasibility::Bool=true,
        mip_gap::Union{Number, Nothing}=nothing,
        output_dir::Union{AbstractString, Nothing}=nothing,
    )
        resolved_mip_gap = isnothing(mip_gap) ? nothing : Float64(mip_gap)
        resolved_output_dir = isnothing(output_dir) ? nothing : String(output_dir)
        new(
            optimizer_env,
            silent,
            show_counts,
            do_optimize,
            warm_start,
            check_feasibility,
            resolved_mip_gap,
            resolved_output_dir,
        )
    end
end

struct DirectSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    max_enumerated_routes::Int
    max_enumeration_time_sec::Float64

    function DirectSolver(
        config::SolverConfig;
        max_enumerated_routes::Int=10_000,
        max_enumeration_time_sec::Number=30.0,
    )
        max_enumerated_routes > 0 ||
            throw(ArgumentError("max_enumerated_routes must be positive"))
        max_enumeration_time_sec > 0 ||
            throw(ArgumentError("max_enumeration_time_sec must be positive"))
        new(config, max_enumerated_routes, Float64(max_enumeration_time_sec))
    end
end

function DirectSolver(;
    config::Union{SolverConfig, Nothing}=nothing,
    max_enumerated_routes::Int=10_000,
    max_enumeration_time_sec::Number=30.0,
    kwargs...
)
    cfg = isnothing(config) ? SolverConfig(; kwargs...) : config
    return DirectSolver(
        cfg;
        max_enumerated_routes=max_enumerated_routes,
        max_enumeration_time_sec=max_enumeration_time_sec,
    )
end

struct ColumnGenerationSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    max_iterations::Int
    max_columns_per_iteration::Int
    n_candidates::Int
    reduced_cost_tol::Float64
    pricing_time_limit_sec::Float64
    final_ip_time_limit_sec::Float64
    log_dir::Union{String, Nothing}

    function ColumnGenerationSolver(;
        config::SolverConfig=SolverConfig(),
        max_iterations::Int=10_000,
        max_columns_per_iteration::Int=20,
        n_candidates::Int=max_columns_per_iteration,
        reduced_cost_tol::Number=1e-6,
        pricing_time_limit_sec::Number=30.0,
        final_ip_time_limit_sec::Number=3600.0,
        log_dir::Union{AbstractString, Nothing}=nothing,
    )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        max_columns_per_iteration > 0 ||
            throw(ArgumentError("max_columns_per_iteration must be positive"))
        n_candidates >= max_columns_per_iteration ||
            throw(ArgumentError("n_candidates must be >= max_columns_per_iteration"))
        reduced_cost_tol >= 0 ||
            throw(ArgumentError("reduced_cost_tol must be non-negative"))
        pricing_time_limit_sec > 0 ||
            throw(ArgumentError("pricing_time_limit_sec must be positive"))
        final_ip_time_limit_sec > 0 ||
            throw(ArgumentError("final_ip_time_limit_sec must be positive"))
        new(
            config,
            max_iterations,
            max_columns_per_iteration,
            n_candidates,
            Float64(reduced_cost_tol),
            Float64(pricing_time_limit_sec),
            Float64(final_ip_time_limit_sec),
            isnothing(log_dir) ? nothing : String(log_dir),
        )
    end
end

abstract type AbstractSolveStrategy end
abstract type AbstractIterativeSolveStrategy <: AbstractSolveStrategy end

struct IterativeSolveIterationSummary
    iteration::Int
    objective_value::Float64
    state_size_before::Int
    state_size_after::Int
    added_count::Int
    removed_count::Int
    state_change_ratio::Float64
    objective_improvement::Union{Nothing, Float64}
    objective_delta::Union{Nothing, Float64}
    relative_objective_improvement::Union{Nothing, Float64}
    metadata::Dict{String, Any}
end

struct IterativeSolveResult
    final_result::OptResult
    iterations::Vector{IterativeSolveIterationSummary}
    convergence_reason::String
    final_state::Any
    metadata::Dict{String, Any}
end
