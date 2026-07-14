export AbstractStationSelectionSolver
export SolverConfig
export DirectSolver
export ColumnGenerationSolver
export AbstractBendersDecomposition
export BendersY
export BendersXY
export AbstractBendersCutMode
export SingleCut
export MultiCut
export BendersSolver
export HeuristicSolver
export HeuristicEnumerationSolver
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

abstract type AbstractBendersDecomposition end

"""
    BendersY

Benders decomposition whose master/cuts are expressed over first-stage design
variables only.
"""
struct BendersY <: AbstractBendersDecomposition end

"""
    BendersXY

Benders decomposition whose master/cuts include first-stage design variables and
linking or assignment variables.
"""
struct BendersXY <: AbstractBendersDecomposition end

abstract type AbstractBendersCutMode end

"""
    SingleCut

Aggregate all scenario subproblem values into one Benders theta/cut.
"""
struct SingleCut <: AbstractBendersCutMode end

"""
    MultiCut(:scenario)

Generate separate Benders theta variables and cuts by scenario.
"""
struct MultiCut <: AbstractBendersCutMode
    dimension::Symbol

    function MultiCut(dimension::Symbol=:scenario)
        dimension == :scenario ||
            throw(ArgumentError("only MultiCut(:scenario) is currently supported"))
        new(dimension)
    end
end

struct BendersSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    decomposition::AbstractBendersDecomposition
    cut_mode::AbstractBendersCutMode
    inner_solver::ColumnGenerationSolver
    max_iterations::Int
    optimality_tol::Float64
    log_dir::Union{String, Nothing}

    function BendersSolver(;
        config::SolverConfig=SolverConfig(),
        decomposition::AbstractBendersDecomposition=BendersY(),
        cut_mode::AbstractBendersCutMode=MultiCut(),
        inner_solver::Union{ColumnGenerationSolver, Nothing}=nothing,
        max_iterations::Int=10_000,
        optimality_tol::Union{Number, Nothing}=nothing,
        reduced_cost_tol::Union{Number, Nothing}=nothing,
        max_columns_per_iteration::Int=20,
        n_candidates::Int=max_columns_per_iteration,
        pricing_time_limit_sec::Number=30.0,
        final_ip_time_limit_sec::Number=3600.0,
        log_dir::Union{AbstractString, Nothing}=nothing,
    )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        resolved_tol = isnothing(optimality_tol) ?
            (isnothing(reduced_cost_tol) ? 1e-6 : Float64(reduced_cost_tol)) :
            Float64(optimality_tol)
        resolved_tol >= 0 || throw(ArgumentError("optimality_tol must be non-negative"))
        resolved_inner = isnothing(inner_solver) ?
            ColumnGenerationSolver(
                config=config,
                max_columns_per_iteration=max_columns_per_iteration,
                n_candidates=n_candidates,
                reduced_cost_tol=isnothing(reduced_cost_tol) ? resolved_tol : Float64(reduced_cost_tol),
                pricing_time_limit_sec=pricing_time_limit_sec,
                final_ip_time_limit_sec=final_ip_time_limit_sec,
                log_dir=log_dir,
            ) :
            inner_solver
        new(
            config,
            decomposition,
            cut_mode,
            resolved_inner,
            max_iterations,
            resolved_tol,
            isnothing(log_dir) ? nothing : String(log_dir),
        )
    end
end

struct HeuristicSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    init_spec::RoutePoolInitSpec
    max_iterations::Int
    route_length_schedule::Vector{Int}
    prune_enabled::Bool
    expand_enabled::Bool
    min_active_value_to_keep::Float64
    pool_target_size::Int
    bucket_multiplier::Float64
    random_retention_seed::Int
    objective_improvement_tol::Float64
    pool_change_tol::Float64
    export_iteration_artifacts::Bool
    enrichment::ExactDARPRouteEnrichmentConfig

    function HeuristicSolver(;
        config::SolverConfig=SolverConfig(),
        init_spec::RoutePoolInitSpec=RoutePoolInitSpec(:direct_only),
        max_iterations::Int=3,
        route_length_schedule::Vector{Int}=Int[],
        prune_enabled::Bool=true,
        expand_enabled::Bool=true,
        min_active_value_to_keep::Number=1e-6,
        pool_target_size::Int=1_000_000,
        bucket_multiplier::Number=100.0,
        random_retention_seed::Int=1234,
        objective_improvement_tol::Number=1e-6,
        pool_change_tol::Number=0.0,
        export_iteration_artifacts::Bool=false,
        enrichment::ExactDARPRouteEnrichmentConfig=ExactDARPRouteEnrichmentConfig(enabled=false),
    )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        min_active_value_to_keep >= 0 ||
            throw(ArgumentError("min_active_value_to_keep must be non-negative"))
        pool_target_size > 0 || throw(ArgumentError("pool_target_size must be positive"))
        bucket_multiplier >= 1 ||
            throw(ArgumentError("bucket_multiplier must be at least 1"))
        pool_change_tol >= 0 || throw(ArgumentError("pool_change_tol must be non-negative"))
        all(v -> v >= 2, route_length_schedule) ||
            throw(ArgumentError("route_length_schedule values must be >= 2"))
        new(
            config,
            init_spec,
            max_iterations,
            route_length_schedule,
            prune_enabled,
            expand_enabled,
            Float64(min_active_value_to_keep),
            pool_target_size,
            Float64(bucket_multiplier),
            random_retention_seed,
            Float64(objective_improvement_tol),
            Float64(pool_change_tol),
            export_iteration_artifacts,
            enrichment,
        )
    end
end

"""
    HeuristicEnumerationSolver

Solve `AggregateODRouteModel` by trying a caller-supplied list of candidate open-station
sets (fixed `y`). For each candidate, the nearest-open assignment is derived and the
resulting fixed-station, fixed-assignment routing sub-problem (`RouteCoveringProblem`) is
solved to proven optimality via column generation. The best-scoring feasible candidate is
then used to warm-start a direct solve of the full `AggregateODRouteModel` (with the
winning routes folded into its column pool).

Candidates are not generated internally — supply them via `candidate_open_stations`
(e.g. station sets read from a prior run).
"""
struct HeuristicEnumerationSolver <: AbstractStationSelectionSolver
    config::SolverConfig
    candidate_open_stations::Vector{Vector{Int}}
    cg_solver::ColumnGenerationSolver

    function HeuristicEnumerationSolver(;
        config::SolverConfig=SolverConfig(),
        candidate_open_stations::Vector{Vector{Int}},
        cg_solver::ColumnGenerationSolver=ColumnGenerationSolver(config=config),
    )
        !isempty(candidate_open_stations) ||
            throw(ArgumentError("candidate_open_stations must not be empty"))
        for candidate in candidate_open_stations
            length(candidate) == length(unique(candidate)) ||
                throw(ArgumentError("candidate_open_stations entries must not contain duplicate station ids"))
        end
        new(config, candidate_open_stations, cg_solver)
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
