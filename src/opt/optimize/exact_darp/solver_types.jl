export HeuristicSolver

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
