export RoutePoolInitSpec
export RoutePoolState
export AlphaRouteBucketPoolsState
export AlphaRouteRunnerConfig
export AlphaRouteIterationSummary
export AlphaRouteRunnerResult

struct RoutePoolInitSpec
    mode::Symbol
    routes_file::Union{String, Nothing}
    alpha_profile_file::Union{String, Nothing}

    function RoutePoolInitSpec(
        mode::Symbol;
        routes_file::Union{String, Nothing}=nothing,
        alpha_profile_file::Union{String, Nothing}=nothing
    )
        mode in (:generated, :file, :combined) ||
            throw(ArgumentError("RoutePoolInitSpec.mode must be :generated, :file, or :combined"))
        mode in (:file, :combined) && isnothing(routes_file) &&
            throw(ArgumentError("routes_file is required for mode=$mode"))
        mode in (:file, :combined) && isnothing(alpha_profile_file) &&
            throw(ArgumentError("alpha_profile_file is required for mode=$mode"))
        new(mode, routes_file, alpha_profile_file)
    end
end

mutable struct RoutePoolState
    scenario_idx::Int
    time_id::Int
    valid_jk_pairs::Set{Tuple{Int, Int}}
    x_candidate_count::Int
    routes_by_id::Dict{Int, RouteData}
    alpha_profile::Dict{NTuple{3, Int}, Float64}
    signature_to_route_id::Dict{String, Int}
    provenance_by_route_id::Dict{Int, Set{Symbol}}
    protected_route_ids::Set{Int}
    direct_seed_route_ids::Set{Int}
    removed_route_ids::Set{Int}
    current_generated_max_route_length::Int
end

mutable struct AlphaRouteBucketPoolsState
    bucket_states::Dict{Tuple{Int, Int}, RoutePoolState}
    next_global_route_id::Int
end

struct AlphaRouteRunnerConfig
    init_spec::RoutePoolInitSpec
    iterative::Bool
    max_iterations::Int
    route_length_schedule::Vector{Int}
    prune_enabled::Bool
    expand_enabled::Bool
    min_theta_to_keep::Float64
    route_pool_target_size::Int
    route_pool_bucket_x_multiplier::Float64
    random_retention_seed::Int
    objective_improvement_tol::Float64
    route_pool_change_tol::Float64
    export_iteration_artifacts::Bool

    function AlphaRouteRunnerConfig(
        init_spec::RoutePoolInitSpec;
        iterative::Bool=true,
        max_iterations::Int=3,
        route_length_schedule::Vector{Int}=Int[],
        prune_enabled::Bool=true,
        expand_enabled::Bool=true,
        min_theta_to_keep::Float64=1e-6,
        route_pool_target_size::Int=1_000_000,
        route_pool_bucket_x_multiplier::Float64=100.0,
        random_retention_seed::Int=1234,
        objective_improvement_tol::Float64=1e-6,
        route_pool_change_tol::Float64=0.0,
        export_iteration_artifacts::Bool=false
    )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        min_theta_to_keep >= 0.0 || throw(ArgumentError("min_theta_to_keep must be non-negative"))
        objective_improvement_tol >= 0.0 || throw(ArgumentError("objective_improvement_tol must be non-negative"))
        route_pool_change_tol >= 0.0 || throw(ArgumentError("route_pool_change_tol must be non-negative"))
        route_pool_target_size > 0 || throw(ArgumentError("route_pool_target_size must be positive"))
        route_pool_bucket_x_multiplier >= 1.0 || throw(ArgumentError("route_pool_bucket_x_multiplier must be at least 1.0"))
        all(v -> v >= 2, route_length_schedule) ||
            throw(ArgumentError("route_length_schedule values must be >= 2"))
        init_spec.mode in (:generated, :combined) && isempty(route_length_schedule) &&
            throw(ArgumentError("route_length_schedule must be non-empty for generated/combined route-pool initialization"))
        new(
            init_spec,
            iterative,
            max_iterations,
            route_length_schedule,
            prune_enabled,
            expand_enabled,
            min_theta_to_keep,
            route_pool_target_size,
            route_pool_bucket_x_multiplier,
            random_retention_seed,
            objective_improvement_tol,
            route_pool_change_tol,
            export_iteration_artifacts,
        )
    end
end

struct AlphaRouteIterationSummary
    iteration::Int
    objective_value::Float64
    route_count_before::Int
    route_count_after::Int
    active_route_count::Int
    added_route_count::Int
    removed_route_count::Int
    pool_change_ratio::Float64
    objective_improvement::Union{Nothing, Float64}
end

struct AlphaRouteRunnerResult
    final_result::OptResult
    iterations::Vector{AlphaRouteIterationSummary}
    convergence_reason::String
    final_route_pool::AlphaRouteBucketPoolsState
end
