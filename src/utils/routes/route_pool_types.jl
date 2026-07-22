export RoutePoolInitSpec
export RoutePoolState
export ExactDARPRouteBucketPoolsState
export ExactDARPRouteEnrichmentConfig
export ExactDARPRouteRunnerConfig
export ExactDARPRouteIterationSummary
export ExactDARPRouteRunnerResult
export ExactDARPRouteColumnGenerationConfig
export ExactDARPRouteColumnGenerationRunnerResult
export IterativeRouteGenerationConfig

struct RoutePoolInitSpec
    mode::Symbol
    routes_file::Union{String, Nothing}
    alpha_profile_file::Union{String, Nothing}

    function RoutePoolInitSpec(
        mode::Symbol;
        routes_file::Union{String, Nothing}=nothing,
        alpha_profile_file::Union{String, Nothing}=nothing
    )
        mode in (:generated, :file, :combined, :direct_only) ||
            throw(ArgumentError("RoutePoolInitSpec.mode must be :generated, :file, :combined, or :direct_only"))
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

struct IterativeRouteGenerationConfig
    max_route_length::Int
    max_iterations::Int
    max_new_routes_per_iter::Int
    max_routes_total::Int
    arc_epsilon::Float64
    top_b_insertions::Int
    knn_replacement::Int
    min_feasible_legs::Int
    min_new_feasible_legs::Int
    mutation_min_new_feasible_legs::Int
    max_travel_time::Union{Nothing, Float64}
    geometry_insertion_quota::Int
    coverage_insertion_quota::Int
    interior_replacement_quota::Int
    endpoint_mutation_quota::Int
    reverse_mutation_quota::Int
    rng_seed::Int
    verbose::Bool

    function IterativeRouteGenerationConfig(;
        max_route_length::Int=4,
        max_iterations::Int=3,
        max_new_routes_per_iter::Int=200,
        max_routes_total::Int=5_000,
        arc_epsilon::Float64=0.25,
        top_b_insertions::Int=8,
        knn_replacement::Int=5,
        min_feasible_legs::Int=1,
        min_new_feasible_legs::Int=1,
        mutation_min_new_feasible_legs::Int=0,
        max_travel_time::Union{Nothing, Float64}=nothing,
        geometry_insertion_quota::Int=75,
        coverage_insertion_quota::Int=75,
        interior_replacement_quota::Int=50,
        endpoint_mutation_quota::Int=50,
        reverse_mutation_quota::Int=50,
        rng_seed::Int=1234,
        verbose::Bool=true,
    )
        max_route_length >= 2 || throw(ArgumentError("max_route_length must be >= 2"))
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        max_new_routes_per_iter > 0 || throw(ArgumentError("max_new_routes_per_iter must be positive"))
        max_routes_total > 0 || throw(ArgumentError("max_routes_total must be positive"))
        arc_epsilon >= 0.0 || throw(ArgumentError("arc_epsilon must be non-negative"))
        top_b_insertions > 0 || throw(ArgumentError("top_b_insertions must be positive"))
        knn_replacement > 0 || throw(ArgumentError("knn_replacement must be positive"))
        min_feasible_legs > 0 || throw(ArgumentError("min_feasible_legs must be positive"))
        min_new_feasible_legs >= 0 || throw(ArgumentError("min_new_feasible_legs must be non-negative"))
        mutation_min_new_feasible_legs >= 0 || throw(ArgumentError("mutation_min_new_feasible_legs must be non-negative"))
        isnothing(max_travel_time) || max_travel_time > 0.0 ||
            throw(ArgumentError("max_travel_time must be positive when set"))
        new(
            max_route_length, max_iterations, max_new_routes_per_iter, max_routes_total,
            arc_epsilon, top_b_insertions, knn_replacement, min_feasible_legs,
            min_new_feasible_legs, mutation_min_new_feasible_legs, max_travel_time, geometry_insertion_quota,
            coverage_insertion_quota, interior_replacement_quota, endpoint_mutation_quota,
            reverse_mutation_quota, rng_seed, verbose,
        )
    end
end

mutable struct ExactDARPRouteBucketPoolsState
    bucket_states::Dict{Tuple{Int, Int}, RoutePoolState}
    next_global_route_id::Int
end

struct ExactDARPRouteEnrichmentConfig
    enabled                             :: Bool
    pressure_threshold                  :: Float64
    binding_threshold                   :: Float64
    alpha_scale_factor                  :: Float64
    min_profile_difference              :: Int
    max_profiles_per_route_sequence     :: Int
    max_new_profiles_per_iteration      :: Int
    max_candidate_routes_for_enrichment :: Int

    function ExactDARPRouteEnrichmentConfig(;
        enabled                             :: Bool    = true,
        pressure_threshold                  :: Float64 = 0.70,
        binding_threshold                   :: Float64 = 0.95,
        alpha_scale_factor                  :: Float64 = 1.5,
        min_profile_difference              :: Int     = 2,
        max_profiles_per_route_sequence     :: Int     = 3,
        max_new_profiles_per_iteration      :: Int     = 30,
        max_candidate_routes_for_enrichment :: Int     = 20,
    )
        pressure_threshold >= 0.0 && pressure_threshold < 1.0 ||
            throw(ArgumentError("pressure_threshold must be in [0, 1)"))
        binding_threshold > pressure_threshold && binding_threshold <= 1.0 ||
            throw(ArgumentError("binding_threshold must be in (pressure_threshold, 1]"))
        alpha_scale_factor >= 0.0 ||
            throw(ArgumentError("alpha_scale_factor must be non-negative"))
        min_profile_difference >= 0 ||
            throw(ArgumentError("min_profile_difference must be non-negative"))
        max_profiles_per_route_sequence >= 1 ||
            throw(ArgumentError("max_profiles_per_route_sequence must be at least 1"))
        new(
            enabled,
            pressure_threshold,
            binding_threshold,
            alpha_scale_factor,
            min_profile_difference,
            max_profiles_per_route_sequence,
            max_new_profiles_per_iteration,
            max_candidate_routes_for_enrichment,
        )
    end
end

struct ExactDARPRouteRunnerConfig
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
    enrichment::ExactDARPRouteEnrichmentConfig

    function ExactDARPRouteRunnerConfig(
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
        export_iteration_artifacts::Bool=false,
        enrichment::ExactDARPRouteEnrichmentConfig=ExactDARPRouteEnrichmentConfig(enabled=false),
    )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        min_theta_to_keep >= 0.0 || throw(ArgumentError("min_theta_to_keep must be non-negative"))
        # objective_improvement_tol may be negative to disable objective-based convergence
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
            enrichment,
        )
    end
end

struct ExactDARPRouteColumnGenerationConfig
    init_spec::RoutePoolInitSpec
    max_iterations::Int
    rc_tolerance::Float64
    max_columns_per_iteration::Int
    pricing_time_limit_sec::Float64
    export_iteration_artifacts::Bool

    function ExactDARPRouteColumnGenerationConfig(;
        init_spec::RoutePoolInitSpec=RoutePoolInitSpec(:direct_only),
        max_iterations::Int=10,
        rc_tolerance::Float64=-1e-6,
        max_columns_per_iteration::Int=10,
        pricing_time_limit_sec::Float64=60.0,
        export_iteration_artifacts::Bool=false,
    )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        max_columns_per_iteration > 0 || throw(ArgumentError("max_columns_per_iteration must be positive"))
        pricing_time_limit_sec > 0.0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
        init_spec.mode == :direct_only ||
            throw(ArgumentError("ExactDARPRouteColumnGenerationConfig currently requires RoutePoolInitSpec(:direct_only)"))
        new(
            init_spec,
            max_iterations,
            rc_tolerance,
            max_columns_per_iteration,
            pricing_time_limit_sec,
            export_iteration_artifacts,
        )
    end
end

struct ExactDARPRouteIterationSummary
    iteration::Int
    objective_value::Float64
    route_count_before::Int
    route_count_after::Int
    active_route_count::Int
    added_route_count::Int
    removed_route_count::Int
    pool_change_ratio::Float64
    objective_improvement::Union{Nothing, Float64}
    objective_delta::Union{Nothing, Float64}
    relative_objective_improvement::Union{Nothing, Float64}
    build_time_sec::Union{Nothing, Float64}
    warm_start_time_sec::Union{Nothing, Float64}
    solve_time_sec::Union{Nothing, Float64}
    runtime_sec::Union{Nothing, Float64}
end

struct ExactDARPRouteRunnerResult
    final_result::OptResult
    iterations::Vector{ExactDARPRouteIterationSummary}
    convergence_reason::String
    final_route_pool::ExactDARPRouteBucketPoolsState
end

struct ExactDARPRouteColumnGenerationRunnerResult
    final_result::OptResult
    iterations::Vector{Any}
    convergence_reason::String
    final_state::Any
end
