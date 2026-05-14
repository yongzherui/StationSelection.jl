export default_iterative_route_generation_config
export generate_routes_for_bucket

function _with_max_route_length(
    config::IterativeRouteGenerationConfig,
    max_route_length::Int
)::IterativeRouteGenerationConfig
    return IterativeRouteGenerationConfig(
        max_route_length=max_route_length,
        max_iterations=config.max_iterations,
        max_new_routes_per_iter=config.max_new_routes_per_iter,
        max_routes_total=config.max_routes_total,
        arc_epsilon=config.arc_epsilon,
        top_b_insertions=config.top_b_insertions,
        knn_replacement=config.knn_replacement,
        min_feasible_legs=config.min_feasible_legs,
        min_new_feasible_legs=config.min_new_feasible_legs,
        max_travel_time=config.max_travel_time,
        geometry_insertion_quota=config.geometry_insertion_quota,
        coverage_insertion_quota=config.coverage_insertion_quota,
        interior_replacement_quota=config.interior_replacement_quota,
        endpoint_mutation_quota=config.endpoint_mutation_quota,
        reverse_mutation_quota=config.reverse_mutation_quota,
        rng_seed=config.rng_seed,
        verbose=config.verbose,
    )
end

function default_iterative_route_generation_config(
    max_route_length::Int;
    max_travel_time::Union{Nothing, Float64}=nothing
)::IterativeRouteGenerationConfig
    return IterativeRouteGenerationConfig(
        max_route_length=max_route_length,
        max_travel_time=max_travel_time,
    )
end

function generate_routes_for_bucket(
    valid_jk_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData;
    route_generation_method::Symbol=:dfs,
    iterative_config::Union{Nothing, IterativeRouteGenerationConfig}=nothing,
    max_route_length::Int,
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
)::Vector{RouteData}
    if route_generation_method == :dfs
        return generate_simple_routes(
            valid_jk_pairs,
            data;
            max_route_length=max_route_length,
            max_detour_time=max_detour_time,
            max_detour_ratio=max_detour_ratio,
            stop_dwell_time=stop_dwell_time,
        )
    elseif route_generation_method == :iterative_insertion
        cfg = isnothing(iterative_config) ?
            default_iterative_route_generation_config(max_route_length) :
            _with_max_route_length(iterative_config, max_route_length)
        return generate_iterative_routes(
            valid_jk_pairs,
            data;
            config=cfg,
            max_detour_time=max_detour_time,
            max_detour_ratio=max_detour_ratio,
            stop_dwell_time=stop_dwell_time,
        )
    end

    throw(ArgumentError("Unsupported route_generation_method=$route_generation_method"))
end
