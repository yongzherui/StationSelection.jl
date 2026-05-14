export IterativeRouteGenerationConfig
export generate_iterative_routes

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
        top_b_insertions::Int=5,
        knn_replacement::Int=3,
        min_feasible_legs::Int=1,
        min_new_feasible_legs::Int=1,
        max_travel_time::Union{Nothing, Float64}=nothing,
        geometry_insertion_quota::Int=75,
        coverage_insertion_quota::Int=75,
        interior_replacement_quota::Int=25,
        endpoint_mutation_quota::Int=25,
        reverse_mutation_quota::Int=25,
        rng_seed::Int=1234,
        verbose::Bool=false,
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
        isnothing(max_travel_time) || max_travel_time > 0.0 ||
            throw(ArgumentError("max_travel_time must be positive when set"))
        new(
            max_route_length,
            max_iterations,
            max_new_routes_per_iter,
            max_routes_total,
            arc_epsilon,
            top_b_insertions,
            knn_replacement,
            min_feasible_legs,
            min_new_feasible_legs,
            max_travel_time,
            geometry_insertion_quota,
            coverage_insertion_quota,
            interior_replacement_quota,
            endpoint_mutation_quota,
            reverse_mutation_quota,
            rng_seed,
            verbose,
        )
    end
end

function _route_coverage_count(routes::Vector{RouteData})::Dict{Tuple{Int, Int}, Int}
    coverage = Dict{Tuple{Int, Int}, Int}()
    for route in routes, leg in route.detour_feasible_legs
        coverage[leg] = get(coverage, leg, 0) + 1
    end
    return coverage
end

function _active_stations(valid_jk_pairs::Set{Tuple{Int, Int}})::Vector{Int}
    active = Set{Int}()
    for (j_idx, k_idx) in valid_jk_pairs
        push!(active, j_idx)
        push!(active, k_idx)
    end
    return sort!(collect(active))
end

function _nearest_station_candidates(
    active_stations::Vector{Int},
    data::StationSelectionData,
    source_idx::Int,
    excluded::Set{Int},
    k::Int,
)::Vector{Int}
    ranked = Tuple{Float64, Int}[]
    for candidate_idx in active_stations
        candidate_idx == source_idx && continue
        candidate_idx in excluded && continue
        cost = get_routing_cost(data, source_idx, candidate_idx)
        isfinite(cost) || continue
        push!(ranked, (cost, candidate_idx))
    end
    sort!(ranked, by=x -> (x[1], x[2]))
    return [station_idx for (_, station_idx) in ranked[1:min(k, length(ranked))]]
end

function _candidate_tiebreak(station_indices::AbstractVector{Int}, seed::Int)::UInt
    return hash((seed, route_sequence_key(station_indices)))
end

function _evaluate_candidate_route(
    station_indices::Vector{Int},
    data::StationSelectionData,
    valid_jk_pairs::Set{Tuple{Int, Int}},
    config::IterativeRouteGenerationConfig;
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
    parent_legs::Union{Nothing, Set{Tuple{Int, Int}}}=nothing,
)::Union{Nothing, Tuple{RouteData, Int}}
    route = evaluate_route_sequence(
        station_indices,
        data;
        max_route_length=config.max_route_length,
        max_travel_time=config.max_travel_time,
        max_detour_time=max_detour_time,
        max_detour_ratio=max_detour_ratio,
        stop_dwell_time=stop_dwell_time,
        relevant_jk_pairs=valid_jk_pairs,
        min_relevant_feasible_legs=config.min_feasible_legs,
    )
    isnothing(route) && return nothing
    new_leg_count = isnothing(parent_legs) ? length(route.detour_feasible_legs) :
        count(leg -> leg ∉ parent_legs, route.detour_feasible_legs)
    new_leg_count >= config.min_new_feasible_legs || return nothing
    return route, new_leg_count
end

function _plausible_insertions(
    route::RouteData,
    active_stations::Vector{Int},
    data::StationSelectionData,
    arc_epsilon::Float64,
)::Vector{Tuple{Float64, Int, Int}}
    insertions = Tuple{Float64, Int, Int}[]
    in_route = Set(route.station_indices)
    for pos in 1:(length(route.station_indices) - 1)
        a = route.station_indices[pos]
        b = route.station_indices[pos + 1]
        base_cost = get_routing_cost(data, a, b)
        isfinite(base_cost) || continue
        for u in active_stations
            u in in_route && continue
            c_au = get_routing_cost(data, a, u)
            c_ub = get_routing_cost(data, u, b)
            (isfinite(c_au) && isfinite(c_ub)) || continue
            new_arc_cost = c_au + c_ub
            new_arc_cost <= (1.0 + arc_epsilon) * base_cost || continue
            push!(insertions, (new_arc_cost - base_cost, pos, u))
        end
    end
    sort!(insertions, by=x -> (x[1], x[2], x[3]))
    return insertions
end

function _geometry_insertion_candidates(
    routes::Vector{RouteData},
    active_stations::Vector{Int},
    data::StationSelectionData,
    valid_jk_pairs::Set{Tuple{Int, Int}},
    config::IterativeRouteGenerationConfig;
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
)
    candidates = NamedTuple[]
    for route in routes
        length(route.station_indices) >= config.max_route_length && continue
        parent_legs = Set(route.detour_feasible_legs)
        for (delta, pos, u) in _plausible_insertions(route, active_stations, data, config.arc_epsilon)[1:min(config.top_b_insertions, end)]
            candidate_seq = vcat(route.station_indices[1:pos], [u], route.station_indices[(pos + 1):end])
            evaluated = _evaluate_candidate_route(
                candidate_seq, data, valid_jk_pairs, config;
                max_detour_time=max_detour_time,
                max_detour_ratio=max_detour_ratio,
                stop_dwell_time=stop_dwell_time,
                parent_legs=parent_legs,
            )
            isnothing(evaluated) && continue
            push!(candidates, (route=evaluated[1], score=-delta, strategy=:geometry))
        end
    end
    sort!(candidates, by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    return candidates
end

function _coverage_balancing_candidates(
    routes::Vector{RouteData},
    active_stations::Vector{Int},
    data::StationSelectionData,
    valid_jk_pairs::Set{Tuple{Int, Int}},
    coverage_count::Dict{Tuple{Int, Int}, Int},
    config::IterativeRouteGenerationConfig;
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
)
    candidates = NamedTuple[]
    for route in routes
        length(route.station_indices) >= config.max_route_length && continue
        parent_legs = Set(route.detour_feasible_legs)
        for (_, pos, u) in _plausible_insertions(route, active_stations, data, config.arc_epsilon)[1:min(config.top_b_insertions, end)]
            candidate_seq = vcat(route.station_indices[1:pos], [u], route.station_indices[(pos + 1):end])
            evaluated = _evaluate_candidate_route(
                candidate_seq, data, valid_jk_pairs, config;
                max_detour_time=max_detour_time,
                max_detour_ratio=max_detour_ratio,
                stop_dwell_time=stop_dwell_time,
                parent_legs=parent_legs,
            )
            isnothing(evaluated) && continue
            route_data = evaluated[1]
            new_legs = [leg for leg in route_data.detour_feasible_legs if leg ∉ parent_legs]
            score = sum(1.0 / (1.0 + get(coverage_count, leg, 0)) for leg in new_legs)
            score > 0.0 || continue
            push!(candidates, (route=route_data, score=score, strategy=:coverage))
        end
    end
    sort!(candidates, by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    return candidates
end

function _interior_replacement_candidates(
    routes::Vector{RouteData},
    active_stations::Vector{Int},
    data::StationSelectionData,
    valid_jk_pairs::Set{Tuple{Int, Int}},
    coverage_count::Dict{Tuple{Int, Int}, Int},
    config::IterativeRouteGenerationConfig;
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
)
    candidates = NamedTuple[]
    for route in routes
        length(route.station_indices) < 3 && continue
        for pos in 2:(length(route.station_indices) - 1)
            excluded = Set(route.station_indices)
            delete!(excluded, route.station_indices[pos])
            replacements = _nearest_station_candidates(active_stations, data, route.station_indices[pos], excluded, config.knn_replacement)
            parent_legs = Set(route.detour_feasible_legs)
            for replacement in replacements
                candidate_seq = copy(route.station_indices)
                candidate_seq[pos] = replacement
                evaluated = _evaluate_candidate_route(
                    candidate_seq, data, valid_jk_pairs, config;
                    max_detour_time=max_detour_time,
                    max_detour_ratio=max_detour_ratio,
                    stop_dwell_time=stop_dwell_time,
                    parent_legs=parent_legs,
                )
                isnothing(evaluated) && continue
                route_data = evaluated[1]
                score = sum((1.0 / (1.0 + get(coverage_count, leg, 0)) for leg in route_data.detour_feasible_legs if leg ∉ parent_legs); init=0.0)
                push!(candidates, (route=route_data, score=score, strategy=:interior))
            end
        end
    end
    sort!(candidates, by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    return candidates
end

function _endpoint_and_reverse_candidates(
    routes::Vector{RouteData},
    active_stations::Vector{Int},
    data::StationSelectionData,
    valid_jk_pairs::Set{Tuple{Int, Int}},
    coverage_count::Dict{Tuple{Int, Int}, Int},
    config::IterativeRouteGenerationConfig;
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
)
    endpoint_candidates = NamedTuple[]
    reverse_candidates = NamedTuple[]
    for route in routes
        parent_legs = Set(route.detour_feasible_legs)
        for pos in (1, length(route.station_indices))
            source_station = route.station_indices[pos]
            excluded = Set(route.station_indices)
            delete!(excluded, source_station)
            for replacement in _nearest_station_candidates(active_stations, data, source_station, excluded, config.knn_replacement)
                candidate_seq = copy(route.station_indices)
                candidate_seq[pos] = replacement
                evaluated = _evaluate_candidate_route(
                    candidate_seq, data, valid_jk_pairs, config;
                    max_detour_time=max_detour_time,
                    max_detour_ratio=max_detour_ratio,
                    stop_dwell_time=stop_dwell_time,
                    parent_legs=parent_legs,
                )
                isnothing(evaluated) && continue
                route_data = evaluated[1]
                score = sum((1.0 / (1.0 + get(coverage_count, leg, 0)) for leg in route_data.detour_feasible_legs if leg ∉ parent_legs); init=0.0)
                push!(endpoint_candidates, (route=route_data, score=score, strategy=:endpoint))
            end
        end

        reverse_seq = reverse(route.station_indices)
        reverse_seq == route.station_indices && continue
        evaluated = _evaluate_candidate_route(
            reverse_seq, data, valid_jk_pairs, config;
            max_detour_time=max_detour_time,
            max_detour_ratio=max_detour_ratio,
            stop_dwell_time=stop_dwell_time,
            parent_legs=parent_legs,
        )
        isnothing(evaluated) && continue
        route_data = evaluated[1]
        score = sum((1.0 / (1.0 + get(coverage_count, leg, 0)) for leg in route_data.detour_feasible_legs if leg ∉ parent_legs); init=0.0)
        push!(reverse_candidates, (route=route_data, score=score, strategy=:reverse))
    end
    sort!(endpoint_candidates, by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    sort!(reverse_candidates, by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    return endpoint_candidates, reverse_candidates
end

function _covered_valid_jk_pair_count(routes::Vector{RouteData}, valid_jk_pairs::Set{Tuple{Int, Int}})::Int
    covered = Set{Tuple{Int, Int}}()
    for route in routes, leg in route.detour_feasible_legs
        leg in valid_jk_pairs && push!(covered, leg)
    end
    return length(covered)
end

function generate_iterative_routes(
    valid_jk_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData;
    config::IterativeRouteGenerationConfig=IterativeRouteGenerationConfig(),
    max_detour_time::Float64=Inf,
    max_detour_ratio::Float64=Inf,
    stop_dwell_time::Float64=10.0,
)::Vector{RouteData}
    isempty(valid_jk_pairs) && return RouteData[]

    routes_by_key = Dict{Tuple, RouteData}()
    next_id = 1
    for (j_idx, k_idx) in sort!(collect(valid_jk_pairs))
        route = evaluate_route_sequence(
            [j_idx, k_idx], data;
            route_id=next_id,
            max_route_length=2,
            max_detour_time=max_detour_time,
            max_detour_ratio=max_detour_ratio,
            stop_dwell_time=stop_dwell_time,
            relevant_jk_pairs=valid_jk_pairs,
            min_relevant_feasible_legs=1,
        )
        isnothing(route) && continue
        routes_by_key[route_sequence_key(route.station_indices)] = route
        next_id += 1
    end

    active_stations = _active_stations(valid_jk_pairs)
    quotas = Dict(
        :geometry => config.geometry_insertion_quota,
        :coverage => config.coverage_insertion_quota,
        :interior => config.interior_replacement_quota,
        :endpoint => config.endpoint_mutation_quota,
        :reverse => config.reverse_mutation_quota,
    )

    for iter in 1:config.max_iterations
        length(routes_by_key) >= config.max_routes_total && break
        current_routes = sort!(collect(values(routes_by_key)), by=r -> (length(r.station_indices), r.station_indices))
        coverage_count = _route_coverage_count(current_routes)
        geometry_candidates = _geometry_insertion_candidates(current_routes, active_stations, data, valid_jk_pairs, config; max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio, stop_dwell_time=stop_dwell_time)
        coverage_candidates = _coverage_balancing_candidates(current_routes, active_stations, data, valid_jk_pairs, coverage_count, config; max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio, stop_dwell_time=stop_dwell_time)
        interior_candidates = _interior_replacement_candidates(current_routes, active_stations, data, valid_jk_pairs, coverage_count, config; max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio, stop_dwell_time=stop_dwell_time)
        endpoint_candidates, reverse_candidates = _endpoint_and_reverse_candidates(current_routes, active_stations, data, valid_jk_pairs, coverage_count, config; max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio, stop_dwell_time=stop_dwell_time)

        candidate_groups = Dict(
            :geometry => geometry_candidates,
            :coverage => coverage_candidates,
            :interior => interior_candidates,
            :endpoint => endpoint_candidates,
            :reverse => reverse_candidates,
        )
        added_this_iter = 0
        added_by_strategy = Dict{Symbol, Int}(strategy => 0 for strategy in keys(candidate_groups))

        for strategy in (:geometry, :coverage, :interior, :endpoint, :reverse)
            for candidate in candidate_groups[strategy][1:min(quotas[strategy], length(candidate_groups[strategy]))]
                route_key = route_sequence_key(candidate.route.station_indices)
                haskey(routes_by_key, route_key) && continue
                routes_by_key[route_key] = RouteData(next_id, candidate.route.station_indices, candidate.route.travel_time, candidate.route.detour_feasible_legs)
                next_id += 1
                added_this_iter += 1
                added_by_strategy[strategy] += 1
                added_this_iter >= config.max_new_routes_per_iter && break
                length(routes_by_key) >= config.max_routes_total && break
            end
            (added_this_iter >= config.max_new_routes_per_iter || length(routes_by_key) >= config.max_routes_total) && break
        end

        if config.verbose
            routes_now = collect(values(routes_by_key))
            lengths = length.(getfield.(routes_now, :station_indices))
            println(
                "    iterative route gen iter $iter: " *
                "generated=(geom=$(length(geometry_candidates)), cov=$(length(coverage_candidates)), int=$(length(interior_candidates)), end=$(length(endpoint_candidates)), rev=$(length(reverse_candidates))) " *
                "added=(geom=$(get(added_by_strategy, :geometry, 0)), cov=$(get(added_by_strategy, :coverage, 0)), int=$(get(added_by_strategy, :interior, 0)), end=$(get(added_by_strategy, :endpoint, 0)), rev=$(get(added_by_strategy, :reverse, 0))) " *
                "filtered=$(length(geometry_candidates) + length(coverage_candidates) + length(interior_candidates) + length(endpoint_candidates) + length(reverse_candidates) - added_this_iter) " *
                "total=$(length(routes_now)) avg_len=$(round(mean(lengths), digits=2)) max_len=$(maximum(lengths)) covered=$(_covered_valid_jk_pair_count(routes_now, valid_jk_pairs))/$(length(valid_jk_pairs))"
            )
            flush(stdout)
        end

        added_this_iter == 0 && break
    end

    return sort!(collect(values(routes_by_key)), by=r -> r.id)
end
