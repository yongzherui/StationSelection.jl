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
        push!(active, j_idx, k_idx)
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
        (candidate_idx == source_idx || candidate_idx in excluded) && continue
        cost = get_routing_cost(data, source_idx, candidate_idx)
        isfinite(cost) || continue
        push!(ranked, (cost, candidate_idx))
    end
    sort!(ranked, by=x -> (x[1], x[2]))
    return [s for (_, s) in ranked[1:min(k, length(ranked))]]
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
    min_new_feasible_legs::Int=config.min_new_feasible_legs,
)::Union{Nothing, Tuple{RouteData, Int}}
    route = evaluate_route_sequence(
        station_indices, data;
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
    new_leg_count >= min_new_feasible_legs || return nothing
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
            result = _evaluate_candidate_route(candidate_seq, data, valid_jk_pairs, config;
                max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio,
                stop_dwell_time=stop_dwell_time, parent_legs=parent_legs)
            isnothing(result) && continue
            push!(candidates, (route=result[1], score=-delta, strategy=:geometry, source_len=length(route.station_indices)))
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
            result = _evaluate_candidate_route(candidate_seq, data, valid_jk_pairs, config;
                max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio,
                stop_dwell_time=stop_dwell_time, parent_legs=parent_legs)
            isnothing(result) && continue
            new_legs = [leg for leg in result[1].detour_feasible_legs if leg ∉ parent_legs]
            score = sum(1.0 / (1.0 + get(coverage_count, leg, 0)) for leg in new_legs)
            score > 0.0 || continue
            push!(candidates, (route=result[1], score=score, strategy=:coverage, source_len=length(route.station_indices)))
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
            parent_legs = Set(route.detour_feasible_legs)
            for replacement in _nearest_station_candidates(active_stations, data, route.station_indices[pos], excluded, config.knn_replacement)
                candidate_seq = copy(route.station_indices)
                candidate_seq[pos] = replacement
                result = _evaluate_candidate_route(candidate_seq, data, valid_jk_pairs, config;
                    max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio,
                    stop_dwell_time=stop_dwell_time, parent_legs=parent_legs,
                    min_new_feasible_legs=config.mutation_min_new_feasible_legs)
                isnothing(result) && continue
                score = sum((1.0 / (1.0 + get(coverage_count, leg, 0))
                    for leg in result[1].detour_feasible_legs if leg ∉ parent_legs); init=0.0)
                push!(candidates, (route=result[1], score=score, strategy=:interior, source_len=length(route.station_indices)))
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
    reverse_candidates  = NamedTuple[]
    for route in routes
        parent_legs = Set(route.detour_feasible_legs)
        for pos in (1, length(route.station_indices))
            source = route.station_indices[pos]
            excluded = Set(route.station_indices)
            delete!(excluded, source)
            for replacement in _nearest_station_candidates(active_stations, data, source, excluded, config.knn_replacement)
                candidate_seq = copy(route.station_indices)
                candidate_seq[pos] = replacement
                result = _evaluate_candidate_route(candidate_seq, data, valid_jk_pairs, config;
                    max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio,
                    stop_dwell_time=stop_dwell_time, parent_legs=parent_legs,
                    min_new_feasible_legs=config.mutation_min_new_feasible_legs)
                isnothing(result) && continue
                score = sum((1.0 / (1.0 + get(coverage_count, leg, 0))
                    for leg in result[1].detour_feasible_legs if leg ∉ parent_legs); init=0.0)
                push!(endpoint_candidates, (route=result[1], score=score, strategy=:endpoint, source_len=length(route.station_indices)))
            end
        end

        reverse_seq = reverse(route.station_indices)
        reverse_seq == route.station_indices && continue
        result = _evaluate_candidate_route(reverse_seq, data, valid_jk_pairs, config;
            max_detour_time=max_detour_time, max_detour_ratio=max_detour_ratio,
            stop_dwell_time=stop_dwell_time, parent_legs=parent_legs,
            min_new_feasible_legs=config.mutation_min_new_feasible_legs)
        isnothing(result) && continue
        score = sum((1.0 / (1.0 + get(coverage_count, leg, 0))
            for leg in result[1].detour_feasible_legs if leg ∉ parent_legs); init=0.0)
        push!(reverse_candidates, (route=result[1], score=score, strategy=:reverse, source_len=length(route.station_indices)))
    end
    sort!(endpoint_candidates, by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    sort!(reverse_candidates,  by=x -> (-x.score, _candidate_tiebreak(x.route.station_indices, config.rng_seed)))
    return endpoint_candidates, reverse_candidates
end

function _covered_valid_jk_pair_count(routes::Vector{RouteData}, valid_jk_pairs::Set{Tuple{Int, Int}})::Int
    covered = Set{Tuple{Int, Int}}()
    for route in routes, leg in route.detour_feasible_legs
        leg in valid_jk_pairs && push!(covered, leg)
    end
    return length(covered)
end

function _histogram_pairs(values::AbstractVector{Int})::Vector{Pair{Int, Int}}
    hist = Dict{Int, Int}()
    for value in values
        hist[value] = get(hist, value, 0) + 1
    end
    return sort!(collect(hist); by=first)
end

function _route_length_histogram(routes::Vector{RouteData})::Vector{Pair{Int, Int}}
    return _histogram_pairs(length.(getfield.(routes, :station_indices)))
end

function _candidate_source_length_histogram(candidates)::Vector{Pair{Int, Int}}
    isempty(candidates) && return Pair{Int, Int}[]
    return _histogram_pairs(Int[getproperty(c, :source_len) for c in candidates])
end

function _candidate_target_length_histogram(candidates)::Vector{Pair{Int, Int}}
    isempty(candidates) && return Pair{Int, Int}[]
    return _histogram_pairs(Int[length(getproperty(c, :route).station_indices) for c in candidates])
end
