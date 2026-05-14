export generate_iterative_routes

function generate_iterative_routes(
    valid_jk_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData;
    config::IterativeRouteGenerationConfig=IterativeRouteGenerationConfig(),
    max_detour_time::Float64=Inf,
    max_detour_ratio::Float64=Inf,
    stop_dwell_time::Float64=10.0,
)::Vector{RouteData}
    isempty(valid_jk_pairs) && return RouteData[]

    config.verbose && @info "generate_iterative_routes: starting" max_route_length=config.max_route_length max_iterations=config.max_iterations max_routes_total=config.max_routes_total max_new_routes_per_iter=config.max_new_routes_per_iter n_valid_pairs=length(valid_jk_pairs) arc_epsilon=config.arc_epsilon top_b_insertions=config.top_b_insertions knn_replacement=config.knn_replacement quotas=(geom=config.geometry_insertion_quota, cov=config.coverage_insertion_quota, int=config.interior_replacement_quota, ep=config.endpoint_mutation_quota, rev=config.reverse_mutation_quota)

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

    config.verbose && @info "generate_iterative_routes: seeded" n_direct_routes=length(routes_by_key) n_valid_pairs=length(valid_jk_pairs)

    active_stations = _active_stations(valid_jk_pairs)
    quotas = Dict(
        :geometry => config.geometry_insertion_quota,
        :coverage => config.coverage_insertion_quota,
        :interior => config.interior_replacement_quota,
        :endpoint => config.endpoint_mutation_quota,
        :reverse  => config.reverse_mutation_quota,
    )
    iter_kw = (
        max_detour_time=max_detour_time,
        max_detour_ratio=max_detour_ratio,
        stop_dwell_time=stop_dwell_time,
    )

    for iter in 1:config.max_iterations
        length(routes_by_key) >= config.max_routes_total && break
        current_routes  = sort!(collect(values(routes_by_key)), by=r -> (length(r.station_indices), r.station_indices))
        coverage_count  = _route_coverage_count(current_routes)

        geom_c = _geometry_insertion_candidates(current_routes, active_stations, data, valid_jk_pairs, config; iter_kw...)
        cov_c  = _coverage_balancing_candidates(current_routes, active_stations, data, valid_jk_pairs, coverage_count, config; iter_kw...)
        int_c  = _interior_replacement_candidates(current_routes, active_stations, data, valid_jk_pairs, coverage_count, config; iter_kw...)
        ep_c, rev_c = _endpoint_and_reverse_candidates(current_routes, active_stations, data, valid_jk_pairs, coverage_count, config; iter_kw...)

        candidate_groups = Dict(:geometry => geom_c, :coverage => cov_c, :interior => int_c, :endpoint => ep_c, :reverse => rev_c)
        added_this_iter  = 0
        added_by         = Dict{Symbol, Int}(s => 0 for s in keys(candidate_groups))

        for strategy in (:geometry, :coverage, :interior, :endpoint, :reverse)
            for cand in candidate_groups[strategy][1:min(quotas[strategy], length(candidate_groups[strategy]))]
                key = route_sequence_key(cand.route.station_indices)
                haskey(routes_by_key, key) && continue
                routes_by_key[key] = RouteData(next_id, cand.route.station_indices, cand.route.travel_time, cand.route.detour_feasible_legs)
                next_id += 1
                added_this_iter += 1
                added_by[strategy] += 1
                (added_this_iter >= config.max_new_routes_per_iter || length(routes_by_key) >= config.max_routes_total) && break
            end
            (added_this_iter >= config.max_new_routes_per_iter || length(routes_by_key) >= config.max_routes_total) && break
        end

        if config.verbose
            routes_now = collect(values(routes_by_key))
            lengths = length.(getfield.(routes_now, :station_indices))
            @info "generate_iterative_routes: iter $iter" candidates=(geom=length(geom_c), cov=length(cov_c), int=length(int_c), ep=length(ep_c), rev=length(rev_c)) added=(geom=added_by[:geometry], cov=added_by[:coverage], int=added_by[:interior], ep=added_by[:endpoint], rev=added_by[:reverse]) total=length(routes_now) avg_len=round(mean(lengths), digits=2) max_len=maximum(lengths) covered_pairs=_covered_valid_jk_pair_count(routes_now, valid_jk_pairs) n_valid_pairs=length(valid_jk_pairs)
        end

        added_this_iter == 0 && break
    end

    result = sort!(collect(values(routes_by_key)), by=r -> r.id)
    config.verbose && @info "generate_iterative_routes: done" n_routes=length(result) covered_pairs=_covered_valid_jk_pair_count(result, valid_jk_pairs) n_valid_pairs=length(valid_jk_pairs)
    return result
end
