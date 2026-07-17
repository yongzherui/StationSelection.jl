"""
Exhaustive route-column enumeration for aggregate OD route models.

Deliberately independent of the column-generation label-setting pricer
(`aggregate_od_route_column_generation.jl`): that engine's dominance-bucket
pruning (keyed only on the current node, ignoring route length / served-pair
signature) is sound for real pricing -- where only *a* negative-reduced-cost
column is needed and duals evolve across iterations -- but is NOT sound for
one-shot exhaustive enumeration, since it can discard a complete, distinct,
strictly cheaper route just because a less-accomplished route happens to
reach the same node first. See
`notes/2026-07-14_nearest_open_solver_alignment.md` for the full derivation
and the concrete fixtures that exposed the gap.

This module instead does a plain bounded depth-first search over station
sequences (no duals, no dominance, no reduced-cost bookkeeping) and checks
route feasibility directly against the model's own constraints
(`max_stops`, `max_visits_per_node`, `detour_factor`, `max_wait_time`).
"""

export enumerate_aggregate_od_route_columns

"""
    _od_route_relevant_nodes(active_pairs) -> Vector{Int}

Only stations that are an origin or destination of some active OD pair can
ever help serve a pair -- routing costs are direct point-to-point (no
underlying road graph to transit through), so restricting the search to
these nodes is lossless, not a heuristic prune.
"""
function _od_route_relevant_nodes(active_pairs::Vector{Tuple{Int, Int}})::Vector{Int}
    nodes = Set{Int}()
    for (j, k) in active_pairs
        push!(nodes, j)
        push!(nodes, k)
    end
    return sort!(collect(nodes))
end

function _od_route_travel_lookup(
    data::StationSelectionData,
    nodes::Vector{Int},
)::Dict{Tuple{Int, Int}, Float64}
    travel = Dict{Tuple{Int, Int}, Float64}()
    for i in nodes, j in nodes
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) ||
            throw(ArgumentError("missing finite routing cost for station arc $((i, j))"))
        travel[(i, j)] = cost
    end
    return travel
end

"""
    _od_route_served_pairs(route, cum_time, active_pairs, travel, detour_factor, max_wait_time)

A pair `(j, k)` is served by `route` if there exists a boarding position `p`
(`route[p] == j`, reachable within `max_wait_time` of route start) and a
later alighting position `q > p` (`route[q] == k`) whose ride time
`cum_time[q] - cum_time[p]` is within `detour_factor` of the direct `(j, k)`
travel time. Existence of *any* such `(p, q)` is checked -- a strictly more
complete definition than tracking only the most recent boarding of `j`, as
the label-setting pricer does.
"""
function _od_route_served_pairs(
    route::Vector{Int},
    cum_time::Vector{Float64},
    active_pairs::Vector{Tuple{Int, Int}},
    travel::Dict{Tuple{Int, Int}, Float64},
    detour_factor::Float64,
    max_wait_time::Float64,
)::Set{Tuple{Int, Int}}
    served = Set{Tuple{Int, Int}}()
    n = length(route)
    for (j, k) in active_pairs
        ride_limit = detour_factor * travel[(j, k)] + 1e-9
        found = false
        for p in 1:(n - 1)
            route[p] == j || continue
            cum_time[p] <= max_wait_time + 1e-9 || continue
            for q in (p + 1):n
                route[q] == k || continue
                (cum_time[q] - cum_time[p]) <= ride_limit && (found = true)
                found && break
            end
            found && break
        end
        found && push!(served, (j, k))
    end
    return served
end

"""
    enumerate_aggregate_od_route_columns(model, data; max_routes=10_000, time_limit_sec=30.0)

Exhaustively enumerate feasible aggregate OD route columns via bounded DFS
over station sequences (stations restricted to OD-pair endpoints, since
routing costs are direct point-to-point). Every prefix of length >= 2 that
serves at least one active pair is emitted as a candidate column; the
cheapest column per distinct served-pairs signature is kept.
"""
function enumerate_aggregate_od_route_columns(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData;
    max_routes::Int=10_000,
    time_limit_sec::Float64=30.0,
)::Vector{AggregateODRouteColumn}
    max_routes > 0 || throw(ArgumentError("max_routes must be positive"))
    time_limit_sec > 0 || throw(ArgumentError("time_limit_sec must be positive"))

    mapping = create_map(model, data)
    base_model = _base_aggregate_od_route_model(model)
    active_pairs = _all_active_aggregate_od_route_pairs(mapping)
    isempty(active_pairs) && return AggregateODRouteColumn[]

    max_stops = base_model.max_stops == typemax(Int) ? data.n_stations : base_model.max_stops
    max_visits_per_node = base_model.max_visits_per_node
    detour_factor = base_model.detour_factor
    max_wait_time = base_model.max_wait_time

    nodes = _od_route_relevant_nodes(active_pairs)
    travel = _od_route_travel_lookup(data, nodes)

    t_start = time()
    exhausted = true
    columns = AggregateODRouteColumn[]
    next_id = 1

    function visit!(route::Vector{Int}, cum_time::Vector{Float64}, visit_counts::Dict{Int, Int})
        if time() - t_start > time_limit_sec
            exhausted = false
            return
        end
        if length(route) >= 2
            served = _od_route_served_pairs(route, cum_time, active_pairs, travel, detour_factor, max_wait_time)
            if !isempty(served)
                push!(columns, AggregateODRouteColumn(
                    next_id,
                    collect(served),
                    cum_time[end];
                    metadata=Dict{String, Any}(
                        "initialization" => "enumeration",
                        "route" => Tuple(route),
                    ),
                ))
                next_id += 1
                length(columns) <= max_routes ||
                    throw(ArgumentError("route enumeration exceeded max_routes=$(max_routes)"))
            end
        end
        length(route) >= max_stops && return
        for next_node in nodes
            next_node == route[end] && continue
            get(visit_counts, next_node, 0) < max_visits_per_node || continue
            new_time = cum_time[end] + travel[(route[end], next_node)]
            push!(route, next_node)
            push!(cum_time, new_time)
            visit_counts[next_node] = get(visit_counts, next_node, 0) + 1
            visit!(route, cum_time, visit_counts)
            pop!(route)
            pop!(cum_time)
            visit_counts[next_node] -= 1
            exhausted || return
        end
    end

    for start_node in nodes
        visit!(Int[start_node], Float64[0.0], Dict{Int, Int}(start_node => 1))
        exhausted || break
    end

    exhausted || throw(ArgumentError("route enumeration did not complete within time_limit_sec=$(time_limit_sec)"))

    append!(columns, mapping.columns)
    return _deduplicate_aggregate_od_route_columns(columns)
end
