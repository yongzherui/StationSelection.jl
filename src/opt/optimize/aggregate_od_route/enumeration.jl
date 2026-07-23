"""
Exhaustive route-column enumeration for aggregate OD route models.

The traversal is deliberately independent of the pricing search orchestration:
the pricer is optimized for finding negative-reduced-cost columns under a
particular dual vector, while direct solves need a dual-free route universe.
See `notes/2026-07-14_nearest_open_solver_alignment.md` for the historical
dominance-pruning gap that originally motivated the separate search.

This module instead does a plain bounded depth-first search over pricing
labels. It deliberately reuses the pricing problem's label initialization,
candidate-extension, pickup-cutoff, and pair-certification transitions, but
does no dominance or reduced-cost pruning. Thus exhaustive enumeration and
pricing have one definition of route service while retaining independent
search strategies.
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
    enumerate_aggregate_od_route_columns(model, data; max_routes=10_000, time_limit_sec=30.0)

Exhaustively enumerate feasible aggregate OD route columns via bounded DFS
over the same label transitions used by column-generation pricing (stations
are restricted to OD-pair endpoints because routing costs are all-pairs
shortest-path costs). Unlike pricing, this traversal applies neither
reduced-cost pruning nor label dominance. Every label serving at least one
active pair is emitted as a candidate column; the cheapest column per
distinct served-pairs signature is kept.
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

    max_visits_per_node = base_model.max_visits_per_node
    nodes = _od_route_relevant_nodes(active_pairs)
    max_stops = _resolve_aggregate_od_route_max_stops(
        base_model.max_stops,
        max_visits_per_node,
        length(nodes),
    )
    travel = _od_route_travel_lookup(data, nodes)
    pricing_data = AggregateODRoutePricingData(
        0,
        nodes,
        travel,
        active_pairs,
        base_model.route_regularization_weight,
        base_model.repositioning_time,
        base_model.max_wait_time,
        base_model.detour_factor,
        max_stops,
        max_visits_per_node,
        base_model.max_stops != typemax(Int),
    )
    # Uniform positive rewards make every active pair visible to the shared
    # pricing transitions. They do not prune or rank this DFS.
    enumeration_duals = AggregateODRoutePricingDuals(Dict(pair => 1.0 for pair in active_pairs))

    t_start = time()
    exhausted = true
    columns = AggregateODRouteColumn[]
    next_id = 1

    function visit!(label::AggregateODRoutePricingLabel)
        if time() - t_start > time_limit_sec
            exhausted = false
            return
        end
        if !isempty(label.served_pairs)
            push!(columns, AggregateODRouteColumn(
                next_id,
                collect(label.served_pairs),
                label.tau;
                metadata=Dict{String, Any}(
                    "initialization" => "enumeration",
                    "route" => Tuple(label.route),
                ),
            ))
            next_id += 1
            length(columns) <= max_routes ||
                throw(ArgumentError("route enumeration exceeded max_routes=$(max_routes)"))
        end
        label.route_length >= max_stops && return
        next_nodes = _aggregate_od_route_candidate_next_nodes(
            label,
            pricing_data,
            enumeration_duals;
            max_visits_per_node=max_visits_per_node,
        )
        for next_node in next_nodes
            for child in extend_aggregate_od_route_pricing_label(
                label,
                next_node,
                pricing_data,
                enumeration_duals,
            )
                visit!(child)
                exhausted || return
            end
        end
    end

    for initial_label in initial_aggregate_od_route_pricing_labels(pricing_data, enumeration_duals)
        visit!(initial_label)
        exhausted || break
    end

    exhausted || throw(ArgumentError("route enumeration did not complete within time_limit_sec=$(time_limit_sec)"))

    append!(columns, mapping.columns)
    return _deduplicate_aggregate_od_route_columns(columns)
end
