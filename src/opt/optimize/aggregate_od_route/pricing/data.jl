"""
Builds the per-scenario pricing graph (nodes, travel costs, active station OD
pairs, and the direct-ride-limit/reduced-cost helpers derived from it) that
the label search in `labels.jl`/`search.jl` operates over.
"""

export create_aggregate_od_route_pricing_data

function _aggregate_od_route_travel(pricing_data::AggregateODRoutePricingData, u::Int, v::Int)::Float64
    cost = get(pricing_data.travel_cost, (u, v), Inf)
    isfinite(cost) || throw(ArgumentError("missing finite routing cost for station arc $((u, v))"))
    return cost
end

function _direct_ride_limit(
    pricing_data::AggregateODRoutePricingData,
    pair::Tuple{Int, Int},
)::Float64
    return pricing_data.detour_factor * _aggregate_od_route_travel(pricing_data, pair[1], pair[2])
end

function _aggregate_od_route_label_reduced_cost(
    tau::Float64,
    served_pairs,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Float64
    dual_credit = sum(get(duals.sigma, pair, 0.0) for pair in served_pairs; init=0.0)
    return pricing_data.route_regularization_weight * (tau + pricing_data.repositioning_time) - dual_credit
end

"""
Resolve the finite route-length ceiling shared by pricing and exhaustive
enumeration. With unbounded `max_stops` and a finite per-node visit limit,
`n_nodes * max_visits_per_node` is the exact combinatorial ceiling. If both
limits are unbounded, exhaustive enumeration (no dominance, no reduced-cost
pruning) has no finite DFS depth and cannot terminate -- that case is a hard
error here. Label-setting pricing does not share this requirement; see
`_resolve_aggregate_od_route_pricing_max_stops` below.
"""
function _resolve_aggregate_od_route_max_stops(
    max_stops::Int,
    max_visits_per_node::Int,
    n_nodes::Int,
)::Int
    max_stops != typemax(Int) && return max_stops
    max_visits_per_node != typemax(Int) || throw(ArgumentError(
        "AggregateODRouteModel route search requires a finite max_stops or " *
        "max_visits_per_node; both are currently unbounded",
    ))
    try
        return Base.Checked.checked_mul(n_nodes, max_visits_per_node)
    catch err
        err isa OverflowError || rethrow()
        throw(ArgumentError(
            "aggregate OD route-length bound n_nodes * max_visits_per_node overflows Int",
        ))
    end
end

"""
Resolve the route-length ceiling for label-setting pricing. Unlike exhaustive
enumeration, the labeling search in `search.jl` terminates via label dominance
and reduced-cost pruning even with no depth ceiling at all -- `bounded_max_stops`
already tells the dominance/comparison code to ignore route_length when
`max_stops` is unbounded (see `labels.jl`), so `route_length >=
pricing_data.max_stops` at the top of the search loop only needs to be a
no-op in that case, not an error. Both limits unbounded is therefore a
legitimate "run pricing with no artificial route-length cap" configuration,
not a misconfiguration.
"""
function _resolve_aggregate_od_route_pricing_max_stops(
    max_stops::Int,
    max_visits_per_node::Int,
    n_nodes::Int,
)::Int
    max_stops != typemax(Int) && return max_stops
    max_visits_per_node == typemax(Int) && return typemax(Int)
    try
        return Base.Checked.checked_mul(n_nodes, max_visits_per_node)
    catch err
        err isa OverflowError || rethrow()
        throw(ArgumentError(
            "aggregate OD route-length bound n_nodes * max_visits_per_node overflows Int",
        ))
    end
end

function create_aggregate_od_route_pricing_data(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    scenario::Int,
    pricing_duals::Union{AggregateODRoutePricingDuals, Nothing}=nothing,
)::AggregateODRoutePricingData
    1 <= scenario <= n_scenarios(data) ||
        throw(ArgumentError("scenario index $scenario is out of range"))
    has_routing_costs(data) ||
        throw(ArgumentError("AggregateODRouteModel pricing requires routing_costs"))

    nodes = collect(1:data.n_stations)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    missing_arcs = Tuple{Int, Int}[]
    for i in nodes, j in nodes
        i == j && continue
        cost = get_routing_cost(data, i, j)
        if isfinite(cost)
            travel_cost[(i, j)] = cost
        else
            push!(missing_arcs, (i, j))
        end
    end
    isempty(missing_arcs) ||
        throw(ArgumentError("missing finite routing costs for aggregate OD route pricing arcs: $(missing_arcs)"))

    all_pairs = filter(!requires_no_vehicle_route, get(mapping.active_jk_s, scenario, Tuple{Int, Int}[]))
    active_pairs = if isnothing(pricing_duals)
        all_pairs
    else
        filter(pair -> get(pricing_duals.sigma, pair, 0.0) > 1e-9, all_pairs)
    end

    bounded_max_stops = model.max_stops != typemax(Int)
    max_stops = _resolve_aggregate_od_route_pricing_max_stops(
        model.max_stops,
        model.max_visits_per_node,
        length(nodes),
    )
    return AggregateODRoutePricingData(
        scenario,
        nodes,
        travel_cost,
        active_pairs,
        model.route_regularization_weight,
        model.repositioning_time,
        model.max_wait_time,
        model.detour_factor,
        max_stops,
        model.max_visits_per_node,
        bounded_max_stops,
    )
end

function _certify_aggregate_od_route_pairs_at_node(
    node::Int,
    station_age::Dict{Int, Float64},
    travel_time::Float64,
    served_pairs::Set{Tuple{Int, Int}},
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)
    certified_pairs = copy(served_pairs)
    reward = 0.0
    for pair in pricing_data.active_pairs
        pair[2] == node || continue
        pair ∈ certified_pairs && continue
        pair_reward = get(duals.sigma, pair, 0.0)
        pair_reward > 1e-9 || continue
        origin_age = get(station_age, pair[1], Inf)
        origin_age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        push!(certified_pairs, pair)
        reward += pair_reward
    end
    return certified_pairs, reward
end

function _prune_irrelevant_aggregate_od_route_station_ages(
    station_age::Dict{Int, Float64},
    served_pairs::Set{Tuple{Int, Int}},
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
    current::Int,
)
    remaining = Dict{Int, Float64}()
    for (station, age) in station_age
        useful = false
        for pair in pricing_data.active_pairs
            pair[1] == station || continue
            pair ∈ served_pairs && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            t_to_dest = pair[2] == current ? 0.0 : _aggregate_od_route_travel(pricing_data, current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            useful = true
            break
        end
        useful && (remaining[station] = age)
    end
    return remaining
end
