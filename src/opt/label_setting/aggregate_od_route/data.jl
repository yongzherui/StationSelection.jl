"""
Builds the per-scenario pricing graph (nodes, travel costs, active station OD
pairs, and the direct-ride-limit/reduced-cost helpers derived from it) that
the label search in `labels.jl`/`search.jl` operates over.
"""

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
Resolve the finite route-length ceiling required by exhaustive enumeration.
Exhaustive enumeration (no dominance, no reduced-cost pruning) has no finite
DFS depth and cannot terminate without a finite `max_stops` -- that case is a
hard error here. Label-setting pricing does not share this requirement; see
`_resolve_aggregate_od_route_pricing_max_stops` below.
"""
function _resolve_aggregate_od_route_max_stops(max_stops::Int)::Int
    max_stops != typemax(Int) && return max_stops
    throw(ArgumentError(
        "AggregateODRouteProblem route search requires a finite max_stops",
    ))
end

"""
Resolve the route-length ceiling for label-setting pricing. Unlike exhaustive
enumeration, the labeling search in `search.jl` terminates via label dominance
and reduced-cost pruning even with no depth ceiling at all -- `bounded_max_stops`
already tells the dominance/comparison code to ignore route_length when
`max_stops` is unbounded (see `labels.jl`), so `route_length >=
pricing_data.max_stops` at the top of the search loop only needs to be a
no-op in that case, not an error. Unbounded `max_stops` is therefore a
legitimate "run pricing with no artificial route-length cap" configuration,
not a misconfiguration.
"""
function _resolve_aggregate_od_route_pricing_max_stops(max_stops::Int)::Int
    return max_stops
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
