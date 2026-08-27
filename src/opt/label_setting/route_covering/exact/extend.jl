"""
How a label grows: which nodes are legal to visit next
(`_route_covering_candidate_next_nodes`, the `_pricing_candidate_next_nodes`
hook) and what visiting one produces (`_extend_route_covering_pricing_label`,
the `_pricing_extend_label` hook, both wired in `hooks.jl`) -- station-age
aging/reset/prune and pair certification happen here. See `seed.jl` for where
a label starts, `prune.jl` for the bound that decides whether extending a
label is worth it at all, and `dominate.jl` for what happens to a child once
it's built.
"""

export extend_route_covering_pricing_label

# True iff `label` still has a live pickup clock (a station visited, whose
# clock hasn't been pruned) that could still reach some uncertified pair's
# destination in time. Used only after the pickup cutoff has passed (see
# below) to decide whether a route with no more pickups ahead is nonetheless
# still worth extending toward a dropoff it already set up.
function _has_useful_live_route_covering_origin(
    label::RouteCoveringPricingLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Bool
    for (station, age) in label.station_age
        for pair in pricing_data.active_pairs
            pair[1] == station || continue                              # station must be this pair's origin
            pair ∈ label.served_pairs && continue                       # already certified, nothing to gain
            get(duals.sigma, pair, 0.0) > 1e-9 || continue               # not worth pursuing under current duals
            t_to_dest = pair[2] == label.current ? 0.0 : _route_covering_travel(pricing_data, label.current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue  # dropoff still reachable in time
            return true
        end
    end
    return false
end

# Every node the search may legally extend `label` to next: either a fresh
# pickup (a pair's origin, reachable before its wait cutoff) or a dropoff for
# a pickup clock already live (a pair's destination, reachable within its
# ride limit). Both are gated on `get(duals.sigma, pair, 0.0) > 1e-9` -- a
# pair with no positive dual reward can't improve reduced cost, so there's no
# point ever visiting either of its endpoints for its sake.
function _route_covering_candidate_next_nodes(
    label::RouteCoveringPricingLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    # Once no fresh pickup is possible (past cutoff), the route is only worth
    # continuing if some already-open pickup clock can still reach a
    # dropoff -- otherwise every future extension is dead weight.
    if past_pickup_cutoff && !_has_useful_live_route_covering_origin(label, pricing_data, duals)
        return Int[]
    end
    for pair in pricing_data.active_pairs
        get(duals.sigma, pair, 0.0) > 1e-9 || continue
        origin, destination = pair
        remembered = pair ∈ label.served_pairs

        # Fresh-pickup candidate: not yet certified, not already standing at
        # the origin, and reachable before this pair's wait cutoff.
        if !remembered && origin != label.current
            arrival_time = label.time + _route_covering_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end

        # Dropoff candidate: skip if already certified or already standing at
        # the destination; otherwise offer it if the origin's pickup clock
        # (if any -- `Inf` if never visited) can still reach it in time.
        remembered && continue
        destination == label.current && continue
        origin_age = get(label.station_age, origin, Inf)
        origin_age + _route_covering_travel(pricing_data, label.current, destination) <=
            _direct_ride_limit(pricing_data, pair) + 1e-9 && push!(candidate_nodes, destination)
    end

    return sort!(collect(candidate_nodes))
end

# Public wrapper: this pricer only ever produces one child per next-node (no
# branching within a single extension), but `_run_label_setting`'s engine
# calls this as a `label -> [children...]`-shaped hook so pricers that *can*
# branch (e.g. multiple ways to serve the same node) share the same call
# site. See `_extend_route_covering_pricing_label` for the actual mechanics.
function extend_route_covering_pricing_label(
    label::RouteCoveringPricingLabel,
    next_node::Int,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{RouteCoveringPricingLabel}
    return RouteCoveringPricingLabel[
        _extend_route_covering_pricing_label(label, next_node, pricing_data, duals),
    ]
end

function _extend_route_covering_pricing_label(
    label::RouteCoveringPricingLabel,
    next_node::Int,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::RouteCoveringPricingLabel
    travel_time = _route_covering_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    # Every existing live pickup clock ages by the travel time to get here --
    # this happens unconditionally, before checking what this visit certifies.
    aged_station = Dict(station => age + travel_time for (station, age) in label.station_age)

    # Certify every pair whose destination is `next_node` and whose origin's
    # (aged) pickup clock still beats its ride limit; `reward` is the summed
    # dual value of everything just certified (see data.jl).
    certified_pairs, reward =
        _certify_route_covering_pairs_at_node(
            next_node,
            label.station_age,
            travel_time,
            label.served_pairs,
            pricing_data,
            duals,
        )
    # This visit also opens a fresh pickup clock at `next_node` itself, but
    # only if we're still within its own wait cutoff -- arriving late means
    # `next_node` can pick up nothing, so no clock is worth starting.
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        aged_station[next_node] = 0.0
    end
    # Drop any clock (including the one possibly just opened above) that can
    # no longer reach any uncertified pair's destination in time -- keeps the
    # label's state lean for the dominance/bitset machinery downstream.
    aged_station = _prune_irrelevant_route_covering_station_ages(
        aged_station,
        certified_pairs,
        pricing_data,
        duals,
        next_node,
    )

    return RouteCoveringPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_pairs,
        new_tau,
        # Reduced cost accrues the travel's regularized cost and is credited
        # back the reward of whatever just got certified -- this is the
        # quantity the search's priority/pruning is driven by, not `tau`.
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )

end
