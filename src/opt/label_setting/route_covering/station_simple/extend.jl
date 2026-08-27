"""
How a label grows: which not-yet-visited nodes are legal to visit next
(`_route_covering_station_simple_candidate_next_nodes`, the
`_pricing_candidate_next_nodes` hook) and what visiting one produces
(`_extend_route_covering_station_simple_label`, the `_pricing_extend_label`
hook, both wired in `hooks.jl`) -- live-clock aging/reset/prune and pair
certification happen here. See `seed.jl` for where a label starts, `prune.jl`
for the bound that decides whether extending a label is worth it at all, and
`dominate.jl` for what happens to a child once it's built.
"""

# Drop any live pickup clock that can no longer reach an uncertified,
# positive-dual pair's destination in time -- once a station's clock is
# useless it's pure dead weight in the label state and dominance signature.
# Unlike the revisit-tolerant pricer, a destination already `visited` can
# never be certified later even by a different clock (a station is visited
# exactly once here), so `pair[2] in visited` alone is enough to rule a pair
# out, with no need to also check `served_pairs`.
function _route_covering_station_simple_prune_live_origins(
    live_origin_age::Dict{Int, Float64},
    current::Int,
    visited::Set{Int},
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Dict{Int, Float64}
    remaining = Dict{Int, Float64}()
    for (origin, age) in live_origin_age
        can_still_reward = false
        for pair in pricing_data.active_pairs
            pair[1] == origin || continue
            pair[2] in visited && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            age + _route_covering_travel(pricing_data, current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            can_still_reward = true
            break
        end
        can_still_reward && (remaining[origin] = age)
    end
    return remaining
end

# Every not-yet-visited node worth extending to next: either a dropoff for a
# clock already live, or (if not that) a fresh origin the route could still
# open in time. `next_node in label.visited && continue` up front is the one
# thing that makes this elementary rather than revisit-tolerant -- everywhere
# else the logic mirrors the revisit-tolerant candidate search.
function _route_covering_station_simple_candidate_next_nodes(
    label::RouteCoveringStationSimpleLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Vector{Int}
    candidates = Int[]
    for next_node in pricing_data.nodes
        next_node in label.visited && continue
        travel_time = _route_covering_travel(pricing_data, label.current, next_node)

        # Is `next_node` a live-clock's dropoff, reachable within its ride limit?
        is_useful = false
        for (origin, age) in label.live_origin_age
            pair = (origin, next_node)
            dual = get(duals.sigma, pair, 0.0)
            dual > 1e-9 || continue
            age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            is_useful = true
            break
        end

        # Not a dropoff -- is it worth visiting to open a fresh pickup clock
        # (reachable before the wait cutoff, and it's some uncertifiable
        # pair's origin)?
        if !is_useful && label.time + travel_time <= pricing_data.max_wait_time + 1e-9
            for pair in pricing_data.active_pairs
                pair[1] == next_node || continue
                pair[2] in label.visited && continue
                get(duals.sigma, pair, 0.0) > 1e-9 || continue
                is_useful = true
                break
            end
        end

        is_useful && push!(candidates, next_node)
    end
    return candidates
end

function _extend_route_covering_station_simple_label(
    label::RouteCoveringStationSimpleLabel,
    next_node::Int,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::RouteCoveringStationSimpleLabel
    next_node in label.visited && throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _route_covering_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_route = vcat(label.route, next_node)
    visited = copy(label.visited)
    push!(visited, next_node)

    # Certify every live clock's pair whose destination is `next_node` and
    # whose ride limit isn't exceeded -- same certification rule as the
    # revisit-tolerant pricer, just scanning `live_origin_age` (this label's
    # explicit clock set) instead of `_certify_route_covering_pairs_at_node`'s
    # scan over every active pair.
    served_pairs = copy(label.served_pairs)
    reward = 0.0
    for (origin, age) in label.live_origin_age
        pair = (origin, next_node)
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        if pair ∉ served_pairs
            push!(served_pairs, pair)
            reward += dual
        end
    end

    # Every existing clock ages by the travel time, same as the
    # revisit-tolerant pricer; then, if this visit opens a fresh clock (some
    # pair has `next_node` as its origin, positive dual) and we're still
    # within the wait cutoff, that clock starts live at age 0.
    live = Dict(origin => age + travel_time for (origin, age) in label.live_origin_age)
    opens_origin = any(
        pair -> pair[1] == next_node && get(duals.sigma, pair, 0.0) > 1e-9,
        pricing_data.active_pairs,
    )
    if opens_origin && arrival_time <= pricing_data.max_wait_time + 1e-9
        live[next_node] = 0.0
    end
    live = _route_covering_station_simple_prune_live_origins(live, next_node, visited, pricing_data, duals)

    new_tau = label.tau + travel_time
    return RouteCoveringStationSimpleLabel(
        next_node,
        new_route,
        visited,
        arrival_time,
        live,
        served_pairs,
        new_tau,
        # Same reduced-cost update rule as the revisit-tolerant pricer: pay
        # the regularized travel cost, credit back what was just certified.
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
    )
end
