"""
The admissible remaining-reward bound behind frontier priority and pop-time
pruning (`_pricing_label_priority`, wired in `hooks.jl`): `_run_label_setting`
(`engine.jl`) uses `label.reduced_cost - bound` both to order the frontier and,
at pop time, to stop extending a label no completion of it could ever improve
on. Unlike the revisit-tolerant twin (`../exact/prune.jl`), this pricer has no
travel matrix or per-pair arrays to precompute, so the bound just iterates
`active_pairs` directly and needs nothing off the context struct beyond
`pricing_data`/`duals` -- no dependency on `context.jl`.
"""

# Upper bound on collectible reward from this label onward. Already-open origins
# need the detour check against their current age; not-yet-visited origins only
# need to still be reachable within the pickup window (ignoring detour, since
# triangle inequality makes direct travel a lower bound on any routed arrival).
function _route_covering_station_simple_future_reward_bound(
    label::RouteCoveringStationSimpleLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Float64
    ub = 0.0
    for pair in pricing_data.active_pairs
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        pair[2] in label.visited && continue
        if pair[1] in label.visited
            age = get(label.live_origin_age, pair[1], Inf)
            isinf(age) && continue
            age + _route_covering_travel(pricing_data, label.current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        else
            label.time + _route_covering_travel(pricing_data, label.current, pair[1]) <=
                pricing_data.max_wait_time + 1e-9 || continue
        end
        ub += dual
    end
    return ub
end

_route_covering_station_simple_label_priority(
    label::RouteCoveringStationSimpleLabel,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
) = label.reduced_cost - _route_covering_station_simple_future_reward_bound(label, pricing_data, duals)
