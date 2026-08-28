"""
Builds the per-scenario pricing graph (nodes, travel costs, active station OD
pairs, and the direct-ride-limit/reduced-cost helpers derived from it) that
the label search in `exact/` (seed.jl/extend.jl/prune.jl/dominate.jl/context.jl/hooks.jl) operates over.
"""

function _route_covering_travel(pricing_data::RouteCoveringPricingData, u::Int, v::Int)::Float64
    cost = get(pricing_data.travel_cost, (u, v), Inf)
    isfinite(cost) || throw(ArgumentError("missing finite routing cost for station arc $((u, v))"))
    return cost
end

function _direct_ride_limit(
    pricing_data::RouteCoveringPricingData,
    pair::Tuple{Int, Int},
)::Float64
    return pricing_data.detour_factor * _route_covering_travel(pricing_data, pair[1], pair[2])
end

function _route_covering_label_reduced_cost(
    tau::Float64,
    served_pairs,
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)::Float64
    dual_credit = sum(get(duals.sigma, pair, 0.0) for pair in served_pairs; init=0.0)
    return pricing_data.route_regularization_weight * (tau + pricing_data.repositioning_time) - dual_credit
end

function _certify_route_covering_pairs_at_node(
    node::Int,
    station_age::Dict{Int, Float64},
    travel_time::Float64,
    served_pairs::Set{Tuple{Int, Int}},
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
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

"""
Is a live clock at `station` (of age `age`, with the vehicle at `current`) still
reachable in time for *any* pair it could certify -- purely a ride-limit expiry
test, deliberately blind to whether those pairs are already in `served_pairs`.
`travel(current, dest)` is the *minimum* additional time before reaching any
destination -- detouring only adds more -- so this test is exact, not heuristic.

A clock used to also die once every reachable pair from it was already served,
on the reasoning that discarding it cost nothing since it represents a
potential certification, not an irrevocable commitment. That entangles the
clock -- a resource query, "can this station's age still reach a ride-limit
deadline" -- with `served_pairs`, which is *per-label* and differs between
whatever two labels a dominance check happens to compare (compensated
dominance explicitly allows one label to have served more pairs than another,
charged against its reduced-cost lead): dropping the clock the moment label
`a`'s own `served_pairs` makes it look exhausted removes it from `a`'s
`station_age` dict, so a later dominance comparison against some `b` that
still carries a live (but, from `b`'s perspective, likewise not-yet-useful)
clock at that station sees `a` as *lacking* a resource `b` has -- `a` reads as
strictly worse there even though the two are equivalent, and a domination that
held before a shared extension can silently stop holding after it. This is the
same bug class fixed for `joint_routing_assignment`'s twin
(`_joint_routing_assignment_age_is_useful`, `joint_routing_assignment/data.jl`)
in commit f644a7c ("Fix joint routing-assignment station-clock dominance
unsoundness") -- confirmed present here too via the randomized
dominance-preservation regression test in `test/opt/test_aggregate_od_route_pricing.jl`.
`route_covering/station_simple/`'s own pruning (`_route_covering_station_simple_prune_live_origins`)
does not have this problem: it keys off `visited`, which is required to be
*exactly equal* between any two labels a dominance check compares (it's part
of the state key itself, `_route_covering_station_simple_state`), so it can
never differ between `a` and `b` the way `served_pairs` can here.

The resource should only expire on its own physical term (the ride limit), not
on a label-local bookkeeping question; keeping it live doesn't cost anything
the dominance test doesn't already price in via the compensated reward-diff
budget.
"""
function _prune_irrelevant_route_covering_station_ages(
    station_age::Dict{Int, Float64},
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
    current::Int,
)
    remaining = Dict{Int, Float64}()
    for (station, age) in station_age
        useful = false
        for pair in pricing_data.active_pairs
            pair[1] == station || continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            t_to_dest = pair[2] == current ? 0.0 : _route_covering_travel(pricing_data, current, pair[2])
            age + t_to_dest <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            useful = true
            break
        end
        useful && (remaining[station] = age)
    end
    return remaining
end
