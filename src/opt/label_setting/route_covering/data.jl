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

function _prune_irrelevant_route_covering_station_ages(
    station_age::Dict{Int, Float64},
    served_pairs::Set{Tuple{Int, Int}},
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
    current::Int,
)
    remaining = Dict{Int, Float64}()
    for (station, age) in station_age
        useful = false
        for pair in pricing_data.active_pairs
            pair[1] == station || continue
            pair ∈ served_pairs && continue
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
