"""
Builds the per-scenario pricing graph (nodes, travel costs, active station OD
pairs, and the direct-ride-limit/reduced-cost helpers derived from it) that
the label search in `labels.jl`/`exact.jl` operates over.
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
