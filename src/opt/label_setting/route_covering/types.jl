"""
Plain data containers shared by every AggregateODRouteProblem pricer variant
(`exact/` and `station_simple/`): the pricing graph itself and the duals it
prices against. Each variant's own label/bitsets/dominance types live under
its own subdirectory instead (`exact/types.jl`) -- `RouteCoveringPricingLabel`
and friends are specific to the revisit-tolerant search, not shared with
`station_simple/`'s elementary-route label.
"""

export RouteCoveringPricingData
export RouteCoveringPricingDuals

struct RouteCoveringPricingData
    scenario::Int
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    active_pairs::Vector{Tuple{Int, Int}}
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    bounded_max_stops::Bool
    # Whether dominance uses the compensated reward test `rc_a + w(A_a \ A_b) <= rc_b`
    # or the plain `A_a subseteq A_b`. A toggle for the same reason as Joint's twin
    # field (`joint_routing_assignment/types.jl`): it trades column diversity per
    # search for speed, and which side wins for column generation is an end-to-end
    # question, not a pricing-speed one.
    compensated_dominance::Bool
end

struct RouteCoveringPricingDuals
    sigma::Dict{Tuple{Int, Int}, Float64}
end
