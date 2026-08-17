"""
Plain data containers for the AggregateODRouteProblem pricing label search.
"""

export AggregateODRoutePricingData
export AggregateODRoutePricingDuals
export AggregateODRoutePricingLabel

struct AggregateODRoutePricingData
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

struct AggregateODRoutePricingDuals
    sigma::Dict{Tuple{Int, Int}, Float64}
end

"""
A partial unlimited-capacity, synchronized-start route.

`time` is absolute time since the common `t=0` route/passenger start;
`station_age[j]` is ride time since the latest pickup-eligible visit to `j`;
and `served_pairs` records independently certified pairs. There is intentionally
no onboard passenger count or capacity resource. See `labels.jl` for the full
pricing contract.
"""
struct AggregateODRoutePricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const AggregateODRouteLabelId = Int
const AggregateODRouteLabelOrderKey = Tuple{Float64, Float64, Int, Int}

struct AggregateODRouteLabelBitsets
    served_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
end

struct AggregateODRouteDominanceFilters
    reduced_cost::Float64
    time::Float64
    age_mask::UInt64
    route_length::Int32
    n_live_ages::Int32
end

"""
    AggregateODRouteDominanceRules{BoundedStops, Compensated}

The two dominance switches, carried in the *type* rather than as `Bool`
arguments, for the same reason as Joint's twin
(`JointRoutingAssignmentDominanceRules`, `joint_routing_assignment/types.jl`):
they are constant for a whole pricing call, and encoding them as type
parameters lets the compiler delete a disabled `BoundedStops` branch outright
and specialize the reward-diff walk on `Compensated`.
"""
struct AggregateODRouteDominanceRules{BoundedStops, Compensated} <: AbstractPricingDominanceRules end

_aggregate_od_route_dominance_rules(bounded_max_stops::Bool, compensated_dominance::Bool) =
    AggregateODRouteDominanceRules{bounded_max_stops, compensated_dominance}()
