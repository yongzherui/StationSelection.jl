"""
Plain data containers shared by every passenger free-assignment pricer variant
(`exact/` and `station_simple/`): the pricing graph/reward-layer types and the
final route-column type. Each variant's own label/bitsets/dominance types live
under its own subdirectory instead (`exact/types.jl`) --
`JointRoutingAssignmentPricingLabel` and friends are specific to the
revisit-tolerant search, not shared with `station_simple/`'s elementary-route
label.

This is a sibling pricer to `RouteCoveringPricingData`/`RouteCoveringPricingLabel`
(see `../types.jl`), not a replacement: it solves a different subproblem, over a
different reward structure (per-passenger maximum certified reward rather than an
aggregate sum over independently-served station pairs), so it gets its own label
and data types rather than overloading the pair-based ones that the Benders/CG
stack already depends on.
"""

export RewardLayerBitset
export PassengerAssignmentCandidate
export PassengerAssignmentOpportunity
export JointRoutingAssignmentPricingData
export JointRoutingAssignmentRouteColumn

"""
One global bit per passenger reward layer `(p, h)`. See the module docstring in
`data.jl` for how candidates are turned into layers.
"""
const RewardLayerBitset = BitSet

"""
    PassengerAssignmentCandidate(p, origin, destination, ride_limit, reward)

Raw input to pricing: one feasible passenger assignment `(p, j, k)` with its
already-computed reward `ρ_pjk = α_p - γ^O_pj - γ^D_pk - w_pjk` and its
passenger-specific ride-time/detour limit `R_pjk`. `p` is the demand group's
index -- the position of `(o,d)` within `mapping.Omega_s[scenario]`. Only
candidates with `reward > 0` matter for pricing (see `_build_passenger_reward_layers`).
"""
struct PassengerAssignmentCandidate
    p::Int
    origin::Int
    destination::Int
    ride_limit::Float64
    reward::Float64
end

"""
A passenger assignment opportunity as consumed at search time: the candidate's
reward has been folded into `layer_mask`, the prefix of that passenger's reward
layers activated by certifying this particular `(j, k)`.
"""
struct PassengerAssignmentOpportunity
    p::Int
    origin::Int
    destination::Int
    ride_limit::Float64
    reward::Float64
    layer_mask::RewardLayerBitset
end

struct JointRoutingAssignmentPricingData
    scenario::Int
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    max_stops::Int
    bounded_max_stops::Bool
    # Whether dominance uses the compensated reward test `rc_a + w(A_a \ A_b) <= rc_b`
    # or the older plain `A_a subseteq A_b`. A toggle only because the compensated
    # rule trades away column diversity per search for speed -- measured 2.5-3.9x
    # faster but ~50% fewer distinct columns harvested -- and which side wins for
    # column generation is an end-to-end question, not a pricing-speed one.
    compensated_dominance::Bool
    n_layers::Int
    layer_weight::Vector{Float64}
    assignment_layer_mask::Dict{Tuple{Int, Int, Int}, RewardLayerBitset}
    assignments_by_destination::Dict{Int, Vector{PassengerAssignmentOpportunity}}
    assignments_by_origin::Dict{Int, Vector{PassengerAssignmentOpportunity}}
    origin_layer_mask::Dict{Int, RewardLayerBitset}
    destination_layer_mask::Dict{Int, RewardLayerBitset}
    opportunities::Vector{PassengerAssignmentOpportunity}
end

"""
    JointRoutingAssignmentRouteColumn(id, route, assignments, tau; metadata)

A priced column: a physical station route paired with the concrete per-passenger
assignments `(p, pickup, dropoff)` selected during route replay (see
`exact/exact.jl`). Unlike `activated_reward_layers`, which only records reward
levels, `assignments` records which stations actually carry the linking
coefficients a master problem would need.
"""
struct JointRoutingAssignmentRouteColumn
    id::Int
    route::Vector{Int}
    assignments::Vector{Tuple{Int, Int, Int}}
    tau::Float64
    metadata::Dict{String, Any}

    function JointRoutingAssignmentRouteColumn(
            id::Int,
            route::AbstractVector{<:Integer},
            assignments::AbstractVector{<:Tuple{Int, Int, Int}},
            tau::Number;
            metadata::Dict{String, Any}=Dict{String, Any}()
        )
        id > 0 || throw(ArgumentError("column id must be positive"))
        isempty(assignments) && throw(ArgumentError(
            "passenger free-assignment route column must cover at least one passenger assignment",
        ))
        tau >= 0 || throw(ArgumentError("tau must be non-negative"))
        unique_assignments = unique(Tuple{Int, Int, Int}.(assignments))
        new(id, collect(Int, route), unique_assignments, Float64(tau), metadata)
    end
end
