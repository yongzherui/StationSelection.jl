"""
Plain data containers for the passenger free-assignment pricing label search.

This is a sibling pricer to `AggregateODRoutePricingData`/`AggregateODRoutePricingLabel`
(see `../types.jl`), not a replacement: it solves a different subproblem, over a
different reward structure (per-passenger maximum certified reward rather than an
aggregate sum over independently-served station pairs), so it gets its own label
and data types rather than overloading the pair-based ones that the Benders/CG
stack already depends on.
"""

export RewardLayerBitset
export PassengerAssignmentCandidate
export PassengerAssignmentOpportunity
export PassengerFreeAssignmentPricingData
export PassengerFreeAssignmentPricingLabel
export PassengerFreeAssignmentRouteColumn

"""
One global bit per passenger reward layer `(p, h)`. See the module docstring in
`data.jl` for how candidates are turned into layers.
"""
const RewardLayerBitset = BitSet

"""
    PassengerAssignmentCandidate(passenger, origin, destination, ride_limit, reward)

Raw input to pricing: one feasible passenger assignment `(p, j, k)` with its
already-computed reward `ρ_pjk = α_p - γ^O_pj - γ^D_pk - w_pjk` and its
passenger-specific ride-time/detour limit `R_pjk`. Only candidates with
`reward > 0` matter for pricing (see `_build_passenger_reward_layers`).
"""
struct PassengerAssignmentCandidate
    passenger::Int
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
    passenger::Int
    origin::Int
    destination::Int
    ride_limit::Float64
    reward::Float64
    layer_mask::RewardLayerBitset
end

struct PassengerFreeAssignmentPricingData
    scenario::Int
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    max_stops::Int
    max_visits_per_node::Int
    bounded_max_stops::Bool
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
A partial unlimited-capacity, synchronized-start route, exactly as in
`AggregateODRoutePricingLabel` (`current`, `route`, `time`, `station_age`, `tau`,
`reduced_cost`, `route_length` all carry the same meaning). The one substantive
change is `activated_reward_layers` in place of `served_pairs`: since a passenger
selects a single, best, certified assignment rather than being "served" by every
pair the route happens to certify, the label only needs to remember the highest
reward level certified so far per passenger -- not which concrete `(j, k)` pairs
were involved. See `labels.jl` for the full pricing contract.
"""
struct PassengerFreeAssignmentPricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    activated_reward_layers::RewardLayerBitset
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const PassengerFreeAssignmentLabelId = Int
const PassengerFreeAssignmentLabelOrderKey = Tuple{Float64, Float64, Int, Int}

"""
Hot-path mirror of a label's pruning-relevant state.

`station_age` is held **sparsely** as parallel sorted arrays rather than a dense
`Vector{Float64}` of length `n_nodes`. Aggressive age pruning means only a
handful of origins are ever live, so a dense vector costs `O(n_nodes)` to
allocate and to scan on every dominance test, while the sparse form costs
`O(#live)`. This matters most in the unbounded-`max_stops` regime, where label
counts are far larger and dominance is the main thing keeping the search finite.

`age_idx` is sorted ascending; `age_val` is parallel to it.
"""
struct PassengerFreeAssignmentLabelBitsets
    activated_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
end

"""
    PassengerFreeAssignmentRouteColumn(id, route, assignments, tau; metadata)

A priced column: a physical station route paired with the concrete per-passenger
assignments `(passenger, pickup, dropoff)` selected during route replay (see
`search.jl`). Unlike `activated_reward_layers`, which only records reward levels,
`assignments` records which stations actually carry the linking coefficients a
master problem would need.
"""
struct PassengerFreeAssignmentRouteColumn
    id::Int
    route::Vector{Int}
    assignments::Vector{Tuple{Int, Int, Int}}
    tau::Float64
    metadata::Dict{String, Any}

    function PassengerFreeAssignmentRouteColumn(
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
