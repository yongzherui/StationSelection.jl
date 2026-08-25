"""
Label/bitsets/dominance types specific to the elementary-route passenger
free-assignment pricer (`station_simple.jl`). See `../types.jl` for the
pricing graph/reward-layer types this pricer shares with `exact/`.
"""

export JointRoutingAssignmentStationSimpleLabel

"""
A partial elementary route. Same fields as `JointRoutingAssignmentPricingLabel`
plus an authoritative `visited` set. `route_length == length(visited)` always
holds here.

`visited` is a `BitSet` (over station ids), not a `Set{Int}`: the dominance scan's
`issubset(a.visited, b.visited)` is then a word-wise AND rather than a per-element
hash probe, and it is compared *directly* -- no separate node-index bitset has to
be rebuilt per label. `activated_reward_layers` is likewise already a `BitSet` and
is compared directly, so the only per-label precompute the search still needs is
the sorted live-age arrays (`JointRoutingAssignmentStationSimpleAges`).
"""
struct JointRoutingAssignmentStationSimpleLabel
    current::Int
    route::Vector{Int}
    visited::BitSet
    time::Float64
    station_age::Dict{Int, Float64}
    activated_reward_layers::RewardLayerBitset
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

"""
The one piece of per-label state the dominance scan can't read straight off the
label: the live pickup clocks, held as parallel sorted arrays (`age_idx` sorted
ascending in node-index space, `age_val` parallel) so the age test is an O(#live)
merge walk rather than a Dict scan. `visited` and `activated_reward_layers` are
already `BitSet`s on the label and are compared there directly, so nothing else
needs mirroring.
"""
struct JointRoutingAssignmentStationSimpleAges
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    # One bit per live station index (folded mod 64) -- the `dom(age_b) ⊆
    # dom(age_a)` half of the age condition as a single instruction, so the merge
    # walk below only runs on pairs that can still pass it. See
    # `JointRoutingAssignmentLabelBitsets.age_mask` for why folding is safe.
    age_mask::UInt64
end

"""
Scalar dominance state, copied out of the label so the state's label-list scan
rejects the common case without dereferencing `label`/`ages` at all -- same rationale as
`JointRoutingAssignmentDominanceFilters` (`../exact/types.jl`). `visited` and
`activated_reward_layers` are not carried here (unlike the revisit-tolerant
pricer's filters): both are already `BitSet`s on the label, compared directly,
so mirroring them would only add a redundant copy -- see `labels.jl`'s dominance
docstrings.
"""
struct JointRoutingAssignmentStationSimpleDominanceFilters
    reduced_cost::Float64
    time::Float64
    route_length::Int32
    n_live_ages::Int32
end

"""Dominance-rule marker for the passenger station-simple pricer; no switches
yet, unlike the revisit-tolerant pricer's four (elementary routes have no
analogous optional caps to toggle)."""
struct JointRoutingAssignmentStationSimpleDominanceRules <: AbstractPricingDominanceRules end
