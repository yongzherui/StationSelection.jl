"""
Label/bitsets/dominance types specific to the elementary-route passenger
free-assignment pricer. See `../types.jl` for the pricing graph/reward-layer
types this pricer shares with `exact/`, and `seed.jl` / `extend.jl` /
`dominate.jl` / `context.jl` / `hooks.jl` for the label-setting functionality
built on top of the types below -- `seed.jl` is the file to start from for
"is the label search correct". This pricer has no `prune.jl`/`accept.jl` of
its own: it reuses `../exact/prune.jl`'s remaining-reward bound and
`../exact/accept.jl`'s route replay directly (both wired in `hooks.jl`),
since neither depends on how the physical route was found.

An alternative to the revisit-tolerant search in `../exact/` (types.jl/
seed.jl/extend.jl/prune.jl/dominate.jl/context.jl/accept.jl/hooks.jl), in
which a physical route may never revisit a station.

# What elementarity changes (and what it does not)

Only the *route universe* changes -- the reward contract is identical to the
revisit-tolerant pricer (see `../exact/types.jl`'s module docstring): a visit to origin
`j` within `max_wait_time` opens a live pickup clock; a later visit to `k`
certifies `(p, j, k)` for passengers whose clock survives the ride limit; and a
passenger banks only its single best certified reward, tracked incrementally via
`activated_reward_layers`.

Crucially, the per-passenger *maximum* reward means elementarity does NOT let us
drop the layer/age bookkeeping the way the aggregate pair-based
`../../route_covering/station_simple/extend.jl` drops `station_age` for
`live_origin_age`: reaching a strictly better dropoff later still activates
incremental layers, so a live clock stays useful even after it has already
certified something. What elementarity removes is clock *resets* -- a station
is visited exactly once, so a clock only ages and is never reopened.

# Why this is faster

Two levers:

1. **Fewer extensions.** Candidate generation drops any already-visited node, so
   the branching factor shrinks as a route grows.

2. **Fine dominance states (`dominance_mode = :exact`, the default).** The state
   is the exact `(current, visited)` pair, so each state's label list is tiny and
   every insertion's dominance scan is short. That is the whole game here: the
   scan is O(list size) per insertion and ~85-90% of wall time, so state
   *granularity* dominates. At n=20 this makes the elementary search 1.6-3.5x
   faster than the revisit-tolerant pricer.

   A `:subset` mode also exists (state = `current` alone, add `U_a ⊆ U_b` to
   dominance). It is a strictly stronger dominance and keeps ~2x fewer live labels,
   yet it is **1.4-6.6x slower** than `:exact` because its coarse `current`-only
   states grow label lists to tens of thousands of entries and the per-insertion
   scan blows up -- fewer labels do not pay for scanning giant lists. Retained for
   research only; measured verdict and numbers in
   `notes/2026-07-30_passenger_station_simple_pricing.md`.

# Correctness caveat

Restricting to elementary routes restricts the column universe the master problem
prices over. Where the model's optimum genuinely wants a revisiting route this
pricer is a *heuristic* -- it can terminate CG with a weaker LP bound or miss
improving columns (the aggregate pair-based `use_station_simple` did exactly this
on some instance families). It is therefore opt-in and off by default; validate
the LP bound against the revisit-tolerant pricer before relying on it.

# Reuse

Shares `JointRoutingAssignmentPricingData` (no new data struct) and the
`_joint_routing_assignment_travel`, `_certify_joint_routing_assignment_layers_at_node`,
`_joint_routing_assignment_age_is_useful`, `_has_useful_live_joint_routing_assignment_origin`,
`_joint_routing_assignment_compensation`, and `_joint_routing_assignment_remaining_reward_bound`
primitives from `../data.jl`/`../exact/extend.jl`/`../exact/dominate.jl`/`../exact/prune.jl`. Emits the same
`JointRoutingAssignmentRouteColumn` via the identical route-replay path
(`_joint_routing_assignment_column_from_route`), since replay is agnostic to how
the physical route was found and replays an elementary route unchanged.
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
so mirroring them would only add a redundant copy -- see `dominate.jl`'s dominance
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
