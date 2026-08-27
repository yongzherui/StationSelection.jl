"""
Label/bitsets/dominance types specific to the revisit-tolerant passenger
free-assignment pricer, plus the operational contract those types encode. See
`../types.jl` for the pricing graph/reward-layer types this pricer shares
with `station_simple/`, and `seed.jl` / `extend.jl` / `prune.jl` /
`dominate.jl` / `context.jl` / `accept.jl` / `hooks.jl` for the label-setting
functionality built on top of the types below (plus `logging.jl`, the
dominance rejection-census dev tooling split out of `dominate.jl`) --
`seed.jl` is the file to start from for "is the label search correct".

# Operational contract of the column being priced

Same unlimited-capacity, synchronized-start physical route as
`RouteCoveringPricingLabel` (see that file's module docstring for the shared
station-age/wait/detour rules) -- this pricer changes only what a route gets
*credit* for:

- a visit to origin `j` within `max_wait_time` opens a live pickup clock for `j`,
  exactly as before;
- a later visit to `k` certifies `(p, j, k)` for every passenger `p` with a
  positive-reward candidate on that pair, when elapsed time since the pickup is
  at most that candidate's own `ride_limit` (passenger-specific, not a single
  `detour_factor * travel_time` shared by all pairs);
- but a passenger is not "served" by the union of everything certified -- of all
  the assignments the finished route certifies for `p`, only the single best one
  counts. `activated_reward_layers` tracks running per-passenger maxima (via the
  prefix-layer encoding from `data.jl`) so labels can compare/dominate on this
  without carrying dense per-passenger reward vectors or a discrete "which
  assignment is current best" pointer -- reaching a strictly better destination
  later activates only the incremental layers between the old and new reward, so
  the layer weight sum is always exactly `sum(max reward per passenger)`, never a
  double count. Reaching a worse destination afterwards activates nothing (its
  prefix mask is already a subset of what's active).

No onboard-passenger state, capacity resource, or drop-off subset enumeration is
modeled -- unbounded capacity means certifying `p`'s assignment never prevents
certifying anyone else's, so there is nothing to branch on beyond the physical
route itself. The concrete `(j, k)` a passenger is ultimately assigned to is
reconstructed only for finished, negative-reduced-cost routes (`accept.jl`'s
route replay) -- not tracked while labels are being extended.
"""

export JointRoutingAssignmentPricingLabel

"""
A partial unlimited-capacity, synchronized-start route, exactly as in
`RouteCoveringPricingLabel` (`current`, `route`, `time`, `station_age`, `tau`,
`reduced_cost`, `route_length` all carry the same meaning). The one substantive
change is `activated_reward_layers` in place of `served_pairs`: since a passenger
selects a single, best, certified assignment rather than being "served" by every
pair the route happens to certify, the label only needs to remember the highest
reward level certified so far per passenger -- not which concrete `(j, k)` pairs
were involved. See this file's own module docstring above for the full pricing
contract.
"""
struct JointRoutingAssignmentPricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    activated_reward_layers::RewardLayerBitset
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const JointRoutingAssignmentLabelId = Int
const JointRoutingAssignmentLabelOrderKey = Tuple{Float64, Float64, Int, Int}

"""
Hot-path mirror of a label's pruning-relevant state.

`station_age` is held **sparsely** as parallel sorted arrays rather than a dense
`Vector{Float64}` of length `n_nodes`. Aggressive age pruning means only a
handful of origins are ever live, so a dense vector costs `O(n_nodes)` to
allocate and to scan on every dominance test, while the sparse form costs
`O(#live)`. This matters most in the unbounded-`max_stops` regime, where label
counts are far larger and dominance is the main thing keeping the search finite.

`age_idx` is sorted ascending; `age_val` is parallel to it.

`age_mask` is a one-bit-per-live-station summary of `age_idx`, and exists purely
as a prefilter for the station-age merge walk, which profiling put at ~20% of the
search once the reward-layer compensation had been moved behind it. Dominance
requires `dom(age_b) ⊆ dom(age_a)`, so `age_mask_b & ~age_mask_a == 0` is
necessary, and it is one instruction against a walk over two heap arrays.

Station indices are folded modulo 64 (`(idx - 1) & 63`). Below 64 nodes that is
exact; above it, two stations can share a bit, which can only make the filter
*weaker* -- it may let a pair through that the walk then rejects, never the
reverse -- so no correctness case depends on the node count.
"""
struct JointRoutingAssignmentLabelBitsets
    activated_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
end

"""
Every piece of label state the dominance test can check with a scalar comparison,
in one `isbits` value so it can be *stored inline* in a label entry and passed by
value. `Int32` for the two counts is what keeps this at 40 bytes, and the entry
that embeds it inside one 64-byte cache line.
"""
struct JointRoutingAssignmentDominanceFilters
    reduced_cost::Float64
    time::Float64
    age_mask::UInt64
    route_length::Int32
    n_live_ages::Int32
end

function JointRoutingAssignmentDominanceFilters(
    label::JointRoutingAssignmentPricingLabel,
    bitsets::JointRoutingAssignmentLabelBitsets,
)
    return JointRoutingAssignmentDominanceFilters(
        label.reduced_cost, label.time, bitsets.age_mask,
        Int32(label.route_length), Int32(length(bitsets.age_idx)),
    )
end

"""
Everything the dominance scan needs about one live label, stored *in* the
state's label list -- `PricingLabelEntry{JointRoutingAssignmentDominanceFilters,
JointRoutingAssignmentPricingLabel, JointRoutingAssignmentLabelBitsets}`
(`pricing/types.jl`).

The scan is the hot loop of the whole search -- measured at ~90% of the
revisit-tolerant pricer's wall time -- and it visits every entry of the state's
label list on every insertion. Keeping only a label id here and looking the
label and its bitsets up in two side `Dict`s cost two hash probes per entry,
which dominated the actual dominance predicate (mostly short-circuiting scalar
comparisons). Inlining them makes the scan a straight walk over the sorted
container.

MEASURED: 1.1-1.15x, with labels and `max_live` bit-identical (it is a pure
data-layout change). Less than the two-hash-probe estimate predicted, and that
shortfall is what pointed at the container itself.

# Why the scalar fields are duplicated out of the label

`label` and `bitsets` are both heap objects (they hold `Vector`s, a `Dict` and a
`BitSet`), so an entry stores *pointers* to them, and every read of
`entry.label.time` is a pointer chase into an unrelated cache line. But almost
every entry the scan visits is rejected by a *scalar* comparison -- reduced cost,
time, route length, the size and support of the live-clock set -- so the common
case paid a cache miss to fetch one `Float64`.

`JointRoutingAssignmentDominanceFilters` gathers exactly those scalars and is
stored **inline** here, so the whole first stage of the dominance test is a walk
over contiguous memory; only entries that survive every scalar filter dereference
`bitsets` (for the station-age values and the reward-layer compensation), and
`label` is never touched on the scan path at all. The entry grows from 24 to 64
bytes -- one cache line, and cheaper than the two chases it removes.

# The per-state label list

A state's label list (`PricingStateLabels{...}`, same three type parameters)
is a **`Vector`, not a `SortedDict`**, kept sorted by
`(reduced_cost, time, route_length, id)` -- see `_pricing_entry_order_key`. A
balanced search tree makes each step a pointer chase into unrelated cache
lines; measured cost was ~76ns per entry, far more than the
mostly-short-circuiting comparisons in the dominance predicate could account
for. A sorted `Vector` walks contiguous memory instead.

The trade is that insertion and eviction become `O(list size)` memmoves rather
than `O(log list size)` tree surgery -- but there is exactly one insertion per
label against a full list scan, and a memmove of a few thousand small structs
runs at memory bandwidth, so it is not close.

MEASURED: 1.3-1.5x, again with labels and `max_live` bit-identical. See
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`.
"""
PricingLabelEntry(
    id::JointRoutingAssignmentLabelId,
    label::JointRoutingAssignmentPricingLabel,
    bitsets::JointRoutingAssignmentLabelBitsets,
) = PricingLabelEntry(JointRoutingAssignmentDominanceFilters(label, bitsets), id, label, bitsets)

"""
    JointRoutingAssignmentDominanceRules{BoundedStops, Compensated, Instrumented}

The three dominance switches, carried in the *type* rather than as `Bool`
arguments.

They are constants for a whole pricing call, but as an ordinary argument
`BoundedStops` costs a branch per scanned label entry, and in the default
configuration it is `false`, so the branch is paid only to skip the code it
guards. Encoding it as a type parameter lets the compiler delete the disabled
condition outright and specialize the reward-layer compensation on
`Compensated`.

The cost is one dynamic dispatch where the rules object reaches the `dominates`
closure each search builds once and hands to the shared
`_add_pricing_label_to_state!` (`pricing/types.jl`) -- once per *label
insertion*, against a label-list scan that is hundreds to thousands of entries
long, so it is not measurable. Do not push the object any deeper (e.g. per
scanned entry) expecting the same to hold.
"""
struct JointRoutingAssignmentDominanceRules{BoundedStops, Compensated, Instrumented} <: AbstractPricingDominanceRules end

function _joint_routing_assignment_dominance_rules(
    bounded_max_stops::Bool,
    compensated_dominance::Bool,
    instrumented::Bool=false,
)
    return JointRoutingAssignmentDominanceRules{
        bounded_max_stops, compensated_dominance, instrumented,
    }()
end
