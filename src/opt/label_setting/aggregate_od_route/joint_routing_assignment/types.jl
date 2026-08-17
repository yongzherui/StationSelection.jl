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
export JointRoutingAssignmentPricingData
export JointRoutingAssignmentPricingLabel
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
A partial unlimited-capacity, synchronized-start route, exactly as in
`AggregateODRoutePricingLabel` (`current`, `route`, `time`, `station_age`, `tau`,
`reduced_cost`, `route_length` all carry the same meaning). The one substantive
change is `activated_reward_layers` in place of `served_pairs`: since a passenger
selects a single, best, certified assignment rather than being "served" by every
pair the route happens to certify, the label only needs to remember the highest
reward level certified so far per passenger -- not which concrete `(j, k)` pairs
were involved. See `labels.jl` for the full pricing contract.
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

"""
    JointRoutingAssignmentRouteColumn(id, route, assignments, tau; metadata)

A priced column: a physical station route paired with the concrete per-passenger
assignments `(p, pickup, dropoff)` selected during route replay (see
`exact.jl`). Unlike `activated_reward_layers`, which only records reward levels,
`assignments` records which stations actually carry the linking coefficients a
master problem would need.
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
