"""
When is one label strictly better than another? State/order keys, the
per-label bitsets mirror the scan needs, the reward-layer compensation
sub-test, and the dominance predicates themselves (`_pricing_dominates_fn`,
wired in `hooks.jl`) all live here. The optional rejection-census
instrumentation each predicate can increment into is declared separately, in
`logging.jl` -- this file is the one to read for the algorithm; `logging.jl`
is dev tooling for measuring it.

# Where the time actually goes (read before optimizing)

Profiled 2026-07-30: **the dominance scan is ~85-90% of wall time.** Everything
else -- the remaining-reward bound (`prune.jl`), candidate generation, label
extension (`extend.jl`), the priority queue -- is together under 2%.
Optimizations that do not either shrink the live-label population or make the
per-state label-list walk cheaper have consistently measured as no-ops here,
however sound they look on paper.

Scoreboard of what was tried, each measured on its own
(`scripts/bench_joint_routing_assignment_labels.jl`; full write-up in
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`).

| change | result |
| --- | --- |
| compensated layer dominance (this file) | **2.5-3.9x** -- halves `max_live` |
| `Vector` per-state label lists (`types.jl`) | **1.3-1.5x** |
| label inlined into label entry (`types.jl`) | **1.1-1.15x** |
| reuse popped priority (`hooks.jl`) | no effect; the bound is ~0.6% of runtime |
| travel-discounted reward bound (`prune.jl`) | no effect at cold-start duals |
| station-budget cap at `l` (removed 2026-08-10) | measured off by default -- pricing-neutral to 1.7x slower, LP bound identical to 10 decimals at n=10/n=15; removed rather than kept dark |
| compatibility-component decomposition | not built: the reward graph is one component holding 100% of opportunities |

| **mechanical pass, 2026-08-03** (below) | **6-17x on the dominance scan, 1.7-9.2x wall** |

The 2026-08-03 pass changed no pruning rule at all -- every case in the benchmark
came back bit-identical on `best_rc`, `labels`, `max_live`, `columns` and the
winning route. What it changed was the cost of the scan:

  - conditions reordered cheapest-and-likeliest-first, with a `UInt64` live-clock
    support mask added in front of the station-age merge walk
    (`_pricing_dominates_at_state`);
  - the reward-layer compensation fused into one word-wise pass instead of
    `issubset` followed by an element walk (`_joint_routing_assignment_compensation`);
  - the scalar dominance state inlined into the label entry so a rejected entry
    is never dereferenced (`JointRoutingAssignmentDominanceFilters` in `types.jl`);
  - the dominance switches moved into the type, so disabled conditions compile
    away (`JointRoutingAssignmentDominanceRules`);
  - `sortperm` and a defensive `BitSet` copy dropped from the per-label mirror
    (`_make_joint_routing_assignment_label_bitsets`).

Measured census over 456M tested pairs
(`julia scripts/diagnose.jl dominance_audit`): 48% of pairs are rejected on
`time`, a further 39-44% on the support mask, 7-9% on the support size. Under 2%
ever reach the station-age walk and under 0.5% the reward compensation -- which is
why the order of these conditions is worth this much.

Any exact change must leave the benchmark's `best_rc` bit-identical.
"""

# ── state key / order key ────────────────────────────────────────────────────
_joint_routing_assignment_state(label::JointRoutingAssignmentPricingLabel) = label.current

"""
The cheap bookkeeping signature used by the label search itself (dedup within
`best_by_signature`, see `hooks.jl`) -- deliberately *not* the final column
signature. Two labels can share this signature (same running per-passenger
maxima) while representing different physical routes with different concrete
`(p, j, k)` assignments and therefore different station-linking coefficients;
only route replay on a finished, accepted route (`accept.jl`)
resolves which concrete assignment each passenger actually gets.
"""
_joint_routing_assignment_layer_signature(label::JointRoutingAssignmentPricingLabel) = label.activated_reward_layers

function _joint_routing_assignment_label_order_key(
    label::JointRoutingAssignmentPricingLabel,
    label_id::JointRoutingAssignmentLabelId,
)::JointRoutingAssignmentLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end


# ── bitsets construction (hot-path dominance mirror) ─────────────────────────
"""
Build the hot-path mirror of `label`'s pruning state.

Called once per generated label, so its allocation count is multiplied by the
whole label population. Two things it deliberately does *not* do:

  - **no `sortperm`.** The old form allocated a permutation vector and two gathered
    copies (`age_idx[perm]`, `age_val[perm]`) on top of the two it had already
    filled -- five allocations to sort what is almost always one to four live
    clocks. Insertion sort over the two parallel arrays does it in place; at this
    length it also beats a comparison sort outright, quite apart from the
    allocations.
  - **no `copy` of the reward-layer set.** Labels are immutable and nothing in the
    search ever mutates `activated_reward_layers` -- `_certify_..._layers_at_node`
    (`../data.jl`) builds a label's set with the non-mutating `union`/`setdiff`,
    and the reward bound (`prune.jl`) only ever reads it as a `union!` *source*
    into its own workspace. The mirror can therefore alias it. (If a future change
    makes any label's layer set mutable in place, this alias is the thing that
    breaks: the state's label list would then be dominance-testing against a set
    that has since changed underneath it.)
"""
function _make_joint_routing_assignment_label_bitsets(
    label::JointRoutingAssignmentPricingLabel,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::JointRoutingAssignmentLabelBitsets
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    return JointRoutingAssignmentLabelBitsets(
        label.activated_reward_layers, age_idx, age_val, age_mask,
    )
end

# ── reward-layer compensation (dominance sub-test) ───────────────────────────
"""
Weight of the layers `a` has activated that `b` has not -- the passenger-layer
instance of the reward-model-independent `_bitset_diff_weight` (`label_setting/
utils.jl`), which carries the full bit-trick/soundness docstring. Kept as a
named wrapper here (rather than calling `_bitset_diff_weight` at every call
site) because `activated_reward_layers`/`layer_weight` read better under this
pricer's own vocabulary than the shared function's generic one.
"""
function _joint_routing_assignment_compensation(
    a_layers::RewardLayerBitset,
    b_layers::RewardLayerBitset,
    layer_weight::Vector{Float64},
    budget::Float64,
    compensated::Bool=true,
)::Float64
    return _bitset_diff_weight(a_layers, b_layers, layer_weight, budget, compensated)
end

function _joint_routing_assignment_compensation(
    a_layers::RewardLayerBitset,
    b_layers::RewardLayerBitset,
    layer_weight::Vector{Float64},
    budget::Float64,
    ::Val{Compensated},
)::Float64 where {Compensated}
    return _bitset_diff_weight(a_layers, b_layers, layer_weight, budget, Val(Compensated))
end

# ── dominance predicates ──────────────────────────────────────────────────────
"""
    _dominates_joint_routing_assignment_label(a, b, layer_weight, bounded_max_stops)

`a` dominates `b`: every completion of `b` has a counterpart from `a` that is at
least as good, so `b` can be discarded.

The reward condition is **compensated** rather than a plain subset test. For a
suffix `sigma` feasible from `b`, `a`'s no-worse station ages and elapsed time
mean everything `sigma` certifies for `b` it also certifies for `a`
(`reach_b ⊆ reach_a`), and since
`reach_b ∖ A_b ⊆ (reach_a ∖ A_a) ∪ (A_a ∖ A_b)` we get
`reward_b(sigma) <= reward_a(sigma) + w(A_a ∖ A_b)`. Both completions pay the
same travel, so `a`'s final reduced cost is no worse as long as

    rc_a + w(A_a ∖ A_b) <= rc_b.

Note the direction: `a` pays for the layers **`a`** holds and `b` lacks, not the
other way round. Charging `w(A_b ∖ A_a)` instead would be unsound -- it would let
a label that has already banked a large layer dominate one that has not, even
though the shared suffix can still pay that layer out to the second label and
overtake the first.

Requiring `A_a ⊆ A_b` (the previous rule) is the special case `w(A_a ∖ A_b) = 0`,
so this only ever adds dominations. It never weakens the `rc_a <= rc_b`
precondition either, since the compensation is non-negative -- which is what
keeps the shared `_add_pricing_label_to_state!`'s reduced-cost-ordered
scan valid.

MEASURED: **the single biggest win in this pricer, 2.5-3.9x.** `max_live` roughly
halves (58,260 -> 21,917 at n=15/max_stops=6; 117,950 -> 52,461 at n=20), and
because the dominance scan is linear in the state's label-list size *per
insertion*, halving the live population quarters the work. The speedup grows
with instance size. See
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`.

Cost control matters here: the element-wise scan in
`_joint_routing_assignment_compensation` is more expensive than the `issubset`
it replaces, so that function tries `issubset` first and bails the scan as soon
as the running weight exceeds the budget. Removing either guard gives back the
win.
"""
function _dominates_joint_routing_assignment_label(
    a::JointRoutingAssignmentPricingLabel,
    b::JointRoutingAssignmentPricingLabel,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_state(a) == _joint_routing_assignment_state(b) || return false  # must share current node
    (!bounded_max_stops || a.route_length <= b.route_length) || return false                  # a can't have used more stops
    a.time <= b.time + 1e-9 || return false                                                    # a can't be running later
    budget = b.reduced_cost - a.reduced_cost + 1e-9  # a's reduced-cost surplus over b, spendable on the compensation test below
    budget >= 0.0 || return false
    _joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
        compensated_dominance,
    ) <= budget || return false
    # Every station either label has a live pickup clock for: a's clock can't
    # be older than b's.
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

function _dominates_joint_routing_assignment_label(
    a::JointRoutingAssignmentPricingLabel,
    b::JointRoutingAssignmentPricingLabel,
    a_bitsets::JointRoutingAssignmentLabelBitsets,
    b_bitsets::JointRoutingAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_state(a) == _joint_routing_assignment_state(b) || return false
    return _pricing_dominates_at_state(
        JointRoutingAssignmentDominanceFilters(a, a_bitsets), a_bitsets,
        JointRoutingAssignmentDominanceFilters(b, b_bitsets), b_bitsets,
        layer_weight,
        _joint_routing_assignment_dominance_rules(
            bounded_max_stops, compensated_dominance,
        ),
    )
end

"""
    _pricing_dominates_at_state(a..., b..., layer_weight, rules)

The dominance predicate as the state's label-list scan calls it: `a` dominates
`b`, so `b` can be discarded. Scalar state is passed by value (the caller reads
it straight out of `PricingLabelEntry`, avoiding a chase into the label) and
only the two `Bitsets` mirrors are passed by reference.

The **state check is absent on purpose**: a state's label list holds only
labels keyed by exactly that state (`label.current`), so every pair the scan
ever tests already agrees on it, and testing it per entry was a
guaranteed-true comparison in the hottest loop of the search. The 6-argument
method above keeps it, for callers that are not scanning within one state's
list.

# Condition order

Conditions are ordered by (cost to evaluate) against (how often they reject),
cheapest-and-likeliest first, so that the expensive tail runs on as few pairs as
possible:

 1. `time` -- one compare, and a strong discriminator: the state's label list is
    sorted by reduced cost, which correlates only weakly with elapsed time.
 2. **live-clock support size** -- `|dom(age_a)| >= |dom(age_b)|` is necessary for
    the station-age condition (`dom(age_b) ⊆ dom(age_a)`), and it is two array
    lengths. It used to sit *after* the reward compensation, i.e. the cheapest
    remaining filter ran last.
 3. **live-clock support mask** -- `dom(age_b) ⊆ dom(age_a)` itself, as one
    `UInt64` test, standing in front of the walk that would otherwise establish
    it element by element out of two heap arrays (see
    `JointRoutingAssignmentLabelBitsets.age_mask`).
 4. `route_length` -- compiled out entirely unless `max_stops` is bounded.
 5. `reduced_cost` -- guaranteed non-negative when called from the state's
    label-list scan (the walk splits on exactly this), kept because the
    compensation's early bail is defined in terms of a non-negative budget.
 6. **station ages** -- an `O(#live)` merge walk over two sorted `Int32`/`Float64`
    arrays, no allocation, no hashing. Everything above it is scalar and reads
    only fields the label entry carries inline, so a rejected pair never touches
    the arrays.
 7. **reward-layer compensation** -- the only test that touches the layer bitsets,
    now last. It was previously ahead of the station-age walk, so every pair that
    the ages were going to reject paid a bitset traversal first.

Swapping 6 and 7, and putting 3 in front of 6, are the reorderings with real
leverage; the rest is a few instructions. Every condition is exact, so no
ordering here can change *which* pairs dominate -- only how much work is done to
find out.

MEASURED (`julia scripts/diagnose.jl dominance_audit`, share of all tested pairs
each condition is the *first* to reject; n=15/ms=6/s=3 and n=20/ms=5/s=3):

| condition | rejects |
| --- | --- |
| 1 time | 49.4% / 47.9% |
| 3 age mask | 39.5% / 43.6% |
| 2 support size | 9.1% / 6.6% |
| 4 route length | 0.7% / 0.6% |
| 6 station ages | 1.1% / 0.7% |
| 7 compensation | 0.2% / 0.5% |
| dominates | 0.10% / 0.06% |

So three scalar tests dispose of ~96% of pairs, and the two conditions that touch
heap data run on under 2%. `reduced_cost` recorded **zero** rejections in 456M
pairs, because the state's label-list walk splits on exactly that comparison,
so it is already guaranteed when this is reached. It is kept as a
one-instruction guard for callers outside that walk and because the
compensation's early bail is defined against a non-negative budget.
"""
@inline function _pricing_dominates_at_state(
    a_filters::JointRoutingAssignmentDominanceFilters,
    a_bitsets::JointRoutingAssignmentLabelBitsets,
    b_filters::JointRoutingAssignmentDominanceFilters,
    b_bitsets::JointRoutingAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    ::JointRoutingAssignmentDominanceRules{BoundedStops, Compensated, Instrumented},
)::Bool where {BoundedStops, Compensated, Instrumented}
    @inline _reject(i::Int) = (Instrumented && (@inbounds JOINT_ROUTING_ASSIGNMENT_DOMINANCE_REJECTIONS[i] += 1); false)

    a_filters.time <= b_filters.time + 1e-9 || return _reject(JRA_REJECT_TIME)

    # `dom(age_b) subseteq dom(age_a)` is required below, so `a` cannot have fewer
    # live clocks than `b`. Both counts ride inline in the filters, ahead of
    # everything that has to look at set contents.
    age_rejection = _sparse_station_age_support_rejection(
        a_bitsets.age_idx, a_filters.age_mask, b_bitsets.age_idx, b_filters.age_mask,
    )
    age_rejection == 1 && return _reject(JRA_REJECT_LIVE_CLOCK_SUPPORT)
    age_rejection == 2 && return _reject(JRA_REJECT_AGE_MASK)

    if BoundedStops
        a_filters.route_length <= b_filters.route_length || return _reject(JRA_REJECT_ROUTE_LENGTH)
    end

    budget = b_filters.reduced_cost - a_filters.reduced_cost + 1e-9
    budget >= 0.0 || return _reject(JRA_REJECT_REDUCED_COST)

    # `dom(age_b) subseteq dom(age_a)` and `age_a(j) <= age_b(j)` for all j in
    # dom(age_b) -- exactly equivalent to the dense "all stations, missing = Inf"
    # rule (a live age absent from `a` compares as Inf and always fails; one
    # absent from `b` compares as <= Inf and always passes), but costs O(#live)
    # instead of O(n_nodes). Both arrays are sorted, so one merge walk suffices.
    _sparse_station_age_values_dominate(
        a_bitsets.age_idx, a_bitsets.age_val, b_bitsets.age_idx, b_bitsets.age_val,
    ) || return _reject(JRA_REJECT_STATION_AGE)

    # See the 6-argument method for why the reward test is a compensated
    # reduced-cost budget rather than `issubset`. There is no `length` prefilter
    # here: `a` holding *more* layers than `b` is a legitimate domination whenever
    # `a`'s reduced-cost advantage covers their weight.
    _joint_routing_assignment_compensation(
        a_bitsets.activated_bits, b_bitsets.activated_bits, layer_weight, budget, Val(Compensated),
    ) <= budget || return _reject(JRA_REJECT_COMPENSATION)

    Instrumented && (@inbounds JOINT_ROUTING_ASSIGNMENT_DOMINANCE_REJECTIONS[JRA_DOMINATES] += 1)
    return true
end
