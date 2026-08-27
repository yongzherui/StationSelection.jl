"""
Core label-DP primitives for passenger free-assignment pricing: label creation,
extension, and dominance. `exact.jl` orchestrates these into a full pricing
pass; this file is the one to audit for "is the label search correct".

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
reconstructed only for finished, negative-reduced-cost routes (`exact.jl`'s
route replay) -- not tracked while labels are being extended.

# Where the time actually goes (read before optimizing)

Profiled 2026-07-30: **the dominance scan is ~85-90% of wall time.** Everything
else -- the remaining-reward bound, candidate generation, label extension, the
priority queue -- is together under 2%. Optimizations that do not either shrink
the live-label population or make the per-state label-list walk cheaper have
consistently measured as no-ops here, however sound they look on paper.

Scoreboard of what was tried, each measured on its own
(`scripts/bench_joint_routing_assignment_labels.jl`; full write-up in
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`):

| change | result |
| --- | --- |
| compensated layer dominance (this file) | **2.5-3.9x** -- halves `max_live` |
| `Vector` per-state label lists (`types.jl`) | **1.3-1.5x** |
| label inlined into label entry (`types.jl`) | **1.1-1.15x** |
| reuse popped priority (`exact.jl`) | no effect; the bound is ~0.6% of runtime |
| travel-discounted reward bound (`exact.jl`) | no effect at cold-start duals |
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

export initial_joint_routing_assignment_pricing_labels
export extend_joint_routing_assignment_pricing_label

# ── label seeding ────────────────────────────────────────────────────────────
function initial_joint_routing_assignment_pricing_labels(
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{JointRoutingAssignmentPricingLabel}
    # Same reasoning as the route-covering pricer's twin: a route can only
    # ever collect reward through one of an opportunity's two endpoints, so
    # seeding elsewhere would waste search on routes that can never certify
    # anything.
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = JointRoutingAssignmentPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        # One depth-1 label per relevant node: route so far is `[node]`,
        # `time = 0`, this node's own pickup clock starts live at age 0, no
        # reward layers activated yet, and `tau`/`reduced_cost` already carry
        # the fixed `repositioning_time` cost every route pays regardless of
        # length.
        push!(labels, JointRoutingAssignmentPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            RewardLayerBitset(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end

# ── candidate next-nodes ─────────────────────────────────────────────────────
# `label` is untyped so both the revisit-tolerant and the elementary
# (`../station_simple/labels.jl`) pricers can share this: it reads only `station_age`,
# `current`, and `activated_reward_layers`, which both label types expose. Julia
# specializes per concrete call site, so there is no dispatch or speed cost.
function _has_useful_live_joint_routing_assignment_origin(
    label,
    pricing_data::JointRoutingAssignmentPricingData,
)::Bool
    for (station, age) in label.station_age
        opportunities = get(pricing_data.assignments_by_origin, station, PassengerAssignmentOpportunity[])
        for opp in opportunities
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            t_to_dest = opp.destination == label.current ? 0.0 :
                _joint_routing_assignment_travel(pricing_data, label.current, opp.destination)
            age + t_to_dest <= opp.ride_limit + 1e-9 || continue
            return true
        end
    end
    return false
end

"""
    is_useful_destination(label, k)

A station `k` is worth visiting next as a destination if some *currently live*
origin age can still certify a currently-inactive layer there. A station `j` is
worth visiting as an origin (only while still inside the pickup window) if
visiting it could open a live clock that later unlocks a currently-inactive
layer, judged via `origin_layer_mask` (the union of everything reachable from
that origin, an optimistic pre-filter -- true feasibility from that origin is
re-checked once its age actually becomes live).
"""
function _joint_routing_assignment_candidate_next_nodes(
    label::JointRoutingAssignmentPricingLabel,
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_joint_routing_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin == label.current && continue
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _joint_routing_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    # Driven from the label's *live origins* rather than from every destination
    # group: only a live origin can make a destination useful, and pruning keeps
    # the live set small, whereas `assignments_by_destination` spans all
    # `~P * n^2` opportunities regardless of label state.
    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination == label.current && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _joint_routing_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

# ── label extension ──────────────────────────────────────────────────────────
"""
Extension always produces exactly one child (an unlimited-capacity route has
nothing to branch on at a stop), so the search calls
`_extend_joint_routing_assignment_pricing_label` and gets the label back
directly. This method wraps it in the one-element `Vector` the public API has
always returned; that wrapper allocation is per *extension*, so it is worth not
paying on the search's hot path.
"""
function extend_joint_routing_assignment_pricing_label(
    label::JointRoutingAssignmentPricingLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{JointRoutingAssignmentPricingLabel}
    return JointRoutingAssignmentPricingLabel[
        _extend_joint_routing_assignment_pricing_label(label, next_node, pricing_data),
    ]
end

function _extend_joint_routing_assignment_pricing_label(
    label::JointRoutingAssignmentPricingLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
)::JointRoutingAssignmentPricingLabel
    travel_time = _joint_routing_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    certified_layers, reward = _certify_joint_routing_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    # Age, reset, and prune in ONE pass. Previously this built an aged Dict and
    # then a second pruned Dict from it -- two allocations and two traversals per
    # extension, on the hottest path in the search.
    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        station == next_node && continue  # handled by the reset below
        aged = age + travel_time
        _joint_routing_assignment_age_is_useful(station, aged, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        # A fresh clock at the arrival station: age 0 is the most useful an age can
        # be, but it still only earns a slot if it can reach some opportunity in time.
        _joint_routing_assignment_age_is_useful(next_node, 0.0, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
        # Past the cutoff the visit creates no new clock, so `next_node`'s existing
        # clock (if any) just ages like the rest.
        aged = label.station_age[next_node] + travel_time
        _joint_routing_assignment_age_is_useful(next_node, aged, pricing_data, next_node) &&
            (aged_station[next_node] = aged)
    end

    return JointRoutingAssignmentPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )
end

# ── state key / order key ────────────────────────────────────────────────────
_joint_routing_assignment_state(label::JointRoutingAssignmentPricingLabel) = label.current

"""
The cheap bookkeeping signature used by the label search itself (dedup within
`best_by_signature`, see `exact.jl`) -- deliberately *not* the final column
signature. Two labels can share this signature (same running per-passenger
maxima) while representing different physical routes with different concrete
`(p, j, k)` assignments and therefore different station-linking coefficients;
only route replay on a finished, accepted route (section 13) resolves which
concrete assignment each passenger actually gets.
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
    builds a label's set with the non-mutating `union`/`setdiff`, and the reward
    bound only ever reads it as a `union!` *source* into its own workspace. The
    mirror can therefore alias it. (If a future change makes any label's layer set
    mutable in place, this alias is the thing that breaks: the state's label
    list would then be dominance-testing against a set that has since changed
    underneath it.)
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
    abs::JointRoutingAssignmentLabelBitsets,
    bbs::JointRoutingAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_state(a) == _joint_routing_assignment_state(b) || return false
    return _pricing_dominates_at_state(
        JointRoutingAssignmentDominanceFilters(a, abs), abs,
        JointRoutingAssignmentDominanceFilters(b, bbs), bbs,
        layer_weight,
        _joint_routing_assignment_dominance_rules(
            bounded_max_stops, compensated_dominance,
        ),
    )
end

"""
    _dominates_joint_routing_assignment_label_reward_aware(a, b, pricing_data, layer_weight, bounded_max_stops, compensated_dominance=true)

Alternative to `_dominates_joint_routing_assignment_label` above: same D1
(shared node), D2 (`a.time <= b.time`) and D4 (compensated reward-cost budget)
conditions, but a strictly more permissive station-age condition (D3).

The existing rule requires, for *every* station either label has a live clock
at, that `a`'s age be no worse than `b`'s -- a clock `a` lacks compares as
`Inf` and always fails. This rule instead only requires that for stations `j`
where `b` *still* has a live clock: either `a` matches it (has a live clock
there, no older), **or** `a` has already collected every reward layer
reachable from origin `j` (`j` is "reward-dead" for `a`, tested against
`pricing_data.origin_layer_mask[j]`, exactly as `_has_useful_live_joint_routing_assignment_origin`
tests it elsewhere). In the reward-dead case, no suffix of `b`'s can use `j`'s
clock to certify anything `a` doesn't already have, so `a` doesn't need the
physical resource to match it -- the difference is already priced by D4.

Unlike the reward-coupled `_joint_routing_assignment_age_is_useful` removed in
commit `f644a7c` (which made this same exemption unsound by baking it into
what `station_age` stores, so it leaked into *every* future comparison,
including against labels for which the station was not reward-dead), the
exemption here is recomputed fresh for `a` and `b`'s own
`activated_reward_layers` at comparison time and never mutates either label's
stored clocks.

Deliberately the plain, `Dict`-based label form -- not the optimized
bitset/filter scan (`_pricing_dominates_at_state`). That scan's condition
order and inlined filters are tuned around the existing D3; this is the
correctness-first version to establish whether the rule is worth adopting
before it earns a place in the hot path.
"""
function _dominates_joint_routing_assignment_label_reward_aware(
    a::JointRoutingAssignmentPricingLabel,
    b::JointRoutingAssignmentPricingLabel,
    pricing_data::JointRoutingAssignmentPricingData,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_state(a) == _joint_routing_assignment_state(b) || return false  # D1
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false                                                   # D2
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    _joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
        compensated_dominance,
    ) <= budget || return false                                                               # D4
    for (station, b_age) in b.station_age                                                     # D3
        reward_dead = !_has_inactive_layer(
            get(pricing_data.origin_layer_mask, station, RewardLayerBitset()),
            a.activated_reward_layers,
        )
        reward_dead && continue
        haskey(a.station_age, station) || return false
        a.station_age[station] <= b_age + 1e-9 || return false
    end
    return true
end

# ── dominance rejection census (diagnostics) ─────────────────────────────────
"""
Rejection census for `_pricing_dominates_at_state` (passenger method), one counter
per condition plus one for "dominates".

Only written when the dominance rules carry `Instrumented = true`, which the
production search never sets; with `Instrumented = false` the increments are
constant-folded away, so an uninstrumented scan pays nothing for their existence.
Ordering the conditions cheapest-and-likeliest-to-reject first is only meaningful
against measured rejection rates, and this is where those come from -- see
`julia scripts/diagnose.jl dominance_audit`.
"""
const PFA_DOMINANCE_CONDITIONS = (
    :time, :live_clock_support, :route_length,
    :reduced_cost, :station_age, :compensation, :dominates, :age_mask,
)
const PFA_DOMINANCE_REJECTIONS = zeros(Int, length(PFA_DOMINANCE_CONDITIONS))

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
    af::JointRoutingAssignmentDominanceFilters, abs::JointRoutingAssignmentLabelBitsets,
    bf::JointRoutingAssignmentDominanceFilters, bbs::JointRoutingAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    ::JointRoutingAssignmentDominanceRules{BoundedStops, Compensated, Instrumented},
)::Bool where {BoundedStops, Compensated, Instrumented}
    @inline _reject(i::Int) = (Instrumented && (@inbounds PFA_DOMINANCE_REJECTIONS[i] += 1); false)

    af.time <= bf.time + 1e-9 || return _reject(1)

    # `dom(age_b) subseteq dom(age_a)` is required below, so `a` cannot have fewer
    # live clocks than `b`. Both counts ride inline in the filters, ahead of
    # everything that has to look at set contents.
    age_rejection = _sparse_station_age_support_rejection(
        abs.age_idx, af.age_mask, bbs.age_idx, bf.age_mask,
    )
    age_rejection == 1 && return _reject(2)
    age_rejection == 2 && return _reject(8)

    if BoundedStops
        af.route_length <= bf.route_length || return _reject(3)
    end

    budget = bf.reduced_cost - af.reduced_cost + 1e-9
    budget >= 0.0 || return _reject(4)

    # `dom(age_b) subseteq dom(age_a)` and `age_a(j) <= age_b(j)` for all j in
    # dom(age_b) -- exactly equivalent to the dense "all stations, missing = Inf"
    # rule (a live age absent from `a` compares as Inf and always fails; one
    # absent from `b` compares as <= Inf and always passes), but costs O(#live)
    # instead of O(n_nodes). Both arrays are sorted, so one merge walk suffices.
    _sparse_station_age_values_dominate(
        abs.age_idx, abs.age_val, bbs.age_idx, bbs.age_val,
    ) || return _reject(5)

    # See the 6-argument method for why the reward test is a compensated
    # reduced-cost budget rather than `issubset`. There is no `length` prefilter
    # here: `a` holding *more* layers than `b` is a legitimate domination whenever
    # `a`'s reduced-cost advantage covers their weight.
    _joint_routing_assignment_compensation(
        abs.activated_bits, bbs.activated_bits, layer_weight, budget, Val(Compensated),
    ) <= budget || return _reject(6)

    Instrumented && (@inbounds PFA_DOMINANCE_REJECTIONS[7] += 1)
    return true
end

"""
Read out and reset the rejection census. Returns
`condition => count` pairs in evaluation order.
"""
function joint_routing_assignment_dominance_rejections(; reset::Bool=true)
    counts = [PFA_DOMINANCE_CONDITIONS[i] => PFA_DOMINANCE_REJECTIONS[i]
              for i in eachindex(PFA_DOMINANCE_CONDITIONS)]
    reset && fill!(PFA_DOMINANCE_REJECTIONS, 0)
    return counts
end


# ── column signature ──────────────────────────────────────────────────────────
function _joint_routing_assignment_column_signature(assignments)::Tuple{Vararg{Tuple{Int, Int, Int}}}
    return Tuple(sort!(collect(assignments)))
end

_joint_routing_assignment_column_signature(column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(column.assignments)
