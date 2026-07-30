"""
Core label-DP primitives for passenger free-assignment pricing: label creation,
extension, and dominance. `search.jl` orchestrates these into a full pricing
pass; this file is the one to audit for "is the label search correct".

# Operational contract of the column being priced

Same unlimited-capacity, synchronized-start physical route as
`AggregateODRoutePricingLabel` (see that file's module docstring for the shared
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
reconstructed only for finished, negative-reduced-cost routes (`search.jl`'s
route replay) -- not tracked while labels are being extended.

# Where the time actually goes (read before optimizing)

Profiled 2026-07-30: **the dominance scan is ~85-90% of wall time.** Everything
else -- the remaining-reward bound, candidate generation, label extension, the
priority queue -- is together under 2%. Optimizations that do not either shrink
the live-label population or make the bucket walk cheaper have consistently
measured as no-ops here, however sound they look on paper.

Scoreboard of what was tried, each measured on its own
(`scripts/bench_passenger_free_assignment_labels.jl`; full write-up in
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`):

| change | result |
| --- | --- |
| compensated layer dominance (this file) | **2.5-3.9x** -- halves `max_live` |
| `Vector` dominance buckets (`types.jl`) | **1.3-1.5x** |
| label inlined into bucket entry (`types.jl`) | **1.1-1.15x** |
| reuse popped priority (`search.jl`) | no effect; the bound is ~0.6% of runtime |
| travel-discounted reward bound (`search.jl`) | no effect at cold-start duals |
| station-budget cap at `l` (this file) | **off by default** -- slower, and the LP bound did not move |
| compatibility-component decomposition | not built: the reward graph is one component holding 100% of opportunities |

Any exact change must leave the benchmark's `best_rc` bit-identical; the
station-budget cap is the one deliberate exception, since it restricts the column
set on purpose.
"""

export initial_passenger_free_assignment_pricing_labels
export extend_passenger_free_assignment_pricing_label

function initial_passenger_free_assignment_pricing_labels(
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{PassengerFreeAssignmentPricingLabel}
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = PassengerFreeAssignmentPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, PassengerFreeAssignmentPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            RewardLayerBitset(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
            get(pricing_data.station_bit, node, UInt64(0)),
        ))
    end
    return labels
end

"""
    _passenger_free_assignment_station_budget_allows(label, next_node, pricing_data)

Whether extending to `next_node` keeps the route inside the station budget
`l` (`max_distinct_stations`), i.e. `|U(route) ∪ {next_node}| <= l`. Revisiting
an already-used station is always free.

# Why capping *visited* stations is valid

Only stations that end up carrying an assignment need `y_j = 1`, and in an
integer master `theta_r >= 1` forces `y_j = 1` for each of them, so
`|A_r| <= sum(y) = l`: columns above that are unusable by any integer solution.
Dropping them leaves the IP optimum untouched and *tightens* the LP bound (the
LP over the restricted pool is still a relaxation of the same IP, so it remains a
valid lower bound while being no smaller).

Capping *visited* stations rather than assignment-carrying ones is a stronger
restriction, and it is still lossless for pricing. A visited station that carries
no assignment in the replayed column can be deleted from the route: by the
triangle inequality `travel(a, b) <= travel(a, k) + travel(k, b)`, removing `k`
lowers `tau` and makes every later arrival *earlier*, which only relaxes ride
limits and opens more pickup clocks -- so reward cannot fall and reduced cost
cannot rise. Hence for every column with `|A_r| <= l` there is one with reduced
cost no worse whose route visits only assignment-carrying stations, and searching
`{|visited| <= l}` attains the same pricing minimum as searching `{|A_r| <= l}`.

The cap is what makes the extra dominance condition in
`_dominates_passenger_free_assignment_label` necessary: once station budget is a
consumed resource, a label that has spent more of it cannot stand in for one that
has spent less.

MEASURED: **default off, because it does not pay on either metric it was meant
to.** Pricing is neutral to 1.7x *slower* with stops unbounded -- the `U_a ⊆ U_b`
companion drops the domination rate 80.7% -> 74.4%, so buckets grow and the
dominance scan costs more than the branch pruning saves. And through the full CG
loop the LP bound is identical to ten decimal places at n=10 and n=15: the LP
optimum never wanted a wider column.

Two structural reasons it binds so weakly, both worth knowing before re-enabling
it: ride limits plus the pickup window already hold the best routes to 6-8
distinct stations at n=15 (so `l = 8` constrains nothing), and the per-passenger
*maximum* reward structure means a route cannot buy extra dual credit by touching
more stations -- unlike the aggregate pricer, whose summed per-pair reward is what
produced the hub-route LP-IP gap this cap was meant to close. Worth revisiting
only where feasibility does not already bound distinct stations below `l`, and
only after re-measuring `lp_bound` there.
"""
function _passenger_free_assignment_station_budget_allows(
    label::PassengerFreeAssignmentPricingLabel,
    next_node::Int,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Bool
    pricing_data.bounded_distinct_stations || return true
    bit = pricing_data.station_bit[next_node]
    label.visited_mask & bit != 0 && return true  # revisit costs no budget
    return count_ones(label.visited_mask) < pricing_data.max_distinct_stations
end

function _has_useful_live_passenger_free_assignment_origin(
    label::PassengerFreeAssignmentPricingLabel,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Bool
    for (station, age) in label.station_age
        opportunities = get(pricing_data.assignments_by_origin, station, PassengerAssignmentOpportunity[])
        for opp in opportunities
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            t_to_dest = opp.destination == label.current ? 0.0 :
                _passenger_free_assignment_travel(pricing_data, label.current, opp.destination)
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
function _passenger_free_assignment_candidate_next_nodes(
    label::PassengerFreeAssignmentPricingLabel,
    pricing_data::PassengerFreeAssignmentPricingData;
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_passenger_free_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin == label.current && continue
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _passenger_free_assignment_travel(pricing_data, label.current, origin)
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
            origin_age + _passenger_free_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    if max_visits_per_node < typemax(Int)
        visit_counts = Dict{Int, Int}()
        for node in label.route
            visit_counts[node] = get(visit_counts, node, 0) + 1
        end
        filter!(node -> get(visit_counts, node, 0) < max_visits_per_node, candidate_nodes)
    end

    if pricing_data.bounded_distinct_stations
        filter!(node -> _passenger_free_assignment_station_budget_allows(label, node, pricing_data), candidate_nodes)
    end

    return sort!(collect(candidate_nodes))
end

function extend_passenger_free_assignment_pricing_label(
    label::PassengerFreeAssignmentPricingLabel,
    next_node::Int,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{PassengerFreeAssignmentPricingLabel}
    travel_time = _passenger_free_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    certified_layers, reward = _certify_passenger_free_assignment_layers_at_node(
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
        _passenger_free_assignment_age_is_useful(station, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        # A fresh clock at the arrival station: age 0 is the most useful an age can
        # be, but it still only earns a slot if it can certify something new.
        _passenger_free_assignment_age_is_useful(next_node, 0.0, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
        # Past the cutoff the visit creates no new clock, so `next_node`'s existing
        # clock (if any) just ages like the rest.
        aged = label.station_age[next_node] + travel_time
        _passenger_free_assignment_age_is_useful(next_node, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = aged)
    end

    child = PassengerFreeAssignmentPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
        label.visited_mask | get(pricing_data.station_bit, next_node, UInt64(0)),
    )

    return PassengerFreeAssignmentPricingLabel[child]
end

_passenger_free_assignment_dominance_signature(label::PassengerFreeAssignmentPricingLabel) = label.current

"""
The cheap bookkeeping signature used by the label search itself (dedup within
`best_by_signature`, see `search.jl`) -- deliberately *not* the final column
signature. Two labels can share this signature (same running per-passenger
maxima) while representing different physical routes with different concrete
`(p, j, k)` assignments and therefore different station-linking coefficients;
only route replay on a finished, accepted route (section 13) resolves which
concrete assignment each passenger actually gets.
"""
_passenger_free_assignment_layer_signature(label::PassengerFreeAssignmentPricingLabel) = label.activated_reward_layers

function _passenger_free_assignment_label_order_key(
    label::PassengerFreeAssignmentPricingLabel,
    label_id::PassengerFreeAssignmentLabelId,
)::PassengerFreeAssignmentLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end

_create_passenger_free_assignment_dominance_bucket() = PassengerFreeAssignmentDominanceBucket()

function _make_passenger_free_assignment_label_bitsets(
    label::PassengerFreeAssignmentPricingLabel,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::PassengerFreeAssignmentLabelBitsets
    n_live = length(label.station_age)
    age_idx = Vector{Int32}(undef, n_live)
    age_val = Vector{Float64}(undef, n_live)
    i = 1
    for (station, age) in label.station_age
        age_idx[i] = Int32(node_index[station])
        age_val[i] = age
        i += 1
    end
    perm = sortperm(age_idx)
    return PassengerFreeAssignmentLabelBitsets(
        copy(label.activated_reward_layers), age_idx[perm], age_val[perm],
    )
end

"""
Weight of the layers `a` has activated that `b` has not.

This is the "catch-up" term in the dominance rule below: those layers are reward
`b` can still bank off a suffix the two labels share, while `a`, having already
banked them, gets nothing more for re-reaching them.

Bails out as soon as the running total exceeds `budget`, because the only use is
the test `compensation <= budget`. In practice `budget` is a small reduced-cost
difference while individual layer weights are large, so the common failing case
exits after one layer.
"""
function _passenger_free_assignment_compensation(
    a_layers::RewardLayerBitset,
    b_layers::RewardLayerBitset,
    layer_weight::Vector{Float64},
    budget::Float64,
)::Float64
    # Word-wise and much cheaper than the element loop; also the single most
    # common case (it is exactly the old, uncompensated dominance rule).
    issubset(a_layers, b_layers) && return 0.0
    total = 0.0
    @inbounds for layer in a_layers
        layer in b_layers && continue
        total += layer_weight[layer]
        total > budget && return total
    end
    return total
end

"""
    _dominates_passenger_free_assignment_label(a, b, layer_weight, bounded_max_stops)

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
keeps `_add_passenger_free_assignment_label_to_bucket!`'s reduced-cost-ordered
scan valid.

MEASURED: **the single biggest win in this pricer, 2.5-3.9x.** `max_live` roughly
halves (58,260 -> 21,917 at n=15/max_stops=6; 117,950 -> 52,461 at n=20), and
because the dominance scan is linear in bucket size *per insertion*, halving the
live population quarters the work. The speedup grows with instance size. See
`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`.

Cost control matters here: the element-wise scan in
`_passenger_free_assignment_compensation` is more expensive than the `issubset`
it replaces, so that function tries `issubset` first and bails the scan as soon
as the running weight exceeds the budget. Removing either guard gives back the
win.
"""
function _dominates_passenger_free_assignment_label(
    a::PassengerFreeAssignmentPricingLabel,
    b::PassengerFreeAssignmentPricingLabel,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    bounded_distinct_stations::Bool=false,
)::Bool
    _passenger_free_assignment_dominance_signature(a) == _passenger_free_assignment_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    # Station budget is a consumed resource once capped: any suffix feasible for
    # `b` must also be feasible for `a`, which needs `U_a subseteq U_b`. Skipped
    # entirely when uncapped, since it would only weaken dominance for nothing.
    (!bounded_distinct_stations || a.visited_mask & ~b.visited_mask == 0) || return false
    a.time <= b.time + 1e-9 || return false
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    _passenger_free_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
    ) <= budget || return false
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

function _dominates_passenger_free_assignment_label(
    a::PassengerFreeAssignmentPricingLabel,
    b::PassengerFreeAssignmentPricingLabel,
    abs::PassengerFreeAssignmentLabelBitsets,
    bbs::PassengerFreeAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    bounded_distinct_stations::Bool=false,
)::Bool
    _passenger_free_assignment_dominance_signature(a) == _passenger_free_assignment_dominance_signature(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    # See the 4-argument method: `U_a subseteq U_b`, one instruction, and only
    # when the station budget is actually capped.
    (!bounded_distinct_stations || a.visited_mask & ~b.visited_mask == 0) || return false
    a.time <= b.time + 1e-9 || return false
    # See the 4-argument method for why the reward test is a compensated
    # reduced-cost budget rather than `issubset`. There is no `length` prefilter
    # here any more: `a` holding *more* layers than `b` is now a legitimate
    # domination whenever `a`'s reduced-cost advantage covers their weight.
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    _passenger_free_assignment_compensation(
        abs.activated_bits, bbs.activated_bits, layer_weight, budget,
    ) <= budget || return false
    # `dom(age_b) subseteq dom(age_a)` and `age_a(j) <= age_b(j)` for all j in
    # dom(age_b) -- exactly equivalent to the dense "all stations, missing = Inf"
    # rule (a live age absent from `a` compares as Inf and always fails; one
    # absent from `b` compares as <= Inf and always passes), but costs O(#live)
    # instead of O(n_nodes). Both arrays are sorted, so one merge walk suffices.
    length(abs.age_idx) >= length(bbs.age_idx) || return false
    ia = 1
    na = length(abs.age_idx)
    @inbounds for ib in eachindex(bbs.age_idx)
        j = bbs.age_idx[ib]
        while ia <= na && abs.age_idx[ia] < j
            ia += 1
        end
        (ia <= na && abs.age_idx[ia] == j) || return false
        abs.age_val[ia] <= bbs.age_val[ib] + 1e-9 || return false
    end
    return true
end

"""
Insert `label` into its dominance bucket, unless an incumbent dominates it, and
evict the incumbents it dominates.

The bucket is ordered by `(reduced_cost, time, route_length, id)`, and domination
in either direction requires `rc_dominator <= rc_dominated` -- true of the
compensated rule too, since the compensation is non-negative. So the walk splits
at the new label's reduced cost: below it only an incumbent can dominate the new
label (and finding one ends the walk), above it only the new label can dominate
incumbents.

Dominated entries are collected as bucket *indices*, which the walk produces in
ascending order, so eviction is a single `deleteat!` and nothing is mutated while
the bucket is being scanned.
"""
function _add_passenger_free_assignment_label_to_bucket!(
    bucket::PassengerFreeAssignmentDominanceBucket,
    live_labels::Dict{Int, PassengerFreeAssignmentPricingLabel},
    label::PassengerFreeAssignmentPricingLabel,
    label_id::Int,
    label_bs::PassengerFreeAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    bounded_distinct_stations::Bool,
    dominated::Vector{Int},
)
    inserted = true
    empty!(dominated)
    switched = false

    @inbounds for i in eachindex(bucket)
        entry = bucket[i]
        existing_label = entry.label

        if !switched && label.reduced_cost > existing_label.reduced_cost + 1e-9
            if _dominates_passenger_free_assignment_label(existing_label, label, entry.bitsets, label_bs, layer_weight, bounded_max_stops, bounded_distinct_stations)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if _dominates_passenger_free_assignment_label(label, existing_label, label_bs, entry.bitsets, layer_weight, bounded_max_stops, bounded_distinct_stations)
            push!(dominated, i)
        end
    end

    n_dominated = length(dominated)
    if inserted
        @inbounds for i in dominated
            delete!(live_labels, bucket[i].id)
        end
        deleteat!(bucket, dominated)
        new_entry = PassengerFreeAssignmentBucketEntry(label_id, label, label_bs)
        # The search value must be an entry, not a bare key: `searchsortedfirst`
        # applies `by` to it as well as to the elements.
        pos = searchsortedfirst(bucket, new_entry; by=_passenger_free_assignment_entry_order_key)
        insert!(bucket, pos, new_entry)
    end
    return inserted, n_dominated
end

function _passenger_free_assignment_column_signature(assignments)::Tuple{Vararg{Tuple{Int, Int, Int}}}
    return Tuple(sort!(collect(assignments)))
end

_passenger_free_assignment_column_signature(column::PassengerFreeAssignmentRouteColumn) =
    _passenger_free_assignment_column_signature(column.assignments)
