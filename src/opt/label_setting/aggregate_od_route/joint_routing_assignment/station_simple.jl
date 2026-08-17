"""
Station-simple (elementary-route) label-setting pricer for the passenger
free-assignment subproblem: an alternative to the revisit-tolerant search in
`labels.jl`/`exact.jl` in which a physical route may never revisit a station.

# What elementarity changes (and what it does not)

Only the *route universe* changes -- the reward contract is identical to the
revisit-tolerant pricer (see `labels.jl`'s module docstring): a visit to origin
`j` within `max_wait_time` opens a live pickup clock; a later visit to `k`
certifies `(p, j, k)` for passengers whose clock survives the ride limit; and a
passenger banks only its single best certified reward, tracked incrementally via
`activated_reward_layers`.

Crucially, the per-passenger *maximum* reward means elementarity does NOT let us
drop the layer/age bookkeeping the way the aggregate pair-based `station_simple.jl`
drops `station_age` for `live_origin_age`: reaching a strictly better dropoff later
still activates incremental layers, so a live clock stays useful even after it has
already certified something. What elementarity removes is clock *resets* -- a
station is visited exactly once, so a clock only ages and is never reopened.

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
primitives from `data.jl`/`labels.jl`/`exact.jl`. Emits the same
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
Delegates to the shared `_make_sparse_station_ages` (`utils.jl`), which runs
the identical insertion sort used by the revisit-tolerant pricer's twin in
`labels.jl` -- only the return type differs (wrapped here in
`JointRoutingAssignmentStationSimpleAges`).
"""
function _make_joint_routing_assignment_station_simple_ages(
    label::JointRoutingAssignmentStationSimpleLabel,
    node_index::Dict{Int, Int},
)::JointRoutingAssignmentStationSimpleAges
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    return JointRoutingAssignmentStationSimpleAges(age_idx, age_val, age_mask)
end

"""
Scalar dominance state, copied out of the label so the state's label-list scan
rejects the common case without dereferencing `label`/`ages` at all -- same rationale as
`JointRoutingAssignmentDominanceFilters` (`types.jl`). `visited` and
`activated_reward_layers` are not carried here (unlike the revisit-tolerant
pricer's filters): both are already `BitSet`s on the label, compared directly,
so mirroring them would only add a redundant copy -- see this file's module
docstring.
"""
struct JointRoutingAssignmentStationSimpleDominanceFilters
    reduced_cost::Float64
    time::Float64
    route_length::Int32
    n_live_ages::Int32
end

JointRoutingAssignmentStationSimpleDominanceFilters(
    label::JointRoutingAssignmentStationSimpleLabel, ages::JointRoutingAssignmentStationSimpleAges,
) = JointRoutingAssignmentStationSimpleDominanceFilters(
    label.reduced_cost, label.time, Int32(label.route_length), Int32(length(ages.age_idx)),
)

PricingLabelEntry(
    id::JointRoutingAssignmentLabelId,
    label::JointRoutingAssignmentStationSimpleLabel,
    ages::JointRoutingAssignmentStationSimpleAges,
) = PricingLabelEntry(JointRoutingAssignmentStationSimpleDominanceFilters(label, ages), id, label, ages)

"""Dominance-rule marker for the passenger station-simple pricer; no switches
yet, unlike the revisit-tolerant pricer's four (elementary routes have no
analogous optional caps to toggle)."""
struct JointRoutingAssignmentStationSimpleDominanceRules <: AbstractPricingDominanceRules end

"""
    _dominates_joint_routing_assignment_station_simple_label(a, b, abs, bbs, layer_weight)

`a` dominates `b`: every completion of `b` has a counterpart from `a` at least as
good. Callers only ever compare labels drawn from the same `current` state, so
`a.current == b.current` is re-checked only as a cheap guard.

The visited resource is a **subset** test, `U_a ⊆ U_b`, not equality. For an
elementary route `visited` is the set of forbidden future stations, so if `a` has
visited a subset of what `b` has, every station `b` may still visit `a` may visit
too -- hence every completion feasible for `b` is feasible from `a`. This is
strictly stronger than the exact `(current, visited)` state it replaced: a
"lean" label (visited a subset) can now kill a "wandered" one that forbade itself
extra stations for no gain, which the exact rule structurally could not, and which
was measured letting the live-label population balloon 3-6x (see the note). Because
`U_a ⊆ U_b` implies `route_length_a <= route_length_b`, the `max_stops` resource is
subsumed and needs no separate check. The remaining conditions are the
revisit-tolerant pricer's:

  - `time_a <= time_b`;
  - the compensated reward-layer budget `rc_a + w(A_a ∖ A_b) <= rc_b` (see
    `_joint_routing_assignment_compensation` and the dominance docstring in
    `labels.jl` for why this, not `issubset` on layers, is the sound test);
  - every live station age in `a` is no larger than `b`'s (sparse merge walk).

Conditions are ordered cheapest-and-likeliest-to-reject first, exactly as in
`_pricing_dominates_at_state` (passenger method): scalars, then the word-wise
`visited` subset, then the `O(#live)` age walk, and only last the reward-layer
compensation, which is the one test that has to sum weights. `a.current ==
b.current` is *not* checked -- both states (`current` under `:subset`,
`(current, visited)` under `:exact`) already include it, so it was a
guaranteed-true compare in the hot loop.
"""
function _dominates_joint_routing_assignment_station_simple_label(
    a::JointRoutingAssignmentStationSimpleLabel,
    b::JointRoutingAssignmentStationSimpleLabel,
    a_ages::JointRoutingAssignmentStationSimpleAges,
    b_ages::JointRoutingAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
)::Bool
    return _pricing_dominates_at_state(
        JointRoutingAssignmentStationSimpleDominanceFilters(a, a_ages), a, a_ages,
        JointRoutingAssignmentStationSimpleDominanceFilters(b, b_ages), b, b_ages,
        layer_weight, JointRoutingAssignmentStationSimpleDominanceRules(),
    )
end

"""
The form the state's label-list scan calls: the scalars come from
`JointRoutingAssignmentStationSimpleDominanceFilters`, so an entry rejected on
time, live-clock count or reduced cost is never dereferenced into its label at
all. `visited`/`activated_reward_layers` are read off `a`/`b` directly (see
`JointRoutingAssignmentStationSimpleDominanceFilters`'s docstring for why they
are not mirrored into the filters/ages, unlike the revisit-tolerant pricer).
"""
@inline function _pricing_dominates_at_state(
    af::JointRoutingAssignmentStationSimpleDominanceFilters,
    a::JointRoutingAssignmentStationSimpleLabel, a_ages::JointRoutingAssignmentStationSimpleAges,
    bf::JointRoutingAssignmentStationSimpleDominanceFilters,
    b::JointRoutingAssignmentStationSimpleLabel, b_ages::JointRoutingAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
    ::JointRoutingAssignmentStationSimpleDominanceRules,
)::Bool
    af.time <= bf.time + 1e-9 || return false
    # `dom(age_b) ⊆ dom(age_a)` is required below, so `a` cannot have fewer live
    # clocks than `b`, nor a mask missing any of `b`'s -- both cheap, ahead of
    # anything that reads set contents. Shared with the revisit-tolerant bitset
    # dominance via `mechanics.jl`.
    _sparse_station_age_support_rejection(
        a_ages.age_idx, a_ages.age_mask, b_ages.age_idx, b_ages.age_mask,
    ) == 0 || return false
    budget = bf.reduced_cost - af.reduced_cost + 1e-9
    budget >= 0.0 || return false
    # `visited` and `activated_reward_layers` are read straight off the labels --
    # both are `BitSet`s, so `issubset`/compensation are word-wise with no per-label
    # bitset reconstruction.
    issubset(a.visited, b.visited) || return false
    # `dom(age_b) ⊆ dom(age_a)` and `age_a(j) <= age_b(j)` for j in dom(age_b) --
    # the same O(#live) merge as the revisit-tolerant bitset dominance, shared via
    # `mechanics.jl`.
    _sparse_station_age_values_dominate(
        a_ages.age_idx, a_ages.age_val, b_ages.age_idx, b_ages.age_val,
    ) || return false
    _joint_routing_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
    ) <= budget || return false
    return true
end

function _initial_joint_routing_assignment_station_simple_labels(
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{JointRoutingAssignmentStationSimpleLabel}
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = JointRoutingAssignmentStationSimpleLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, JointRoutingAssignmentStationSimpleLabel(
            node,
            [node],
            BitSet((node,)),
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

"""
Candidate next nodes for an elementary label: the revisit-tolerant
`_joint_routing_assignment_candidate_next_nodes` restricted to unvisited nodes,
with the station-budget branch removed (subsumed by elementarity).
"""
function _joint_routing_assignment_station_simple_candidate_next_nodes(
    label::JointRoutingAssignmentStationSimpleLabel,
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_joint_routing_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin in label.visited && continue  # elementary: no revisit (also excludes current)
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _joint_routing_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination in label.visited && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _joint_routing_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

"""
Extend an elementary label to `next_node` (which must be unvisited). Reward
certification and clock aging are identical to
`extend_joint_routing_assignment_pricing_label`; the revisit-tolerant version's
special handling of a re-visited `next_node` clock is simply unreachable here
(`next_node ∉ visited ⊇ keys(station_age)`), so it is omitted.
"""
function _extend_joint_routing_assignment_station_simple_label(
    label::JointRoutingAssignmentStationSimpleLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
)::JointRoutingAssignmentStationSimpleLabel
    next_node in label.visited &&
        throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _joint_routing_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    new_visited = copy(label.visited)
    push!(new_visited, next_node)

    certified_layers, reward = _certify_joint_routing_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        aged = age + travel_time
        _joint_routing_assignment_age_is_useful(station, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _joint_routing_assignment_age_is_useful(next_node, 0.0, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    end

    return JointRoutingAssignmentStationSimpleLabel(
        next_node,
        new_route,
        new_visited,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )
end

"""
Context for the elementary-route passenger free-assignment search: bundles
`pricing_data`, `dominance_mode` (`:exact` states on `(current, visited)`;
`:subset` states on `current` alone, pairing it with a shared `empty_visited`
so both modes share one concrete state type), the once-built `dominates`
closure, and the `search_index`/`bound_workspace` the shared remaining-reward
bound needs. Plugs into `_run_pricing_label_search` (`engine.jl`) the same way
`JointRoutingAssignmentSearchContext` does (`exact.jl`). Not currently
reachable from `joint_routing_assignment/pricing_round.jl`'s
`_pricing_build_unit_context` (always builds the revisit-tolerant context
today) -- kept as a real, independently usable capability.
"""
struct JointRoutingAssignmentStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentStationSimpleDominanceFilters, JointRoutingAssignmentStationSimpleLabel, JointRoutingAssignmentStationSimpleAges,
    Tuple{Int, BitSet}, RewardLayerBitset,
}
    pricing_data::JointRoutingAssignmentPricingData
    dominance_mode::Symbol
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    node_index::Dict{Int, Int}
    empty_visited::BitSet
end

function JointRoutingAssignmentStationSimpleSearchContext(
    pricing_data::JointRoutingAssignmentPricingData; dominance_mode::Symbol=:exact,
)
    dominance_mode in (:subset, :exact) ||
        throw(ArgumentError("dominance_mode must be :subset or :exact, got $(dominance_mode)"))
    rules = JointRoutingAssignmentStationSimpleDominanceRules()
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.label, x.bitsets, y.filters, y.label, y.bitsets, pricing_data.layer_weight, rules,
    )
    n_nodes = length(pricing_data.nodes)
    search_index = _build_joint_routing_assignment_search_index(pricing_data)
    bound_workspace = _create_joint_routing_assignment_bound_workspace(n_nodes)
    return JointRoutingAssignmentStationSimpleSearchContext(
        pricing_data, dominance_mode, dominates, search_index, bound_workspace, search_index.node_index, BitSet(),
    )
end

_pricing_initial_labels(ctx::JointRoutingAssignmentStationSimpleSearchContext) =
    _initial_joint_routing_assignment_station_simple_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    _make_joint_routing_assignment_station_simple_ages(label, ctx.node_index)

# `:subset` states on `current` alone; pairing it with the shared `empty_visited`
# keeps a single concrete key type for both modes.
_pricing_state(
    ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, ::JointRoutingAssignmentStationSimpleAges,
) = ctx.dominance_mode === :exact ? (label.current, label.visited) : (label.current, ctx.empty_visited)

# The reward bound reads only `current`/`time`/`activated_reward_layers` from the
# label and `age_idx`/`age_val` from the ages mirror, so the revisit-tolerant
# pricer's bound (`exact.jl`) applies unchanged here.
_pricing_label_priority(
    ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, label_ages::JointRoutingAssignmentStationSimpleAges,
) = label.reduced_cost -
    _joint_routing_assignment_remaining_reward_bound(label, label_ages, ctx.pricing_data, ctx.search_index, ctx.bound_workspace)

_pricing_best_signature(::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    isempty(label.activated_reward_layers) ? nothing : label.activated_reward_layers

_pricing_route_length(::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentStationSimpleSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel) =
    _joint_routing_assignment_station_simple_candidate_next_nodes(label, ctx.pricing_data)

_pricing_extend_label(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel, next_node::Int) =
    _extend_joint_routing_assignment_station_simple_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentStationSimpleSearchContext) = ctx.dominates

"""
Elementary-route counterpart of the revisit-tolerant pricer's
`_pricing_candidate_from_label` (`exact.jl`): identical route-replay, since
replay is agnostic to how the physical route was found."""
function _pricing_candidate_from_label(ctx::JointRoutingAssignmentStationSimpleSearchContext, label::JointRoutingAssignmentStationSimpleLabel)
    assignments, tau, reduced_cost = _joint_routing_assignment_column_from_route(
        label.route, ctx.pricing_data; label_reduced_cost=label.reduced_cost,
    )
    isempty(assignments) && return nothing
    return (
        signature=_joint_routing_assignment_column_signature(assignments),
        tau=tau, reduced_cost=reduced_cost, payload=(route=label.route, assignments=assignments),
    )
end

_pricing_pool_signature(::JointRoutingAssignmentStationSimpleSearchContext, existing_column::JointRoutingAssignmentRouteColumn) =
    _joint_routing_assignment_column_signature(existing_column)

_pricing_make_column(ctx::JointRoutingAssignmentStationSimpleSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
        ),
    )

"""Same master reduced-cost cross-check as the revisit-tolerant context's twin
(`exact.jl`) -- the master doesn't know which route universe a column came
from, only the assignments it carries."""
function _pricing_verify_column(::JointRoutingAssignmentStationSimpleSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
