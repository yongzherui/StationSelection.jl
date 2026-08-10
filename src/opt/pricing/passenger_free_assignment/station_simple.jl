"""
Station-simple (elementary-route) label-setting pricer for the passenger
free-assignment subproblem: an alternative to the revisit-tolerant search in
`labels.jl`/`search.jl` in which a physical route may never revisit a station.

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

2. **Fine dominance buckets (`dominance_mode = :exact`, the default).** Buckets are
   keyed on the exact `(current, visited)` pair, so each bucket is tiny and every
   insertion's dominance scan is short. That is the whole game here: the scan is
   O(bucket) per insertion and ~85-90% of wall time, so bucket *granularity*
   dominates. At n=20 this makes the elementary search 1.6-3.5x faster than the
   revisit-tolerant pricer.

   A `:subset` mode also exists (bucket on `current`, add `U_a ⊆ U_b` to
   dominance). It is a strictly stronger dominance and keeps ~2x fewer live labels,
   yet it is **1.4-6.6x slower** than `:exact` because its coarse `current`-only
   buckets grow to tens of thousands of entries and the per-insertion scan blows
   up -- fewer labels do not pay for scanning giant buckets. Retained for research
   only; measured verdict and numbers in
   `notes/2026-07-30_passenger_station_simple_pricing.md`.

# Correctness caveat

Restricting to elementary routes restricts the column universe the master problem
prices over. Where the model's optimum genuinely wants a revisiting route this
pricer is a *heuristic* -- it can terminate CG with a weaker LP bound or miss
improving columns (the aggregate pair-based `use_station_simple` did exactly this
on some instance families). It is therefore opt-in and off by default; validate
the LP bound against the revisit-tolerant pricer before relying on it.

# Reuse

Shares `PassengerFreeAssignmentPricingData` (no new data struct) and the
`_passenger_free_assignment_travel`, `_certify_passenger_free_assignment_layers_at_node`,
`_passenger_free_assignment_age_is_useful`, `_has_useful_live_passenger_free_assignment_origin`,
`_passenger_free_assignment_compensation`, and `_passenger_free_assignment_remaining_reward_bound`
primitives from `data.jl`/`labels.jl`/`search.jl`. Emits the same
`PassengerFreeAssignmentRouteColumn` via the identical route-replay path
(`_passenger_free_assignment_column_from_route`), since replay is agnostic to how
the physical route was found and replays an elementary route unchanged.
"""

export PassengerFreeAssignmentStationSimpleLabel
export passenger_free_assignment_pricing_by_station_simple_label_setting

"""
A partial elementary route. Same fields as `PassengerFreeAssignmentPricingLabel`
plus an authoritative `visited` set. `route_length == length(visited)` always
holds here.

`visited` is a `BitSet` (over station ids), not a `Set{Int}`: the dominance scan's
`issubset(a.visited, b.visited)` is then a word-wise AND rather than a per-element
hash probe, and it is compared *directly* -- no separate node-index bitset has to
be rebuilt per label. `activated_reward_layers` is likewise already a `BitSet` and
is compared directly, so the only per-label precompute the search still needs is
the sorted live-age arrays (`PassengerFreeAssignmentStationSimpleAges`).
"""
struct PassengerFreeAssignmentStationSimpleLabel
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
struct PassengerFreeAssignmentStationSimpleAges
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    # One bit per live station index (folded mod 64) -- the `dom(age_b) ⊆
    # dom(age_a)` half of the age condition as a single instruction, so the merge
    # walk below only runs on pairs that can still pass it. See
    # `PassengerFreeAssignmentLabelBitsets.age_mask` for why folding is safe.
    age_mask::UInt64
end

"""
Delegates to the shared `_make_sparse_station_ages` (`mechanics.jl`), which runs
the identical insertion sort used by the revisit-tolerant pricer's twin in
`labels.jl` -- only the return type differs (wrapped here in
`PassengerFreeAssignmentStationSimpleAges`).
"""
function _make_passenger_free_assignment_station_simple_ages(
    label::PassengerFreeAssignmentStationSimpleLabel,
    node_index::Dict{Int, Int},
)::PassengerFreeAssignmentStationSimpleAges
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    return PassengerFreeAssignmentStationSimpleAges(age_idx, age_val, age_mask)
end

"""
Scalar dominance state, copied out of the label so the bucket scan rejects the
common case without dereferencing `label`/`ages` at all -- same rationale as
`PassengerFreeAssignmentDominanceFilters` (`types.jl`). `visited` and
`activated_reward_layers` are not carried here (unlike the revisit-tolerant
pricer's filters): both are already `BitSet`s on the label, compared directly,
so mirroring them would only add a redundant copy -- see this file's module
docstring.
"""
struct PassengerFreeAssignmentStationSimpleDominanceFilters
    reduced_cost::Float64
    time::Float64
    route_length::Int32
    n_live_ages::Int32
end

PassengerFreeAssignmentStationSimpleDominanceFilters(
    label::PassengerFreeAssignmentStationSimpleLabel, ages::PassengerFreeAssignmentStationSimpleAges,
) = PassengerFreeAssignmentStationSimpleDominanceFilters(
    label.reduced_cost, label.time, Int32(label.route_length), Int32(length(ages.age_idx)),
)

PricingBucketEntry(
    id::PassengerFreeAssignmentLabelId,
    label::PassengerFreeAssignmentStationSimpleLabel,
    ages::PassengerFreeAssignmentStationSimpleAges,
) = PricingBucketEntry(PassengerFreeAssignmentStationSimpleDominanceFilters(label, ages), id, label, ages)

"""Dominance-rule marker for the passenger station-simple pricer; no switches
yet, unlike the revisit-tolerant pricer's four (elementary routes have no
analogous optional caps to toggle)."""
struct PassengerFreeAssignmentStationSimpleDominanceRules <: AbstractPricingDominanceRules end

"""
    _dominates_passenger_free_assignment_station_simple_label(a, b, abs, bbs, layer_weight)

`a` dominates `b`: every completion of `b` has a counterpart from `a` at least as
good. Callers only ever compare labels drawn from the same `current` bucket, so
`a.current == b.current` is re-checked only as a cheap guard.

The visited resource is a **subset** test, `U_a ⊆ U_b`, not equality. For an
elementary route `visited` is the set of forbidden future stations, so if `a` has
visited a subset of what `b` has, every station `b` may still visit `a` may visit
too -- hence every completion feasible for `b` is feasible from `a`. This is
strictly stronger than the exact-`(current, visited)` bucketing it replaced: a
"lean" label (visited a subset) can now kill a "wandered" one that forbade itself
extra stations for no gain, which the exact rule structurally could not, and which
was measured letting the live-label population balloon 3-6x (see the note). Because
`U_a ⊆ U_b` implies `route_length_a <= route_length_b`, the `max_stops` resource is
subsumed and needs no separate check. The remaining conditions are the
revisit-tolerant pricer's:

  - `time_a <= time_b`;
  - the compensated reward-layer budget `rc_a + w(A_a ∖ A_b) <= rc_b` (see
    `_passenger_free_assignment_compensation` and the dominance docstring in
    `labels.jl` for why this, not `issubset` on layers, is the sound test);
  - every live station age in `a` is no larger than `b`'s (sparse merge walk).

Conditions are ordered cheapest-and-likeliest-to-reject first, exactly as in
`_pricing_dominates_in_bucket` (passenger method): scalars, then the word-wise
`visited` subset, then the `O(#live)` age walk, and only last the reward-layer
compensation, which is the one test that has to sum weights. `a.current ==
b.current` is *not* checked -- both bucket signatures (`current` under `:subset`,
`(current, visited)` under `:exact`) already include it, so it was a
guaranteed-true compare in the hot loop.
"""
function _dominates_passenger_free_assignment_station_simple_label(
    a::PassengerFreeAssignmentStationSimpleLabel,
    b::PassengerFreeAssignmentStationSimpleLabel,
    a_ages::PassengerFreeAssignmentStationSimpleAges,
    b_ages::PassengerFreeAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
)::Bool
    return _pricing_dominates_in_bucket(
        PassengerFreeAssignmentStationSimpleDominanceFilters(a, a_ages), a, a_ages,
        PassengerFreeAssignmentStationSimpleDominanceFilters(b, b_ages), b, b_ages,
        layer_weight, PassengerFreeAssignmentStationSimpleDominanceRules(),
    )
end

"""
The form the bucket scan calls: the scalars come from
`PassengerFreeAssignmentStationSimpleDominanceFilters`, so an entry rejected on
time, live-clock count or reduced cost is never dereferenced into its label at
all. `visited`/`activated_reward_layers` are read off `a`/`b` directly (see
`PassengerFreeAssignmentStationSimpleDominanceFilters`'s docstring for why they
are not mirrored into the filters/ages, unlike the revisit-tolerant pricer).
"""
@inline function _pricing_dominates_in_bucket(
    af::PassengerFreeAssignmentStationSimpleDominanceFilters,
    a::PassengerFreeAssignmentStationSimpleLabel, a_ages::PassengerFreeAssignmentStationSimpleAges,
    bf::PassengerFreeAssignmentStationSimpleDominanceFilters,
    b::PassengerFreeAssignmentStationSimpleLabel, b_ages::PassengerFreeAssignmentStationSimpleAges,
    layer_weight::Vector{Float64},
    ::PassengerFreeAssignmentStationSimpleDominanceRules,
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
    _passenger_free_assignment_compensation(
        a.activated_reward_layers, b.activated_reward_layers, layer_weight, budget,
    ) <= budget || return false
    return true
end

function _initial_passenger_free_assignment_station_simple_labels(
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{PassengerFreeAssignmentStationSimpleLabel}
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = PassengerFreeAssignmentStationSimpleLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, PassengerFreeAssignmentStationSimpleLabel(
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
`_passenger_free_assignment_candidate_next_nodes` restricted to unvisited nodes,
with the station-budget branch removed (subsumed by elementarity).
"""
function _passenger_free_assignment_station_simple_candidate_next_nodes(
    label::PassengerFreeAssignmentStationSimpleLabel,
    pricing_data::PassengerFreeAssignmentPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_passenger_free_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin in label.visited && continue  # elementary: no revisit (also excludes current)
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _passenger_free_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination in label.visited && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _passenger_free_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

"""
Extend an elementary label to `next_node` (which must be unvisited). Reward
certification and clock aging are identical to
`extend_passenger_free_assignment_pricing_label`; the revisit-tolerant version's
special handling of a re-visited `next_node` clock is simply unreachable here
(`next_node ∉ visited ⊇ keys(station_age)`), so it is omitted.
"""
function _extend_passenger_free_assignment_station_simple_label(
    label::PassengerFreeAssignmentStationSimpleLabel,
    next_node::Int,
    pricing_data::PassengerFreeAssignmentPricingData,
)::PassengerFreeAssignmentStationSimpleLabel
    next_node in label.visited &&
        throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _passenger_free_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    new_visited = copy(label.visited)
    push!(new_visited, next_node)

    certified_layers, reward = _certify_passenger_free_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        aged = age + travel_time
        _passenger_free_assignment_age_is_useful(station, aged, certified_layers, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _passenger_free_assignment_age_is_useful(next_node, 0.0, certified_layers, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    end

    return PassengerFreeAssignmentStationSimpleLabel(
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
Context for the elementary-route `PassengerFreeAssignmentCG` search
(`use_station_simple`/`station_simple_warm_start`): bundles `pricing_data`,
`dominance_mode` (`:exact` buckets on `(current, visited)`; `:subset` buckets
on `current` alone, pairing it with a shared `empty_visited` so both modes
share one concrete bucket-key type), the once-built `dominates` closure, and
the `search_index`/`bound_workspace` the shared remaining-reward bound needs.
Plugs into `_run_pricing_label_search` (`pricing/types.jl`); see
`_enumerate_passenger_free_assignment_station_simple_pricing_labels` below.
"""
struct PassengerFreeAssignmentStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    PassengerFreeAssignmentStationSimpleDominanceFilters, PassengerFreeAssignmentStationSimpleLabel, PassengerFreeAssignmentStationSimpleAges,
    Tuple{Int, BitSet}, RewardLayerBitset,
}
    pricing_data::PassengerFreeAssignmentPricingData
    dominance_mode::Symbol
    dominates::D
    search_index::PassengerFreeAssignmentSearchIndex
    bound_workspace::PassengerFreeAssignmentBoundWorkspace
    node_index::Dict{Int, Int}
    empty_visited::BitSet
end

function PassengerFreeAssignmentStationSimpleSearchContext(
    pricing_data::PassengerFreeAssignmentPricingData; dominance_mode::Symbol=:exact,
)
    dominance_mode in (:subset, :exact) ||
        throw(ArgumentError("dominance_mode must be :subset or :exact, got $(dominance_mode)"))
    rules = PassengerFreeAssignmentStationSimpleDominanceRules()
    dominates(x::PricingBucketEntry, y::PricingBucketEntry) = _pricing_dominates_in_bucket(
        x.filters, x.label, x.bitsets, y.filters, y.label, y.bitsets, pricing_data.layer_weight, rules,
    )
    n_nodes = length(pricing_data.nodes)
    search_index = _build_passenger_free_assignment_search_index(pricing_data)
    bound_workspace = _create_passenger_free_assignment_bound_workspace(n_nodes)
    return PassengerFreeAssignmentStationSimpleSearchContext(
        pricing_data, dominance_mode, dominates, search_index, bound_workspace, search_index.node_index, BitSet(),
    )
end

_pricing_initial_labels(ctx::PassengerFreeAssignmentStationSimpleSearchContext) =
    _initial_passenger_free_assignment_station_simple_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel) =
    _make_passenger_free_assignment_station_simple_ages(label, ctx.node_index)

# `:subset` buckets on `current` alone; pairing it with the shared `empty_visited`
# keeps a single concrete key type for both modes.
_pricing_bucket_signature(
    ctx::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel, ::PassengerFreeAssignmentStationSimpleAges,
) = ctx.dominance_mode === :exact ? (label.current, label.visited) : (label.current, ctx.empty_visited)

# The reward bound reads only `current`/`time`/`activated_reward_layers` from the
# label and `age_idx`/`age_val` from the ages mirror, so the revisit-tolerant
# pricer's bound (`search.jl`) applies unchanged here.
_pricing_label_priority(
    ctx::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel, label_ages::PassengerFreeAssignmentStationSimpleAges,
) = label.reduced_cost -
    _passenger_free_assignment_remaining_reward_bound(label, label_ages, ctx.pricing_data, ctx.search_index, ctx.bound_workspace)

_pricing_best_signature(::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel) =
    isempty(label.activated_reward_layers) ? nothing : label.activated_reward_layers

_pricing_route_length(::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel) = label.route_length

_pricing_max_route_length(ctx::PassengerFreeAssignmentStationSimpleSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel) =
    _passenger_free_assignment_station_simple_candidate_next_nodes(label, ctx.pricing_data)

_pricing_extend_label(ctx::PassengerFreeAssignmentStationSimpleSearchContext, label::PassengerFreeAssignmentStationSimpleLabel, next_node::Int) =
    _extend_passenger_free_assignment_station_simple_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::PassengerFreeAssignmentStationSimpleSearchContext) = ctx.dominates

function _enumerate_passenger_free_assignment_station_simple_pricing_labels(
    pricing_data::PassengerFreeAssignmentPricingData;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    use_reduced_cost_pruning::Bool=true,
    dominance_mode::Symbol=:exact,
    profile::Bool=false,
    stop_if=label -> false,
)
    ctx = PassengerFreeAssignmentStationSimpleSearchContext(pricing_data; dominance_mode=dominance_mode)
    return _run_pricing_label_search(
        ctx;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        use_reduced_cost_pruning=use_reduced_cost_pruning,
        profile=profile,
        stop_if=stop_if,
    )
end

"""
    passenger_free_assignment_pricing_by_station_simple_label_setting(pricing_data, existing_columns; kwargs...)

Elementary-route counterpart of `passenger_free_assignment_pricing_by_label_setting`:
runs the station-simple label search, replays every finished candidate route to
recover concrete assignments (via the shared
`_passenger_free_assignment_column_from_route`), and returns up to
`max_new_columns` improving, pool-novel `PassengerFreeAssignmentRouteColumn`s.
Acceptance/dedup operates on the real assignment signature, exactly as in the
revisit-tolerant driver.
"""
function passenger_free_assignment_pricing_by_station_simple_label_setting(
    pricing_data::PassengerFreeAssignmentPricingData,
    existing_columns::Vector{PassengerFreeAssignmentRouteColumn};
    next_column_id::Int,
    reduced_cost_tol::Float64=1e-6,
    max_new_columns::Int=1,
    n_candidates::Int=max_new_columns,
    time_limit::Float64=30.0,
    dominance_mode::Symbol=:exact,
    profile::Bool=false,
)
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _passenger_free_assignment_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored_by_signature = Dict{Any, NamedTuple}()

    function try_accept_route!(route::Vector{Int}, label_reduced_cost::Float64)::Bool
        assignments, tau, reduced_cost = _passenger_free_assignment_column_from_route(
            route, pricing_data; label_reduced_cost=label_reduced_cost,
        )
        isempty(assignments) && return false
        reduced_cost < -reduced_cost_tol || return false
        signature = _passenger_free_assignment_column_signature(assignments)
        tau < get(best_pool_tau, signature, Inf) - 1e-9 || return false
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                reduced_cost < current.reduced_cost - 1e-9 ||
                (abs(reduced_cost - current.reduced_cost) <= 1e-9 && tau < current.tau - 1e-9)
            scored_by_signature[signature] = (reduced_cost=reduced_cost, route=route, assignments=assignments, tau=tau)
        end
        return length(scored_by_signature) >= n_candidates
    end

    labels, exhausted, stats = _enumerate_passenger_free_assignment_station_simple_pricing_labels(
        pricing_data;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        dominance_mode=dominance_mode,
        profile=profile,
        stop_if=label -> try_accept_route!(label.route, label.reduced_cost),
    )

    for label in labels
        try_accept_route!(label.route, label.reduced_cost)
    end

    scored = collect(values(scored_by_signature))
    # Decorate-sort-undecorate, for the reason given at the twin in `search.jl`:
    # `by` runs per comparison, so `string(route)` inside it was the single
    # largest non-GC cost in this pricer's flame graph. Same ordering, one key
    # construction per column.
    scored = scored[sortperm([(e.reduced_cost, e.tau, string(e.route)) for e in scored])]
    scored = scored[1:min(length(scored), n_candidates)]
    scored = scored[1:min(length(scored), max_new_columns)]

    columns = PassengerFreeAssignmentRouteColumn[]
    column_id = next_column_id
    for entry in scored
        push!(columns, PassengerFreeAssignmentRouteColumn(
            column_id,
            entry.route,
            entry.assignments,
            entry.tau;
            metadata=Dict{String, Any}(
                "scenario" => pricing_data.scenario,
                "route" => Tuple(entry.route),
                "reduced_cost" => entry.reduced_cost,
            ),
        ))
        column_id += 1
    end
    return columns, exhausted, stats
end
