"""
Generic label-setting engine shared by every pricer: the dominance-bucket
entry/container, the single-walk bucket-insertion primitive, and the
priority-queue search context/loop. See `mechanics.jl` for the shared
reward-model-independent dominance math, and `aggregate_od_route/types.jl` /
`joint_routing_assignment/types.jl` for each pricer's own data/label/filters
types that plug into the generics here.
"""

"""
Marker supertype for a pricer's dominance-rule switches, which each concrete
pricer encodes as its own type parameters (e.g. `AggregateODRouteDominanceRules{BoundedStops}`
in `aggregate_od_route/types.jl`, `JointRoutingAssignmentDominanceRules{BoundedStops,Compensated,Instrumented}`
in `joint_routing_assignment/types.jl`) for zero-cost specialization -- see either
concrete type's own docstring for why. This common supertype does not unify
their field/parameter shapes (those differ for real reasons); it exists so
future shared dispatch has somewhere to hang. Defined here (rather than in
`mechanics.jl`, where the rest of the cross-pricer shared code lives) because
this file loads first among the pricing files, before any concrete dominance
rules struct needs it as a supertype.
"""
abstract type AbstractPricingDominanceRules end

"""
Generic dominance-bucket entry/container shared by every label-setting pricer.
Each concrete pricer parameterizes this by its own `XxxDominanceFilters`, label,
and bitsets types (e.g. `PricingBucketEntry{AggregateODRouteDominanceFilters,
AggregateODRoutePricingLabel, AggregateODRouteLabelBitsets}`); `filters` is kept
inline (not behind a `label`/`bitsets` lookup) so bucket scans stay cache-friendly.
`_pricing_entry_order_key` is the sort key each bucket is kept sorted by
(ascending reduced cost, then time, then route length, then id as a tiebreaker).
"""
struct PricingBucketEntry{F, L, B}
    filters::F
    id::Int
    label::L
    bitsets::B
end

const PricingDominanceBucket{F, L, B} = Vector{PricingBucketEntry{F, L, B}}

_pricing_entry_order_key(entry::PricingBucketEntry) =
    (entry.filters.reduced_cost, entry.filters.time, entry.filters.route_length, entry.id)

"""
    _add_pricing_label_to_bucket!(bucket, live_labels, label, label_id, label_bs, dominates, dominated)

Single-walk bucket insertion/eviction shared by every label-setting pricer's
dominance bucket (`aggregate_od_route`, its station-simple variant, and both of
`joint_routing_assignment`'s). The bucket is kept sorted by
`_pricing_entry_order_key`, and domination in either direction requires
`rc_dominator <= rc_dominated`, so the walk splits at the new label's own
reduced cost: below it only an incumbent can dominate the new label (and
finding one ends the walk), above it only the new label can dominate
incumbents. Both directions are resolved in this single walk, and dominated
entries are collected as ascending bucket indices, so eviction is one
`deleteat!` and nothing is mutated mid-scan.

`dominates(x::PricingBucketEntry, y::PricingBucketEntry)::Bool` -- "does `x`
dominate `y`?" -- is supplied by the caller rather than fixed here, since each
pricer's actual dominance test differs (served-pairs subset vs. reward-layer
compensated budget, with or without a distinct-stations cap, ...) and some
need extra per-search-constant context (e.g. the passenger pricers' reward
`layer_weight`) that this walk has no business knowing about. Callers close
over that context and their own `_pricing_dominates_in_bucket` method once per
search and hand the closure to `_run_pricing_label_search` via the
`_pricing_dominates_fn` hook below; see any concrete `AbstractPricingSearchContext`
subtype's `dominates(...)` definition for the pattern.
"""
function _add_pricing_label_to_bucket!(
    bucket::PricingDominanceBucket{F, L, B},
    live_labels::Vector{Union{Nothing, L}},
    label::L,
    label_id::Int,
    label_bs::B,
    dominates::Function,
    dominated::Vector{Int},
) where {F, L, B}
    inserted = true
    empty!(dominated)
    switched = false

    new_entry = PricingBucketEntry(label_id, label, label_bs)
    new_rc = new_entry.filters.reduced_cost

    @inbounds for i in eachindex(bucket)
        entry = bucket[i]
        if !switched && new_rc > entry.filters.reduced_cost + 1e-9
            if dominates(entry, new_entry)
                inserted = false
                break
            end
            continue
        end

        switched = true
        if dominates(new_entry, entry)
            push!(dominated, i)
        end
    end

    if inserted
        @inbounds for i in dominated
            live_labels[bucket[i].id] = nothing
        end
        deleteat!(bucket, dominated)
        insert!(bucket, searchsortedfirst(bucket, new_entry; by=_pricing_entry_order_key), new_entry)
    end
    return inserted, length(dominated)
end

"""
    AbstractPricingSearchContext{F, L, B, Sig, BestSig}

Base type for the per-algorithm context threaded through one label-setting
priority-queue search (`_run_pricing_label_search`). `F`/`L`/`B` are the
dominance-filters/label/bitsets types (same three parameters `PricingBucketEntry`
and `PricingDominanceBucket` take); `Sig` is the dominance-bucket key type
(`dominance_buckets::Dict{Sig, ...}`) and `BestSig` is the per-request
best-column key type (`best_by_signature::Dict{BestSig, L}`). Both are carried
as explicit type parameters -- not `Any` -- because at least one pricer's own
comments call out that an `Any`-keyed dominance-bucket `Dict` measurably boxes
every signature and dispatches its hash dynamically; concrete typing here keeps
that already-paid-for optimization.

A concrete context (`AggregateODRouteSearchContext`, `AggregateODRouteStationSimpleSearchContext`,
`JointRoutingAssignmentSearchContext`, `JointRoutingAssignmentStationSimpleSearchContext`)
bundles whatever that algorithm's hooks need -- `pricing_data`, `duals` if
applicable, dominance rules, precomputed indices for its reward bound -- built
once by the driver function before calling `_run_pricing_label_search`. Each
hook below has no default method (bar `_pricing_on_label_inserted`, a true
no-op default): every concrete context must implement all of them.
"""
abstract type AbstractPricingSearchContext{F, L, B, Sig, BestSig} end

"""Initial (depth-1) labels the search seeds the frontier with."""
_pricing_initial_labels(ctx::AbstractPricingSearchContext) =
    error("_pricing_initial_labels not implemented for $(typeof(ctx))")

"""Build the per-label bitsets mirror (`B`) the dominance test and bucket key need."""
_pricing_make_bitsets(ctx::AbstractPricingSearchContext, label) =
    error("_pricing_make_bitsets not implemented for $(typeof(ctx))")

"""Dominance-bucket key (`Sig`) for `label`/`label_bs` -- labels only ever
dominate one another within the same bucket, so this defines the granularity."""
_pricing_bucket_signature(ctx::AbstractPricingSearchContext, label, label_bs) =
    error("_pricing_bucket_signature not implemented for $(typeof(ctx))")

"""Frontier priority: a lower bound on the label's own reduced cost plus every
reward it could still collect, so a label popped at `>= -reduced_cost_tol` can
never improve regardless of how it is extended."""
_pricing_label_priority(ctx::AbstractPricingSearchContext, label, label_bs)::Float64 =
    error("_pricing_label_priority not implemented for $(typeof(ctx))")

"""Key (`Union{Nothing, BestSig}`) under which a *finished* label competes for
being the best-so-far candidate toward a column, or `nothing` if `label`
represents no certified reward yet and should not be tracked at all."""
_pricing_best_signature(ctx::AbstractPricingSearchContext, label) =
    error("_pricing_best_signature not implemented for $(typeof(ctx))")

"""Number of stops `label`'s route already has (the resource the `max_stops`
cap is measured against)."""
_pricing_route_length(ctx::AbstractPricingSearchContext, label)::Int =
    error("_pricing_route_length not implemented for $(typeof(ctx))")

"""The `max_stops` cap itself."""
_pricing_max_route_length(ctx::AbstractPricingSearchContext)::Int =
    error("_pricing_max_route_length not implemented for $(typeof(ctx))")

"""Nodes `label` may legally extend to next."""
_pricing_candidate_next_nodes(ctx::AbstractPricingSearchContext, label) =
    error("_pricing_candidate_next_nodes not implemented for $(typeof(ctx))")

"""Extend `label` with a visit to `next_node`, returning the child label."""
_pricing_extend_label(ctx::AbstractPricingSearchContext, label, next_node) =
    error("_pricing_extend_label not implemented for $(typeof(ctx))")

"""`(x::PricingBucketEntry, y::PricingBucketEntry) -> Bool` -- does `x`
dominate `y`? -- built once per search and forwarded to
`_add_pricing_label_to_bucket!` unchanged; see that function's docstring for
why this is a closure rather than a fixed call."""
_pricing_dominates_fn(ctx::AbstractPricingSearchContext)::Function =
    error("_pricing_dominates_fn not implemented for $(typeof(ctx))")

"""Optional diagnostic hook, called once per label that survives dominance and
enters the frontier. The default is a true no-op (not merely unused): most
pricers never override it, so this must cost nothing beyond the dispatch
itself. Only `JointRoutingAssignmentSearchContext` overrides it today, for
`julia scripts/diagnose.jl split_census`."""
_pricing_on_label_inserted(ctx::AbstractPricingSearchContext, label) = nothing

"""Optional hook, called once right before the search loop starts, with its
start time and time budget. The default is a no-op; only
`JointRoutingAssignmentSearchContext` overrides it, stashing both into mutable
fields so its post-`W` completion bound (computed inside `_pricing_label_priority`,
which has no other way to see the search's remaining time budget) can size its
own sub-search's `time_limit`."""
_pricing_search_started!(ctx::AbstractPricingSearchContext, t_start::Float64, time_limit::Float64) = nothing

"""
    _run_pricing_label_search(ctx; time_limit, reduced_cost_tol, use_reduced_cost_pruning=true, profile=false, stop_if=label->false)

The priority-queue label-setting search shared by every pricer's enumerate
function: seed the frontier from `_pricing_initial_labels`, then repeatedly pop
the most promising live label, record it as a best-so-far candidate under
`_pricing_best_signature` (offering it to `stop_if` for early exit), and -- if
it is not at the stops cap and its priority still beats `reduced_cost_tol` --
extend it along every `_pricing_candidate_next_nodes` result, inserting each
child through the shared `_add_pricing_label_to_bucket!`.

This function is the generalization of what were four independently written,
near-identical copies of this exact loop (frontier/live-labels/dominance-bucket
bookkeeping, the same `stats` shape, the same profiling timers, the same
stale-pop handling) -- one per pricer × route-universe combination. Everything
that differs between them (candidate generation, label extension, the reward
bound behind the priority, what counts as a "finished" label worth tracking) is
a hook on `ctx`; this loop knows none of it. See any of the four concrete
`AbstractPricingSearchContext` subtypes and their `_enumerate_*` wrapper for
the pattern.
"""
function _run_pricing_label_search(
    ctx::AbstractPricingSearchContext{F, L, B, Sig, BestSig};
    time_limit::Float64,
    reduced_cost_tol::Float64,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
) where {F, L, B, Sig, BestSig}
    frontier = PriorityQueue{Int, Float64}()
    live_labels = Union{Nothing, L}[]
    n_live_labels = 0
    dominance_buckets = Dict{Sig, PricingDominanceBucket{F, L, B}}()
    best_by_signature = Dict{BestSig, L}()
    dominated_scratch = Int[]
    dominates = _pricing_dominates_fn(ctx)

    exhausted = true
    t_start = time()
    _pricing_search_started!(ctx, t_start, time_limit)
    next_label_id = 1
    labels_generated = 0
    labels_rejected_by_dominance = 0
    labels_removed_by_dominance = 0
    stale_pops = 0
    max_frontier_size = 0
    max_live_labels = 0
    t_queue = UInt64(0)
    t_candidates = UInt64(0)
    t_extension = UInt64(0)
    t_dominance = UInt64(0)

    function add_label!(label::L)
        label_id = next_label_id
        next_label_id += 1
        labels_generated += 1
        push!(live_labels, label)
        n_live_labels += 1
        label_bs = _pricing_make_bitsets(ctx, label)
        signature = _pricing_bucket_signature(ctx, label, label_bs)
        bucket = get!(() -> PricingDominanceBucket{F, L, B}(), dominance_buckets, signature)

        t0 = profile ? time_ns() : UInt64(0)
        inserted, removed = _add_pricing_label_to_bucket!(
            bucket, live_labels, label, label_id, label_bs, dominates, dominated_scratch,
        )
        profile && (t_dominance += time_ns() - t0)
        labels_removed_by_dominance += removed
        n_live_labels -= removed

        if !inserted
            live_labels[label_id] = nothing
            n_live_labels -= 1
            labels_rejected_by_dominance += 1
            return nothing
        end

        t0 = profile ? time_ns() : UInt64(0)
        push!(frontier, label_id => _pricing_label_priority(ctx, label, label_bs))
        profile && (t_queue += time_ns() - t0)
        max_frontier_size = max(max_frontier_size, length(frontier))
        max_live_labels = max(max_live_labels, n_live_labels)
        _pricing_on_label_inserted(ctx, label)
        return nothing
    end

    for label in _pricing_initial_labels(ctx)
        add_label!(label)
    end

    while !isempty(frontier)
        if time() - t_start > time_limit
            exhausted = false
            break
        end

        t0 = profile ? time_ns() : UInt64(0)
        label_id, popped_priority = popfirst!(frontier)
        profile && (t_queue += time_ns() - t0)
        maybe_label = live_labels[label_id]
        if isnothing(maybe_label)
            stale_pops += 1
            continue
        end
        label = maybe_label::L

        maybe_signature = _pricing_best_signature(ctx, label)
        if !isnothing(maybe_signature)
            signature = maybe_signature::BestSig
            incumbent = get(best_by_signature, signature, nothing)
            if isnothing(incumbent) || label.tau < incumbent.tau - 1e-9
                best_by_signature[signature] = label
                if stop_if(label)
                    exhausted = false
                    break
                end
            end
        end

        _pricing_route_length(ctx, label) >= _pricing_max_route_length(ctx) && continue
        if use_reduced_cost_pruning
            popped_priority >= -reduced_cost_tol && continue
        end

        t0 = profile ? time_ns() : UInt64(0)
        next_nodes = _pricing_candidate_next_nodes(ctx, label)
        profile && (t_candidates += time_ns() - t0)

        for next_node in next_nodes
            t0 = profile ? time_ns() : UInt64(0)
            child = _pricing_extend_label(ctx, label, next_node)
            profile && (t_extension += time_ns() - t0)
            add_label!(child)
        end
    end

    stats = (
        labels_generated=labels_generated,
        labels_rejected_by_dominance=labels_rejected_by_dominance,
        labels_removed_by_dominance=labels_removed_by_dominance,
        stale_pops=stale_pops,
        max_frontier_size=max_frontier_size,
        max_live_labels=max_live_labels,
        t_queue_sec=t_queue * 1e-9,
        t_candidates_sec=t_candidates * 1e-9,
        t_extension_sec=t_extension * 1e-9,
        t_dominance_sec=t_dominance * 1e-9,
    )
    return collect(values(best_by_signature)), exhausted, stats
end
