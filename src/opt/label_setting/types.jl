"""
The per-state label list and the `AbstractPricingSearchContext` hook
contract shared by every label-setting pricer -- the "what a pricer must
supply" half of the engine. See `engine.jl` for the loop that actually calls
these hooks, and `aggregate_od_route/base/types.jl` / `joint_routing_assignment/types.jl`
for each pricer's own data/label/filters types that plug into the generics here.
"""

"""
Marker supertype for a pricer's dominance-rule switches, which each concrete
pricer encodes as its own type parameters (e.g. `AggregateODRouteDominanceRules{BoundedStops}`
in `aggregate_od_route/base/types.jl`, `JointRoutingAssignmentDominanceRules{BoundedStops,Compensated,Instrumented}`
in `joint_routing_assignment/types.jl`) for zero-cost specialization -- see either
concrete type's own docstring for why. This common supertype does not unify
their field/parameter shapes (those differ for real reasons); it exists so
future shared dispatch has somewhere to hang.
"""
# ── dominance-rule marker type ──────────────────────────────────────────────
abstract type AbstractPricingDominanceRules end

"""
Generic per-label entry shared by every label-setting pricer. Two labels can
only ever dominate one another when they occupy the same search **state**
(same current node, or, for the elementary-route pricer, the same
`(current, visited)`) -- outside that, their future extensions diverge and
comparison is meaningless. `_run_pricing_label_search` (`engine.jl`) therefore
keeps one list of live `PricingLabelEntry`s per state, and every dominance
check happens within a single such list.

Each concrete pricer parameterizes this entry by its own `XxxDominanceFilters`,
label, and bitsets types (e.g. `PricingLabelEntry{AggregateODRouteDominanceFilters,
AggregateODRoutePricingLabel, AggregateODRouteLabelBitsets}`); `filters` is kept
inline (not behind a `label`/`bitsets` lookup) so a state's label-list scan stays
cache-friendly. `_pricing_entry_order_key` is the sort key a state's label list
is kept sorted by (ascending reduced cost, then time, then route length, then id
as a tiebreaker).
"""
# ── per-state label list: PricingLabelEntry / PricingStateLabels ───────────
struct PricingLabelEntry{F, L, B}
    filters::F
    id::Int
    label::L
    bitsets::B
end

"""The live labels currently occupying one search state -- kept sorted by
`_pricing_entry_order_key`. See `PricingLabelEntry`'s docstring for why
dominance is only ever tested within one of these lists."""
const PricingStateLabels{F, L, B} = Vector{PricingLabelEntry{F, L, B}}

_pricing_entry_order_key(entry::PricingLabelEntry) =
    (entry.filters.reduced_cost, entry.filters.time, entry.filters.route_length, entry.id)

"""
    _add_pricing_label_to_state!(state_labels, live_labels, label, label_id, label_bs, dominates, dominated)

Single-walk insertion/eviction into one state's live-label list
(`PricingStateLabels`), shared by every label-setting pricer's dominance test
(`aggregate_od_route`, its station-simple variant, and both of
`joint_routing_assignment`'s). `state_labels` is kept sorted by
`_pricing_entry_order_key`, and domination in either direction requires
`rc_dominator <= rc_dominated`, so the walk splits at the new label's own
reduced cost: below it only an incumbent can dominate the new label (and
finding one ends the walk), above it only the new label can dominate
incumbents. Both directions are resolved in this single walk, and dominated
entries are collected as ascending list indices, so eviction is one
`deleteat!` and nothing is mutated mid-scan.

`dominates(x::PricingLabelEntry, y::PricingLabelEntry)::Bool` -- "does `x`
dominate `y`?" -- is supplied by the caller rather than fixed here, since each
pricer's actual dominance test differs (served-pairs subset vs. reward-layer
compensated budget, with or without a distinct-stations cap, ...) and some
need extra per-search-constant context (e.g. the passenger pricers' reward
`layer_weight`) that this walk has no business knowing about. Callers close
over that context and their own `_pricing_dominates_at_state` method once per
search and hand the closure to `_run_pricing_label_search` (`engine.jl`) via
the `_pricing_dominates_fn` hook below; see any concrete
`AbstractPricingSearchContext` subtype's `dominates(...)` definition for the
pattern.
"""
function _add_pricing_label_to_state!(
    state_labels::PricingStateLabels{F, L, B},
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

    new_entry = PricingLabelEntry(label_id, label, label_bs)
    new_rc = new_entry.filters.reduced_cost

    @inbounds for i in eachindex(state_labels)
        entry = state_labels[i]
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
            live_labels[state_labels[i].id] = nothing
        end
        deleteat!(state_labels, dominated)
        insert!(state_labels, searchsortedfirst(state_labels, new_entry; by=_pricing_entry_order_key), new_entry)
    end
    return inserted, length(dominated)
end

"""
    AbstractPricingSearchContext{F, L, B, State, BestSig}

Base type for the per-algorithm context threaded through one label-setting
priority-queue search (`_run_pricing_label_search`, `engine.jl`). `F`/`L`/`B`
are the dominance-filters/label/bitsets types (same three parameters
`PricingLabelEntry` and `PricingStateLabels` take); `State` is the search-state
type each label is keyed by (`labels_by_state::Dict{State, ...}`) and `BestSig`
is the per-request best-column key type (`best_by_signature::Dict{BestSig, L}`)
-- a different notion of "signature": the *served-pairs/route identity* a
finished label competes on, not the DP state two labels must share to be
comparable at all. Both are carried as explicit type parameters -- not `Any`
-- because at least one pricer's own comments call out that an `Any`-keyed
`Dict` measurably boxes every key and dispatches its hash dynamically;
concrete typing here keeps that already-paid-for optimization.

A concrete context (`AggregateODRouteSearchContext`, `AggregateODRouteStationSimpleSearchContext`,
`JointRoutingAssignmentSearchContext`, `JointRoutingAssignmentStationSimpleSearchContext`)
bundles whatever that algorithm's hooks need -- `pricing_data`, `duals` if
applicable, dominance rules, precomputed indices for its reward bound -- built
once by the driver function before calling `_run_pricing_label_search`. Each
hook below has no default method (bar `_pricing_on_label_inserted`, a true
no-op default): every concrete context must implement all of them.
"""
# ── AbstractPricingSearchContext: the hook contract ─────────────────────────
abstract type AbstractPricingSearchContext{F, L, B, State, BestSig} end

"""Initial (depth-1) labels the search seeds the frontier with."""
_pricing_initial_labels(ctx::AbstractPricingSearchContext) =
    error("_pricing_initial_labels not implemented for $(typeof(ctx))")

"""Build the per-label bitsets mirror (`B`) the dominance test and state key need."""
_pricing_make_bitsets(ctx::AbstractPricingSearchContext, label) =
    error("_pricing_make_bitsets not implemented for $(typeof(ctx))")

"""Search state (`State`) for `label`/`label_bs` -- labels only ever dominate
one another within the same state, so this defines the granularity two labels
must share (same current node, or `(current, visited)` for an elementary-route
pricer) before comparison is even meaningful."""
_pricing_state(ctx::AbstractPricingSearchContext, label, label_bs) =
    error("_pricing_state not implemented for $(typeof(ctx))")

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

"""`(x::PricingLabelEntry, y::PricingLabelEntry) -> Bool` -- does `x`
dominate `y`? -- built once per search and forwarded to
`_add_pricing_label_to_state!` unchanged; see that function's docstring for
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
