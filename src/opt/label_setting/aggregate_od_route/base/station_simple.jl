"""
Station-simple (elementary-route) label-setting pricer for AggregateODRouteProblem:
an alternative to the revisit-tolerant search in `labels.jl`/`exact.jl`. A route
may never revisit a station, so a certified `(j,k)` pair settles permanently the
first (and only) time `k` is visited after `j` -- there is no need for a
`station_age` Dict tracking every past visit, only `live_origin_age` for stations
already on the route whose destination hasn't been reached yet.

Because `visited` only grows and is part of the dominance signature (exact match,
not subset), a dominating label's `served_pairs` need not be compared separately:
identical `(current, visited)` plus reduced-cost/time/live-origin-age domination
already implies every future extension available to the dominated label is at
least as good from the dominating one, since which nodes remain reachable depends
only on `visited` and rewards not yet banked are fully described by
`live_origin_age`. Do not add a served-pairs comparison here without re-deriving
that invariant; see `_dominates_aggregate_od_route_station_simple_label` below.

This shares `AggregateODRoutePricingData`/`AggregateODRoutePricingDuals` and the
`_aggregate_od_route_travel`/`_direct_ride_limit` helpers from `data.jl`, and
produces `AggregateODRouteColumn`s via the same `_aggregate_od_route_column_from_label`
convention as the revisit-tolerant pricer (a new method dispatched on this file's
label type).
"""

export AggregateODRouteStationSimpleLabel

struct AggregateODRouteStationSimpleLabel
    current::Int
    route::Vector{Int}
    visited::Set{Int}
    time::Float64
    live_origin_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
end

"""
`live_origin_age` is stored sparse -- `(age_idx, age_val, age_mask)`, the same
representation `_make_sparse_station_ages` (`mechanics.jl`) builds for the
revisit-tolerant pricer and for the PFA station-simple pricer -- rather than a
dense `Vector` over every node, since a partial elementary route typically has
only a handful of live pickup clocks regardless of network size.
"""
struct AggregateODRouteStationSimpleBitsets
    visited_bits::BitSet
    age_idx::Vector{Int32}
    age_val::Vector{Float64}
    age_mask::UInt64
end

function _make_aggregate_od_route_station_simple_bitsets(
    label::AggregateODRouteStationSimpleLabel,
    node_index::Dict{Int, Int},
)::AggregateODRouteStationSimpleBitsets
    visited_bits = BitSet()
    for node in label.visited
        push!(visited_bits, node_index[node])
    end

    age_idx, age_val, age_mask = _make_sparse_station_ages(label.live_origin_age, node_index)

    return AggregateODRouteStationSimpleBitsets(visited_bits, age_idx, age_val, age_mask)
end

_aggregate_od_route_station_simple_state(
    label::AggregateODRouteStationSimpleLabel,
    bs::AggregateODRouteStationSimpleBitsets,
) = (label.current, bs.visited_bits)

function _dominates_aggregate_od_route_station_simple_label(
    a::AggregateODRouteStationSimpleLabel,
    b::AggregateODRouteStationSimpleLabel,
    abs::AggregateODRouteStationSimpleBitsets,
    bbs::AggregateODRouteStationSimpleBitsets,
)::Bool
    a.current == b.current || return false
    abs.visited_bits == bbs.visited_bits || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    a.time <= b.time + 1e-9 || return false
    # dom(b) ⊆ dom(a) and age_a(j) <= age_b(j) for j in dom(b) -- shared with the
    # revisit-tolerant bitset dominance and the PFA station-simple pricer via
    # `mechanics.jl`.
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end

"""
Inline scalar fields the state's label-list scan needs, read straight off the
entry rather than chasing into `label`/`bitsets` -- same rationale as
`AggregateODRouteDominanceFilters` (`labels.jl`). `current`/`visited_bits` are
not carried here because the state itself is exactly that key (see
`_aggregate_od_route_station_simple_state`), so every pair the
state's label-list scan tests already agrees on them.
"""
struct AggregateODRouteStationSimpleDominanceFilters
    reduced_cost::Float64
    time::Float64
    route_length::Int32
end

AggregateODRouteStationSimpleDominanceFilters(
    label::AggregateODRouteStationSimpleLabel, ::AggregateODRouteStationSimpleBitsets,
) = AggregateODRouteStationSimpleDominanceFilters(
    label.reduced_cost, label.time, Int32(length(label.route)),
)

PricingLabelEntry(id::Int, label::AggregateODRouteStationSimpleLabel, bs::AggregateODRouteStationSimpleBitsets) =
    PricingLabelEntry(AggregateODRouteStationSimpleDominanceFilters(label, bs), id, label, bs)

"""Dominance-rule marker for the station-simple pricer; no switches yet, unlike
the revisit-tolerant pricer's `BoundedStops` (elementary routes have no
analogous optional cap to toggle)."""
struct AggregateODRouteStationSimpleDominanceRules <: AbstractPricingDominanceRules end

"""
State-scan fast path for `_dominates_aggregate_od_route_station_simple_label`:
identical dominance test, minus the `current`/`visited_bits` check, which the
state itself already guarantees for every pair this is called on (see the
4-argument method's docstring above, and `_dominates_joint_routing_assignment_at_state`
in `joint_routing_assignment/labels.jl` for the same convention on the other
pricer).
"""
@inline function _pricing_dominates_at_state(
    af::AggregateODRouteStationSimpleDominanceFilters, abs::AggregateODRouteStationSimpleBitsets,
    bf::AggregateODRouteStationSimpleDominanceFilters, bbs::AggregateODRouteStationSimpleBitsets,
    ::AggregateODRouteStationSimpleDominanceRules,
)::Bool
    af.time <= bf.time + 1e-9 || return false
    af.reduced_cost <= bf.reduced_cost + 1e-9 || return false
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end

function _initial_aggregate_od_route_station_simple_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{AggregateODRouteStationSimpleLabel}
    positive_origins = Set{Int}(
        pair[1] for pair in pricing_data.active_pairs if get(duals.sigma, pair, 0.0) > 1e-9
    )
    labels = AggregateODRouteStationSimpleLabel[]
    for node in pricing_data.nodes
        live = Dict{Int, Float64}()
        node in positive_origins && (live[node] = 0.0)
        push!(labels, AggregateODRouteStationSimpleLabel(
            node,
            [node],
            Set{Int}([node]),
            0.0,
            live,
            Set{Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
        ))
    end
    return labels
end

function _aggregate_od_route_station_simple_prune_live_origins(
    live_origin_age::Dict{Int, Float64},
    current::Int,
    visited::Set{Int},
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Dict{Int, Float64}
    remaining = Dict{Int, Float64}()
    for (origin, age) in live_origin_age
        can_still_reward = false
        for pair in pricing_data.active_pairs
            pair[1] == origin || continue
            pair[2] in visited && continue
            get(duals.sigma, pair, 0.0) > 1e-9 || continue
            age + _aggregate_od_route_travel(pricing_data, current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            can_still_reward = true
            break
        end
        can_still_reward && (remaining[origin] = age)
    end
    return remaining
end

function _aggregate_od_route_station_simple_candidate_next_nodes(
    label::AggregateODRouteStationSimpleLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Vector{Int}
    candidates = Int[]
    for next_node in pricing_data.nodes
        next_node in label.visited && continue
        travel_time = _aggregate_od_route_travel(pricing_data, label.current, next_node)

        is_useful = false
        for (origin, age) in label.live_origin_age
            pair = (origin, next_node)
            dual = get(duals.sigma, pair, 0.0)
            dual > 1e-9 || continue
            age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
            is_useful = true
            break
        end

        if !is_useful && label.time + travel_time <= pricing_data.max_wait_time + 1e-9
            for pair in pricing_data.active_pairs
                pair[1] == next_node || continue
                pair[2] in label.visited && continue
                get(duals.sigma, pair, 0.0) > 1e-9 || continue
                is_useful = true
                break
            end
        end

        is_useful && push!(candidates, next_node)
    end
    return candidates
end

function _extend_aggregate_od_route_station_simple_label(
    label::AggregateODRouteStationSimpleLabel,
    next_node::Int,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::AggregateODRouteStationSimpleLabel
    next_node in label.visited && throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _aggregate_od_route_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_route = vcat(label.route, next_node)
    visited = copy(label.visited)
    push!(visited, next_node)

    served_pairs = copy(label.served_pairs)
    reward = 0.0
    for (origin, age) in label.live_origin_age
        pair = (origin, next_node)
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        age + travel_time <= _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        if pair ∉ served_pairs
            push!(served_pairs, pair)
            reward += dual
        end
    end

    live = Dict(origin => age + travel_time for (origin, age) in label.live_origin_age)
    opens_origin = any(
        pair -> pair[1] == next_node && get(duals.sigma, pair, 0.0) > 1e-9,
        pricing_data.active_pairs,
    )
    if opens_origin && arrival_time <= pricing_data.max_wait_time + 1e-9
        live[next_node] = 0.0
    end
    live = _aggregate_od_route_station_simple_prune_live_origins(live, next_node, visited, pricing_data, duals)

    new_tau = label.tau + travel_time
    return AggregateODRouteStationSimpleLabel(
        next_node,
        new_route,
        visited,
        arrival_time,
        live,
        served_pairs,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
    )
end

# Upper bound on collectible reward from this label onward. Already-open origins
# need the detour check against their current age; not-yet-visited origins only
# need to still be reachable within the pickup window (ignoring detour, since
# triangle inequality makes direct travel a lower bound on any routed arrival).
function _aggregate_od_route_station_simple_future_reward_bound(
    label::AggregateODRouteStationSimpleLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)::Float64
    ub = 0.0
    for pair in pricing_data.active_pairs
        dual = get(duals.sigma, pair, 0.0)
        dual > 1e-9 || continue
        pair[2] in label.visited && continue
        if pair[1] in label.visited
            age = get(label.live_origin_age, pair[1], Inf)
            isinf(age) && continue
            age + _aggregate_od_route_travel(pricing_data, label.current, pair[2]) <=
                _direct_ride_limit(pricing_data, pair) + 1e-9 || continue
        else
            label.time + _aggregate_od_route_travel(pricing_data, label.current, pair[1]) <=
                pricing_data.max_wait_time + 1e-9 || continue
        end
        ub += dual
    end
    return ub
end

_aggregate_od_route_station_simple_label_priority(
    label::AggregateODRouteStationSimpleLabel,
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
) = label.reduced_cost - _aggregate_od_route_station_simple_future_reward_bound(label, pricing_data, duals)

_aggregate_od_route_column_from_label(
    label::AggregateODRouteStationSimpleLabel,
    column_id::Int,
    scenario::Int,
)::AggregateODRouteColumn = AggregateODRouteColumn(
    column_id,
    collect(label.served_pairs),
    label.tau;
    metadata=Dict{String, Any}(
        "scenario" => scenario,
        "route" => Tuple(label.route),
        "reduced_cost" => label.reduced_cost,
    ),
)

"""
Context for the elementary-route pricer: bundles `pricing_data`/`duals`, the
once-built `dominates` closure, and `node_index` (the only precomputed index
this pricer's reward bound needs -- unlike the revisit-tolerant twin, it has
no travel matrix or per-pair arrays to precompute, since
`_aggregate_od_route_station_simple_future_reward_bound` iterates
`active_pairs` directly). Plugs into the shared `_run_pricing_label_search`
(`engine.jl`) the same way `AggregateODRouteSearchContext` does (`exact.jl`).
Not currently reachable from `base/pricing_round.jl`'s `_pricing_build_unit_context`
(`AggregateODRouteBaseFormulation` always builds the revisit-tolerant context
today) -- kept as a real, independently usable capability for whoever wants to
wire station-simple pricing back into the hub.
"""
struct AggregateODRouteStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    AggregateODRouteStationSimpleDominanceFilters, AggregateODRouteStationSimpleLabel, AggregateODRouteStationSimpleBitsets,
    Tuple{Int, BitSet}, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::AggregateODRoutePricingData
    duals::AggregateODRoutePricingDuals
    dominates::D
    node_index::Dict{Int, Int}
end

function AggregateODRouteStationSimpleSearchContext(
    pricing_data::AggregateODRoutePricingData, duals::AggregateODRoutePricingDuals,
)
    rules = AggregateODRouteStationSimpleDominanceRules()
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, rules)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    return AggregateODRouteStationSimpleSearchContext(pricing_data, duals, dominates, node_index)
end

_pricing_initial_labels(ctx::AggregateODRouteStationSimpleSearchContext) =
    _initial_aggregate_od_route_station_simple_labels(ctx.pricing_data, ctx.duals)

_pricing_make_bitsets(ctx::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel) =
    _make_aggregate_od_route_station_simple_bitsets(label, ctx.node_index)

_pricing_state(
    ::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel, label_bs::AggregateODRouteStationSimpleBitsets,
) = _aggregate_od_route_station_simple_state(label, label_bs)

_pricing_label_priority(ctx::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel, ::AggregateODRouteStationSimpleBitsets) =
    _aggregate_od_route_station_simple_label_priority(label, ctx.pricing_data, ctx.duals)

_pricing_best_signature(::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel) =
    isempty(label.served_pairs) ? nothing : _aggregate_od_route_column_signature(label.served_pairs)

_pricing_route_length(::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel) = length(label.route)

_pricing_max_route_length(ctx::AggregateODRouteStationSimpleSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel) =
    _aggregate_od_route_station_simple_candidate_next_nodes(label, ctx.pricing_data, ctx.duals)

_pricing_extend_label(ctx::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel, next_node::Int) =
    _extend_aggregate_od_route_station_simple_label(label, next_node, ctx.pricing_data, ctx.duals)

_pricing_dominates_fn(ctx::AggregateODRouteStationSimpleSearchContext) = ctx.dominates

# ── round-level hooks (engine.jl's `_run_pricing_round`, dispatched on ctx) ──

function _pricing_candidate_from_label(::AggregateODRouteStationSimpleSearchContext, label::AggregateODRouteStationSimpleLabel)
    isempty(label.served_pairs) && return nothing
    return (
        signature=_aggregate_od_route_column_signature(label.served_pairs),
        tau=label.tau, reduced_cost=label.reduced_cost, payload=label,
    )
end

_pricing_pool_signature(::AggregateODRouteStationSimpleSearchContext, existing_column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(existing_column)

_pricing_make_column(ctx::AggregateODRouteStationSimpleSearchContext, column_id::Int, candidate) =
    _aggregate_od_route_column_from_label(candidate.payload, column_id, ctx.pricing_data.scenario)

"""Same reduced-cost cross-check as the revisit-tolerant context's twin
(`exact.jl`) -- the master doesn't know which route universe a column came
from, only the pair set it carries."""
function _pricing_verify_column(ctx::AggregateODRouteStationSimpleSearchContext, column::AggregateODRouteColumn, ::JuMP.Model, mapping, duals; atol::Float64=1e-5)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = aggregate_od_route_column_objective_coefficient(
        ctx.pricing_data.route_regularization_weight, ctx.pricing_data.repositioning_time, column,
    ) - sum(get(ctx.duals.sigma, pair, 0.0) for pair in column.od_pairs; init=0.0)
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
