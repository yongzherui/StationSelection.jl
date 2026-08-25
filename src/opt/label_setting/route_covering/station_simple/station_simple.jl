"""
`RouteCoveringStationSimpleSearchContext`: the elementary-route pricer's plug
into the shared search loop (`_run_label_setting`, `engine.jl`), mirroring
`RouteCoveringSearchContext`'s hook set (`../exact/exact.jl`) one for one. See
`labels.jl` for the label-DP primitives (candidate generation, extension,
dominance) and `types.jl` for the underlying label/bitsets/dominance types.
"""

"""
Context for the elementary-route pricer: bundles `pricing_data`/`duals`, the
once-built `dominates` closure, and `node_index` (the only precomputed index
this pricer's reward bound needs -- unlike the revisit-tolerant twin, it has
no travel matrix or per-pair arrays to precompute, since
`_route_covering_station_simple_future_reward_bound` (`labels.jl`) iterates
`active_pairs` directly). Plugs into the shared `_run_label_setting`
(`engine.jl`) the same way `RouteCoveringSearchContext` does (`../exact/exact.jl`).
Not currently reachable from `../exact/pricing_round.jl`'s `_pricing_build_scenario_context`
(`AggregateODRouteBaseFormulation` always builds the revisit-tolerant context
today) -- kept as a real, independently usable capability for whoever wants to
wire station-simple pricing back into the hub.
"""
struct RouteCoveringStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    RouteCoveringStationSimpleDominanceFilters, RouteCoveringStationSimpleLabel, RouteCoveringStationSimpleBitsets,
    Tuple{Int, BitSet}, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::RouteCoveringPricingData
    duals::RouteCoveringPricingDuals
    dominates::D
    node_index::Dict{Int, Int}
end

function RouteCoveringStationSimpleSearchContext(
    pricing_data::RouteCoveringPricingData, duals::RouteCoveringPricingDuals,
)
    rules = RouteCoveringStationSimpleDominanceRules()
    # Built once per search and handed to `_add_pricing_label_to_state!`
    # unchanged for every dominance test, same convention as the
    # revisit-tolerant pricer's own `dominates` closure (`../exact/exact.jl`).
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, rules)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    return RouteCoveringStationSimpleSearchContext(pricing_data, duals, dominates, node_index)
end

_pricing_initial_labels(ctx::RouteCoveringStationSimpleSearchContext) =
    _initial_route_covering_station_simple_labels(ctx.pricing_data, ctx.duals)

_pricing_make_bitsets(ctx::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel) =
    _make_route_covering_station_simple_bitsets(label, ctx.node_index)

_pricing_state(
    ::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel, label_bs::RouteCoveringStationSimpleBitsets,
) = _route_covering_station_simple_state(label, label_bs)

_pricing_label_priority(ctx::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel, ::RouteCoveringStationSimpleBitsets) =
    _route_covering_station_simple_label_priority(label, ctx.pricing_data, ctx.duals)

_pricing_best_signature(::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel) =
    isempty(label.served_pairs) ? nothing : _aggregate_od_route_column_signature(label.served_pairs)

_pricing_route_length(::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel) = length(label.route)

_pricing_max_route_length(ctx::RouteCoveringStationSimpleSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel) =
    _route_covering_station_simple_candidate_next_nodes(label, ctx.pricing_data, ctx.duals)

_pricing_extend_label(ctx::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel, next_node::Int) =
    _extend_route_covering_station_simple_label(label, next_node, ctx.pricing_data, ctx.duals)

_pricing_dominates_fn(ctx::RouteCoveringStationSimpleSearchContext) = ctx.dominates

# ── round-level hooks (engine.jl's `_run_pricing_round`, dispatched on ctx) ──

function _pricing_candidate_from_label(::RouteCoveringStationSimpleSearchContext, label::RouteCoveringStationSimpleLabel)
    isempty(label.served_pairs) && return nothing
    return (
        signature=_aggregate_od_route_column_signature(label.served_pairs),
        tau=label.tau, reduced_cost=label.reduced_cost, payload=label,
    )
end

_pricing_pool_signature(::RouteCoveringStationSimpleSearchContext, existing_column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(existing_column)

_pricing_make_column(ctx::RouteCoveringStationSimpleSearchContext, column_id::Int, candidate) =
    _aggregate_od_route_column_from_label(candidate.payload, column_id, ctx.pricing_data.scenario)

"""Same reduced-cost cross-check as the revisit-tolerant context's twin
(`../exact/exact.jl`) -- the master doesn't know which route universe a column came
from, only the pair set it carries."""
function _pricing_verify_column(ctx::RouteCoveringStationSimpleSearchContext, column::AggregateODRouteColumn, ::JuMP.Model, mapping, duals; atol::Float64=1e-5)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = aggregate_od_route_column_objective_coefficient(
        ctx.pricing_data.route_regularization_weight, ctx.pricing_data.repositioning_time, column,
    ) - sum(get(ctx.duals.sigma, pair, 0.0) for pair in column.od_pairs; init=0.0)
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
