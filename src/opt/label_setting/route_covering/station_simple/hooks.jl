"""
All wiring, no logic: every method that plugs
`RouteCoveringStationSimpleSearchContext` (`context.jl`) into the two generic
hook contracts this pricer implements, mirroring `../exact/hooks.jl`'s hook
set one for one. Two contracts, both forwarded from here:

  - the twelve `AbstractPricingSearchContext` hooks (`../../types.jl`) that
    `_run_label_setting` (`engine.jl`) calls during the search itself --
    forwarded to `seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl`;
  - the four context-level hooks `round.jl` calls once per surviving label to
    harvest it into a column.

Like the revisit-tolerant twin, this pricer's round-level hooks have no
route-replay step (`_pricing_candidate_from_label` is a trivial projection off
`label.served_pairs`), so `_aggregate_od_route_column_from_label` (this
label type's own method) lives inline here rather than in a separate
`accept.jl`; `_aggregate_od_route_column_signature` itself is reused directly
from `../exact/hooks.jl` (generic over any `pairs`/`AggregateODRouteColumn`,
not label-type-specific).
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
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

# ── round.jl context-level hooks (candidate → column → master verification) ──
_aggregate_od_route_column_from_label(
    label::RouteCoveringStationSimpleLabel,
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
(`../exact/hooks.jl`) -- the master doesn't know which route universe a column came
from, only the pair set it carries."""
function _pricing_verify_column(ctx::RouteCoveringStationSimpleSearchContext, column::AggregateODRouteColumn, ::JuMP.Model, mapping, duals; atol::Float64=1e-5)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = aggregate_od_route_column_objective_coefficient(
        ctx.pricing_data.route_regularization_weight, ctx.pricing_data.repositioning_time, column,
    ) - sum(get(ctx.duals.sigma, pair, 0.0) for pair in column.od_pairs; init=0.0)
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
