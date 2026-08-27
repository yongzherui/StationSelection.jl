"""
All wiring, no logic: every method that plugs `RouteCoveringSearchContext`
(`context.jl`) into the two generic hook contracts this pricer implements.
Two contracts, both forwarded from here:

  - the twelve `AbstractPricingSearchContext` hooks (`../../types.jl`) that
    `_run_label_setting` (`engine.jl`) calls during the search itself --
    forwarded to `seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl`;
  - the four context-level hooks `round.jl` calls once per surviving label to
    harvest it into a column.

Unlike Joint's twin, this pricer's round-level hooks have no separate
route-replay step to forward to (`_pricing_candidate_from_label` is a
trivial projection off `label.served_pairs` -- see `types.jl`'s module
docstring for why no capacity/replay reconstruction is needed here), so the
column-shape helpers (`_aggregate_od_route_column_signature`,
`_aggregate_od_route_column_from_label`) live inline in this file rather
than in a separate `accept.jl`.
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
_pricing_initial_labels(ctx::RouteCoveringSearchContext) =
    initial_route_covering_pricing_labels(ctx.pricing_data, ctx.duals)

_pricing_make_bitsets(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) =
    _make_route_covering_label_bitsets(label, ctx.pair_index, ctx.n_pairs, ctx.node_index, ctx.n_nodes)

_pricing_state(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel, ::RouteCoveringLabelBitsets) =
    _route_covering_state(label)

_pricing_label_priority(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel, label_bs::RouteCoveringLabelBitsets)::Float64 =
    label.reduced_cost - _route_covering_remaining_reward_bound(label, label_bs, ctx)

_pricing_best_signature(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) =
    isempty(label.served_pairs) ? nothing : _aggregate_od_route_column_signature(label.served_pairs)

_pricing_route_length(::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) = label.route_length

_pricing_max_route_length(ctx::RouteCoveringSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) =
    _route_covering_candidate_next_nodes(label, ctx.pricing_data, ctx.duals)

_pricing_extend_label(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel, next_node::Int) =
    _extend_route_covering_pricing_label(label, next_node, ctx.pricing_data, ctx.duals)

_pricing_dominates_fn(ctx::RouteCoveringSearchContext) = ctx.dominates

# ── round.jl context-level hooks (candidate → column → master verification) ──
function _aggregate_od_route_column_signature(pairs)::Tuple{Vararg{Tuple{Int, Int}}}
    return Tuple(sort!(collect(pairs)))
end

_aggregate_od_route_column_signature(column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(column.od_pairs)

function _aggregate_od_route_column_from_label(
    label::RouteCoveringPricingLabel,
    column_id::Int,
    scenario::Int,
)::AggregateODRouteColumn
    return AggregateODRouteColumn(
        column_id,
        collect(label.served_pairs),
        label.tau;
        metadata=Dict{String, Any}(
            "scenario" => scenario,
            "route" => Tuple(label.route),
            "reduced_cost" => label.reduced_cost,
        ),
    )
end

function _pricing_candidate_from_label(::RouteCoveringSearchContext, label::RouteCoveringPricingLabel)
    isempty(label.served_pairs) && return nothing
    return (
        signature=_aggregate_od_route_column_signature(label.served_pairs),
        tau=label.tau, reduced_cost=label.reduced_cost, payload=label,
    )
end

_pricing_pool_signature(::RouteCoveringSearchContext, existing_column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(existing_column)

_pricing_make_column(ctx::RouteCoveringSearchContext, column_id::Int, candidate) =
    _aggregate_od_route_column_from_label(candidate.payload, column_id, ctx.pricing_data.scenario)

"""
Cross-check that the pricer's reported reduced cost equals the one implied by
the master's own duals. A single-term formula (unlike Joint's three-constraint-
family sum, `joint_routing_assignment/exact/hooks.jl`): a Base column only ever
touches `route_link`. `m`/`mapping`/`duals` are unused -- everything needed
already lives on `ctx.pricing_data`/`ctx.duals` -- but the hook signature is
shared with every other pricer's `_pricing_verify_column`.
"""
function _pricing_verify_column(ctx::RouteCoveringSearchContext, column::AggregateODRouteColumn, ::JuMP.Model, mapping, duals; atol::Float64=1e-5)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = aggregate_od_route_column_objective_coefficient(
        ctx.pricing_data.route_regularization_weight, ctx.pricing_data.repositioning_time, column,
    ) - sum(get(ctx.duals.sigma, pair, 0.0) for pair in column.od_pairs; init=0.0)
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
