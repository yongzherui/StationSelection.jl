"""
`AggregateODRouteSearchContext`: the revisit-tolerant pricer's plug into the
shared search loop (`_run_pricing_label_search`, `engine.jl`) -- the ten
inner search hooks (dominance, extension, pruning, ...) plus the three
round-level hooks (`round.jl`'s `_pricing_candidate_from_label`/
`_pricing_pool_signature`/`_pricing_make_column`) that let
`_run_pricing_round` harvest this context's surviving labels into
`AggregateODRouteColumn`s.
"""

"""
Context for the revisit-tolerant `AggregateODRouteCG` search: bundles
`pricing_data`/`duals`, the once-built `dominates` closure, and the
precomputed indices `_pricing_label_priority`'s remaining-reward bound needs
(`pair_index`, `node_index`, the pair-origin/destination index arrays, the
per-pair ride limit, the dense travel matrix, and the positive dual rewards).
Plugs into the shared `_run_pricing_label_search` (`engine.jl`); built once per
scenario by `base/pricing_round.jl`'s `_pricing_build_unit_context` and handed
to `round.jl`'s `_run_pricing_round`.
"""
struct AggregateODRouteSearchContext{D<:Function} <: AbstractPricingSearchContext{
    AggregateODRouteDominanceFilters, AggregateODRoutePricingLabel, AggregateODRouteLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::AggregateODRoutePricingData
    duals::AggregateODRoutePricingDuals
    dominates::D
    n_pairs::Int
    pair_index::Dict{Tuple{Int, Int}, Int}
    node_index::Dict{Int, Int}
    n_nodes::Int
    pair_origin_idx::Vector{Int}
    pair_dest_idx::Vector{Int}
    pair_ride_limit::Vector{Float64}
    travel_matrix::Matrix{Float64}
    positive_pair_rewards::Vector{Float64}
end

function AggregateODRouteSearchContext(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals,
)
    n_pairs = length(pricing_data.active_pairs)
    pair_index = Dict(pair => i for (i, pair) in enumerate(pricing_data.active_pairs))
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    n_nodes = length(pricing_data.nodes)
    pair_origin_idx = [node_index[pair[1]] for pair in pricing_data.active_pairs]
    pair_dest_idx = [node_index[pair[2]] for pair in pricing_data.active_pairs]
    pair_ride_limit = [_direct_ride_limit(pricing_data, pair) for pair in pricing_data.active_pairs]
    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) &&
            (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    rules = AggregateODRouteDominanceRules{pricing_data.bounded_max_stops}()
    dominates(x::PricingBucketEntry, y::PricingBucketEntry) =
        _pricing_dominates_in_bucket(x.filters, x.bitsets, y.filters, y.bitsets, rules)
    positive_pair_rewards = Float64[
        max(0.0, get(duals.sigma, pair, 0.0)) for pair in pricing_data.active_pairs
    ]
    return AggregateODRouteSearchContext(
        pricing_data, duals, dominates,
        n_pairs, pair_index, node_index, n_nodes,
        pair_origin_idx, pair_dest_idx, pair_ride_limit, travel_matrix, positive_pair_rewards,
    )
end

_pricing_initial_labels(ctx::AggregateODRouteSearchContext) =
    initial_aggregate_od_route_pricing_labels(ctx.pricing_data, ctx.duals)

_pricing_make_bitsets(ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel) =
    _make_aggregate_od_route_label_bitsets(label, ctx.pair_index, ctx.n_pairs, ctx.node_index, ctx.n_nodes)

_pricing_bucket_signature(ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel, ::AggregateODRouteLabelBitsets) =
    _aggregate_od_route_dominance_signature(label)

function _pricing_label_priority(
    ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel, label_bs::AggregateODRouteLabelBitsets,
)::Float64
    past_pickup_cutoff = label.time > ctx.pricing_data.max_wait_time + 1e-9
    current_idx = ctx.node_index[label.current]
    ub = 0.0
    @inbounds for i in 1:ctx.n_pairs
        ctx.positive_pair_rewards[i] > 0 || continue
        i in label_bs.served_bits && continue
        pos = searchsortedfirst(label_bs.age_idx, Int32(ctx.pair_origin_idx[i]))
        origin_age = pos <= length(label_bs.age_idx) && label_bs.age_idx[pos] == ctx.pair_origin_idx[i] ?
            label_bs.age_val[pos] : Inf
        can_claim_current = isfinite(origin_age) &&
            origin_age + ctx.travel_matrix[current_idx, ctx.pair_dest_idx[i]] <= ctx.pair_ride_limit[i] + 1e-9
        can_refresh = !past_pickup_cutoff &&
            label.time + ctx.travel_matrix[current_idx, ctx.pair_origin_idx[i]] <= ctx.pricing_data.max_wait_time + 1e-9
        can_claim_current || can_refresh || continue
        ub += ctx.positive_pair_rewards[i]
    end
    return label.reduced_cost - ub
end

_pricing_best_signature(ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel) =
    isempty(label.served_pairs) ? nothing : _aggregate_od_route_column_signature(label.served_pairs)

_pricing_route_length(::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel) = label.route_length

_pricing_max_route_length(ctx::AggregateODRouteSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel) =
    _aggregate_od_route_candidate_next_nodes(label, ctx.pricing_data, ctx.duals)

_pricing_extend_label(ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel, next_node::Int) =
    _extend_aggregate_od_route_pricing_label(label, next_node, ctx.pricing_data, ctx.duals)

_pricing_dominates_fn(ctx::AggregateODRouteSearchContext) = ctx.dominates

# ── round-level hooks (engine.jl's `_run_pricing_round`, dispatched on ctx) ──

function _pricing_candidate_from_label(::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel)
    isempty(label.served_pairs) && return nothing
    return (
        signature=_aggregate_od_route_column_signature(label.served_pairs),
        tau=label.tau, reduced_cost=label.reduced_cost, payload=label,
    )
end

_pricing_pool_signature(::AggregateODRouteSearchContext, existing_column::AggregateODRouteColumn) =
    _aggregate_od_route_column_signature(existing_column)

_pricing_make_column(ctx::AggregateODRouteSearchContext, column_id::Int, candidate) =
    _aggregate_od_route_column_from_label(candidate.payload, column_id, ctx.pricing_data.scenario)

"""
Cross-check that the pricer's reported reduced cost equals the one implied by
the master's own duals. A single-term formula (unlike Joint's three-constraint-
family sum, `joint_routing_assignment/exact.jl`): a Base column only ever
touches `route_link`. `m`/`mapping`/`duals` are unused -- everything needed
already lives on `ctx.pricing_data`/`ctx.duals` -- but the hook signature is
shared with every other pricer's `_pricing_verify_column`.
"""
function _pricing_verify_column(ctx::AggregateODRouteSearchContext, column::AggregateODRouteColumn, ::JuMP.Model, mapping, duals; atol::Float64=1e-5)
    pricer_rc = Float64(get(column.metadata, "reduced_cost", NaN))
    isnan(pricer_rc) && return true, pricer_rc, NaN
    master_rc = aggregate_od_route_column_objective_coefficient(
        ctx.pricing_data.route_regularization_weight, ctx.pricing_data.repositioning_time, column,
    ) - sum(get(ctx.duals.sigma, pair, 0.0) for pair in column.od_pairs; init=0.0)
    return isapprox(pricer_rc, master_rc; atol=atol), pricer_rc, master_rc
end
