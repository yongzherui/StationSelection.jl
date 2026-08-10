"""
Orchestrates the `labels.jl` primitives into a full label-setting pricing
pass: a priority-queue search over live labels (`_enumerate_aggregate_od_route_pricing_labels`),
and the per-request driver that turns surviving labels into candidate columns
(`aggregate_od_route_pricing_by_label_setting`).
"""

export aggregate_od_route_pricing_by_label_setting

"""
Context for the revisit-tolerant `AggregateODRouteCG` search: bundles
`pricing_data`/`duals`, the once-built `dominates` closure, and the
precomputed indices `_pricing_label_priority`'s remaining-reward bound needs
(`pair_index`, `node_index`, the pair-origin/destination index arrays, the
per-pair ride limit, the dense travel matrix, and the positive dual rewards).
Plugs into the shared `_run_pricing_label_search` (`pricing/types.jl`); see
`_enumerate_aggregate_od_route_pricing_labels` below for how it is built.
"""
struct AggregateODRouteSearchContext{D<:Function} <: AbstractPricingSearchContext{
    AggregateODRouteDominanceFilters, AggregateODRoutePricingLabel, AggregateODRouteLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::AggregateODRoutePricingData
    duals::AggregateODRoutePricingDuals
    max_visits_per_node::Int
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
    max_visits_per_node::Int,
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
        pricing_data, duals, max_visits_per_node, dominates,
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
    _aggregate_od_route_candidate_next_nodes(
        label, ctx.pricing_data, ctx.duals; max_visits_per_node=ctx.max_visits_per_node,
    )

_pricing_extend_label(ctx::AggregateODRouteSearchContext, label::AggregateODRoutePricingLabel, next_node::Int) =
    _extend_aggregate_od_route_pricing_label(label, next_node, ctx.pricing_data, ctx.duals)

_pricing_dominates_fn(ctx::AggregateODRouteSearchContext) = ctx.dominates

function _enumerate_aggregate_od_route_pricing_labels(
    pricing_data::AggregateODRoutePricingData,
    duals::AggregateODRoutePricingDuals;
    time_limit::Float64,
    reduced_cost_tol::Float64,
    max_visits_per_node::Int,
    use_reduced_cost_pruning::Bool=true,
    profile::Bool=false,
    stop_if=label -> false,
)
    ctx = AggregateODRouteSearchContext(pricing_data, duals, max_visits_per_node)
    return _run_pricing_label_search(
        ctx;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        use_reduced_cost_pruning=use_reduced_cost_pruning,
        profile=profile,
        stop_if=stop_if,
    )
end

function aggregate_od_route_pricing_by_label_setting(
    pricing_data::AggregateODRoutePricingData,
    existing_columns::Vector{AggregateODRouteColumn},
    duals::AggregateODRoutePricingDuals;
    next_column_id::Int,
    reduced_cost_tol::Float64=1e-6,
    max_new_columns::Int=1,
    n_candidates::Int=max_new_columns,
    time_limit::Float64=30.0,
    max_visits_per_node::Int=pricing_data.max_visits_per_node,
    profile::Bool=false,
)
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    time_limit > 0 || throw(ArgumentError("time_limit must be positive"))

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _aggregate_od_route_column_signature(column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored_by_signature = Dict{Any, Tuple{Float64, AggregateODRoutePricingLabel}}()

    function accept_pricing_label!(label::AggregateODRoutePricingLabel)
        isempty(label.served_pairs) && return false
        label.reduced_cost < -reduced_cost_tol || return false
        signature = _aggregate_od_route_column_signature(label.served_pairs)
        # This assumes `duals` came from the optimal RMP over `existing_columns`.
        # Under that condition, an existing column cannot have negative reduced
        # cost, so subset-served dominance cannot hide a missing improving column
        # behind a non-improving duplicate signature.
        label.tau < get(best_pool_tau, signature, Inf) - 1e-9 || return false
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                label.reduced_cost < current[1] - 1e-9 ||
                (abs(label.reduced_cost - current[1]) <= 1e-9 && label.tau < current[2].tau - 1e-9)
            scored_by_signature[signature] = (label.reduced_cost, label)
        end
        return length(scored_by_signature) >= n_candidates
    end

    labels, exhausted, stats = _enumerate_aggregate_od_route_pricing_labels(
        pricing_data,
        duals;
        time_limit=time_limit,
        reduced_cost_tol=reduced_cost_tol,
        max_visits_per_node=max_visits_per_node,
        profile=profile,
        stop_if=accept_pricing_label!,
    )

    for label in labels
        isempty(label.served_pairs) && continue
        label.reduced_cost < -reduced_cost_tol || continue
        signature = _aggregate_od_route_column_signature(label.served_pairs)
        label.tau < get(best_pool_tau, signature, Inf) - 1e-9 || continue
        current = get(scored_by_signature, signature, nothing)
        if isnothing(current) ||
                label.reduced_cost < current[1] - 1e-9 ||
                (abs(label.reduced_cost - current[1]) <= 1e-9 && label.tau < current[2].tau - 1e-9)
            scored_by_signature[signature] = (label.reduced_cost, label)
        end
    end

    scored = collect(values(scored_by_signature))
    scored = _sort_pricing_results_by_route(scored,
        entry -> (entry[1], entry[2].tau, string(entry[2].route)))
    scored = scored[1:min(length(scored), n_candidates)]
    scored = scored[1:min(length(scored), max_new_columns)]

    columns = AggregateODRouteColumn[]
    column_id = next_column_id
    for (_, label) in scored
        push!(columns, _aggregate_od_route_column_from_label(label, column_id, pricing_data.scenario))
        column_id += 1
    end
    return columns, exhausted, stats
end
