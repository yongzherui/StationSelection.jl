"""
`RouteCoveringSearchContext`: the revisit-tolerant pricer's plug into the
shared search loop (`_run_label_setting`, `engine.jl`) -- the ten
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
Plugs into the shared `_run_label_setting` (`engine.jl`); built once per
scenario by `pricing_round.jl`'s `_pricing_build_scenario_context` and handed
to `round.jl`'s `_run_pricing_round`.
"""
struct RouteCoveringSearchContext{D<:Function} <: AbstractPricingSearchContext{
    RouteCoveringDominanceFilters, RouteCoveringPricingLabel, RouteCoveringLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::RouteCoveringPricingData
    duals::RouteCoveringPricingDuals
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

function RouteCoveringSearchContext(
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)
    # Everything below is precomputed once per scenario so that
    # `_pricing_label_priority` (the remaining-reward bound, called once per
    # popped label -- the hottest hook after dominance) can look things up by
    # dense array index instead of hashing a `(Int,Int)` pair or station id
    # every time.
    n_pairs = length(pricing_data.active_pairs)
    pair_index = Dict(pair => i for (i, pair) in enumerate(pricing_data.active_pairs))    # pair -> dense index (matches served_bits)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))           # station id -> dense index
    n_nodes = length(pricing_data.nodes)
    pair_origin_idx = [node_index[pair[1]] for pair in pricing_data.active_pairs]         # pair i's origin, as a node index
    pair_dest_idx = [node_index[pair[2]] for pair in pricing_data.active_pairs]           # pair i's destination, as a node index
    pair_ride_limit = [_direct_ride_limit(pricing_data, pair) for pair in pricing_data.active_pairs]
    # Dense node x node travel matrix (Inf where no arc exists) -- a matrix
    # lookup is faster than a Dict hash in the bound's hot inner loop below.
    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) &&
            (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    # Only positive dual rewards can ever improve reduced cost, so this is
    # both the reward-bound's per-pair weight and (via `max(0.0, ...)`) an
    # implicit filter: a pair with a non-positive dual contributes 0 to any
    # bound and is effectively invisible to the search.
    positive_pair_rewards = Float64[
        max(0.0, get(duals.sigma, pair, 0.0)) for pair in pricing_data.active_pairs
    ]
    rules = _route_covering_dominance_rules(pricing_data.bounded_max_stops, pricing_data.compensated_dominance)
    # Built once, closing over `positive_pair_rewards`/`rules` for this
    # scenario, and handed to `_add_pricing_label_to_state!` unchanged for
    # every dominance test in this search (see that function's docstring in
    # `label_setting/types.jl` for why the closure is supplied by the caller).
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, positive_pair_rewards, rules)
    return RouteCoveringSearchContext(
        pricing_data, duals, dominates,
        n_pairs, pair_index, node_index, n_nodes,
        pair_origin_idx, pair_dest_idx, pair_ride_limit, travel_matrix, positive_pair_rewards,
    )
end

_pricing_initial_labels(ctx::RouteCoveringSearchContext) =
    initial_route_covering_pricing_labels(ctx.pricing_data, ctx.duals)

_pricing_make_bitsets(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) =
    _make_route_covering_label_bitsets(label, ctx.pair_index, ctx.n_pairs, ctx.node_index, ctx.n_nodes)

_pricing_state(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel, ::RouteCoveringLabelBitsets) =
    _route_covering_state(label)

# Priority = a lower bound on the reduced cost of *any* descendant of
# `label`: `label.reduced_cost` minus an upper bound `ub` on every dual
# reward this route could still possibly collect. The search treats this as
# admissible (never overestimates remaining reward), so once a popped
# priority is no longer beating `reduced_cost_tol`, nothing reachable from it
# can be either -- that's what licenses `engine.jl`'s reduced-cost pruning.
#
# `ub` sums, over every not-yet-served pair with positive dual reward, that
# reward if the pair is *still reachable one way or another* -- ignoring
# whether the route could realistically detour to reach several such pairs
# at once. That slack (every reachable pair counted as if independently
# certifiable) is what makes this a bound rather than an exact remaining
# value, and is the whole point: cheap to compute, never wrong-signed.
function _pricing_label_priority(
    ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel, label_bs::RouteCoveringLabelBitsets,
)::Float64
    past_pickup_cutoff = label.time > ctx.pricing_data.max_wait_time + 1e-9
    current_idx = ctx.node_index[label.current]
    ub = 0.0
    @inbounds for i in 1:ctx.n_pairs
        ctx.positive_pair_rewards[i] > 0 || continue   # no reward, no contribution to the bound
        i in label_bs.served_bits && continue          # already certified, already counted in reduced_cost
        # `label_bs.age_idx` is sorted, so this is a binary search for pair
        # i's origin among the label's currently-live pickup clocks; `Inf` if
        # that origin was never visited (or its clock has since been pruned).
        pos = searchsortedfirst(label_bs.age_idx, Int32(ctx.pair_origin_idx[i]))
        origin_age = pos <= length(label_bs.age_idx) && label_bs.age_idx[pos] == ctx.pair_origin_idx[i] ?
            label_bs.age_val[pos] : Inf
        # Two independent ways pair i could still be certified: (1) a pickup
        # clock is already live and a direct trip from here to the
        # destination still beats the ride limit, or (2) the pickup cutoff
        # hasn't passed yet, so the route could still detour to open a fresh
        # clock at the origin. Either is enough to count the reward.
        can_claim_current = isfinite(origin_age) &&
            origin_age + ctx.travel_matrix[current_idx, ctx.pair_dest_idx[i]] <= ctx.pair_ride_limit[i] + 1e-9
        can_refresh = !past_pickup_cutoff &&
            label.time + ctx.travel_matrix[current_idx, ctx.pair_origin_idx[i]] <= ctx.pricing_data.max_wait_time + 1e-9
        can_claim_current || can_refresh || continue
        ub += ctx.positive_pair_rewards[i]
    end
    return label.reduced_cost - ub
end

_pricing_best_signature(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) =
    isempty(label.served_pairs) ? nothing : _aggregate_od_route_column_signature(label.served_pairs)

_pricing_route_length(::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) = label.route_length

_pricing_max_route_length(ctx::RouteCoveringSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel) =
    _route_covering_candidate_next_nodes(label, ctx.pricing_data, ctx.duals)

_pricing_extend_label(ctx::RouteCoveringSearchContext, label::RouteCoveringPricingLabel, next_node::Int) =
    _extend_route_covering_pricing_label(label, next_node, ctx.pricing_data, ctx.duals)

_pricing_dominates_fn(ctx::RouteCoveringSearchContext) = ctx.dominates

# ── round-level hooks (engine.jl's `_run_pricing_round`, dispatched on ctx) ──

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
family sum, `joint_routing_assignment/exact/exact.jl`): a Base column only ever
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
