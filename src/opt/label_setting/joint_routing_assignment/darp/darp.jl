"""
`JointRoutingAssignmentDarpSearchContext`: `darp/`'s plug into the shared
search loop (`_run_label_setting`, `engine.jl`) -- the ten inner search hooks
plus the four round-level hooks (`round.jl`'s `_pricing_candidate_from_label`/
`_pricing_pool_signature`/`_pricing_make_column`/`_pricing_verify_column`)
that let `_run_pricing_round` harvest this context's surviving labels into
`JointRoutingAssignmentRouteColumn`s, exactly like `exact/exact.jl`'s
context. Built by `_pricing_build_scenario_context`
(`joint_routing_assignment/pricing_round.jl`) when
`AggregateODRouteJointRoutingAssignmentFormulation.pricing_mode === :darp` --
a controlled comparison point for how much `exact/`'s reward-layer running-max
trick is worth *computationally*, not a different reward model: run to
exhaustion, this pricer's optimum is required to equal `exact/`'s (see
`types.jl`'s module docstring for that invariant and the branching that makes
it hold), selectable per solve alongside `:exact` (the default). Also
exercisable standalone, bypassing the CG hub entirely, via
`joint_routing_assignment_pricing_by_darp_label_setting` below -- the same
driver shape the now-removed pre-hub driver functions used.

Unlike `exact/exact.jl`, this pricer needs no route-replay step to recover
concrete `(p,j,k)` assignments: a finished label's `served` field already is
the exact answer (`types.jl`), so `_pricing_candidate_from_label` below is a
trivial projection, the same shape as `route_covering/exact/exact.jl`'s.
"""

export joint_routing_assignment_pricing_by_darp_label_setting

# ── remaining-reward bound (drives frontier priority + pop-time pruning) ────
"""
Admissible bound on the additional reward still reachable from `label`: the
summed `passenger_weight` upper bound of every not-yet-served passenger who
still has some live-or-refreshable candidate reaching them. Same two-source
structure (live origins / refreshable origins) as
`route_covering/exact/exact.jl`'s and
`joint_routing_assignment/exact/exact.jl`'s twins, generalized from "per pair"
to "per passenger, scanning that passenger's own candidates" via
`candidates_by_passenger` (`data.jl`) -- since a passenger can have several
candidate pairs with different origins/destinations/ride limits, unlike
`route_covering`'s one-weight-per-pair simplicity.
"""
function _joint_routing_assignment_darp_remaining_reward_bound(
    label::JointRoutingAssignmentDarpPricingLabel,
    label_bs::JointRoutingAssignmentDarpLabelBitsets,
    ctx,
)::Float64
    pricing_data = ctx.pricing_data
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    current_idx = ctx.node_index[label.current]
    ub = 0.0
    @inbounds for p in 1:pricing_data.n_passengers
        pricing_data.passenger_weight[p] > 0 || continue    # no reward, no contribution to the bound
        p in label_bs.served_bits && continue                # already certified, already counted in reduced_cost
        reachable = false
        for idx in pricing_data.candidates_by_passenger[p]
            c = pricing_data.candidates[idx]
            origin_idx = ctx.node_index[c.origin]
            pos = searchsortedfirst(label_bs.age_idx, Int32(origin_idx))
            origin_age = pos <= length(label_bs.age_idx) && label_bs.age_idx[pos] == origin_idx ?
                label_bs.age_val[pos] : Inf
            dest_idx = ctx.node_index[c.destination]
            can_claim_current = isfinite(origin_age) &&
                origin_age + ctx.travel_matrix[current_idx, dest_idx] <= c.ride_limit + 1e-9
            can_refresh = !past_pickup_cutoff &&
                label.time + ctx.travel_matrix[current_idx, origin_idx] <= pricing_data.max_wait_time + 1e-9
            if can_claim_current || can_refresh
                reachable = true
                break
            end
        end
        reachable && (ub += pricing_data.passenger_weight[p])
    end
    return label.reduced_cost - ub
end

# ── search context: struct + constructor ────────────────────────────────────
struct JointRoutingAssignmentDarpSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDarpDominanceFilters, JointRoutingAssignmentDarpPricingLabel, JointRoutingAssignmentDarpLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int, Int}}},
}
    pricing_data::JointRoutingAssignmentDarpPricingData
    dominates::D
    node_index::Dict{Int, Int}
    n_nodes::Int
    travel_matrix::Matrix{Float64}
end

function JointRoutingAssignmentDarpSearchContext(pricing_data::JointRoutingAssignmentDarpPricingData)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    n_nodes = length(pricing_data.nodes)
    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) && (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    rules = _joint_routing_assignment_darp_dominance_rules(pricing_data.bounded_max_stops, pricing_data.compensated_dominance)
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, pricing_data.passenger_weight, rules,
    )
    return JointRoutingAssignmentDarpSearchContext(pricing_data, dominates, node_index, n_nodes, travel_matrix)
end

# ── context hooks (AbstractPricingSearchContext contract) ───────────────────
_pricing_initial_labels(ctx::JointRoutingAssignmentDarpSearchContext) =
    initial_joint_routing_assignment_darp_pricing_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) =
    _make_joint_routing_assignment_darp_label_bitsets(label, ctx.node_index, ctx.n_nodes)

_pricing_state(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel, ::JointRoutingAssignmentDarpLabelBitsets) =
    _joint_routing_assignment_darp_state(label)

_pricing_label_priority(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel, label_bs::JointRoutingAssignmentDarpLabelBitsets)::Float64 =
    _joint_routing_assignment_darp_remaining_reward_bound(label, label_bs, ctx)

_pricing_best_signature(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) =
    isempty(label.served) ? nothing : _joint_routing_assignment_darp_column_signature(label.served)

_pricing_route_length(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentDarpSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) =
    _joint_routing_assignment_darp_candidate_next_nodes(label, ctx.pricing_data)

# `action`, not a bare node id: see `labels.jl`'s module docstring for why
# commit/skip branching is expressed as one action per branch rather than as
# multiple children of one action.
_pricing_extend_label(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel, action::JointRoutingAssignmentDarpAction) =
    _extend_joint_routing_assignment_darp_pricing_label(label, action, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentDarpSearchContext) = ctx.dominates

# ── round-level hooks (round.jl's `_run_pricing_round`, dispatched on ctx) ──
_joint_routing_assignment_darp_column_signature(served::Dict{Int, Tuple{Int, Int}}) =
    Tuple(sort!([(p, o, k) for (p, (o, k)) in served]))

function _pricing_candidate_from_label(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel)
    isempty(label.served) && return nothing
    assignments = Tuple{Int, Int, Int}[(p, o, k) for (p, (o, k)) in label.served]
    return (
        signature=_joint_routing_assignment_darp_column_signature(label.served),
        tau=label.tau, reduced_cost=label.reduced_cost,
        payload=(route=label.route, assignments=assignments),
    )
end

_pricing_pool_signature(::JointRoutingAssignmentDarpSearchContext, existing_column::JointRoutingAssignmentRouteColumn) =
    Tuple(sort!(collect(existing_column.assignments)))

_pricing_make_column(ctx::JointRoutingAssignmentDarpSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
        ),
    )

"""
Reused from `exact/exact.jl` as-is: `JointRoutingAssignmentRouteColumn`
carries the same `assignments`/`tau` shape regardless of which pricer
produced it, and the master doesn't care how a column was priced, only what
it contains -- see `_verify_joint_routing_assignment_master_reduced_cost`
(`../duals.jl`)."""
function _pricing_verify_column(::JointRoutingAssignmentDarpSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end

# ── standalone driver (comparison/benchmark entrypoint, not wired into the CG hub) ──
"""
    joint_routing_assignment_pricing_by_darp_label_setting(pricing_data, existing_columns;
        next_column_id=1, max_new_columns=typemax(Int)÷2, n_candidates=typemax(Int)÷2,
        time_limit=30.0, reduced_cost_tol=1e-6, profile=false) -> (columns, exhausted, stats)

Run this pricer's label search to completion against one scenario's existing
column pool and return improving columns -- the accept/dedupe + harvest logic
`round.jl`'s `_run_pricing_round` applies per scenario, standalone here (no
master model/duals cross-check, since there is no live master in a bare
comparison run) so `darp/` can be benchmarked against `exact/` directly, the
same shape the pre-hub driver functions (`aggregate_od_route_pricing_by_label_setting`
and friends) used.
"""
function joint_routing_assignment_pricing_by_darp_label_setting(
    pricing_data::JointRoutingAssignmentDarpPricingData,
    existing_columns::AbstractVector{JointRoutingAssignmentRouteColumn};
    next_column_id::Int=1,
    max_new_columns::Int=typemax(Int) ÷ 2,
    n_candidates::Int=typemax(Int) ÷ 2,
    time_limit::Float64=30.0,
    reduced_cost_tol::Float64=1e-6,
    profile::Bool=false,
)
    ctx = JointRoutingAssignmentDarpSearchContext(pricing_data)

    best_pool_tau = Dict{Any, Float64}()
    for column in existing_columns
        signature = _pricing_pool_signature(ctx, column)
        best_pool_tau[signature] = min(get(best_pool_tau, signature, Inf), column.tau)
    end

    scored = Dict{Any, Any}()
    function accept!(label)
        candidate = _pricing_candidate_from_label(ctx, label)
        isnothing(candidate) && return false
        candidate.reduced_cost < -reduced_cost_tol || return false
        candidate.tau < get(best_pool_tau, candidate.signature, Inf) - 1e-9 || return false
        current = get(scored, candidate.signature, nothing)
        if isnothing(current) ||
                candidate.reduced_cost < current.reduced_cost - 1e-9 ||
                (abs(candidate.reduced_cost - current.reduced_cost) <= 1e-9 && candidate.tau < current.tau - 1e-9)
            scored[candidate.signature] = candidate
        end
        return length(scored) >= n_candidates
    end

    _labels, exhausted, stats = _run_label_setting(
        ctx; time_limit=time_limit, reduced_cost_tol=reduced_cost_tol, profile=profile, stop_if=accept!,
    )

    sorted = sort!(collect(values(scored)); by=c -> (c.reduced_cost, c.tau))
    truncated = sorted[1:min(length(sorted), max_new_columns)]
    columns = JointRoutingAssignmentRouteColumn[
        _pricing_make_column(ctx, next_column_id + offset - 1, candidate)
        for (offset, candidate) in enumerate(truncated)
    ]
    return columns, exhausted, stats
end
