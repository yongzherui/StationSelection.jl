"""
`JointRoutingAssignmentDarpSearchContext`: `darp/`'s plug into the shared
search loop (`_run_label_setting`, `engine.jl`) -- the ten inner search hooks
plus the four round-level hooks (`round.jl`'s `_pricing_candidate_from_label`/
`_pricing_pool_signature`/`_pricing_make_column`/`_pricing_verify_column`)
that let `_run_pricing_round` harvest this context's surviving labels into
`JointRoutingAssignmentRouteColumn`s. Built by `_pricing_build_scenario_context`
(`joint_routing_assignment/pricing_round.jl`) when
`AggregateODRouteJointRoutingAssignmentFormulation.pricing_mode === :darp` --
the literal onboard-bitset DARP-style comparison point, selectable per solve
alongside `:exact` and `:darp_modified`. Also exercisable standalone via
`joint_routing_assignment_pricing_by_darp_label_setting` below, bypassing the
CG hub entirely.

Like `darp_modified/`, needs no route-replay step: a label's `served` field is
already a valid delivered assignment set. If the label still has onboard
commitments, `_pricing_candidate_from_label` projects them away by refunding
their pickup-time rewards; the physical route remains valid, and the shared
round-level `accept!` hook decides whether that projected column is improving.
"""

export joint_routing_assignment_pricing_by_darp_label_setting

# ── remaining-reward bound (drives frontier priority + pop-time pruning) ────
"""
Admissible bound on the additional reward still reachable from `label`.
Onboard commitments are excluded because their reward was credited at pickup;
while still within the pickup window, the bound adds the `passenger_weight`
upper bound of every not-yet-resolved passenger. Looser than
`darp_modified/`'s twin (no
reachability check for not-yet-resolved passengers beyond the pickup-window
gate) -- acceptable here since this pricer's whole point is measuring the
cost of a less-clever representation, not chasing bound tightness.
"""
function _joint_routing_assignment_darp_remaining_reward_bound(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Float64
    ub = 0.0
    if label.time <= pricing_data.max_wait_time + 1e-9
        resolved = _joint_routing_assignment_darp_resolved_passengers(label)
        for p in 1:pricing_data.n_passengers
            p in resolved && continue
            ub += pricing_data.passenger_weight[p]
        end
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
end

function JointRoutingAssignmentDarpSearchContext(pricing_data::JointRoutingAssignmentDarpPricingData)
    rules = _joint_routing_assignment_darp_dominance_rules(pricing_data.bounded_max_stops)
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, rules)
    return JointRoutingAssignmentDarpSearchContext(pricing_data, dominates)
end

# ── context hooks (AbstractPricingSearchContext contract) ───────────────────
_pricing_initial_labels(ctx::JointRoutingAssignmentDarpSearchContext) =
    initial_joint_routing_assignment_darp_pricing_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) =
    _make_joint_routing_assignment_darp_label_bitsets(label, ctx.pricing_data)

_pricing_state(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel, ::JointRoutingAssignmentDarpLabelBitsets) =
    _joint_routing_assignment_darp_state(label)

_pricing_label_priority(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel, ::JointRoutingAssignmentDarpLabelBitsets)::Float64 =
    _joint_routing_assignment_darp_remaining_reward_bound(label, ctx.pricing_data)

_pricing_best_signature(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) =
    isempty(label.served) ? nothing : _joint_routing_assignment_darp_column_signature(label.served)

_pricing_route_length(::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentDarpSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel) =
    _joint_routing_assignment_darp_candidate_next_nodes(label, ctx.pricing_data)

_pricing_extend_label(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel, action::JointRoutingAssignmentDarpAction) =
    _extend_joint_routing_assignment_darp_pricing_label(label, action, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentDarpSearchContext) = ctx.dominates

# ── round-level hooks (round.jl's `_run_pricing_round`, dispatched on ctx) ──
_joint_routing_assignment_darp_column_signature(served::Set{Tuple{Int, Int, Int}}) =
    Tuple(sort!(collect(served)))

"""Project any label with delivered passengers into a valid column.

Pickup reward enters `label.reduced_cost` immediately. For an incomplete
label, the projected column simply declines every still-onboard assignment,
so their rewards must be refunded. The route itself needs no change: visiting
a passenger's candidate origin never forces the master column to assign that
passenger. The existing pricing-round `accept!` closure remains the sole gate
that admits only sufficiently negative projected reduced costs.
"""
function _pricing_candidate_from_label(ctx::JointRoutingAssignmentDarpSearchContext, label::JointRoutingAssignmentDarpPricingLabel)
    isempty(label.served) && return nothing
    onboard_refund = sum((
        ctx.pricing_data.candidates[
            ctx.pricing_data.candidate_index[(p, j, k)]
        ].reward
        for (p, (j, k, _age)) in label.onboard
    ); init=0.0)
    assignments = Tuple{Int, Int, Int}[t for t in label.served]
    return (
        signature=_joint_routing_assignment_darp_column_signature(label.served),
        tau=label.tau, reduced_cost=label.reduced_cost + onboard_refund,
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

"""Reused from `exact/exact.jl` as-is: `JointRoutingAssignmentRouteColumn`
carries the same `assignments`/`tau` shape regardless of which pricer
produced it."""
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
column pool and return improving columns -- same accept/dedupe + harvest
shape as `darp_modified/darp_modified.jl`'s twin, standalone (no master
model/duals cross-check).
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
