"""
All wiring, no logic: every method that plugs
`JointRoutingAssignmentDarpSearchContext` (`context.jl`) into the two
generic hook contracts this pricer implements. Two contracts, both forwarded
from here:

  - the twelve `AbstractPricingSearchContext` hooks (`../../types.jl`) that
    `_run_label_setting` (`engine.jl`) calls during the search itself --
    forwarded to `seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl`;
  - the four context-level hooks `round.jl` calls once per surviving label to
    harvest it into a column.

Like `darp_modified/`, needs no route-replay step: a label's `served` field
is already a valid delivered assignment set. If the label still has onboard
commitments, `_pricing_candidate_from_label` projects them away by refunding
their pickup-time rewards; the physical route remains valid, and the shared
round-level `accept!` hook decides whether that projected column is
improving. See `driver.jl` for the standalone comparison entrypoint that
also goes through these hooks, bypassing the CG hub entirely.
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
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

# ── round.jl context-level hooks (candidate → column → master verification) ──
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

"""Reused from `../exact/hooks.jl` as-is: `JointRoutingAssignmentRouteColumn`
carries the same `assignments`/`tau` shape regardless of which pricer
produced it."""
function _pricing_verify_column(::JointRoutingAssignmentDarpSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
