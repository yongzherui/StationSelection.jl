"""
All wiring, no logic: every method that plugs
`JointRoutingAssignmentDarpModifiedSearchContext` (`context.jl`) into the two
generic hook contracts this pricer implements, exactly like `../exact/hooks.jl`'s
hooks. Two contracts, both forwarded from here:

  - the twelve `AbstractPricingSearchContext` hooks (`../../types.jl`) that
    `_run_label_setting` (`engine.jl`) calls during the search itself --
    forwarded to `seed.jl` / `extend.jl` / `prune.jl` / `dominate.jl`;
  - the four context-level hooks `round.jl` calls once per surviving label to
    harvest it into a column.

Unlike `../exact/`'s twin, this pricer's round-level hooks have no
route-replay step to forward to: a finished label's `served` field already
*is* the final answer (`types.jl`'s module docstring), so
`_pricing_candidate_from_label` is a trivial projection -- the column-shape
helper (`_joint_routing_assignment_darp_modified_column_signature`) lives
inline in this file rather than in a separate `accept.jl`. See `driver.jl`
for the standalone comparison entrypoint that also goes through these hooks,
bypassing the CG hub entirely.
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
_pricing_initial_labels(ctx::JointRoutingAssignmentDarpModifiedSearchContext) =
    initial_joint_routing_assignment_darp_modified_pricing_labels(ctx.pricing_data)

_pricing_make_bitsets(ctx::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel) =
    _make_joint_routing_assignment_darp_modified_label_bitsets(label, ctx.node_index, ctx.n_nodes)

_pricing_state(::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel, ::JointRoutingAssignmentDarpModifiedLabelBitsets) =
    _joint_routing_assignment_darp_modified_state(label)

_pricing_label_priority(ctx::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel, label_bs::JointRoutingAssignmentDarpModifiedLabelBitsets)::Float64 =
    _joint_routing_assignment_darp_modified_remaining_reward_bound(label, label_bs, ctx)

_pricing_best_signature(::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel) =
    isempty(label.served) ? nothing : _joint_routing_assignment_darp_modified_column_signature(label.served)

_pricing_route_length(::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentDarpModifiedSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(ctx::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel) =
    _joint_routing_assignment_darp_modified_candidate_next_nodes(label, ctx.pricing_data)

# `action`, not a bare node id: see `extend.jl`'s module docstring for why
# commit/skip branching is expressed as one action per branch rather than as
# multiple children of one action.
_pricing_extend_label(ctx::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel, action::JointRoutingAssignmentDarpModifiedAction) =
    _extend_joint_routing_assignment_darp_modified_pricing_label(label, action, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentDarpModifiedSearchContext) = ctx.dominates

# ── round.jl context-level hooks (candidate → column → master verification) ──
_joint_routing_assignment_darp_modified_column_signature(served::Dict{Int, Tuple{Int, Int}}) =
    Tuple(sort!([(p, o, k) for (p, (o, k)) in served]))

function _pricing_candidate_from_label(::JointRoutingAssignmentDarpModifiedSearchContext, label::JointRoutingAssignmentDarpModifiedPricingLabel)
    isempty(label.served) && return nothing
    assignments = Tuple{Int, Int, Int}[(p, o, k) for (p, (o, k)) in label.served]
    return (
        signature=_joint_routing_assignment_darp_modified_column_signature(label.served),
        tau=label.tau, reduced_cost=label.reduced_cost,
        payload=(route=label.route, assignments=assignments),
    )
end

_pricing_pool_signature(::JointRoutingAssignmentDarpModifiedSearchContext, existing_column::JointRoutingAssignmentRouteColumn) =
    Tuple(sort!(collect(existing_column.assignments)))

_pricing_make_column(ctx::JointRoutingAssignmentDarpModifiedSearchContext, column_id::Int, candidate) =
    JointRoutingAssignmentRouteColumn(
        column_id, candidate.payload.route, candidate.payload.assignments, candidate.tau;
        metadata=Dict{String, Any}(
            "scenario" => ctx.pricing_data.scenario,
            "route" => Tuple(candidate.payload.route),
            "reduced_cost" => candidate.reduced_cost,
        ),
    )

"""
Reused from `../exact/hooks.jl` as-is: `JointRoutingAssignmentRouteColumn`
carries the same `assignments`/`tau` shape regardless of which pricer
produced it, and the master doesn't care how a column was priced, only what
it contains -- see `_verify_joint_routing_assignment_master_reduced_cost`
(`../duals.jl`)."""
function _pricing_verify_column(::JointRoutingAssignmentDarpModifiedSearchContext, column::JointRoutingAssignmentRouteColumn, m::JuMP.Model, mapping, duals)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    return _verify_joint_routing_assignment_master_reduced_cost(column, m, data, mapping, alpha, gamma_o, gamma_d)
end
