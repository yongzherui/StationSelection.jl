"""
All wiring, no logic: every method plugging
`JointRoutingAssignmentRelaxedClusterSearchContext` (`context.jl`) into the
`AbstractPricingSearchContext` hook contract (`../../types.jl`) that
`_run_label_setting` (`engine.jl`) calls during the search.

Unlike every sibling pricer's `hooks.jl`, **`round.jl`'s four context-level
hooks are refusals, not implementations.** This pricer's labels are cluster
routes, not real routes: there is nothing to replay, no `(p, j, k)` assignment
to recover, and no column a master could accept. Its result is consumed by
`certify.jl` instead, which reads reduced costs directly off the labels. The
refusals below exist so that a future change routing this context through
`_run_pricing_round` fails loudly at the point of the mistake rather than
silently minting nonsense columns -- see `types.jl` for the full argument.
"""

# ── AbstractPricingSearchContext hooks (engine.jl / label_setting/types.jl) ──
_pricing_initial_labels(ctx::JointRoutingAssignmentRelaxedClusterSearchContext) =
    _initial_joint_routing_assignment_relaxed_cluster_labels(ctx.pricing_data)

_pricing_make_bitsets(
    ctx::JointRoutingAssignmentRelaxedClusterSearchContext, label::JointRoutingAssignmentPricingLabel,
) = _make_joint_routing_assignment_label_bitsets(label, ctx.search_index.node_index, ctx.n_nodes)

_pricing_state(
    ::JointRoutingAssignmentRelaxedClusterSearchContext, label::JointRoutingAssignmentPricingLabel,
    ::JointRoutingAssignmentLabelBitsets,
) = _joint_routing_assignment_state(label)

# The bound reads only current/time/activated layers off the label and the sparse ages off
# the mirror, and is admissible over the cluster graph for exactly the reasons it is over
# the station graph -- see `RelaxedClusterPricingData`'s docstring for the one place that
# needed care (intra-cluster reward must stay visible to it).
function _pricing_label_priority(
    ctx::JointRoutingAssignmentRelaxedClusterSearchContext,
    label::JointRoutingAssignmentPricingLabel,
    label_bs::JointRoutingAssignmentLabelBitsets,
)::Float64
    return label.reduced_cost - _joint_routing_assignment_remaining_reward_bound(
        label, label_bs, ctx.pricing_data.inner, ctx.search_index, ctx.bound_workspace,
    )
end

_pricing_best_signature(
    ::JointRoutingAssignmentRelaxedClusterSearchContext, label::JointRoutingAssignmentPricingLabel,
) = isempty(label.activated_reward_layers) ? nothing : _joint_routing_assignment_layer_signature(label)

_pricing_route_length(
    ::JointRoutingAssignmentRelaxedClusterSearchContext, label::JointRoutingAssignmentPricingLabel,
) = label.route_length

_pricing_max_route_length(ctx::JointRoutingAssignmentRelaxedClusterSearchContext) =
    ctx.pricing_data.inner.max_stops

_pricing_candidate_next_nodes(
    ctx::JointRoutingAssignmentRelaxedClusterSearchContext, label::JointRoutingAssignmentPricingLabel,
) = _joint_routing_assignment_relaxed_cluster_candidate_next_nodes(label, ctx.pricing_data)

_pricing_extend_label(
    ctx::JointRoutingAssignmentRelaxedClusterSearchContext, label::JointRoutingAssignmentPricingLabel,
    next_node::Int,
) = _extend_joint_routing_assignment_relaxed_cluster_label(label, next_node, ctx.pricing_data)

_pricing_dominates_fn(ctx::JointRoutingAssignmentRelaxedClusterSearchContext) = ctx.dominates

# ── round.jl context-level hooks: deliberately unimplemented ────────────────
const _RELAXED_CLUSTER_NO_COLUMNS_MESSAGE =
    "the relaxed-cluster pricer searches a relaxed cluster graph, so its labels are not " *
    "real routes and cannot become master columns -- it answers only 'does an improving " *
    "column possibly exist' (see certify.jl). Route it through " *
    "_run_relaxed_cluster_certification_round, not _run_pricing_round."

_pricing_candidate_from_label(::JointRoutingAssignmentRelaxedClusterSearchContext, label) =
    error(_RELAXED_CLUSTER_NO_COLUMNS_MESSAGE)

_pricing_pool_signature(::JointRoutingAssignmentRelaxedClusterSearchContext, existing_column) =
    error(_RELAXED_CLUSTER_NO_COLUMNS_MESSAGE)

_pricing_make_column(::JointRoutingAssignmentRelaxedClusterSearchContext, column_id::Int, candidate) =
    error(_RELAXED_CLUSTER_NO_COLUMNS_MESSAGE)

_pricing_verify_column(
    ::JointRoutingAssignmentRelaxedClusterSearchContext, column, m::JuMP.Model, mapping, duals,
) = error(_RELAXED_CLUSTER_NO_COLUMNS_MESSAGE)
