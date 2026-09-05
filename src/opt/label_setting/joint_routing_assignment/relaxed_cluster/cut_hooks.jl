"""
All wiring, no logic: every method that plugs `RelaxedClusterCutSearchContext`
(`cut_context.jl`) into the `AbstractPricingSearchContext` contract
(`../../types.jl`) that `_run_label_setting` (`../../engine.jl`) calls during the
search -- forwarded to `cut_seed.jl` / `cut_extend.jl`, with the dominance
predicate, the label-bitsets mirror and the remaining-reward bound taken
directly from `../exact/`.

Only the search-level contract is implemented here. There are no `round.jl`
harvest hooks (`_pricing_candidate_from_label` and friends): a cluster route is
not a real route and can never become a column, so this context is only ever
driven by `nogood_certify.jl`, which reads reduced costs off the returned labels
itself.
"""

_pricing_initial_labels(ctx::RelaxedClusterCutSearchContext) =
    _initial_relaxed_cluster_cut_labels(ctx.pricing_data, ctx.cuts)

# Same mirror as `../exact/dominate.jl`'s `_make_joint_routing_assignment_label_bitsets`,
# built off the cut label's own fields -- a cut affects nothing the bitsets carry, and
# projecting through `_relaxed_cluster_base_label` would materialize a base label per
# insertion for no gain.
function _pricing_make_bitsets(ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel)
    age_idx, age_val, age_mask =
        _make_sparse_station_ages(label.station_age, ctx.search_index.node_index)
    return JointRoutingAssignmentLabelBitsets(
        label.activated_reward_layers, age_idx, age_val, age_mask,
    )
end

# The mask is part of the STATE, not just the signature: two labels that have escaped
# different cuts have genuinely different futures and must not dominate one another.
_pricing_state(
    ::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
    ::JointRoutingAssignmentLabelBitsets,
) = (label.current, label.satisfied)

# The bound reads only current/time/activated layers off the label, which this type
# exposes directly, so `../exact/prune.jl` applies unchanged.
_pricing_label_priority(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
    label_bs::JointRoutingAssignmentLabelBitsets,
)::Float64 = label.reduced_cost - _joint_routing_assignment_remaining_reward_bound(
    label, label_bs, ctx.pricing_data, ctx.search_index, ctx.bound_workspace,
)

"""A label is an answer only once it has escaped EVERY active cut. Labels that have
not are still extended -- they may escape later -- they simply cannot be reported.
This is the filter, and it has to live here rather than over the returned labels:
the search keeps one best label per signature, so a post-hoc filter would silently
discard a cut-satisfying route in favour of a cut-violating one and certify."""
_pricing_best_signature(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
) = (isempty(label.activated_reward_layers) || label.satisfied != ctx.cuts.all_satisfied) ?
    nothing : (label.activated_reward_layers, label.satisfied)

_pricing_route_length(::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel) =
    label.route_length

_pricing_max_route_length(ctx::RelaxedClusterCutSearchContext) = ctx.pricing_data.max_stops

_pricing_candidate_next_nodes(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel,
) = _relaxed_cluster_cut_candidate_next_nodes(label, ctx.pricing_data, ctx.cuts)

_pricing_extend_label(
    ctx::RelaxedClusterCutSearchContext, label::RelaxedClusterCutLabel, next_node::Int,
) = _extend_relaxed_cluster_cut_label(label, next_node, ctx.pricing_data, ctx.cuts)

_pricing_dominates_fn(ctx::RelaxedClusterCutSearchContext) = ctx.dominates
