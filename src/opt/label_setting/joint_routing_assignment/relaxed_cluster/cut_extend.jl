"""
How a cut-aware label grows: which nodes are legal to visit next
(`_relaxed_cluster_cut_candidate_next_nodes`, the `_pricing_candidate_next_nodes`
hook) and what visiting one produces (`_extend_relaxed_cluster_cut_label`, the
`_pricing_extend_label` hook, both wired in `cut_hooks.jl`).

Both delegate to `../exact/extend.jl` through `_relaxed_cluster_base_label`
(`cut_types.jl`) and then fold the visited node's mask word in, so the travel,
certification and reduced-cost arithmetic have exactly one definition. The
candidate rule is the one place a cut genuinely changes the search -- see below.
"""

"""
    _relaxed_cluster_cut_candidate_next_nodes(label, pricing_data, cuts) -> Vector{Int}

The exact pricer's candidate rule, **widened by every node that would newly satisfy
an outstanding cut**.

That widening is not an optimization, it is required for completeness.
`_joint_routing_assignment_candidate_next_nodes` is reward-driven: it proposes a node only
if visiting it can unlock a not-yet-activated reward layer. Under a cut, a route may have
to visit a cluster *purely to leave the cut set*, collecting nothing there -- and the
reward-driven rule would never propose it, so the search could not build the route at all.

MEASURED: without this, a cut of `{1,2}` made the search report a minimum of -114.6 where
exhaustive enumeration over cut-satisfying routes found -250.5. The winning route was
`[2, 1, 4, 3]`, whose last stop (cluster 3) opened no reward and existed only to escape the
cut. Under-reporting here is the false-certificate failure mode: the loop sees "nothing
improving survives" and certifies while an improving route exists.

Only labels that still owe a cut pay for the widening; once `satisfied` is complete the
rule is the exact pricer's untouched.
"""
function _relaxed_cluster_cut_candidate_next_nodes(
    label::RelaxedClusterCutLabel,
    pricing_data::JointRoutingAssignmentPricingData,
    cuts::RelaxedClusterNoGoodCuts,
)::Vector{Int}
    base = _joint_routing_assignment_candidate_next_nodes(
        _relaxed_cluster_base_label(label), pricing_data,
    )
    label.satisfied == cuts.all_satisfied && return base

    escapes = Int[]
    @inbounds for v in eachindex(cuts.node_mask)
        v == label.current && continue
        # Would visiting `v` turn on a cut bit this label does not yet have?
        (label.satisfied | cuts.node_mask[v]) == label.satisfied && continue
        v in base && continue
        push!(escapes, v)
    end
    isempty(escapes) && return base
    return sort!(vcat(base, escapes))
end

"""
    _extend_relaxed_cluster_cut_label(label, next_node, pricing_data, cuts) -> RelaxedClusterCutLabel

`../exact/extend.jl`'s extension, with `next_node`'s mask word OR'd into the
child's `satisfied`. The mask is monotone: bits only ever turn on, so a route
that has escaped a cut can never un-escape it by travelling further.
"""
function _extend_relaxed_cluster_cut_label(
    label::RelaxedClusterCutLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
    cuts::RelaxedClusterNoGoodCuts,
)::RelaxedClusterCutLabel
    child = _extend_joint_routing_assignment_pricing_label(
        _relaxed_cluster_base_label(label), next_node, pricing_data,
    )
    return RelaxedClusterCutLabel(
        child.current, child.route, child.time, child.station_age,
        child.activated_reward_layers, child.tau, child.reduced_cost, child.route_length,
        label.satisfied | cuts.node_mask[next_node],
    )
end
