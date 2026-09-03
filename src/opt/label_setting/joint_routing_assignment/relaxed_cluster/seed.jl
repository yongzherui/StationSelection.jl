"""
Label seeding for the relaxed-cluster pricer: the depth-1 labels
`_run_label_setting` (`engine.jl`) seeds its frontier from, via the
`_pricing_initial_labels` hook (wired in `hooks.jl`).

Two deviations from `../exact/seed.jl`, both consequences of the intra-cluster
credit (`types.jl`, equation (4)):

1. **A one-cluster "route" can already be worth something.** A real route that
   picks up and drops off entirely inside cluster `C` maps to the relaxed route
   `[C]`, so the seed at `C` must already bank `C`'s intra-cluster reward --
   otherwise that real route's image earns nothing and the bound breaks. The
   seed's reduced cost is therefore `beta * repositioning_time - intra reward`,
   which can be negative on its own.

2. **Seeding is driven by `origin_layer_mask`, not `opportunities`.** The
   exact pricer seeds at every station that is some opportunity's origin;
   here intra opportunities have been stripped out of `assignments_by_origin`
   (`data.jl`), but they are still in `origin_layer_mask`, which is the table
   that answers "could a route starting here ever earn anything". Using
   `opportunities` instead would silently skip a cluster whose only value is
   intra-cluster.

Restricting seeds to reward-reachable clusters is WLOG exactly as it is for the
exact pricer: a prefix of clusters visited before the first one that can earn
anything only adds non-negative travel cost, so truncating it yields a relaxed
route that is at least as good.
"""

function _initial_joint_routing_assignment_relaxed_cluster_labels(
    data::RelaxedClusterPricingData,
)::Vector{JointRoutingAssignmentPricingLabel}
    inner = data.inner
    labels = JointRoutingAssignmentPricingLabel[]
    for node in inner.nodes
        haskey(inner.origin_layer_mask, node) || continue
        # `t = 0` is inside any non-negative pickup window, so the intra credit is
        # always available at a seed.
        activated, reward = _relaxed_cluster_collect_intra(
            data, node, 0.0, RewardLayerBitset(),
        )
        push!(labels, JointRoutingAssignmentPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            activated,
            0.0,
            # Same fixed `repositioning_time` charge every route pays, less whatever
            # this cluster's intra-cluster passengers are already worth.
            inner.route_regularization_weight * inner.repositioning_time - reward,
            1,
        ))
    end
    return labels
end
