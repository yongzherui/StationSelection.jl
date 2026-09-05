"""
Label seeding for the cut-aware relaxed search: the depth-1 labels
`_run_label_setting` (`../../engine.jl`) seeds its frontier from, via the
`_pricing_initial_labels` hook (wired in `cut_hooks.jl`).

Seeding is `../exact/seed.jl`'s, with each seed's satisfied-mask initialized to
whatever cuts its own starting node already escapes -- a route that begins
outside every cut set owes nothing from the start. See `cut_extend.jl` for how
the mask grows from here.
"""

function _initial_relaxed_cluster_cut_labels(
    pricing_data::JointRoutingAssignmentPricingData, cuts::RelaxedClusterNoGoodCuts,
)::Vector{RelaxedClusterCutLabel}
    return RelaxedClusterCutLabel[
        RelaxedClusterCutLabel(
            base.current, base.route, base.time, base.station_age,
            base.activated_reward_layers, base.tau, base.reduced_cost, base.route_length,
            cuts.node_mask[base.current],
        )
        for base in initial_joint_routing_assignment_pricing_labels(pricing_data)
    ]
end
