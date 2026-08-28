"""
Label seeding for the revisit-tolerant passenger free-assignment pricer: the
depth-1 labels `_run_label_setting` (`engine.jl`) seeds its frontier from, via
the `_pricing_initial_labels` hook (wired in `hooks.jl`). See `types.jl` for
what `JointRoutingAssignmentPricingLabel` means and `extend.jl` for how a
seeded label grows from here.
"""

export initial_joint_routing_assignment_pricing_labels

function initial_joint_routing_assignment_pricing_labels(
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{JointRoutingAssignmentPricingLabel}
    # Same reasoning as the route-covering pricer's twin: a route can only
    # ever collect reward through one of an opportunity's two endpoints, so
    # seeding elsewhere would waste search on routes that can never certify
    # anything.
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
    end

    labels = JointRoutingAssignmentPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        # One depth-1 label per relevant node: route so far is `[node]`,
        # `time = 0`, this node's own pickup clock starts live at age 0, no
        # reward layers activated yet, and `tau`/`reduced_cost` already carry
        # the fixed `repositioning_time` cost every route pays regardless of
        # length.
        push!(labels, JointRoutingAssignmentPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            RewardLayerBitset(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end
