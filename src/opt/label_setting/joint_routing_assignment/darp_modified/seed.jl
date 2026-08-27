"""
Label seeding for `darp_modified/`'s branching passenger free-assignment
pricer: the depth-1 labels `_run_label_setting` (`engine.jl`) seeds its
frontier from, via the `_pricing_initial_labels` hook (wired in `hooks.jl`).
See `types.jl` for what `JointRoutingAssignmentDarpModifiedPricingLabel`
means and `extend.jl` for how a seeded label grows from here.
"""

export initial_joint_routing_assignment_darp_modified_pricing_labels

function initial_joint_routing_assignment_darp_modified_pricing_labels(
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::Vector{JointRoutingAssignmentDarpModifiedPricingLabel}
    # Same reasoning as every sibling pricer's twin: a route can only ever
    # collect reward through one of a candidate's two endpoints, so seeding
    # elsewhere would waste search on routes that can never certify anything.
    endpoints = Set{Int}()
    for c in pricing_data.candidates
        push!(endpoints, c.origin)
        push!(endpoints, c.destination)
    end

    labels = JointRoutingAssignmentDarpModifiedPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, JointRoutingAssignmentDarpModifiedPricingLabel(
            node,
            [node],
            0.0,
            Dict(node => 0.0),
            Dict{Int, Tuple{Int, Int}}(),
            0.0,
            pricing_data.route_regularization_weight * pricing_data.repositioning_time,
            1,
        ))
    end
    return labels
end
