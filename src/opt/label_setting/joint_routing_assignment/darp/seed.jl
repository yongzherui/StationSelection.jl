"""
Label seeding for `darp/`'s literal onboard-bitset pricer: the depth-1
labels `_run_label_setting` (`engine.jl`) seeds its frontier from, via the
`_pricing_initial_labels` hook (wired in `hooks.jl`). Unlike every sibling
pricer, a seed label can itself already have onboard commitments (boarding
is decided the instant the vehicle is *at* a candidate origin, including the
route's initial station -- see `types.jl`'s module docstring). See
`extend.jl` for how a seeded label grows from here.
"""

export initial_joint_routing_assignment_darp_pricing_labels

function initial_joint_routing_assignment_darp_pricing_labels(
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{JointRoutingAssignmentDarpPricingLabel}
    endpoints = Set{Int}()
    for c in pricing_data.candidates
        push!(endpoints, c.origin)
        push!(endpoints, c.destination)
    end

    labels = JointRoutingAssignmentDarpPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        options = _joint_routing_assignment_darp_board_options(node, 0.0, Set{Int}(), pricing_data)
        for board_subset in _joint_routing_assignment_darp_board_subsets(node, options)
            onboard = Dict{Int, Tuple{Int, Int, Float64}}(
                p => (j, k, 0.0) for (p, j, k, _reward) in board_subset
            )
            boarded_reward = sum(choice[4] for choice in board_subset; init=0.0)
            push!(labels, JointRoutingAssignmentDarpPricingLabel(
                node,
                [node],
                0.0,
                onboard,
                Set{Tuple{Int, Int, Int}}(),
                0.0,
                pricing_data.route_regularization_weight * pricing_data.repositioning_time - boarded_reward,
                1,
            ))
        end
    end
    return labels
end
