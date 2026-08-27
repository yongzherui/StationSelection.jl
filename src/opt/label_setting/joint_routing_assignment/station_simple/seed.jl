"""
Label seeding for the elementary-route (station-simple) passenger
free-assignment pricer: the depth-1 labels `_run_label_setting` (`engine.jl`)
seeds its frontier from, via the `_pricing_initial_labels` hook (wired in
`hooks.jl`). See `types.jl` for what `JointRoutingAssignmentStationSimpleLabel`
means and `extend.jl` for how a seeded label grows from here.
"""

function _initial_joint_routing_assignment_station_simple_labels(
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{JointRoutingAssignmentStationSimpleLabel}
    # Same reasoning as the revisit-tolerant twin: only nodes that are some
    # opportunity's origin or destination can ever collect reward.
    endpoints = Set{Int}()
    for opp in pricing_data.opportunities
        push!(endpoints, opp.origin)
        push!(endpoints, opp.destination)
    end

    labels = JointRoutingAssignmentStationSimpleLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        # One depth-1 label per relevant node: `visited = {node}` (this
        # pricer's authoritative no-revisit set), pickup clock live at age 0,
        # no reward layers activated, `tau`/`reduced_cost` carrying the fixed
        # `repositioning_time` cost -- otherwise identical to the
        # revisit-tolerant pricer's seeding.
        push!(labels, JointRoutingAssignmentStationSimpleLabel(
            node,
            [node],
            BitSet((node,)),
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
