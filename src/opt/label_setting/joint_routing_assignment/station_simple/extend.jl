"""
How a label grows: which not-yet-visited nodes are legal to visit next
(`_joint_routing_assignment_station_simple_candidate_next_nodes`, the
`_pricing_candidate_next_nodes` hook) and what visiting one produces
(`_extend_joint_routing_assignment_station_simple_label`, the
`_pricing_extend_label` hook, both wired in `hooks.jl`). Reuses
`../exact/extend.jl`'s `_has_useful_live_joint_routing_assignment_origin` and
`../data.jl`'s certification/aging primitives directly -- see `seed.jl` for
where a label starts and `dominate.jl` for what happens to a child once it's
built. This pricer reuses `../exact/prune.jl`'s remaining-reward bound as-is
(wired straight in `hooks.jl`), so there is no `prune.jl` here.
"""

"""
Candidate next nodes for an elementary label: the revisit-tolerant
`_joint_routing_assignment_candidate_next_nodes` restricted to unvisited nodes,
with the station-budget branch removed (subsumed by elementarity).
"""
function _joint_routing_assignment_station_simple_candidate_next_nodes(
    label::JointRoutingAssignmentStationSimpleLabel,
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_joint_routing_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin in label.visited && continue  # elementary: no revisit (also excludes current)
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _joint_routing_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination in label.visited && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _joint_routing_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

"""
Extend an elementary label to `next_node` (which must be unvisited). Reward
certification and clock aging are identical to
`extend_joint_routing_assignment_pricing_label`; the revisit-tolerant version's
special handling of a re-visited `next_node` clock is simply unreachable here
(`next_node ∉ visited ⊇ keys(station_age)`), so it is omitted.
"""
function _extend_joint_routing_assignment_station_simple_label(
    label::JointRoutingAssignmentStationSimpleLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
)::JointRoutingAssignmentStationSimpleLabel
    next_node in label.visited &&
        throw(ArgumentError("station-simple extension cannot revisit $next_node"))

    travel_time = _joint_routing_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)
    new_visited = copy(label.visited)
    push!(new_visited, next_node)

    # Certify/activate whatever `next_node` newly unlocks (shared with the
    # revisit-tolerant pricer -- see `data.jl`), then age every existing clock
    # and open a fresh one at `next_node` if still inside the wait cutoff. No
    # "re-visited node" branch here (unlike the revisit-tolerant twin): it is
    # unreachable by construction, since `next_node ∉ visited` always holds
    # for an elementary extension.
    certified_layers, reward = _certify_joint_routing_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        aged = age + travel_time
        _joint_routing_assignment_age_is_useful(station, aged, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _joint_routing_assignment_age_is_useful(next_node, 0.0, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    end

    return JointRoutingAssignmentStationSimpleLabel(
        next_node,
        new_route,
        new_visited,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        # Same reduced-cost update rule as the revisit-tolerant pricer.
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )
end
