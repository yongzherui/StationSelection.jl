"""
How a label grows: which nodes are legal to visit next
(`_joint_routing_assignment_candidate_next_nodes`, the `_pricing_candidate_next_nodes`
hook) and what visiting one produces (`_extend_joint_routing_assignment_pricing_label`,
the `_pricing_extend_label` hook, both wired in `hooks.jl`) -- station-age
aging/reset/prune and reward-layer certification happen here. See `seed.jl`
for where a label starts, `prune.jl` for the bound that decides whether
extending a label is worth it at all, and `dominate.jl` for what happens to
a child once it's built.
"""

# ── candidate next-nodes ─────────────────────────────────────────────────────
# `label` is untyped so both the revisit-tolerant and the elementary
# (`../station_simple/extend.jl`) pricers can share this: it reads only `station_age`,
# `current`, and `activated_reward_layers`, which both label types expose. Julia
# specializes per concrete call site, so there is no dispatch or speed cost.
function _has_useful_live_joint_routing_assignment_origin(
    label,
    pricing_data::JointRoutingAssignmentPricingData,
)::Bool
    for (station, age) in label.station_age
        opportunities = get(pricing_data.assignments_by_origin, station, PassengerAssignmentOpportunity[])
        for opp in opportunities
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            t_to_dest = opp.destination == label.current ? 0.0 :
                _joint_routing_assignment_travel(pricing_data, label.current, opp.destination)
            age + t_to_dest <= opp.ride_limit + 1e-9 || continue
            return true
        end
    end
    return false
end

"""
    is_useful_destination(label, k)

A station `k` is worth visiting next as a destination if some *currently live*
origin age can still certify a currently-inactive layer there. A station `j` is
worth visiting as an origin (only while still inside the pickup window) if
visiting it could open a live clock that later unlocks a currently-inactive
layer, judged via `origin_layer_mask` (the union of everything reachable from
that origin, an optimistic pre-filter -- true feasibility from that origin is
re-checked once its age actually becomes live).
"""
function _joint_routing_assignment_candidate_next_nodes(
    label::JointRoutingAssignmentPricingLabel,
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_joint_routing_assignment_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, mask) in pricing_data.origin_layer_mask
            origin == label.current && continue
            _has_inactive_layer(mask, label.activated_reward_layers) || continue
            arrival_time = label.time + _joint_routing_assignment_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    # Driven from the label's *live origins* rather than from every destination
    # group: only a live origin can make a destination useful, and pruning keeps
    # the live set small, whereas `assignments_by_destination` spans all
    # `~P * n^2` opportunities regardless of label state.
    for (origin, origin_age) in label.station_age
        for opp in get(pricing_data.assignments_by_origin, origin, PassengerAssignmentOpportunity[])
            opp.destination == label.current && continue
            opp.destination in candidate_nodes && continue
            _has_inactive_layer(opp.layer_mask, label.activated_reward_layers) || continue
            origin_age + _joint_routing_assignment_travel(pricing_data, label.current, opp.destination) <=
                opp.ride_limit + 1e-9 || continue
            push!(candidate_nodes, opp.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

# ── label extension ──────────────────────────────────────────────────────────
export extend_joint_routing_assignment_pricing_label

"""
Extension always produces exactly one child (an unlimited-capacity route has
nothing to branch on at a stop), so the search calls
`_extend_joint_routing_assignment_pricing_label` and gets the label back
directly. This method wraps it in the one-element `Vector` the public API has
always returned; that wrapper allocation is per *extension*, so it is worth not
paying on the search's hot path.
"""
function extend_joint_routing_assignment_pricing_label(
    label::JointRoutingAssignmentPricingLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
)::Vector{JointRoutingAssignmentPricingLabel}
    return JointRoutingAssignmentPricingLabel[
        _extend_joint_routing_assignment_pricing_label(label, next_node, pricing_data),
    ]
end

function _extend_joint_routing_assignment_pricing_label(
    label::JointRoutingAssignmentPricingLabel,
    next_node::Int,
    pricing_data::JointRoutingAssignmentPricingData,
)::JointRoutingAssignmentPricingLabel
    travel_time = _joint_routing_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    certified_layers, reward = _certify_joint_routing_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )

    # Age, reset, and prune in ONE pass. Previously this built an aged Dict and
    # then a second pruned Dict from it -- two allocations and two traversals per
    # extension, on the hottest path in the search.
    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        station == next_node && continue  # handled by the reset below
        aged = age + travel_time
        _joint_routing_assignment_age_is_useful(station, aged, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        # A fresh clock at the arrival station: age 0 is the most useful an age can
        # be, but it still only earns a slot if it can reach some opportunity in time.
        _joint_routing_assignment_age_is_useful(next_node, 0.0, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
        # Past the cutoff the visit creates no new clock, so `next_node`'s existing
        # clock (if any) just ages like the rest.
        aged = label.station_age[next_node] + travel_time
        _joint_routing_assignment_age_is_useful(next_node, aged, pricing_data, next_node) &&
            (aged_station[next_node] = aged)
    end

    return JointRoutingAssignmentPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified_layers,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )
end
