"""
How a relaxed-cluster label grows.

Candidate generation is `../exact/extend.jl`'s, unchanged, run against the
inner cluster-node pricing data -- the relaxed graph is just a smaller graph,
and the intra-cluster credit needs no arc of its own, so nothing about *which
cluster to visit next* differs. (A cluster whose only value is intra-cluster is
still proposed while inside the pickup window, because its intra candidates
remain in `origin_layer_mask`; see `data.jl` for why they are absent from
`assignments_by_origin`.)

Extension is `../exact/extend.jl`'s with exactly one addition: after the
ordinary destination-certification, arrival at a cluster also banks that
cluster's intra-cluster reward (`types.jl`, equation (4)). The two are
composed in that order, but the order is immaterial -- both are unions of
per-passenger prefix masks into the same activated set, so the incremental
reward telescopes the same way either way.
"""

# ── candidate next-nodes ────────────────────────────────────────────────────
"""
Candidate next clusters: `../exact/extend.jl`'s rule verbatim, against the
inner (cluster-node) pricing data.
"""
_joint_routing_assignment_relaxed_cluster_candidate_next_nodes(
    label::JointRoutingAssignmentPricingLabel, data::RelaxedClusterPricingData,
)::Vector{Int} = _joint_routing_assignment_candidate_next_nodes(label, data.inner)

# ── label extension ─────────────────────────────────────────────────────────
"""
    _extend_joint_routing_assignment_relaxed_cluster_label(label, next_node, data)

Visit cluster `next_node`. Identical to
`_extend_joint_routing_assignment_pricing_label` (`../exact/extend.jl`) --
same aging/reset/prune single pass, same reduced-cost update -- except that the
reward collected at the new cluster is the ordinary destination certification
*plus* the intra-cluster credit, and the latter is what makes a real route
serving two stations of one cluster representable at all.
"""
function _extend_joint_routing_assignment_relaxed_cluster_label(
    label::JointRoutingAssignmentPricingLabel,
    next_node::Int,
    data::RelaxedClusterPricingData,
)::JointRoutingAssignmentPricingLabel
    pricing_data = data.inner
    travel_time = _joint_routing_assignment_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    # (a) ordinary certification: live clocks at other clusters reaching this one.
    certified_layers, reward = _certify_joint_routing_assignment_layers_at_node(
        next_node,
        label.station_age,
        travel_time,
        label.activated_reward_layers,
        pricing_data,
    )
    # (b) the relaxation's own addition: passengers whose whole trip fits inside this
    # cluster are credited on arrival, with no arc and no internal travel time.
    certified_layers, intra_reward = _relaxed_cluster_collect_intra(
        data, next_node, arrival_time, certified_layers,
    )
    reward += intra_reward

    # Age, reset, and prune in one pass -- see the exact pricer's twin for why this is
    # a single traversal rather than an aged Dict followed by a pruned one.
    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        station == next_node && continue  # handled by the reset below
        aged = age + travel_time
        _joint_routing_assignment_age_is_useful(station, aged, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _joint_routing_assignment_age_is_useful(next_node, 0.0, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
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
