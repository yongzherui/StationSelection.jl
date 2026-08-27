"""
How a label grows: which nodes are legal to visit next, bundled with which
board selection each visit takes
(`_joint_routing_assignment_darp_candidate_next_nodes`, the
`_pricing_candidate_next_nodes` hook) and what taking one action produces
(`_extend_joint_routing_assignment_darp_pricing_label`, the
`_pricing_extend_label` hook, both wired in `hooks.jl`) -- onboard aging,
delivery resolution, and fresh boarding happen here. See `seed.jl` for where
a label starts, `prune.jl` for the bound that decides whether extending a
label is worth it at all, and `dominate.jl` for what happens to a child once
it's built.
"""

# ── resolved-passenger helper ────────────────────────────────────────────────
"""`p => true` for every passenger already onboard or served -- computed once
per extension/candidate-generation call rather than rescanned per candidate,
since `served` is a set of triples and passenger membership isn't free to
test repeatedly."""
_joint_routing_assignment_darp_resolved_passengers(label::JointRoutingAssignmentDarpPricingLabel)::Set{Int} =
    union(keys(label.onboard), (t[1] for t in label.served))

# ── candidate next-nodes: physical reachability + hard-infeasibility filter ──
"""
Physical reachability only, before the feasibility filter below: nodes worth
visiting are (a) any current onboard commitment's destination (owed, not
optional) and, while still within the pickup window, (b) any not-yet-resolved
passenger's candidate origin reachable in time. Unlike every other pricer in
this package there is no `station_age`-driven "is a live clock still useful"
branch -- boarding is decided at the instant of arrival, not retroactively,
so there is nothing to check reachability of except the raw
`max_wait_time`/`label.time` comparison.
"""
function _joint_routing_assignment_darp_reachable_next_nodes(
    label::JointRoutingAssignmentDarpPricingLabel,
    resolved::Set{Int},
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    for (_p, (_j, k, _age)) in label.onboard
        k == label.current && continue
        push!(candidate_nodes, k)
    end

    if label.time <= pricing_data.max_wait_time + 1e-9
        for (origin, idxs) in pricing_data.candidates_by_origin
            origin == label.current && continue
            any(idx -> !(pricing_data.candidates[idx].p in resolved), idxs) || continue
            arrival_time = label.time + _joint_routing_assignment_darp_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    return sort!(collect(candidate_nodes))
end

"""
Every reachable node, paired with every valid board selection there, after a
hard-infeasibility filter: a node is offered at all only if visiting it would
not make it impossible to still honor *every* current onboard commitment
(including ones not being resolved at this node) -- `age + travel_time +
(remaining travel to that commitment's own destination) <= its ride limit`,
optimistically assuming the cheapest possible remaining path. If this fails
for any commitment, the whole node is excluded -- see `types.jl`'s module
docstring for why this is enforced here, as an action-generation filter,
rather than inside `_pricing_extend_label` (which cannot itself decline to
produce a child).
"""
function _joint_routing_assignment_darp_candidate_next_nodes(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{JointRoutingAssignmentDarpAction}
    resolved = _joint_routing_assignment_darp_resolved_passengers(label)
    actions = JointRoutingAssignmentDarpAction[]
    for next_node in _joint_routing_assignment_darp_reachable_next_nodes(label, resolved, pricing_data)
        travel_time = _joint_routing_assignment_darp_travel(pricing_data, label.current, next_node)

        feasible = true
        for (p, (j, k, age)) in label.onboard
            aged = age + travel_time
            remaining = k == next_node ? 0.0 : _joint_routing_assignment_darp_travel(pricing_data, next_node, k)
            ride_limit = pricing_data.candidates[pricing_data.candidate_index[(p, j, k)]].ride_limit
            aged + remaining <= ride_limit + 1e-9 || (feasible = false; break)
        end
        feasible || continue

        arrival_time = label.time + travel_time
        options = _joint_routing_assignment_darp_board_options(next_node, arrival_time, resolved, pricing_data)
        for board_subset in _joint_routing_assignment_darp_board_subsets(next_node, options)
            push!(actions, (next_node, board_subset))
        end
    end
    return actions
end

# ── label extension ──────────────────────────────────────────────────────────
export extend_joint_routing_assignment_darp_pricing_label

"""
Extension always produces exactly one child per *action* -- as with
`darp_modified/`, branching is in how many actions
`_joint_routing_assignment_darp_candidate_next_nodes` returns for one label,
not in how many children one action produces here."""
function extend_joint_routing_assignment_darp_pricing_label(
    label::JointRoutingAssignmentDarpPricingLabel,
    action::JointRoutingAssignmentDarpAction,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{JointRoutingAssignmentDarpPricingLabel}
    return JointRoutingAssignmentDarpPricingLabel[
        _extend_joint_routing_assignment_darp_pricing_label(label, action, pricing_data),
    ]
end

function _extend_joint_routing_assignment_darp_pricing_label(
    label::JointRoutingAssignmentDarpPricingLabel,
    action::JointRoutingAssignmentDarpAction,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::JointRoutingAssignmentDarpPricingLabel
    next_node, board_subset = action
    travel_time = _joint_routing_assignment_darp_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    # Age every existing commitment; resolve (deterministically) any whose
    # destination is next_node -- feasibility of every one of these was
    # already guaranteed by candidate_next_nodes's hard-infeasibility filter,
    # so nothing here can fail.
    new_onboard = Dict{Int, Tuple{Int, Int, Float64}}()
    served = copy(label.served)
    for (p, (j, k, age)) in label.onboard
        aged = age + travel_time
        if k == next_node
            push!(served, (p, j, k))
        else
            new_onboard[p] = (j, k, aged)
        end
    end

    # Board this action's chosen selection, fresh (age 0).
    for (p, j, k, _r) in board_subset
        new_onboard[p] = (j, k, 0.0)
    end
    boarded_reward = sum(choice[4] for choice in board_subset; init=0.0)

    return JointRoutingAssignmentDarpPricingLabel(
        next_node,
        new_route,
        arrival_time,
        new_onboard,
        served,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - boarded_reward,
        label.route_length + 1,
    )
end
