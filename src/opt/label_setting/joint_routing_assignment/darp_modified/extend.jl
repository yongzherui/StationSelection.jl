"""
How a label grows: which nodes are legal to visit next, bundled with which
commit-or-skip branch each visit takes
(`_joint_routing_assignment_darp_modified_candidate_next_nodes`, the
`_pricing_candidate_next_nodes` hook) and what taking one action produces
(`_extend_joint_routing_assignment_darp_modified_pricing_label`, the
`_pricing_extend_label` hook, both wired in `hooks.jl`) -- station-age
aging/reset/prune and commit-subset crediting happen here. See `seed.jl` for
where a label starts, `prune.jl` for the bound that decides whether
extending a label is worth it at all, and `dominate.jl` for what happens to
a child once it's built.

# The action type: physical move + commit choice, bundled

`_run_label_setting`'s (`engine.jl`) hook contract is one action in, one
child out (`_pricing_extend_label`) -- it has no notion of "one physical move
can fan out into several children". To get branching (see
`_joint_routing_assignment_darp_modified_commit_subsets`, `data.jl`) without
changing that shared contract,
`_joint_routing_assignment_darp_modified_candidate_next_nodes` below returns
*actions*, not bare node ids: one `(next_node, commit_subset)` pair per
`(reachable node, subset of that node's newly-eligible passengers to
actually commit)` combination. Each action becomes its own label via
`_pricing_extend_label`, exactly the way a bare `next_node` would for any
other pricer -- branching lives entirely in how many actions one node
produces, not in how many children one action produces.
"""

# ── candidate next-nodes ─────────────────────────────────────────────────────
function _has_useful_live_joint_routing_assignment_darp_modified_origin(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::Bool
    for (station, age) in label.station_age
        for idx in get(pricing_data.candidates_by_origin, station, Int[])
            c = pricing_data.candidates[idx]
            haskey(label.served, c.p) && continue
            t_to_dest = c.destination == label.current ? 0.0 :
                _joint_routing_assignment_darp_modified_travel(pricing_data, label.current, c.destination)
            age + t_to_dest <= c.ride_limit + 1e-9 || continue
            return true
        end
    end
    return false
end

"""
Physical reachability only -- which nodes `label` may legally move to next.
Unaffected by branching: whether some other label chose to commit or skip a
passenger doesn't change which stations are physically reachable, only
`label.served`, which this already reads (so a passenger left uncommitted by
an earlier "skip" branch still correctly looks eligible again here, exactly
as it should).
"""
function _joint_routing_assignment_darp_modified_reachable_next_nodes(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_joint_routing_assignment_darp_modified_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, idxs) in pricing_data.candidates_by_origin
            origin == label.current && continue
            any(!haskey(label.served, pricing_data.candidates[idx].p) for idx in idxs) || continue
            arrival_time = label.time + _joint_routing_assignment_darp_modified_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    # Driven from the label's *live origins*, same rationale as
    # `../exact/extend.jl`'s twin: only a live origin can make a destination
    # useful, and pruning keeps the live set small.
    for (origin, origin_age) in label.station_age
        for idx in get(pricing_data.candidates_by_origin, origin, Int[])
            c = pricing_data.candidates[idx]
            c.destination == label.current && continue
            c.destination in candidate_nodes && continue
            haskey(label.served, c.p) && continue
            origin_age + _joint_routing_assignment_darp_modified_travel(pricing_data, label.current, c.destination) <=
                c.ride_limit + 1e-9 || continue
            push!(candidate_nodes, c.destination)
        end
    end

    return sort!(collect(candidate_nodes))
end

"""
One `(next_node, commit_subset)` action per reachable node × every subset of
that node's newly-eligible not-yet-served passengers -- see this file's
module docstring for why actions, not children, carry the branching, and
`_joint_routing_assignment_darp_modified_commit_subsets` (`data.jl`) for the subset
enumeration itself. A node with no newly-eligible passengers still produces
exactly one action (the empty subset), so this degenerates to the old
one-action-per-node behavior wherever there is nothing to decide.
"""
function _joint_routing_assignment_darp_modified_candidate_next_nodes(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::Vector{JointRoutingAssignmentDarpModifiedAction}
    actions = JointRoutingAssignmentDarpModifiedAction[]
    for next_node in _joint_routing_assignment_darp_modified_reachable_next_nodes(label, pricing_data)
        travel_time = _joint_routing_assignment_darp_modified_travel(pricing_data, label.current, next_node)
        eligible = _joint_routing_assignment_darp_modified_eligible_at_node(
            next_node, label.station_age, travel_time, label.served, pricing_data,
        )
        for commits in _joint_routing_assignment_darp_modified_commit_subsets(eligible)
            push!(actions, (next_node, commits))
        end
    end
    return actions
end

# ── label extension ──────────────────────────────────────────────────────────
export extend_joint_routing_assignment_darp_modified_pricing_label

"""
Extension always produces exactly one child per *action* -- the branching is
in how many actions `_joint_routing_assignment_darp_modified_candidate_next_nodes`
returns for one label, not in how many children one action produces here --
same wrapper convention as every sibling pricer's public `extend_*`
entrypoint."""
function extend_joint_routing_assignment_darp_modified_pricing_label(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    action::JointRoutingAssignmentDarpModifiedAction,
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::Vector{JointRoutingAssignmentDarpModifiedPricingLabel}
    return JointRoutingAssignmentDarpModifiedPricingLabel[
        _extend_joint_routing_assignment_darp_modified_pricing_label(label, action, pricing_data),
    ]
end

function _extend_joint_routing_assignment_darp_modified_pricing_label(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    action::JointRoutingAssignmentDarpModifiedAction,
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData,
)::JointRoutingAssignmentDarpModifiedPricingLabel
    next_node, commits = action
    travel_time = _joint_routing_assignment_darp_modified_travel(pricing_data, label.current, next_node)
    arrival_time = label.time + travel_time
    new_tau = label.tau + travel_time
    new_route = vcat(label.route, next_node)

    # Apply exactly this action's chosen subset -- anyone eligible here but
    # left out of `commits` stays unserved, exactly as if this visit had
    # never certified anything for them (a deliberate "skip" branch).
    certified = copy(label.served)
    reward = 0.0
    for (p, origin, r) in commits
        certified[p] = (origin, next_node)
        reward += r
    end

    # Age, reset, and prune in one pass -- same convention as
    # `../exact/extend.jl`'s twin. Uses `certified` (this action's actual
    # commitments), so a skipped-but-still-eligible passenger correctly keeps
    # whatever station ages remain useful to it.
    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        station == next_node && continue  # handled by the reset below
        aged = age + travel_time
        _joint_routing_assignment_darp_modified_age_is_useful(station, aged, certified, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _joint_routing_assignment_darp_modified_age_is_useful(next_node, 0.0, certified, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
        aged = label.station_age[next_node] + travel_time
        _joint_routing_assignment_darp_modified_age_is_useful(next_node, aged, certified, pricing_data, next_node) &&
            (aged_station[next_node] = aged)
    end

    return JointRoutingAssignmentDarpModifiedPricingLabel(
        next_node,
        new_route,
        arrival_time,
        aged_station,
        certified,
        new_tau,
        label.reduced_cost + pricing_data.route_regularization_weight * travel_time - reward,
        label.route_length + 1,
    )
end
