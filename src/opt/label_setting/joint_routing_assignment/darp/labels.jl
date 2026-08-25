"""
Core label-DP primitives for `darp/`'s onboard-bitset pricing: label
creation, extension, and dominance. `darp.jl` orchestrates these into a full
pricing pass; this file is the one to audit for "is the label search
correct". See `types.jl`'s module docstring for the reward model,
no-`station_age` rationale, and dominance argument this file implements.
"""

export initial_joint_routing_assignment_darp_pricing_labels
export extend_joint_routing_assignment_darp_pricing_label

# ── label seeding ────────────────────────────────────────────────────────────
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

# ── state key / order key ────────────────────────────────────────────────────
_joint_routing_assignment_darp_state(label::JointRoutingAssignmentDarpPricingLabel) = label.current

function _joint_routing_assignment_darp_label_order_key(
    label::JointRoutingAssignmentDarpPricingLabel,
    label_id::JointRoutingAssignmentDarpLabelId,
)::JointRoutingAssignmentDarpLabelOrderKey
    return (label.reduced_cost, label.time, label.route_length, label_id)
end

# ── bitsets construction (hot-path dominance mirror) ─────────────────────────
function _make_joint_routing_assignment_darp_label_bitsets(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::JointRoutingAssignmentDarpLabelBitsets
    served_bits = BitSet(pricing_data.candidate_index[t] for t in label.served)

    n = length(label.onboard)
    onboard_bits = BitSet()
    age_idx = Vector{Int32}(undef, n)
    age_val = Vector{Float64}(undef, n)
    age_mask = UInt64(0)
    i = 0
    for (p, (j, k, age)) in label.onboard
        idx = Int32(pricing_data.candidate_index[(p, j, k)])
        push!(onboard_bits, idx)
        age_mask |= UInt64(1) << ((idx - 1) & 63)
        pos = i
        while pos >= 1 && age_idx[pos] > idx
            age_idx[pos + 1] = age_idx[pos]
            age_val[pos + 1] = age_val[pos]
            pos -= 1
        end
        age_idx[pos + 1] = idx
        age_val[pos + 1] = age
        i += 1
    end

    return JointRoutingAssignmentDarpLabelBitsets(served_bits, onboard_bits, age_idx, age_val, age_mask)
end

JointRoutingAssignmentDarpDominanceFilters(label::JointRoutingAssignmentDarpPricingLabel, bs::JointRoutingAssignmentDarpLabelBitsets) =
    JointRoutingAssignmentDarpDominanceFilters(
        label.reduced_cost, label.time, bs.onboard_age_mask,
        Int32(label.route_length), Int32(length(bs.onboard_age_idx)),
    )

PricingLabelEntry(id::Int, label::JointRoutingAssignmentDarpPricingLabel, bs::JointRoutingAssignmentDarpLabelBitsets) =
    PricingLabelEntry(JointRoutingAssignmentDarpDominanceFilters(label, bs), id, label, bs)

# ── dominance ─────────────────────────────────────────────────────────────────
"""
Onboard-commitment age dominance: the *opposite* support direction from
`station_age`'s (`label_setting/utils.jl`'s `_sparse_station_ages_dominate`)
-- an onboard commitment is a liability, not an opportunity, so `a`'s support
must be a SUBSET of `b`'s (fewer or equal obligations), not a superset, while
the value requirement (`a`'s ages no older, on whatever's shared) stays the
same sense. That utility couples "bigger support" with "fresher values" in
one fixed direction, which doesn't fit here, so this is its own small
merge-walk rather than a reuse -- see `types.jl`'s module docstring.
"""
@inline function _joint_routing_assignment_darp_onboard_ages_dominate(
    a_idx::Vector{Int32}, a_val::Vector{Float64}, a_mask::UInt64, n_a::Int32,
    b_idx::Vector{Int32}, b_val::Vector{Float64}, b_mask::UInt64, n_b::Int32,
)::Bool
    n_a <= n_b || return false
    a_mask & ~b_mask == 0 || return false   # a's support must be a subset of b's
    ib = 1
    nb = Int(n_b)
    @inbounds for ia in 1:Int(n_a)
        idx = a_idx[ia]
        while ib <= nb && b_idx[ib] < idx
            ib += 1
        end
        ib <= nb && b_idx[ib] == idx || return false
        a_val[ia] <= b_val[ib] + 1e-9 || return false
    end
    return true
end

"""
Set-based dominance test, for tests/callers working straight off labels
rather than bitsets -- the counterpart of `darp_modified/labels.jl`'s
twin, adapted for `served`/`onboard`'s plain (not compensated) subset rule
and `onboard`'s age requirement."""
function _dominates_joint_routing_assignment_darp_label(
    a::JointRoutingAssignmentDarpPricingLabel,
    b::JointRoutingAssignmentDarpPricingLabel,
    bounded_max_stops::Bool,
)::Bool
    _joint_routing_assignment_darp_state(a) == _joint_routing_assignment_darp_state(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    issubset(a.served, b.served) || return false
    length(a.onboard) <= length(b.onboard) || return false
    for (p, (j, k, age_a)) in a.onboard
        current = get(b.onboard, p, nothing)
        isnothing(current) && return false
        (bj, bk, age_b) = current
        (bj == j && bk == k) || return false
        age_a <= age_b + 1e-9 || return false
    end
    return true
end

"""
State-scan-equivalent bitset dominance, expressed by delegating to
`_pricing_dominates_at_state` rather than reimplementing it a third time --
matching the convention every sibling pricer's twin already follows."""
function _dominates_joint_routing_assignment_darp_label(
    a::JointRoutingAssignmentDarpPricingLabel,
    b::JointRoutingAssignmentDarpPricingLabel,
    abs::JointRoutingAssignmentDarpLabelBitsets,
    bbs::JointRoutingAssignmentDarpLabelBitsets,
    bounded_max_stops::Bool,
)::Bool
    _joint_routing_assignment_darp_state(a) == _joint_routing_assignment_darp_state(b) || return false
    return _pricing_dominates_at_state(
        JointRoutingAssignmentDarpDominanceFilters(a, abs), abs,
        JointRoutingAssignmentDarpDominanceFilters(b, bbs), bbs,
        JointRoutingAssignmentDarpDominanceRules{bounded_max_stops}(),
    )
end

"""
    _pricing_dominates_at_state(af, abs, bf, bbs, rules)

The dominance predicate as the state's label-list scan calls it: time, route
length (if bounded), plain `served_bits` subset, then the onboard-age
merge-walk above. No `weight` argument, unlike every sibling pricer's twin --
there is nothing to compensate here (see `types.jl`'s module docstring)."""
@inline function _pricing_dominates_at_state(
    af::JointRoutingAssignmentDarpDominanceFilters, abs::JointRoutingAssignmentDarpLabelBitsets,
    bf::JointRoutingAssignmentDarpDominanceFilters, bbs::JointRoutingAssignmentDarpLabelBitsets,
    ::JointRoutingAssignmentDarpDominanceRules{BoundedStops},
)::Bool where {BoundedStops}
    af.time <= bf.time + 1e-9 || return false
    BoundedStops && af.route_length > bf.route_length && return false
    af.reduced_cost <= bf.reduced_cost + 1e-9 || return false
    issubset(abs.served_bits, bbs.served_bits) || return false
    _joint_routing_assignment_darp_onboard_ages_dominate(
        abs.onboard_age_idx, abs.onboard_age_val, af.onboard_age_mask, af.n_onboard,
        bbs.onboard_age_idx, bbs.onboard_age_val, bf.onboard_age_mask, bf.n_onboard,
    ) || return false
    return true
end
