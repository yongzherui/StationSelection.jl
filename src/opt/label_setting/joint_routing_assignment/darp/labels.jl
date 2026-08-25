"""
Core label-DP primitives for `darp/`'s branching passenger free-assignment
pricing: label creation, extension, and dominance. `darp.jl` orchestrates
these into a full pricing pass; this file is the one to audit for "is the
label search correct". See `types.jl`'s module docstring for the reward
model and the dominance-soundness argument this file implements.

# The action type: physical move + commit choice, bundled

`_run_label_setting`'s (`engine.jl`) hook contract is one action in, one
child out (`_pricing_extend_label`) -- it has no notion of "one physical move
can fan out into several children". To get branching (see
`_joint_routing_assignment_darp_commit_subsets`, `data.jl`) without changing
that shared contract, `_joint_routing_assignment_darp_candidate_next_nodes`
below returns *actions*, not bare node ids: one
`(next_node, commit_subset)` pair per `(reachable node, subset of that node's
newly-eligible passengers to actually commit)` combination. Each action
becomes its own label via `_pricing_extend_label`, exactly the way a bare
`next_node` would for any other pricer -- branching lives entirely in how
many actions one node produces, not in how many children one action produces.
"""

export initial_joint_routing_assignment_darp_pricing_labels
export extend_joint_routing_assignment_darp_pricing_label

# ── label seeding ────────────────────────────────────────────────────────────
function initial_joint_routing_assignment_darp_pricing_labels(
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{JointRoutingAssignmentDarpPricingLabel}
    # Same reasoning as every sibling pricer's twin: a route can only ever
    # collect reward through one of a candidate's two endpoints, so seeding
    # elsewhere would waste search on routes that can never certify anything.
    endpoints = Set{Int}()
    for c in pricing_data.candidates
        push!(endpoints, c.origin)
        push!(endpoints, c.destination)
    end

    labels = JointRoutingAssignmentDarpPricingLabel[]
    for node in pricing_data.nodes
        node in endpoints || continue
        push!(labels, JointRoutingAssignmentDarpPricingLabel(
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

# ── candidate next-nodes ─────────────────────────────────────────────────────
function _has_useful_live_joint_routing_assignment_darp_origin(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Bool
    for (station, age) in label.station_age
        for idx in get(pricing_data.candidates_by_origin, station, Int[])
            c = pricing_data.candidates[idx]
            haskey(label.served, c.p) && continue
            t_to_dest = c.destination == label.current ? 0.0 :
                _joint_routing_assignment_darp_travel(pricing_data, label.current, c.destination)
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
function _joint_routing_assignment_darp_reachable_next_nodes(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{Int}
    candidate_nodes = Set{Int}()
    past_pickup_cutoff = label.time > pricing_data.max_wait_time + 1e-9
    if past_pickup_cutoff && !_has_useful_live_joint_routing_assignment_darp_origin(label, pricing_data)
        return Int[]
    end

    if !past_pickup_cutoff
        for (origin, idxs) in pricing_data.candidates_by_origin
            origin == label.current && continue
            any(!haskey(label.served, pricing_data.candidates[idx].p) for idx in idxs) || continue
            arrival_time = label.time + _joint_routing_assignment_darp_travel(pricing_data, label.current, origin)
            arrival_time <= pricing_data.max_wait_time + 1e-9 && push!(candidate_nodes, origin)
        end
    end

    # Driven from the label's *live origins*, same rationale as
    # `exact/labels.jl`'s twin: only a live origin can make a destination
    # useful, and pruning keeps the live set small.
    for (origin, origin_age) in label.station_age
        for idx in get(pricing_data.candidates_by_origin, origin, Int[])
            c = pricing_data.candidates[idx]
            c.destination == label.current && continue
            c.destination in candidate_nodes && continue
            haskey(label.served, c.p) && continue
            origin_age + _joint_routing_assignment_darp_travel(pricing_data, label.current, c.destination) <=
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
`_joint_routing_assignment_darp_commit_subsets` (`data.jl`) for the subset
enumeration itself. A node with no newly-eligible passengers still produces
exactly one action (the empty subset), so this degenerates to the old
one-action-per-node behavior wherever there is nothing to decide.
"""
function _joint_routing_assignment_darp_candidate_next_nodes(
    label::JointRoutingAssignmentDarpPricingLabel,
    pricing_data::JointRoutingAssignmentDarpPricingData,
)::Vector{JointRoutingAssignmentDarpAction}
    actions = JointRoutingAssignmentDarpAction[]
    for next_node in _joint_routing_assignment_darp_reachable_next_nodes(label, pricing_data)
        travel_time = _joint_routing_assignment_darp_travel(pricing_data, label.current, next_node)
        eligible = _joint_routing_assignment_darp_eligible_at_node(
            next_node, label.station_age, travel_time, label.served, pricing_data,
        )
        for commits in _joint_routing_assignment_darp_commit_subsets(eligible)
            push!(actions, (next_node, commits))
        end
    end
    return actions
end

# ── label extension ──────────────────────────────────────────────────────────
"""
Extension always produces exactly one child per *action* -- the branching is
in how many actions `_joint_routing_assignment_darp_candidate_next_nodes`
returns for one label, not in how many children one action produces here --
same wrapper convention as every sibling pricer's public `extend_*`
entrypoint."""
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
    next_node, commits = action
    travel_time = _joint_routing_assignment_darp_travel(pricing_data, label.current, next_node)
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
    # `exact/labels.jl`'s twin. Uses `certified` (this action's actual
    # commitments), so a skipped-but-still-eligible passenger correctly keeps
    # whatever station ages remain useful to it.
    aged_station = Dict{Int, Float64}()
    for (station, age) in label.station_age
        station == next_node && continue  # handled by the reset below
        aged = age + travel_time
        _joint_routing_assignment_darp_age_is_useful(station, aged, certified, pricing_data, next_node) &&
            (aged_station[station] = aged)
    end
    if arrival_time <= pricing_data.max_wait_time + 1e-9
        _joint_routing_assignment_darp_age_is_useful(next_node, 0.0, certified, pricing_data, next_node) &&
            (aged_station[next_node] = 0.0)
    elseif haskey(label.station_age, next_node)
        aged = label.station_age[next_node] + travel_time
        _joint_routing_assignment_darp_age_is_useful(next_node, aged, certified, pricing_data, next_node) &&
            (aged_station[next_node] = aged)
    end

    return JointRoutingAssignmentDarpPricingLabel(
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
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::JointRoutingAssignmentDarpLabelBitsets
    served_bits = BitSet(keys(label.served))
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    return JointRoutingAssignmentDarpLabelBitsets(served_bits, age_idx, age_val, age_mask)
end

JointRoutingAssignmentDarpDominanceFilters(label::JointRoutingAssignmentDarpPricingLabel, bs::JointRoutingAssignmentDarpLabelBitsets) =
    JointRoutingAssignmentDarpDominanceFilters(label.reduced_cost, label.time, bs.age_mask, Int32(label.route_length), Int32(length(bs.age_idx)))

PricingLabelEntry(id::Int, label::JointRoutingAssignmentDarpPricingLabel, bs::JointRoutingAssignmentDarpLabelBitsets) =
    PricingLabelEntry(JointRoutingAssignmentDarpDominanceFilters(label, bs), id, label, bs)

# ── dominance ─────────────────────────────────────────────────────────────────
"""
Set-based dominance test, for tests/callers working straight off labels
rather than bitsets -- the passenger-`Dict`-keyed counterpart of
`route_covering/exact/labels.jl`'s `_dominates_route_covering_label`. See
`types.jl`'s module docstring for the compensated-dominance soundness
argument (upper-bound `passenger_weight` only ever overcharges, never
undercharges, the budget test below).
"""
function _dominates_joint_routing_assignment_darp_label(
    a::JointRoutingAssignmentDarpPricingLabel,
    b::JointRoutingAssignmentDarpPricingLabel,
    bounded_max_stops::Bool;
    passenger_weight::Vector{Float64}=Float64[],
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_darp_state(a) == _joint_routing_assignment_darp_state(b) || return false
    (!bounded_max_stops || a.route_length <= b.route_length) || return false
    a.time <= b.time + 1e-9 || return false
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    compensation = 0.0
    for p in keys(a.served)
        haskey(b.served, p) && continue                     # `b` already served p too, free
        compensated_dominance || return false                # plain mode: `a`'s served set must be a subset of `b`'s
        compensation += get(passenger_weight, p, 0.0)
        compensation > budget && return false
    end
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

"""
State-scan-equivalent bitset dominance, expressed by delegating to
`_pricing_dominates_at_state` rather than reimplementing it a third time --
matching the convention `route_covering/exact/labels.jl`'s and
`joint_routing_assignment/exact/labels.jl`'s twins already follow."""
function _dominates_joint_routing_assignment_darp_label(
    a::JointRoutingAssignmentDarpPricingLabel,
    b::JointRoutingAssignmentDarpPricingLabel,
    abs::JointRoutingAssignmentDarpLabelBitsets,
    bbs::JointRoutingAssignmentDarpLabelBitsets,
    bounded_max_stops::Bool;
    weight::Vector{Float64}=Float64[],
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_darp_state(a) == _joint_routing_assignment_darp_state(b) || return false
    return _pricing_dominates_at_state(
        JointRoutingAssignmentDarpDominanceFilters(a, abs), abs,
        JointRoutingAssignmentDarpDominanceFilters(b, bbs), bbs,
        weight,
        JointRoutingAssignmentDarpDominanceRules{bounded_max_stops, compensated_dominance}(),
    )
end

"""
    _pricing_dominates_at_state(af, abs, bf, bbs, weight, rules)

The dominance predicate as the state's label-list scan calls it -- see
`route_covering/exact/labels.jl`'s twin for the full condition-ordering
rationale, which applies unchanged here. `weight` is `passenger_weight`
(`JointRoutingAssignmentDarpPricingData`), indexed exactly like `served_bits`.
"""
@inline function _pricing_dominates_at_state(
    af::JointRoutingAssignmentDarpDominanceFilters, abs::JointRoutingAssignmentDarpLabelBitsets,
    bf::JointRoutingAssignmentDarpDominanceFilters, bbs::JointRoutingAssignmentDarpLabelBitsets,
    weight::Vector{Float64},
    ::JointRoutingAssignmentDarpDominanceRules{BoundedStops, Compensated},
)::Bool where {BoundedStops, Compensated}
    af.time <= bf.time + 1e-9 || return false
    af.n_live_ages >= bf.n_live_ages || return false
    bf.age_mask & ~af.age_mask == 0 || return false
    BoundedStops && af.route_length > bf.route_length && return false
    budget = bf.reduced_cost - af.reduced_cost + 1e-9
    budget >= 0.0 || return false
    ia = 1
    na = Int(af.n_live_ages)
    @inbounds for ib in Base.OneTo(Int(bf.n_live_ages))
        idx = bbs.age_idx[ib]
        while ia <= na && abs.age_idx[ia] < idx
            ia += 1
        end
        ia <= na && abs.age_idx[ia] == idx || return false
        abs.age_val[ia] <= bbs.age_val[ib] + 1e-9 || return false
    end
    _bitset_diff_weight(abs.served_bits, bbs.served_bits, weight, budget, Val(Compensated)) <= budget || return false
    return true
end
