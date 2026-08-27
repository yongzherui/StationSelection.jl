"""
When is one label strictly better than another? State/order keys, the
per-label bitsets mirror the scan needs, and the dominance predicates
themselves (`_pricing_dominates_fn`, wired in `hooks.jl`) all live here. See
`types.jl`'s module docstring for the full compensated-dominance soundness
argument (upper-bound `passenger_weight` only ever overcharges, never
undercharges, the budget test below).
"""

# ── state key / order key ────────────────────────────────────────────────────
_joint_routing_assignment_darp_modified_state(label::JointRoutingAssignmentDarpModifiedPricingLabel) = label.current

function _joint_routing_assignment_darp_modified_label_order_key(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    label_id::JointRoutingAssignmentDarpModifiedLabelId,
)::JointRoutingAssignmentDarpModifiedLabelOrderKey
    return (label.reduced_cost, label.time, label.route_length, label_id)
end

# ── bitsets construction (hot-path dominance mirror) ─────────────────────────
function _make_joint_routing_assignment_darp_modified_label_bitsets(
    label::JointRoutingAssignmentDarpModifiedPricingLabel,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::JointRoutingAssignmentDarpModifiedLabelBitsets
    served_bits = BitSet(keys(label.served))
    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)
    return JointRoutingAssignmentDarpModifiedLabelBitsets(served_bits, age_idx, age_val, age_mask)
end

JointRoutingAssignmentDarpModifiedDominanceFilters(label::JointRoutingAssignmentDarpModifiedPricingLabel, bs::JointRoutingAssignmentDarpModifiedLabelBitsets) =
    JointRoutingAssignmentDarpModifiedDominanceFilters(label.reduced_cost, label.time, bs.age_mask, Int32(label.route_length), Int32(length(bs.age_idx)))

PricingLabelEntry(id::Int, label::JointRoutingAssignmentDarpModifiedPricingLabel, bs::JointRoutingAssignmentDarpModifiedLabelBitsets) =
    PricingLabelEntry(JointRoutingAssignmentDarpModifiedDominanceFilters(label, bs), id, label, bs)

# ── dominance predicates ──────────────────────────────────────────────────────
"""
Set-based dominance test, for tests/callers working straight off labels
rather than bitsets -- the passenger-`Dict`-keyed counterpart of
`route_covering/exact/dominate.jl`'s `_dominates_route_covering_label`. See
`types.jl`'s module docstring for the compensated-dominance soundness
argument (upper-bound `passenger_weight` only ever overcharges, never
undercharges, the budget test below).
"""
function _dominates_joint_routing_assignment_darp_modified_label(
    a::JointRoutingAssignmentDarpModifiedPricingLabel,
    b::JointRoutingAssignmentDarpModifiedPricingLabel,
    bounded_max_stops::Bool;
    passenger_weight::Vector{Float64}=Float64[],
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_darp_modified_state(a) == _joint_routing_assignment_darp_modified_state(b) || return false
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
matching the convention `route_covering/exact/dominate.jl`'s and
`joint_routing_assignment/exact/dominate.jl`'s twins already follow."""
function _dominates_joint_routing_assignment_darp_modified_label(
    a::JointRoutingAssignmentDarpModifiedPricingLabel,
    b::JointRoutingAssignmentDarpModifiedPricingLabel,
    abs::JointRoutingAssignmentDarpModifiedLabelBitsets,
    bbs::JointRoutingAssignmentDarpModifiedLabelBitsets,
    bounded_max_stops::Bool;
    weight::Vector{Float64}=Float64[],
    compensated_dominance::Bool=true,
)::Bool
    _joint_routing_assignment_darp_modified_state(a) == _joint_routing_assignment_darp_modified_state(b) || return false
    return _pricing_dominates_at_state(
        JointRoutingAssignmentDarpModifiedDominanceFilters(a, abs), abs,
        JointRoutingAssignmentDarpModifiedDominanceFilters(b, bbs), bbs,
        weight,
        JointRoutingAssignmentDarpModifiedDominanceRules{bounded_max_stops, compensated_dominance}(),
    )
end

"""
    _pricing_dominates_at_state(af, abs, bf, bbs, weight, rules)

The dominance predicate as the state's label-list scan calls it -- see
`route_covering/exact/dominate.jl`'s twin for the full condition-ordering
rationale, which applies unchanged here. `weight` is `passenger_weight`
(`JointRoutingAssignmentDarpModifiedPricingData`), indexed exactly like `served_bits`.
"""
@inline function _pricing_dominates_at_state(
    af::JointRoutingAssignmentDarpModifiedDominanceFilters, abs::JointRoutingAssignmentDarpModifiedLabelBitsets,
    bf::JointRoutingAssignmentDarpModifiedDominanceFilters, bbs::JointRoutingAssignmentDarpModifiedLabelBitsets,
    weight::Vector{Float64},
    ::JointRoutingAssignmentDarpModifiedDominanceRules{BoundedStops, Compensated},
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
