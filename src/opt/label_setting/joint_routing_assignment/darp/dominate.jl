"""
When is one label strictly better than another? State/order keys, the
per-label bitsets mirror the scan needs, and the dominance predicates
themselves (`_pricing_dominates_fn`, wired in `hooks.jl`) all live here. See
`types.jl`'s module docstring for why `onboard` dominance runs the opposite
support direction from every other pricer's station-age twin, and why
dominance here is unconditionally plain/exact (no `Compensated` parameter).
"""

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

# ── dominance predicates ──────────────────────────────────────────────────────
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
rather than bitsets -- the counterpart of `darp_modified/dominate.jl`'s
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
