"""
When is one label strictly better than another? Bitsets construction, state
key, and the dominance predicates themselves (`_pricing_dominates_fn`, wired
in `hooks.jl`) all live here. Because `visited` only grows and is part of the
dominance *state* (exact match, not subset), the predicate needs no
served-pairs comparison at all -- see `types.jl`'s module docstring for the
invariant that makes that sound; do not add one here without re-deriving it.
"""

# ── bitsets construction (hot-path dominance mirror) ─────────────────────────
function _make_route_covering_station_simple_bitsets(
    label::RouteCoveringStationSimpleLabel,
    node_index::Dict{Int, Int},
)::RouteCoveringStationSimpleBitsets
    visited_bits = BitSet()
    for node in label.visited
        push!(visited_bits, node_index[node])
    end

    age_idx, age_val, age_mask = _make_sparse_station_ages(label.live_origin_age, node_index)

    return RouteCoveringStationSimpleBitsets(visited_bits, age_idx, age_val, age_mask)
end

# ── state key ─────────────────────────────────────────────────────────────────
_route_covering_station_simple_state(
    label::RouteCoveringStationSimpleLabel,
    bs::RouteCoveringStationSimpleBitsets,
) = (label.current, bs.visited_bits)

# ── dominance predicates ──────────────────────────────────────────────────────
function _dominates_route_covering_station_simple_label(
    a::RouteCoveringStationSimpleLabel,
    b::RouteCoveringStationSimpleLabel,
    abs::RouteCoveringStationSimpleBitsets,
    bbs::RouteCoveringStationSimpleBitsets,
)::Bool
    a.current == b.current || return false
    abs.visited_bits == bbs.visited_bits || return false
    a.reduced_cost <= b.reduced_cost + 1e-9 || return false
    a.time <= b.time + 1e-9 || return false
    # dom(b) ⊆ dom(a) and age_a(j) <= age_b(j) for j in dom(b) -- shared with the
    # revisit-tolerant bitset dominance and the PFA station-simple pricer via
    # `label_setting/utils.jl`.
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end

RouteCoveringStationSimpleDominanceFilters(
    label::RouteCoveringStationSimpleLabel, ::RouteCoveringStationSimpleBitsets,
) = RouteCoveringStationSimpleDominanceFilters(
    label.reduced_cost, label.time, Int32(length(label.route)),
)

PricingLabelEntry(id::Int, label::RouteCoveringStationSimpleLabel, bs::RouteCoveringStationSimpleBitsets) =
    PricingLabelEntry(RouteCoveringStationSimpleDominanceFilters(label, bs), id, label, bs)

"""
State-scan fast path for `_dominates_route_covering_station_simple_label`:
identical dominance test, minus the `current`/`visited_bits` check, which the
state itself already guarantees for every pair this is called on (see the
4-argument method's docstring above, and `_dominates_joint_routing_assignment_at_state`
in `joint_routing_assignment/exact/dominate.jl` for the same convention on the other
pricer).
"""
@inline function _pricing_dominates_at_state(
    af::RouteCoveringStationSimpleDominanceFilters, abs::RouteCoveringStationSimpleBitsets,
    bf::RouteCoveringStationSimpleDominanceFilters, bbs::RouteCoveringStationSimpleBitsets,
    ::RouteCoveringStationSimpleDominanceRules,
)::Bool
    af.time <= bf.time + 1e-9 || return false
    af.reduced_cost <= bf.reduced_cost + 1e-9 || return false
    _sparse_station_ages_dominate(
        abs.age_idx, abs.age_val, abs.age_mask, bbs.age_idx, bbs.age_val, bbs.age_mask,
    ) || return false
    return true
end
