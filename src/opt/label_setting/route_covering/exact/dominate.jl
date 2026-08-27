"""
When is one label strictly better than another? State/order keys, the
per-label bitsets mirror the scan needs, and the dominance predicates
themselves (`_pricing_dominates_fn`, wired in `hooks.jl`) all live here. The
reward-diff ("catch-up"/compensated) sub-test itself is generic across
pricers and lives in `label_setting/utils.jl`'s `_bitset_diff_weight` -- this
file calls it, rather than wrapping it in a named pricer-local function the
way `joint_routing_assignment/exact/dominate.jl` does, since this pricer has
no extra vocabulary (served pairs are already "pairs", no "reward layer"
translation needed).
"""

_route_covering_state(label::RouteCoveringPricingLabel) = label.current

function _route_covering_label_order_key(
    label::RouteCoveringPricingLabel,
    label_id::RouteCoveringLabelId,
)::RouteCoveringLabelOrderKey
    return (
        label.reduced_cost,
        label.time,
        label.route_length,
        label_id,
    )
end

# ── bitsets construction (hot-path dominance mirror) ─────────────────────────
function _make_route_covering_label_bitsets(
    label::RouteCoveringPricingLabel,
    pair_index::Dict{Tuple{Int, Int}, Int},
    n_pairs::Int,
    node_index::Dict{Int, Int},
    n_nodes::Int,
)::RouteCoveringLabelBitsets
    served_bits = BitSet()
    for pair in label.served_pairs
        push!(served_bits, pair_index[pair])
    end

    age_idx, age_val, age_mask = _make_sparse_station_ages(label.station_age, node_index)

    return RouteCoveringLabelBitsets(served_bits, age_idx, age_val, age_mask)
end

RouteCoveringDominanceFilters(label::RouteCoveringPricingLabel, bs::RouteCoveringLabelBitsets) =
    RouteCoveringDominanceFilters(label.reduced_cost, label.time, bs.age_mask,
        Int32(label.route_length), Int32(length(bs.age_idx)))

PricingLabelEntry(id::Int, label::RouteCoveringPricingLabel, bs::RouteCoveringLabelBitsets) =
    PricingLabelEntry(RouteCoveringDominanceFilters(label, bs), id, label, bs)

# ── dominance predicates ──────────────────────────────────────────────────────
function _dominates_route_covering_label(
    a::RouteCoveringPricingLabel,
    b::RouteCoveringPricingLabel,
    bounded_max_stops::Bool;
    pair_weight::Dict{Tuple{Int, Int}, Float64}=Dict{Tuple{Int, Int}, Float64}(),
    compensated_dominance::Bool=true,
)::Bool
    _route_covering_state(a) == _route_covering_state(b) || return false          # must share current node
    (!bounded_max_stops || a.route_length <= b.route_length) || return false      # a can't have used more stops
    a.time <= b.time + 1e-9 || return false                                       # a can't be running later
    # `a` is allowed a strictly better (lower) reduced cost than `b` -- that
    # surplus is exactly the "budget" `a` can spend below to still dominate
    # despite otherwise-worse served-pairs coverage (see the loop below).
    budget = b.reduced_cost - a.reduced_cost + 1e-9
    budget >= 0.0 || return false
    # See `_pricing_dominates_at_state` below for the compensated-vs-plain-subset
    # soundness argument; this is its `Set`-based counterpart for tests/callers
    # working straight off labels rather than bitsets.
    compensation = 0.0
    for pair in a.served_pairs
        pair in b.served_pairs && continue                    # `b` already has this pair too, free
        compensated_dominance || return false                 # plain mode: `a` must be a subset of `b`, full stop
        compensation += get(pair_weight, pair, 0.0)            # compensated mode: charge `a`'s reduced-cost budget
        compensation > budget && return false                 # for holding a pair `b` lacks
    end
    # Every station either label has a live pickup clock for: `a`'s clock
    # can't be older than `b`'s (an older clock is strictly worse -- less
    # time left before the ride limit), on pain of not dominating.
    all_stations = union(keys(a.station_age), keys(b.station_age), (a.current, b.current))
    for station in all_stations
        get(a.station_age, station, Inf) <= get(b.station_age, station, Inf) + 1e-9 || return false
    end
    return true
end

"""
State-scan-equivalent bitset dominance, expressed by delegating to
`_pricing_dominates_at_state` rather than reimplementing it a third time --
matching the convention `joint_routing_assignment/exact/dominate.jl`'s twin already
follows. The state check stays here (label-level, cheap, and outside what
`_pricing_dominates_at_state` tests, since a state's label-list scan already
guarantees it for every pair it's called on)."""
function _dominates_route_covering_label(
    a::RouteCoveringPricingLabel,
    b::RouteCoveringPricingLabel,
    abs::RouteCoveringLabelBitsets,
    bbs::RouteCoveringLabelBitsets,
    bounded_max_stops::Bool;
    weight::Vector{Float64}=Float64[],
    compensated_dominance::Bool=true,
)::Bool
    _route_covering_state(a) == _route_covering_state(b) || return false
    return _pricing_dominates_at_state(
        RouteCoveringDominanceFilters(a, abs), abs,
        RouteCoveringDominanceFilters(b, bbs), bbs,
        weight,
        RouteCoveringDominanceRules{bounded_max_stops, compensated_dominance}(),
    )
end

"""
    _pricing_dominates_at_state(af, abs, bf, bbs, weight, rules)

The dominance predicate as the state's label-list scan calls it (see
`joint_routing_assignment/exact/dominate.jl`'s twin for the full condition-ordering
rationale, which applies unchanged here). `weight` is the per-pair reward
(`RouteCoveringSearchContext.positive_pair_rewards`, indexed exactly like
`served_bits`) the reward-diff test in `_bitset_diff_weight`
(`label_setting/utils.jl`) charges `a` for holding pairs `b` lacks.
"""
@inline function _pricing_dominates_at_state(
    af::RouteCoveringDominanceFilters, abs::RouteCoveringLabelBitsets,
    bf::RouteCoveringDominanceFilters, bbs::RouteCoveringLabelBitsets,
    weight::Vector{Float64},
    ::RouteCoveringDominanceRules{BoundedStops, Compensated},
)::Bool where {BoundedStops, Compensated}
    af.time <= bf.time + 1e-9 || return false                          # a can't be running later
    af.n_live_ages >= bf.n_live_ages || return false                   # cheap prefilter before the real support check below
    bf.age_mask & ~af.age_mask == 0 || return false                    # b's live stations must be a subset of a's (folded-bit prefilter)
    BoundedStops && af.route_length > bf.route_length && return false  # a can't have used more stops (only checked if the cap is finite)
    budget = bf.reduced_cost - af.reduced_cost + 1e-9                  # a's reduced-cost surplus over b, spendable below
    budget >= 0.0 || return false
    # Sorted-merge walk: for every station b has a live clock at (age_idx is
    # ascending in both), the matching entry in a must exist and be no older.
    # `ia` only ever advances, never resets, since both arrays are sorted --
    # this is what makes the whole check O(n_live_ages) instead of O(n^2).
    ia = 1
    na = Int(af.n_live_ages)
    @inbounds for ib in Base.OneTo(Int(bf.n_live_ages))
        idx = bbs.age_idx[ib]
        while ia <= na && abs.age_idx[ia] < idx
            ia += 1
        end
        ia <= na && abs.age_idx[ia] == idx || return false  # b has a live station a doesn't -- a can't dominate
        abs.age_val[ia] <= bbs.age_val[ib] + 1e-9 || return false  # a's clock at this station must be no older
    end
    # Final, most expensive check last: does a's served-pairs surplus over b
    # (weighted by dual reward) fit inside the reduced-cost budget computed
    # above? `Val(Compensated)` picks the compensated-vs-plain-subset rule at
    # compile time (see `_bitset_diff_weight`, `label_setting/utils.jl`).
    _bitset_diff_weight(abs.served_bits, bbs.served_bits, weight, budget, Val(Compensated)) <= budget || return false
    return true
end
