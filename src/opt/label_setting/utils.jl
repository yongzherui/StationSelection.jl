"""
Reward-model-independent math shared by every label-setting pricer: sparse
station-age dominance mechanics (used by the aggregate and PFA pricers alike)
and a sort helper. No dependency on any pricer's own types -- see `types.jl`
for the per-state-label-list/search-context contract that builds on this, and
`engine.jl` for the search loop itself.
"""

# ── sparse station-age mechanics (dominance sub-tests, shared across pricers) ──
function _make_sparse_station_ages(
    station_age,
    node_index::Dict{Int, Int},
)::Tuple{Vector{Int32}, Vector{Float64}, UInt64}
    n_live = length(station_age)
    age_idx = Vector{Int32}(undef, n_live)
    age_val = Vector{Float64}(undef, n_live)
    age_mask = UInt64(0)
    i = 0
    @inbounds for (station, age) in station_age
        idx = Int32(node_index[station])
        age_mask |= UInt64(1) << ((idx - 1) & 63)
        j = i
        while j >= 1 && age_idx[j] > idx
            age_idx[j + 1] = age_idx[j]
            age_val[j + 1] = age_val[j]
            j -= 1
        end
        age_idx[j + 1] = idx
        age_val[j + 1] = age
        i += 1
    end
    return age_idx, age_val, age_mask
end

@inline function _sparse_station_ages_dominate(
    a_idx::Vector{Int32}, a_val::Vector{Float64}, a_mask::UInt64,
    b_idx::Vector{Int32}, b_val::Vector{Float64}, b_mask::UInt64,
)::Bool
    _sparse_station_age_support_rejection(a_idx, a_mask, b_idx, b_mask) == 0 || return false
    return _sparse_station_age_values_dominate(a_idx, a_val, b_idx, b_val)
end

"""Return 0 when support may dominate, or 1=size and 2=mask rejection."""
@inline function _sparse_station_age_support_rejection(
    a_idx::Vector{Int32}, a_mask::UInt64,
    b_idx::Vector{Int32}, b_mask::UInt64,
)::Int
    length(a_idx) >= length(b_idx) || return 1
    b_mask & ~a_mask == 0 || return 2
    return 0
end

@inline function _sparse_station_age_values_dominate(
    a_idx::Vector{Int32}, a_val::Vector{Float64},
    b_idx::Vector{Int32}, b_val::Vector{Float64},
)::Bool
    ia = 1
    na = length(a_idx)
    @inbounds for ib in eachindex(b_idx)
        idx = b_idx[ib]
        while ia <= na && a_idx[ia] < idx
            ia += 1
        end
        ia <= na && a_idx[ia] == idx || return false
        a_val[ia] <= b_val[ib] + 1e-9 || return false
    end
    return true
end

# ── bitset-weight compensation (dominance sub-test, shared across pricers) ───
"""
    _bitset_diff_weight(a_bits, b_bits, weight, budget, compensated) -> Float64

Weight of the bits `a_bits` holds that `b_bits` lacks, capped at a running
early-exit against `budget`. Reward-model-independent: works identically
whether a bit stands for a passenger reward layer
(`joint_routing_assignment/exact/types.jl`'s `activated_reward_layers`) or a
certified OD pair (`route_covering/exact/types.jl`'s `served_pairs`) --
`weight` is just "cost of holding bit `i` that the other label doesn't".

This is the "catch-up" term in the compensated dominance rule: for label `a`
to dominate `b` despite holding bits `b` lacks, `a`'s reduced-cost advantage
must cover the weight of that excess (`rc_a + w(A_a \\ A_b) <= rc_b`; see
`joint_routing_assignment/exact/dominate.jl`'s `_dominates_joint_routing_assignment_label`
docstring for the full soundness argument, which is generic in what a bit
means). Requiring `A_a ⊆ A_b` (`compensated = false`) is the special case
`w(A_a ∖ A_b) = 0`, so this only ever adds dominations relative to the plain
subset test.

Bails out as soon as the running total exceeds `budget`, since the only use is
the test `compensation <= budget`. In practice `budget` is a small reduced-cost
difference while individual weights are large, so the common failing case
exits after one bit.

# One pass, not two

The naive form runs `issubset(a, b)` first (word-wise, allocation-free) and,
when that fails, restarts with an element-wise walk that re-tests `bit in b`
one integer at a time -- so the interesting case, where `a` does hold bits `b`
lacks, traverses the bit data twice and pays a per-element `in` probe on the
second pass.

Here both are the same walk. `w = a.bits[i] & ~b.bits[j]` is the set difference
restricted to one 64-bit chunk: all-zero chunks (the subset case, still the most
common outcome) are skipped at exactly the cost `issubset` used to pay, and a
non-zero chunk is drained bit by bit with `trailing_zeros`/`w &= w - 1` right
where it was found, with no second lookup. Chunk `i` of a `BitSet` holds the
integers `((i - 1 + offset) << 6) .+ (0:63)`, which is what turns a set bit back
into its weight-vector index.
"""
function _bitset_diff_weight(
    a_bits::BitSet,
    b_bits::BitSet,
    weight::Vector{Float64},
    budget::Float64,
    compensated::Bool=true,
)::Float64
    # Static dispatch on a loop-invariant flag: the specialized method below drops
    # the branch entirely rather than re-testing it per chunk.
    return compensated ?
        _bitset_diff_weight(a_bits, b_bits, weight, budget, Val(true)) :
        _bitset_diff_weight(a_bits, b_bits, weight, budget, Val(false))
end

function _bitset_diff_weight(
    a_bits::BitSet,
    b_bits::BitSet,
    weight::Vector{Float64},
    budget::Float64,
    ::Val{Compensated},
)::Float64 where {Compensated}
    a_data = a_bits.bits
    b_data = b_bits.bits
    # Chunk `i` of `a` lines up with chunk `i - shift` of `b`.
    shift = b_bits.offset - a_bits.offset
    n_b = length(b_data)
    total = 0.0
    @inbounds for i in eachindex(a_data)
        w = a_data[i]
        w == 0 && continue
        j = i - shift
        if 1 <= j <= n_b
            w &= ~b_data[j]
            w == 0 && continue
        end
        # With compensation off this IS the plain subset rule: a non-subset never
        # dominates, which `Inf` expresses without duplicating the surrounding
        # predicate.
        Compensated || return Inf
        base = (a_bits.offset + i - 1) << 6
        while w != 0
            total += weight[base + trailing_zeros(w)]
            total > budget && return total
            w &= w - one(UInt64)  # clear the lowest set bit
        end
    end
    return total
end

# ── sort helper ──────────────────────────────────────────────────────────────
"""Sort pricing results while constructing the expensive route string once."""
function _sort_pricing_results_by_route(scored, key)
    isempty(scored) && return scored
    return scored[sortperm([key(entry) for entry in scored])]
end

# ── max_stops resolution (shared by every AggregateODRoute-family pricer) ───
"""
Resolve the finite route-length ceiling required by exhaustive enumeration.
Exhaustive enumeration (no dominance, no reduced-cost pruning) has no finite
DFS depth and cannot terminate without a finite `max_stops` -- that case is a
hard error here. Label-setting pricing does not share this requirement; see
`_resolve_aggregate_od_route_pricing_max_stops` below. Used by
`route_covering/exact/enumeration.jl`, the only exhaustive enumerator in
this package.
"""
function _resolve_aggregate_od_route_max_stops(max_stops::Int)::Int
    max_stops != typemax(Int) && return max_stops
    throw(ArgumentError(
        "AggregateODRouteProblem route search requires a finite max_stops",
    ))
end

"""
Resolve the route-length ceiling for label-setting pricing. Unlike exhaustive
enumeration, a labeling search terminates via label dominance and reduced-cost
pruning even with no depth ceiling at all -- `bounded_max_stops` already tells
the dominance/comparison code to ignore route_length when `max_stops` is
unbounded, so `route_length >= pricing_data.max_stops` at the top of the
search loop only needs to be a no-op in that case, not an error. Unbounded
`max_stops` is therefore a legitimate "run pricing with no artificial
route-length cap" configuration, not a misconfiguration. Shared by both
`route_covering/data.jl` and
`joint_routing_assignment/data.jl` -- the one piece of
pricing-data assembly genuinely common to both pricers, unlike everything
else in `route_covering/`, which has no counterpart Joint reuses.
"""
function _resolve_aggregate_od_route_pricing_max_stops(max_stops::Int)::Int
    return max_stops
end
