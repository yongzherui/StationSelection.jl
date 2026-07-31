"""
Sublinear dominance bucket for the revisit-tolerant passenger pricer.

The default bucket (`labels.jl`) is a `Vector` sorted by `(reduced_cost, time, ...)`,
walked in full on every insertion -- O(bucket), and ~85-90% of pricing wall time.
Two of the dominance conditions are the scalars `rc_a <= rc_b` and `time_a <= time_b`
(the compensation term is >= 0, so `rc_a <= rc_b` is necessary); that is a 2-D
dominance relation, which a 1-D sorted vector cannot skip on.

This is a treap (randomized balanced BST) keyed by `(rc, time, id)` whose every node
also carries the MIN and MAX `time` over its subtree. That augmentation turns the two
queries `_add_..._to_treap!` performs into pruned descents:

  * "am I dominated?"  -> descend the `rc' <= rc, time' <= time` region, pruning
    subtrees with `submin > time`; run the full predicate only on survivors, short-
    circuit on the first real dominator. O(log n + #candidates).
  * "whom do I dominate?" -> descend the `rc' >= rc, time' >= time` region, pruning
    subtrees with `submax < time`. O(log n + #candidates).

`(rc, time)` is a *necessary* condition for full dominance, so no real dominator is
ever skipped -- the expensive bitset/age predicate runs only on the handful of
`(rc, time)`-survivors instead of the whole bucket. Priorities come from a
fixed-seed RNG so the structure (and therefore results) are deterministic.

Prototyped and measured standalone in `scripts/prototype_dominance_index.jl`: the
crossover where this beats the `Vector` on the COMBINED (query + insert) cost is
~bucket 10-20k, i.e. exactly the per-`current` bucket sizes the revisit pricer hits
at n >= 25. Selected via `dominance_index=:treap`; `:vector` (default) is unchanged.
"""

mutable struct PassengerFreeAssignmentTreapNode
    id::Int
    rc::Float64
    time::Float64
    label::PassengerFreeAssignmentPricingLabel
    bitsets::PassengerFreeAssignmentLabelBitsets
    prio::UInt64
    left::Union{PassengerFreeAssignmentTreapNode, Nothing}
    right::Union{PassengerFreeAssignmentTreapNode, Nothing}
    submin::Float64
    submax::Float64
end

mutable struct PassengerFreeAssignmentDominanceTreap
    root::Union{PassengerFreeAssignmentTreapNode, Nothing}
    rng::MersenneTwister
    evict::Vector{PassengerFreeAssignmentTreapNode}
end
_create_passenger_free_assignment_dominance_treap() =
    PassengerFreeAssignmentDominanceTreap(nothing, MersenneTwister(0x5eed), PassengerFreeAssignmentTreapNode[])

@inline _tsubmin(nd::Union{PassengerFreeAssignmentTreapNode, Nothing}) = nd === nothing ? Inf : nd.submin
@inline _tsubmax(nd::Union{PassengerFreeAssignmentTreapNode, Nothing}) = nd === nothing ? -Inf : nd.submax

@inline function _tupdate!(nd::PassengerFreeAssignmentTreapNode)
    nd.submin = min(nd.time, _tsubmin(nd.left), _tsubmin(nd.right))
    nd.submax = max(nd.time, _tsubmax(nd.left), _tsubmax(nd.right))
    return nd
end

@inline _tkey(nd::PassengerFreeAssignmentTreapNode) = (nd.rc, nd.time, nd.id)

function _trotate_right(y::PassengerFreeAssignmentTreapNode)
    x = y.left::PassengerFreeAssignmentTreapNode
    y.left = x.right; x.right = y
    _tupdate!(y); _tupdate!(x); return x
end
function _trotate_left(x::PassengerFreeAssignmentTreapNode)
    y = x.right::PassengerFreeAssignmentTreapNode
    x.right = y.left; y.left = x
    _tupdate!(x); _tupdate!(y); return y
end

function _tinsert(nd::Union{PassengerFreeAssignmentTreapNode, Nothing}, new::PassengerFreeAssignmentTreapNode)
    nd === nothing && return new
    if _tkey(new) < _tkey(nd)
        nd.left = _tinsert(nd.left, new)
        (nd.left::PassengerFreeAssignmentTreapNode).prio < nd.prio && (nd = _trotate_right(nd))
    else
        nd.right = _tinsert(nd.right, new)
        (nd.right::PassengerFreeAssignmentTreapNode).prio < nd.prio && (nd = _trotate_left(nd))
    end
    return _tupdate!(nd)
end

function _tdelete(nd::Union{PassengerFreeAssignmentTreapNode, Nothing}, k)
    nd === nothing && return nothing
    if k < _tkey(nd)
        nd.left = _tdelete(nd.left, k)
    elseif k > _tkey(nd)
        nd.right = _tdelete(nd.right, k)
    else
        nd.left === nothing && return nd.right
        nd.right === nothing && return nd.left
        if (nd.left::PassengerFreeAssignmentTreapNode).prio < (nd.right::PassengerFreeAssignmentTreapNode).prio
            nd = _trotate_right(nd); nd.right = _tdelete(nd.right, k)
        else
            nd = _trotate_left(nd); nd.left = _tdelete(nd.left, k)
        end
    end
    nd === nothing ? nothing : _tupdate!(nd)
end

# Is the incoming label (rc R, time T) dominated by any stored node? Descends the
# `rc' <= R, time' <= T` region (mirrors `_each_below_left`), pruning by `submin`,
# and short-circuits on the first node that actually dominates it.
function _treap_has_dominator(
    nd::Union{PassengerFreeAssignmentTreapNode, Nothing}, R::Float64, T::Float64,
    blabel::PassengerFreeAssignmentPricingLabel, bbs::PassengerFreeAssignmentLabelBitsets,
    lw::Vector{Float64}, bms::Bool, bds::Bool, cd::Bool,
)::Bool
    nd === nothing && return false
    nd.submin > T && return false
    _treap_has_dominator(nd.left, R, T, blabel, bbs, lw, bms, bds, cd) && return true
    if nd.rc <= R
        (nd.time <= T && _dominates_passenger_free_assignment_label(
            nd.label, blabel, nd.bitsets, bbs, lw, bms, bds, cd)) && return true
        _treap_has_dominator(nd.right, R, T, blabel, bbs, lw, bms, bds, cd) && return true
    end
    return false
end

# Collect stored nodes the incoming label dominates. Descends `rc' >= R, time' >= T`
# (mirrors `_each_above_right`), pruning by `submax`.
function _treap_collect_dominated(
    nd::Union{PassengerFreeAssignmentTreapNode, Nothing}, R::Float64, T::Float64,
    blabel::PassengerFreeAssignmentPricingLabel, bbs::PassengerFreeAssignmentLabelBitsets,
    lw::Vector{Float64}, bms::Bool, bds::Bool, cd::Bool,
    out::Vector{PassengerFreeAssignmentTreapNode},
)
    nd === nothing && return
    nd.submax < T && return
    if nd.rc >= R
        (nd.time >= T && _dominates_passenger_free_assignment_label(
            blabel, nd.label, bbs, nd.bitsets, lw, bms, bds, cd)) && push!(out, nd)
        _treap_collect_dominated(nd.left, R, T, blabel, bbs, lw, bms, bds, cd, out)
    end
    _treap_collect_dominated(nd.right, R, T, blabel, bbs, lw, bms, bds, cd, out)
end

"""
Treap counterpart of `_add_passenger_free_assignment_label_to_bucket!`, with the
identical signature and `(inserted, n_removed)` contract so the two are drop-in
interchangeable in the enumerate loop. `dominated` is accepted for signature parity
and unused (the treap keeps its own eviction scratch).
"""
function _add_passenger_free_assignment_label_to_treap!(
    treap::PassengerFreeAssignmentDominanceTreap,
    live_labels::Dict{Int, PassengerFreeAssignmentPricingLabel},
    label::PassengerFreeAssignmentPricingLabel,
    label_id::Int,
    label_bs::PassengerFreeAssignmentLabelBitsets,
    layer_weight::Vector{Float64},
    bounded_max_stops::Bool,
    bounded_distinct_stations::Bool,
    compensated_dominance::Bool,
    dominated::Vector{Int},
)
    R = label.reduced_cost
    T = label.time
    # 1. rejected if any stored label dominates the newcomer.
    _treap_has_dominator(treap.root, R, T, label, label_bs, layer_weight,
        bounded_max_stops, bounded_distinct_stations, compensated_dominance) && return false, 0

    # 2. evict every stored label the newcomer dominates.
    evict = treap.evict
    empty!(evict)
    _treap_collect_dominated(treap.root, R, T, label, label_bs, layer_weight,
        bounded_max_stops, bounded_distinct_stations, compensated_dominance, evict)
    n_removed = length(evict)
    for nd in evict
        delete!(live_labels, nd.id)
        treap.root = _tdelete(treap.root, _tkey(nd))
    end

    # 3. insert the newcomer.
    node = PassengerFreeAssignmentTreapNode(
        label_id, R, T, label, label_bs, rand(treap.rng, UInt64), nothing, nothing, T, T)
    treap.root = _tinsert(treap.root, node)
    return true, n_removed
end
