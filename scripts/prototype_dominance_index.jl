"""
    scripts/prototype_dominance_index.jl   (standalone; run with `julia this.jl`)

Prototype for attacking the dominance scan mechanically. The passenger pricers keep
each bucket as a `Vector` sorted by `(reduced_cost, time, ...)` and, on every label
insertion, walk the whole bucket -- O(bucket) -- which profiling puts at ~85-90% of
pricing wall time and which blows up at n>=25 where buckets are large.

Dominance requires (among other, harder conditions) BOTH `rc_a <= rc_b` and
`time_a <= time_b`. Those two scalars are a 2-D dominance (Pareto) relation, and a
1-D sorted vector cannot skip on a 2-D order -- hence the linear scan. This builds a
structure that CAN:

  an rc-keyed treap (randomized balanced BST) whose every node also stores the MIN
  and MAX `time` over its subtree. That augmentation turns the two queries the
  scan actually performs into pruned descents:

    * `any_below_left(rc, time)`  -- "is there a stored point with rc' <= rc AND
      time' <= time?"  The sound fast-reject for "am I dominated?": O(log n).
    * `each_below_left(rc, time, f)` / `each_above_right(rc, time, f)` -- enumerate
      only the points that pass the (rc, time) necessary-condition filter, pruning
      whole subtrees via submin/submax. Output-sensitive: O(log n + #hits), i.e.
      sublinear whenever few points pass the filter (the common case).

The (rc, time) test is a *necessary condition* for full dominance, so it never skips
a real dominator (sound); the remaining bitset/age checks run only on the few
survivors instead of the whole bucket.

This file: (1) the structure, (2) a brute-force reference, (3) a randomized
correctness cross-check, (4) a scaling benchmark vs the linear-vector scan.
"""

using Random, Printf, DataStructures
import Base: insert!, delete!   # extend (not shadow) so the Vector methods still resolve

# ─────────────────────────────────────────────────────────────────────────────
# Augmented treap: BST keyed by (rc, time, id); heap-balanced by random priority;
# each node carries submin/submax = min/max `time` over its subtree.
# ─────────────────────────────────────────────────────────────────────────────

mutable struct Node
    id::Int
    rc::Float64
    time::Float64
    prio::UInt64
    left::Union{Node, Nothing}
    right::Union{Node, Nothing}
    submin::Float64
    submax::Float64
end
Node(id, rc, time, prio) = Node(id, rc, time, prio, nothing, nothing, time, time)

mutable struct RcTimeIndex
    root::Union{Node, Nothing}
    n::Int
    rng::MersenneTwister
end
RcTimeIndex(seed::Int=1) = RcTimeIndex(nothing, 0, MersenneTwister(seed))

_key(nd::Node) = (nd.rc, nd.time, nd.id)

@inline _submin(nd::Union{Node, Nothing}) = nd === nothing ? Inf : nd.submin
@inline _submax(nd::Union{Node, Nothing}) = nd === nothing ? -Inf : nd.submax

@inline function _update!(nd::Node)
    nd.submin = min(nd.time, _submin(nd.left), _submin(nd.right))
    nd.submax = max(nd.time, _submax(nd.left), _submax(nd.right))
    return nd
end

function _rotate_right(y::Node)
    x = y.left::Node
    y.left = x.right
    x.right = y
    _update!(y); _update!(x)
    return x
end

function _rotate_left(x::Node)
    y = x.right::Node
    x.right = y.left
    y.left = x
    _update!(x); _update!(y)
    return y
end

function _insert(nd::Union{Node, Nothing}, new::Node)
    nd === nothing && return new
    if _key(new) < _key(nd)
        nd.left = _insert(nd.left, new)
        (nd.left::Node).prio < nd.prio && (nd = _rotate_right(nd))
    else
        nd.right = _insert(nd.right, new)
        (nd.right::Node).prio < nd.prio && (nd = _rotate_left(nd))
    end
    return _update!(nd)
end

function insert!(idx::RcTimeIndex, id::Int, rc::Float64, time::Float64)
    idx.root = _insert(idx.root, Node(id, rc, time, rand(idx.rng, UInt64)))
    idx.n += 1
    return idx
end

function _delete(nd::Union{Node, Nothing}, k)
    nd === nothing && return nothing
    if k < _key(nd)
        nd.left = _delete(nd.left, k)
    elseif k > _key(nd)
        nd.right = _delete(nd.right, k)
    else
        nd.left === nothing && return nd.right
        nd.right === nothing && return nd.left
        if (nd.left::Node).prio < (nd.right::Node).prio
            nd = _rotate_right(nd); nd.right = _delete(nd.right, k)
        else
            nd = _rotate_left(nd); nd.left = _delete(nd.left, k)
        end
    end
    nd === nothing ? nothing : _update!(nd)
end

function delete!(idx::RcTimeIndex, id::Int, rc::Float64, time::Float64)
    idx.root = _delete(idx.root, (rc, time, id))
    idx.n -= 1
    return idx
end

"""
`any_below_left(idx, R, T)` -- is there a stored point with `rc <= R` and
`time <= T`? Sound fast-reject for "am I dominated?". O(log n) expected: at each
node it either prunes a subtree by `submin > T` or descends toward one child.
"""
function any_below_left(idx::RcTimeIndex, R::Float64, T::Float64)::Bool
    nd = idx.root
    while nd !== nothing
        nd.submin > T && return false               # nothing in this subtree has time <= T
        if nd.rc <= R
            (nd.time <= T) && return true            # this node qualifies
            (_submin(nd.left) <= T) && return true   # whole left subtree has rc <= R
            nd = nd.right                            # rc in (nd.rc, R] lives right
        else
            nd = nd.left                             # only rc <= R can be left
        end
    end
    return false
end

"""Enumerate stored points with `rc <= R` and `time <= T`, pruning subtrees whose
`submin > T`. Output-sensitive: O(log n + #hits)."""
function each_below_left(f, idx::RcTimeIndex, R::Float64, T::Float64)
    _each_below_left(f, idx.root, R, T)
end
function _each_below_left(f, nd::Union{Node, Nothing}, R::Float64, T::Float64)
    nd === nothing && return
    nd.submin > T && return
    # left subtree: all rc <= nd.rc, so keep only if nd.rc <= R OR recurse (rc smaller)
    _each_below_left(f, nd.left, R, T)
    if nd.rc <= R
        nd.time <= T && f(nd)
        _each_below_left(f, nd.right, R, T)
    end
end

"""Enumerate stored points with `rc >= R` and `time >= T`, pruning subtrees whose
`submax < T`. The eviction query ("whom do I dominate?"). O(log n + #hits)."""
function each_above_right(f, idx::RcTimeIndex, R::Float64, T::Float64)
    _each_above_right(f, idx.root, R, T)
end
function _each_above_right(f, nd::Union{Node, Nothing}, R::Float64, T::Float64)
    nd === nothing && return
    nd.submax < T && return
    if nd.rc >= R
        nd.time >= T && f(nd)
        _each_above_right(f, nd.left, R, T)
    end
    _each_above_right(f, nd.right, R, T)
end

# ─────────────────────────────────────────────────────────────────────────────
# Brute-force reference (what a linear bucket scan computes)
# ─────────────────────────────────────────────────────────────────────────────

struct Pt; id::Int; rc::Float64; time::Float64; end

lin_any_below_left(v::Vector{Pt}, R, T) = any(p -> p.rc <= R && p.time <= T, v)
lin_below_left(v::Vector{Pt}, R, T) = sort!([p.id for p in v if p.rc <= R && p.time <= T])
lin_above_right(v::Vector{Pt}, R, T) = sort!([p.id for p in v if p.rc >= R && p.time >= T])

function idx_below_left(idx, R, T)
    out = Int[]; each_below_left(nd -> push!(out, nd.id), idx, R, T); sort!(out)
end
function idx_above_right(idx, R, T)
    out = Int[]; each_above_right(nd -> push!(out, nd.id), idx, R, T); sort!(out)
end

# ─────────────────────────────────────────────────────────────────────────────
# (3) correctness: random insert/delete, cross-check every query against brute force
# ─────────────────────────────────────────────────────────────────────────────

function correctness(; trials=4000, seed=7)
    rng = MersenneTwister(seed)
    idx = RcTimeIndex(99)
    v = Pt[]
    next_id = 1
    fails = 0
    for _ in 1:trials
        if isempty(v) || rand(rng) < 0.6
            rc = round(rand(rng) * 20 - 10; digits=2)
            t  = round(rand(rng) * 20; digits=2)
            insert!(idx, next_id, rc, t); push!(v, Pt(next_id, rc, t)); next_id += 1
        else
            k = rand(rng, 1:length(v)); p = v[k]
            delete!(idx, p.id, p.rc, p.time); deleteat!(v, k)
        end
        R = round(rand(rng) * 20 - 10; digits=2); T = round(rand(rng) * 20; digits=2)
        any_below_left(idx, R, T) == lin_any_below_left(v, R, T) || (fails += 1)
        idx_below_left(idx, R, T) == lin_below_left(v, R, T) || (fails += 1)
        idx_above_right(idx, R, T) == lin_above_right(v, R, T) || (fails += 1)
        idx.n == length(v) || (fails += 1)
    end
    println(fails == 0 ? "correctness: PASS ($trials trials, all queries match brute force)" :
                         "correctness: FAIL ($fails mismatches)")
    return fails == 0
end

# ─────────────────────────────────────────────────────────────────────────────
# (4) scaling: "am I dominated?" query cost, treap vs linear scan, per bucket size
# ─────────────────────────────────────────────────────────────────────────────

# Public-package contender: DataStructures.SortedSet keyed by (rc, time, id). This
# is the fair "use a public balanced tree" option -- O(log n) insert/delete/locate,
# but NO subtree augmentation, so "am I dominated?" still iterates the rc<=R prefix.
ss_any_below_left(ss::SortedSet, R, T) = begin
    for x in ss
        x[1] > R && break        # sorted by rc; nothing further can qualify
        x[2] <= T && return true
    end
    false
end

function bench(; sizes=(1_000, 5_000, 20_000, 100_000), queries=50_000, seed=3)
    rng = MersenneTwister(seed)
    println("\n-- QUERY cost: \"am I dominated?\" (ns per query) --")
    @printf("%-9s %13s %13s %13s %10s %12s\n",
        "bucket", "vector", "sortedset", "treap", "treap/vec", "treap_hits/q")
    for N in sizes
        rcs = rand(rng, N) .* 20 .- 10
        ts  = rand(rng, N) .* 20
        v = [Pt(i, rcs[i], ts[i]) for i in 1:N]
        ss = SortedSet{Tuple{Float64, Float64, Int}}()
        for i in 1:N; push!(ss, (rcs[i], ts[i], i)); end
        idx = RcTimeIndex(N)
        for i in 1:N; insert!(idx, i, rcs[i], ts[i]); end

        qr = rand(rng, queries) .* 20 .- 10
        qt = rand(rng, queries) .* 20

        s = false
        t_vec = @elapsed for q in 1:queries; s ⊻= lin_any_below_left(v, qr[q], qt[q]); end
        s2 = false
        t_ss = @elapsed for q in 1:queries; s2 ⊻= ss_any_below_left(ss, qr[q], qt[q]); end
        s3 = false; hits = 0
        t_idx = @elapsed for q in 1:queries; s3 ⊻= any_below_left(idx, qr[q], qt[q]); end
        for q in 1:200; each_above_right(_ -> (hits += 1), idx, qr[q], qt[q]); end
        @printf("%-9d %13.1f %13.1f %13.1f %10.1fx %12.1f\n",
            N, t_vec / queries * 1e9, t_ss / queries * 1e9, t_idx / queries * 1e9,
            t_vec / t_idx, hits / 200)
        (s, s2, s3)
    end

    println("\n-- INSERT cost: build the structure by N sorted inserts (ns per insert) --")
    @printf("%-9s %13s %13s %13s\n", "bucket", "vector", "sortedset", "treap")
    for N in sizes
        rcs = rand(rng, N) .* 20 .- 10; ts = rand(rng, N) .* 20
        # vector: keep sorted by (rc,time,id) via searchsorted + insert (the pricer's pattern)
        vv = Tuple{Float64, Float64, Int}[]
        t_vec = @elapsed for i in 1:N
            key = (rcs[i], ts[i], i)
            insert!(vv, searchsortedfirst(vv, key), key)
        end
        ss = SortedSet{Tuple{Float64, Float64, Int}}()
        t_ss = @elapsed for i in 1:N; push!(ss, (rcs[i], ts[i], i)); end
        idx = RcTimeIndex(N)
        t_idx = @elapsed for i in 1:N; insert!(idx, i, rcs[i], ts[i]); end
        @printf("%-9d %13.1f %13.1f %13.1f\n",
            N, t_vec / N * 1e9, t_ss / N * 1e9, t_idx / N * 1e9)
    end
end

function main()
    println("=== augmented-treap dominance index prototype ===")
    ok = correctness()
    ok || error("correctness cross-check failed -- do not trust the benchmark")
    bench()
    println("\nInterpretation: linear ns/q grows ~O(bucket); treap ns/q grows ~O(log bucket).")
    println("The crossover is where the pricer's buckets already are at n>=20.")
end

main()
