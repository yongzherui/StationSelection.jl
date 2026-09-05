"""
No-good cuts on cluster sets: the *resource* the cut-aware relaxed search
carries, and how it is compiled for the hot path.

This is what the no-good-cut certification loop in `nogood_certify.jl` adds to,
one cut per barren cluster support. The search that respects them is this
directory's second pricer, split the way every other pricer directory is
(`../../README.md`): `cut_types.jl` (label), `cut_seed.jl`, `cut_extend.jl`,
`cut_context.jl`, `cut_hooks.jl`. It has no `cut_dominate.jl` or `cut_prune.jl`
-- the dominance predicate, filters and remaining-reward bound are `../exact/`'s,
reused verbatim, since a cut changes which routes may be *reported*, not what
makes one label better than another at a state.

The loop's step is:

    relaxed search  ->  best improving cluster route, support T
    exact search over stations(T)
        found an improving real column  ->  refuted, stop
        exhausted with nothing          ->  T is BARREN; cut it and search again

# The cut, and why it is shaped this way

Exhausting the exact pricer over `stations(T)` proves exactly one thing: *no
real route confined to those stations is improving*. So the sound cut is

    every relaxed route must visit at least one cluster OUTSIDE T             (C)

**not** "at most |T|-1 clusters of T". The weaker-looking form is the correct
one; the stronger one is invalid. Take a real improving route `R` touching
clusters `A, B, C, D` after `T = {A,B,C}` was cut. `R` was never examined by
step 3 (it uses a station in `D`), yet a cut of the form
`|route ∩ T| <= 2` deletes its image, and the relaxed search would then certify
with `R` still out there -- a false certificate.

Under (C) that cannot happen: if a real improving `R` exists then
`stations(R) ⊄ stations(T)` (or step 3 would have found it), so `R` touches a
station whose cluster is not in `T`, so its image visits a cluster outside `T`
and survives every cut. Validity is preserved cut after cut, so the loop can
only ever end in a true certificate.

(C) is also stronger than it looks operationally: it kills every route confined
to *any subset* of `T` at once, so each cut removes a downward-closed family
and the loop cannot revisit ground it has covered. With at most `2^K` cluster
supports, it terminates.

**A cut may only be added after an EXHAUSTED exact search.** One that merely ran
out of time has proved nothing about `stations(T)`, and cutting on it would be
the same false certificate by a different route.

# Why the mask has to live inside the search

Satisfying (C) is a property of the *finished* route -- a partial route can sit
inside `T` now and leave later -- so it cannot be enforced by pruning during
extension, and it must not be applied as a filter over the search's results
either. `_run_label_setting` keeps only the best label per signature; if that
label violates a cut while a slightly worse one satisfies it, the satisfying one
is already gone and filtering afterwards silently certifies. So the
satisfied-cuts mask is part of the label (`cut_types.jl`), part of the search
*state* (`cut_hooks.jl`'s `_pricing_state` -- labels with different masks never
dominate one another) and part of the best-so-far signature.

The mask is monotone -- bits only ever turn on -- and one `UInt64` covers 64
simultaneous cuts, which is also the loop's round cap.
"""

export RelaxedClusterNoGoodCuts

"""
    RelaxedClusterNoGoodCuts(cluster_sets, node_mask, all_satisfied)

The active cuts, compiled for the hot path.

`node_mask[v]` has bit `c` set when relaxed-graph node `v` lies OUTSIDE cut
`c`'s cluster set -- i.e. visiting `v` satisfies cut `c`. A label's satisfied
mask is then just an `OR` along its route, and "obeys every cut" is one
equality against `all_satisfied`.
"""
struct RelaxedClusterNoGoodCuts
    cluster_sets::Vector{Set{Int}}
    node_mask::Vector{UInt64}
    all_satisfied::UInt64
end

"""Cap on simultaneously active cuts, set by the `UInt64` mask. The
certification loop stops adding cuts here and reports itself inconclusive
rather than silently dropping one (which would re-admit an already-refuted
cluster support and could loop forever)."""
const RELAXED_CLUSTER_MAX_CUTS = 64

"""
    _relaxed_cluster_cuts(data, cluster_sets) -> RelaxedClusterNoGoodCuts

Compile `cluster_sets` against one relaxed graph: one mask word per node, plus
the `all_satisfied` target the hooks compare against.
"""
function _relaxed_cluster_cuts(
    data::RelaxedClusterPricingData, cluster_sets::AbstractVector{Set{Int}},
)::RelaxedClusterNoGoodCuts
    length(cluster_sets) <= RELAXED_CLUSTER_MAX_CUTS || throw(ArgumentError(
        "at most $RELAXED_CLUSTER_MAX_CUTS cuts fit the UInt64 satisfied-mask, got " *
        "$(length(cluster_sets))",
    ))
    # Dense node -> cluster map (`types.jl`), so compilation is
    # `O(nodes x cuts)` rather than re-scanning `service_node` per node.
    node_clusters = _relaxed_cluster_node_clusters(data)
    node_mask = zeros(UInt64, length(node_clusters))
    for (c, forbidden) in enumerate(cluster_sets)
        bit = UInt64(1) << (c - 1)
        for node in eachindex(node_clusters)
            node_clusters[node] in forbidden || (node_mask[node] |= bit)
        end
    end
    all_satisfied = isempty(cluster_sets) ? UInt64(0) :
        (length(cluster_sets) == 64 ? typemax(UInt64) :
         (UInt64(1) << length(cluster_sets)) - UInt64(1))
    return RelaxedClusterNoGoodCuts(collect(Set{Int}, cluster_sets), node_mask, all_satisfied)
end
