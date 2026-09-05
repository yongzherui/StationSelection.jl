"""
The context struct: bundles what `cut_hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- the relaxed graph's `pricing_data`,
the compiled `cuts` (`cuts.jl`), the once-built `dominates` closure, and the
`search_index`/`bound_workspace` the shared remaining-reward bound
(`../exact/prune.jl`, reused as-is) needs. No hook methods and no search logic
of its own live here.

Same graph and same dominance predicate as the plain relaxed search; what
differs is that the state key carries the satisfied-cuts mask
(`cut_hooks.jl`'s `_pricing_state`), so labels that have escaped different cut
sets never compete, and that only a fully-satisfying label can be an answer.

Keying the state on the mask is the conservative choice: the sharper rule is
that `a` may dominate `b` when `satisfied(a) ⊇ satisfied(b)` (a has escaped
everything `b` has), which would let more labels compete. That is a strict
improvement to make later if the live-label population turns out to be the
bottleneck -- it is not needed for correctness.
"""

struct RelaxedClusterCutSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDominanceFilters, RelaxedClusterCutLabel,
    JointRoutingAssignmentLabelBitsets, Tuple{Int, UInt64}, Tuple{RewardLayerBitset, UInt64},
}
    pricing_data::JointRoutingAssignmentPricingData
    cuts::RelaxedClusterNoGoodCuts
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
end

function RelaxedClusterCutSearchContext(
    data::RelaxedClusterPricingData, cluster_sets::AbstractVector{Set{Int}},
)
    inner = data.inner
    cuts = _relaxed_cluster_cuts(data, cluster_sets)
    search_index = _build_joint_routing_assignment_search_index(inner)
    rules = _joint_routing_assignment_dominance_rules(
        inner.bounded_max_stops, inner.compensated_dominance, false,
    )
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, inner.layer_weight, rules,
    )
    return RelaxedClusterCutSearchContext(
        inner, cuts, dominates, search_index,
        _create_joint_routing_assignment_bound_workspace(),
    )
end
