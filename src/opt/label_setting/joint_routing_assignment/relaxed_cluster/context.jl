"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- the relaxed pricing data, the
once-built `dominates` closure, and the `search_index`/`bound_workspace` the
shared remaining-reward bound (`../exact/prune.jl`, reused as-is) needs. No
hook methods and no search logic of its own live here.

Label, bitsets, dominance filters and the dominance predicate are all
`../exact/`'s -- the search *is* the exact search, run over the cluster graph
-- so this context's type parameters are identical to
`JointRoutingAssignmentSearchContext`'s and its `dominates` closure is built
the same way. Everything the relaxation changes lives in the data
(`data.jl`) or in the two reward-collection points (`seed.jl`/`extend.jl`).

Both the search index and the bound are built against `data.inner`, whose
`opportunities` deliberately still contain the intra-cluster entries: the bound
reads the index, and an admissible bound has to count intra-cluster reward as
reachable (see `RelaxedClusterPricingData`'s docstring).
"""

struct JointRoutingAssignmentRelaxedClusterSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDominanceFilters, JointRoutingAssignmentPricingLabel,
    JointRoutingAssignmentLabelBitsets, Int, RewardLayerBitset,
}
    pricing_data::RelaxedClusterPricingData
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    n_nodes::Int
end

function JointRoutingAssignmentRelaxedClusterSearchContext(
    pricing_data::RelaxedClusterPricingData;
    # Same census switch the exact context carries, and for the same reason: it selects
    # an instrumented specialization of the dominance predicate, so it costs nothing when
    # `false`.
    dominance_census::Bool=false,
)
    inner = pricing_data.inner
    n_nodes = length(inner.nodes)
    search_index = _build_joint_routing_assignment_search_index(inner)
    bound_workspace = _create_joint_routing_assignment_bound_workspace()
    dominance_rules = _joint_routing_assignment_dominance_rules(
        inner.bounded_max_stops, inner.compensated_dominance, dominance_census,
    )
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, inner.layer_weight, dominance_rules,
    )
    return JointRoutingAssignmentRelaxedClusterSearchContext(
        pricing_data, dominates, search_index, bound_workspace, n_nodes,
    )
end
