"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data`, the once-built
`dominates` closure, and the precomputed `search_index`/`bound_workspace`
`prune.jl`'s remaining-reward bound needs. No hook methods and no
search logic of its own live here; see `hooks.jl` for how this struct gets
wired into `_run_label_setting` (`engine.jl`) and `round.jl`.
"""

# ── search context: struct + constructor ────────────────────────────────────
"""
Context for the revisit-tolerant `JointRoutingAssignmentCG` search: bundles
`pricing_data`, the once-built `dominates` closure, the precomputed
`search_index`/`bound_workspace` the remaining-reward bound (`prune.jl`)
needs, and one optional production knob:

  - `label_observer` -- an optional diagnostic callback invoked via
    `_pricing_on_label_inserted` (`hooks.jl`).

(The post-`W` exact completion bound this context used to also support has been
removed along with `post_w_completion.jl` -- peripheral to the base search, not
part of it.)
"""
struct JointRoutingAssignmentSearchContext{D<:Function, O} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDominanceFilters, JointRoutingAssignmentPricingLabel, JointRoutingAssignmentLabelBitsets,
    Int, RewardLayerBitset,
}
    pricing_data::JointRoutingAssignmentPricingData
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    n_nodes::Int
    label_observer::O
end

function JointRoutingAssignmentSearchContext(
    pricing_data::JointRoutingAssignmentPricingData;
    # Count which dominance condition rejected each tested pair, into
    # `logging.jl`'s `JOINT_ROUTING_ASSIGNMENT_DOMINANCE_REJECTIONS`. Off in
    # production: it selects an instrumented specialization of the dominance
    # predicate, so the counters cost nothing at all when this is `false`.
    # See `julia scripts/diagnose.jl dominance_audit`.
    dominance_census::Bool=false,
    # Diagnostic hook: called once per label that survives dominance and enters the
    # frontier. `nothing` (the default) costs one branch per insertion and nothing
    # else -- production pricing never sets it. Used by
    # `julia scripts/diagnose.jl split_census` to census the live-label population
    # (live-clock support, pickup-phase membership) without duplicating this loop.
    label_observer=nothing,
)
    n_nodes = length(pricing_data.nodes)
    search_index = _build_joint_routing_assignment_search_index(pricing_data)
    bound_workspace = _create_joint_routing_assignment_bound_workspace()
    # Built once per search: the dominance switches live in the type, so the scan
    # compiles down to only the conditions this configuration actually uses.
    dominance_rules = _joint_routing_assignment_dominance_rules(
        pricing_data.bounded_max_stops,
        pricing_data.compensated_dominance,
        dominance_census,
    )
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, pricing_data.layer_weight, dominance_rules,
    )
    return JointRoutingAssignmentSearchContext(
        pricing_data, dominates, search_index, bound_workspace, n_nodes, label_observer,
    )
end
