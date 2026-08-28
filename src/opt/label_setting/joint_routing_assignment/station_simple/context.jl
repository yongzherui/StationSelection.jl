"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data`, the once-built
`dominates` closure, and the `search_index`/`bound_workspace` the shared
remaining-reward bound (reused from `../exact/prune.jl` as-is) needs. States
are keyed on `current` alone (see `hooks.jl`'s `_pricing_state`); the
elementarity resource `U_a ⊆ U_b` is enforced inside the dominance predicate
itself (`dominate.jl`), not by the state key -- see `types.jl` for why. No
hook methods and no search logic of its own live here; see `hooks.jl` for how
this struct gets wired into `_run_label_setting` (`engine.jl`) and `round.jl`.
Not currently reachable from `joint_routing_assignment/pricing_round.jl`'s
`_pricing_build_scenario_context` (always builds the revisit-tolerant context
today) -- kept as a real, independently usable capability.
"""

struct JointRoutingAssignmentStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentStationSimpleDominanceFilters, JointRoutingAssignmentStationSimpleLabel, JointRoutingAssignmentStationSimpleAges,
    Int, RewardLayerBitset,
}
    pricing_data::JointRoutingAssignmentPricingData
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    node_index::Dict{Int, Int}
end

function JointRoutingAssignmentStationSimpleSearchContext(
    pricing_data::JointRoutingAssignmentPricingData,
)
    rules = JointRoutingAssignmentStationSimpleDominanceRules()
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.label, x.bitsets, y.filters, y.label, y.bitsets, pricing_data.layer_weight, rules,
    )
    search_index = _build_joint_routing_assignment_search_index(pricing_data)
    bound_workspace = _create_joint_routing_assignment_bound_workspace()
    return JointRoutingAssignmentStationSimpleSearchContext(
        pricing_data, dominates, search_index, bound_workspace, search_index.node_index,
    )
end
