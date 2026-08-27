"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data`, `dominance_mode`
(`:exact` states on `(current, visited)`; `:subset` states on `current`
alone, pairing it with a shared `empty_visited` so both modes share one
concrete state type), the once-built `dominates` closure, and the
`search_index`/`bound_workspace` the shared remaining-reward bound
(reused from `../exact/prune.jl` as-is) needs. No hook methods and no search
logic of its own live here; see `hooks.jl` for how this struct gets wired
into `_run_label_setting` (`engine.jl`) and `round.jl`. Not currently
reachable from `joint_routing_assignment/pricing_round.jl`'s
`_pricing_build_scenario_context` (always builds the revisit-tolerant context
today) -- kept as a real, independently usable capability.
"""

struct JointRoutingAssignmentStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentStationSimpleDominanceFilters, JointRoutingAssignmentStationSimpleLabel, JointRoutingAssignmentStationSimpleAges,
    Tuple{Int, BitSet}, RewardLayerBitset,
}
    pricing_data::JointRoutingAssignmentPricingData
    dominance_mode::Symbol
    dominates::D
    search_index::JointRoutingAssignmentSearchIndex
    bound_workspace::JointRoutingAssignmentBoundWorkspace
    node_index::Dict{Int, Int}
    empty_visited::BitSet
end

function JointRoutingAssignmentStationSimpleSearchContext(
    pricing_data::JointRoutingAssignmentPricingData; dominance_mode::Symbol=:exact,
)
    dominance_mode in (:subset, :exact) ||
        throw(ArgumentError("dominance_mode must be :subset or :exact, got $(dominance_mode)"))
    rules = JointRoutingAssignmentStationSimpleDominanceRules()
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.label, x.bitsets, y.filters, y.label, y.bitsets, pricing_data.layer_weight, rules,
    )
    search_index = _build_joint_routing_assignment_search_index(pricing_data)
    bound_workspace = _create_joint_routing_assignment_bound_workspace()
    return JointRoutingAssignmentStationSimpleSearchContext(
        pricing_data, dominance_mode, dominates, search_index, bound_workspace, search_index.node_index, BitSet(),
    )
end
