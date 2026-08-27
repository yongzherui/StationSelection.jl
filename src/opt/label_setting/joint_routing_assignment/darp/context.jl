"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data` and the once-built
`dominates` closure. Simplest of the three `joint_routing_assignment/`
contexts: no precomputed index at all (`prune.jl`'s bound and `dominate.jl`'s
bitsets construction both take `pricing_data` directly). No hook methods and
no search logic of its own live here; see `hooks.jl` for how this struct
gets wired into `_run_label_setting` (`engine.jl`) and `round.jl`.
"""

struct JointRoutingAssignmentDarpSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDarpDominanceFilters, JointRoutingAssignmentDarpPricingLabel, JointRoutingAssignmentDarpLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int, Int}}},
}
    pricing_data::JointRoutingAssignmentDarpPricingData
    dominates::D
end

function JointRoutingAssignmentDarpSearchContext(pricing_data::JointRoutingAssignmentDarpPricingData)
    rules = _joint_routing_assignment_darp_dominance_rules(pricing_data.bounded_max_stops)
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, rules)
    return JointRoutingAssignmentDarpSearchContext(pricing_data, dominates)
end
