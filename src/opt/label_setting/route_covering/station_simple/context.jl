"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data`/`duals`, the
once-built `dominates` closure, and `node_index` (the only precomputed index
this pricer's reward bound needs -- unlike the revisit-tolerant twin, it has
no travel matrix or per-pair arrays to precompute, since `prune.jl`'s bound
iterates `active_pairs` directly). No hook methods and no search logic of its
own live here; see `hooks.jl` for how this struct gets wired into
`_run_label_setting` (`engine.jl`) and `round.jl`.
"""

struct RouteCoveringStationSimpleSearchContext{D<:Function} <: AbstractPricingSearchContext{
    RouteCoveringStationSimpleDominanceFilters, RouteCoveringStationSimpleLabel, RouteCoveringStationSimpleBitsets,
    Tuple{Int, BitSet}, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::RouteCoveringPricingData
    duals::RouteCoveringPricingDuals
    dominates::D
    node_index::Dict{Int, Int}
end

function RouteCoveringStationSimpleSearchContext(
    pricing_data::RouteCoveringPricingData, duals::RouteCoveringPricingDuals,
)
    rules = RouteCoveringStationSimpleDominanceRules()
    # Built once per search and handed to `_add_pricing_label_to_state!`
    # unchanged for every dominance test, same convention as the
    # revisit-tolerant pricer's own `dominates` closure (`../exact/context.jl`).
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, rules)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    return RouteCoveringStationSimpleSearchContext(pricing_data, duals, dominates, node_index)
end
