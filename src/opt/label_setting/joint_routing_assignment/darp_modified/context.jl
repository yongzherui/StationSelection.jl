"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data`, the once-built
`dominates` closure, and the precomputed `node_index`/`travel_matrix`
`prune.jl`'s remaining-reward bound needs. No hook methods and no search
logic of its own live here; see `hooks.jl` for how this struct gets wired
into `_run_label_setting` (`engine.jl`) and `round.jl`.
"""

struct JointRoutingAssignmentDarpModifiedSearchContext{D<:Function} <: AbstractPricingSearchContext{
    JointRoutingAssignmentDarpModifiedDominanceFilters, JointRoutingAssignmentDarpModifiedPricingLabel, JointRoutingAssignmentDarpModifiedLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int, Int}}},
}
    pricing_data::JointRoutingAssignmentDarpModifiedPricingData
    dominates::D
    node_index::Dict{Int, Int}
    n_nodes::Int
    travel_matrix::Matrix{Float64}
end

function JointRoutingAssignmentDarpModifiedSearchContext(pricing_data::JointRoutingAssignmentDarpModifiedPricingData)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    n_nodes = length(pricing_data.nodes)
    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) && (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    rules = _joint_routing_assignment_darp_modified_dominance_rules(pricing_data.bounded_max_stops, pricing_data.compensated_dominance)
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) = _pricing_dominates_at_state(
        x.filters, x.bitsets, y.filters, y.bitsets, pricing_data.passenger_weight, rules,
    )
    return JointRoutingAssignmentDarpModifiedSearchContext(pricing_data, dominates, node_index, n_nodes, travel_matrix)
end
