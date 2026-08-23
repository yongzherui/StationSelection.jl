"""
Indexed, search-local data for passenger free-assignment label setting.

`JointRoutingAssignmentPricingData` is the semantic pricing input. The types in
this file are implementation details: immutable indexes built once per search
and mutable scratch buffers reused by the remaining-reward bound.
"""

# ── search index (immutable, built once per search) ─────────────────────────
struct JointRoutingAssignmentSearchIndex
    node_index::Dict{Int, Int}
    travel_matrix::Matrix{Float64}
    opp_dest_idx::Vector{Int}
    opp_ride_limit::Vector{Float64}
    opp_layer_mask::Vector{RewardLayerBitset}
    opps_by_origin_idx::Vector{Vector{Int}}
    origin_union_mask::Vector{RewardLayerBitset}
end

"""Mutable scratch reused across the whole search: `layer_scratch` is where
`_joint_routing_assignment_remaining_reward_bound` dedups reward layers it has
already counted, cleared at the start of every call."""
struct JointRoutingAssignmentBoundWorkspace
    layer_scratch::RewardLayerBitset
end

function _build_joint_routing_assignment_search_index(
    pricing_data::JointRoutingAssignmentPricingData,
)::JointRoutingAssignmentSearchIndex
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))
    n_nodes = length(pricing_data.nodes)
    opp_dest_idx = [node_index[opp.destination] for opp in pricing_data.opportunities]
    opp_ride_limit = [opp.ride_limit for opp in pricing_data.opportunities]
    opp_layer_mask = [opp.layer_mask for opp in pricing_data.opportunities]
    opps_by_origin_idx = [Int[] for _ in 1:n_nodes]
    origin_union_mask = [RewardLayerBitset() for _ in 1:n_nodes]
    for (i, opp) in enumerate(pricing_data.opportunities)
        origin_idx = node_index[opp.origin]
        push!(opps_by_origin_idx[origin_idx], i)
        union!(origin_union_mask[origin_idx], opp.layer_mask)
    end

    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) &&
            (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    return JointRoutingAssignmentSearchIndex(
        node_index, travel_matrix, opp_dest_idx, opp_ride_limit, opp_layer_mask,
        opps_by_origin_idx, origin_union_mask,
    )
end

_create_joint_routing_assignment_bound_workspace() =
    JointRoutingAssignmentBoundWorkspace(RewardLayerBitset())
