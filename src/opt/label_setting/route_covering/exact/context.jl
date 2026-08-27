"""
The context struct: bundles what `hooks.jl` needs to answer the
`AbstractPricingSearchContext` contract -- `pricing_data`/`duals`, the
once-built `dominates` closure, and the precomputed indices `prune.jl`'s
remaining-reward bound needs (`pair_index`, `node_index`, the pair-origin/
destination index arrays, the per-pair ride limit, the dense travel matrix,
and the positive dual rewards). No hook methods and no search logic of its
own live here; see `hooks.jl` for how this struct gets wired into
`_run_label_setting` (`engine.jl`) and `round.jl`.
"""

struct RouteCoveringSearchContext{D<:Function} <: AbstractPricingSearchContext{
    RouteCoveringDominanceFilters, RouteCoveringPricingLabel, RouteCoveringLabelBitsets,
    Int, Tuple{Vararg{Tuple{Int, Int}}},
}
    pricing_data::RouteCoveringPricingData
    duals::RouteCoveringPricingDuals
    dominates::D
    n_pairs::Int
    pair_index::Dict{Tuple{Int, Int}, Int}
    node_index::Dict{Int, Int}
    n_nodes::Int
    pair_origin_idx::Vector{Int}
    pair_dest_idx::Vector{Int}
    pair_ride_limit::Vector{Float64}
    travel_matrix::Matrix{Float64}
    positive_pair_rewards::Vector{Float64}
end

function RouteCoveringSearchContext(
    pricing_data::RouteCoveringPricingData,
    duals::RouteCoveringPricingDuals,
)
    # Everything below is precomputed once per scenario so that
    # `_pricing_label_priority` (the remaining-reward bound, called once per
    # popped label -- the hottest hook after dominance) can look things up by
    # dense array index instead of hashing a `(Int,Int)` pair or station id
    # every time.
    n_pairs = length(pricing_data.active_pairs)
    pair_index = Dict(pair => i for (i, pair) in enumerate(pricing_data.active_pairs))    # pair -> dense index (matches served_bits)
    node_index = Dict(node => i for (i, node) in enumerate(pricing_data.nodes))           # station id -> dense index
    n_nodes = length(pricing_data.nodes)
    pair_origin_idx = [node_index[pair[1]] for pair in pricing_data.active_pairs]         # pair i's origin, as a node index
    pair_dest_idx = [node_index[pair[2]] for pair in pricing_data.active_pairs]           # pair i's destination, as a node index
    pair_ride_limit = [_direct_ride_limit(pricing_data, pair) for pair in pricing_data.active_pairs]
    # Dense node x node travel matrix (Inf where no arc exists) -- a matrix
    # lookup is faster than a Dict hash in the bound's hot inner loop below.
    travel_matrix = fill(Inf, n_nodes, n_nodes)
    for (i, u) in enumerate(pricing_data.nodes), (j, v) in enumerate(pricing_data.nodes)
        i == j && (travel_matrix[i, j] = 0.0; continue)
        haskey(pricing_data.travel_cost, (u, v)) &&
            (travel_matrix[i, j] = pricing_data.travel_cost[(u, v)])
    end
    # Only positive dual rewards can ever improve reduced cost, so this is
    # both the reward-bound's per-pair weight and (via `max(0.0, ...)`) an
    # implicit filter: a pair with a non-positive dual contributes 0 to any
    # bound and is effectively invisible to the search.
    positive_pair_rewards = Float64[
        max(0.0, get(duals.sigma, pair, 0.0)) for pair in pricing_data.active_pairs
    ]
    rules = _route_covering_dominance_rules(pricing_data.bounded_max_stops, pricing_data.compensated_dominance)
    # Built once, closing over `positive_pair_rewards`/`rules` for this
    # scenario, and handed to `_add_pricing_label_to_state!` unchanged for
    # every dominance test in this search (see that function's docstring in
    # `label_setting/types.jl` for why the closure is supplied by the caller).
    dominates(x::PricingLabelEntry, y::PricingLabelEntry) =
        _pricing_dominates_at_state(x.filters, x.bitsets, y.filters, y.bitsets, positive_pair_rewards, rules)
    return RouteCoveringSearchContext(
        pricing_data, duals, dominates,
        n_pairs, pair_index, node_index, n_nodes,
        pair_origin_idx, pair_dest_idx, pair_ride_limit, travel_matrix, positive_pair_rewards,
    )
end
