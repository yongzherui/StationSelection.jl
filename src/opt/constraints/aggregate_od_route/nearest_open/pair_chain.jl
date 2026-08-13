"""
`:pair_chain` nearest-open style: feasible station pairs ranked jointly by combined
walking cost, no `z`/endpoint selector at all -- a pair is usable only if every
strictly-cheaper pair is closed on at least one side.
"""

export add_nearest_open_assignment_constraints!

function _aggregate_od_route_assignment_pair_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pair::Tuple{Int, Int},
)::Float64
    j, k = pair
    return get_walking_cost(data, o, j) + get_walking_cost(data, k, d)
end

function _rank_aggregate_od_route_pairs_by_assignment_cost(
    data::StationSelectionData,
    o::Int,
    d::Int,
    pairs::AbstractVector{Tuple{Int, Int}},
)::Vector{Int}
    idxs = collect(eachindex(pairs))
    sort!(idxs, by=i -> (_aggregate_od_route_assignment_pair_cost(data, o, d, pairs[i]), pairs[i][1], pairs[i][2]))
    return idxs
end

function add_nearest_open_assignment_constraints!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
)::Int
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand > 0 || continue
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            pairs = get_valid_jk_pairs(mapping, o, d)
            ranked_pair_idxs = _rank_aggregate_od_route_pairs_by_assignment_cost(data, o, d, pairs)
            for rank_pos in 2:length(ranked_pair_idxs)
                pair_idx = ranked_pair_idxs[rank_pos]
                for prior_rank_pos in 1:(rank_pos - 1)
                    prior_pair_idx = ranked_pair_idxs[prior_rank_pos]
                    prior_j, prior_k = pairs[prior_pair_idx]
                    @constraint(m, x_od[pair_idx] <= 2.0 - y[prior_j] - y[prior_k])
                end
            end
        end
    end
    return _total_num_constraints(m) - before
end
