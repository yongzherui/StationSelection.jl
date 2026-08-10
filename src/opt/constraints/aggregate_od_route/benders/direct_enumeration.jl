"""
    add_direct_enumeration_guide_coverage_constraints!(m, theta_direct, requests, feasible_pairs, full_pool, x_by_pair_full)

`BendersSolver(direct_enumeration_guide=true)`'s coverage rows, linking `theta_direct` to the
master's own per-request assignment `x_by_pair_full`:
`sum(theta_direct[idx,s] for idx covering pair) >= x_by_pair_full[(request,pair)]`. Builds a
`pair -> column indices` reverse index once up front rather than scanning `full_pool` per
`(request, pair)` (unlike `add_benders_route_coverage_constraints!`), since the enumerated pool
can be large.
"""
function add_direct_enumeration_guide_coverage_constraints!(
    m::JuMP.Model,
    theta_direct,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    full_pool::Vector{AggregateODRouteColumn},
    x_by_pair_full::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
)
    pair_to_columns = Dict{Tuple{Int, Int}, Vector{Int}}()
    for (idx, column) in enumerate(full_pool), pair in column.od_pairs
        push!(get!(pair_to_columns, pair, Int[]), idx)
    end
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            requires_no_vehicle_route(pair) && continue
            covering = get(pair_to_columns, pair, Int[])
            @constraint(
                m, sum(theta_direct[idx, s] for idx in covering; init = 0.0) >= x_by_pair_full[(request, pair)]
            )
        end
    end
    return nothing
end
