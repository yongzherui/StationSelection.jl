"""
Objective assembly for `AggregateODRouteBaseFormulation`'s `y`/`x`/`θ` master.
"""

export set_aggregate_od_route_base_objective!

"""
    set_aggregate_od_route_base_objective!(m, data, mapping, x, x_walk, theta,
        walk_cost_weight, route_regularization_weight, repositioning_time)

Demand-weighted walking cost of every `x` assignment and every `x_walk` direct-walk
(`od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)` for the latter -- the same helper
every real `(j,k)` pair uses), plus `aggregate_od_route_column_objective_coefficient`
(`constraints/aggregate_od_route/core.jl`, shared with the Benders objective expressions)
for every route's `theta` (`tau` read off `mapping.columns`, keyed by the same
`column_id` `theta` is keyed by).
"""
function set_aggregate_od_route_base_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x::Dict{NTuple{4, Int}, VariableRef},
    x_walk::Dict{Tuple{Int, Int}, VariableRef},
    theta::Dict{Tuple{Int, Int}, VariableRef},
    walk_cost_weight::Float64,
    route_regularization_weight::Float64,
    repositioning_time::Float64,
)
    obj = AffExpr(0.0)
    for ((s, p, j, k), var) in x
        o, d = mapping.Omega_s[s][p]
        demand = mapping.Q_s[s][p]
        cost = walk_cost_weight * demand * od_pair_walking_cost(data, o, d, (j, k))
        add_to_expression!(obj, cost, var)
    end
    for ((s, p), var) in x_walk
        o, d = mapping.Omega_s[s][p]
        demand = mapping.Q_s[s][p]
        cost = walk_cost_weight * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
        add_to_expression!(obj, cost, var)
    end
    columns_by_id = Dict(column.id => column for column in mapping.columns)
    for ((column_id, _s), var) in theta
        cost = aggregate_od_route_column_objective_coefficient(
            route_regularization_weight, repositioning_time, columns_by_id[column_id],
        )
        add_to_expression!(obj, cost, var)
    end
    @objective(m, Min, obj)
    return nothing
end
