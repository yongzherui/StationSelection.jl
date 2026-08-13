"""
Objective assembly for `AggregateODRouteBaseFormulation`'s `y`/`x`/`θ` master.
"""

export set_aggregate_od_route_base_objective!

"""
    set_aggregate_od_route_base_objective!(m, data, mapping, x, theta,
        walk_cost_weight, route_regularization_weight, repositioning_time)

Demand-weighted walking cost of every `x` assignment, plus
`aggregate_od_route_column_objective_coefficient` (`constraints/aggregate_od_route/core.jl`,
shared with the Benders objective expressions) for every route's `theta` (`tau` read off
`mapping.columns`, keyed by the same `column_id` `theta` is keyed by).
"""
function set_aggregate_od_route_base_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x::Dict{NTuple{5, Int}, VariableRef},
    theta::Dict{Tuple{Int, Int}, VariableRef},
    walk_cost_weight::Float64,
    route_regularization_weight::Float64,
    repositioning_time::Float64,
)
    obj = AffExpr(0.0)
    for ((s, o, d, j, k), var) in x
        demand = get(mapping.Q_s[s], (o, d), 0)
        cost = walk_cost_weight * demand * od_pair_walking_cost(data, o, d, (j, k))
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
