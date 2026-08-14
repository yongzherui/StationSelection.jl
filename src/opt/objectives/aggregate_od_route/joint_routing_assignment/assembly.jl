"""
Objective assembly for the joint routing+assignment CG master, built directly off
`AggregateODRouteMap`.
"""

export set_joint_routing_assignment_objective!

"""
    set_joint_routing_assignment_objective!(m, data, mapping, v, x_same, unserved_penalty)

Base objective before any route columns exist: unserved-demand-group penalty plus
same-station walking cost (`od_pair_walking_cost(data, o, d, (j,j))` -- the same helper
every real `(j,k)` pair uses, since a same-station pair is just the `j==k` case of it).
Route `theta` coefficients are added later, per-column, via `set_objective_coefficient`
inside `add_joint_routing_assignment_column!`
(`constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl`).
"""
function set_joint_routing_assignment_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    v::Dict{Tuple{Int, Int}, VariableRef},
    x_same::Dict{Tuple{Tuple{Int, Int}, Int}, VariableRef},
    unserved_penalty::Float64,
)
    walk_cost_weight = Float64(m[:joint_routing_assignment_walk_cost_weight])
    obj = AffExpr(0.0)
    for var in values(v)
        add_to_expression!(obj, unserved_penalty, var)
    end
    for ((s, p), j) in keys(x_same)
        o, d = mapping.Omega_s[s][p]
        cost = walk_cost_weight * od_pair_walking_cost(data, o, d, (j, j))
        add_to_expression!(obj, cost, x_same[((s, p), j)])
    end
    @objective(m, Min, obj)
    return nothing
end
