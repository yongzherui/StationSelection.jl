"""
Objective assembly for the joint routing+assignment CG master, built directly off
`AggregateODRouteMap`.
"""

export set_joint_routing_assignment_objective!

"""
    set_joint_routing_assignment_objective!(m, data, mapping, x_walk)

Base objective before any route columns exist: demand-weighted direct-walking cost
(`od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)`, the same helper every real `(j,k)`
pair uses). Route `theta` coefficients are added later, per-column, via
`set_objective_coefficient` inside `add_joint_routing_assignment_column!`
(`constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl`).
"""
function set_joint_routing_assignment_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    x_walk::Dict{Tuple{Int, Int}, VariableRef},
)
    walk_cost_weight = Float64(m[:joint_routing_assignment_walk_cost_weight])
    obj = AffExpr(0.0)
    for ((s, p), var) in x_walk
        o, d = mapping.Omega_s[s][p]
        demand = mapping.Q_s[s][p]
        cost = walk_cost_weight * demand * od_pair_walking_cost(data, o, d, WALK_ONLY_PAIR)
        add_to_expression!(obj, cost, var)
    end
    @objective(m, Min, obj)
    return nothing
end
