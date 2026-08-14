"""
Constraint helpers for the aggregate-OD-route problem that are genuinely shared across
more than one formulation, so they don't belong under `base/` or
`joint_routing_assignment/` specifically:

- `aggregate_od_route_column_objective_coefficient`: the route-column cost formula
  shared by `objectives/aggregate_od_route/base/assembly.jl`
  (`AggregateODRouteBaseFormulation`) and `constraints/aggregate_od_route/
  joint_routing_assignment/routing_and_assignment.jl`
  (`AggregateODRouteJointRoutingAssignmentFormulation`'s `add_joint_routing_assignment_column!`).
"""

export aggregate_od_route_column_objective_coefficient

aggregate_od_route_column_objective_coefficient(
    route_regularization_weight::Real,
    repositioning_time::Real,
    column::AggregateODRouteColumn,
) = Float64(route_regularization_weight) * (column.tau + Float64(repositioning_time))
