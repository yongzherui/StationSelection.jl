"""
Constraint helpers for the aggregate-OD-route problem that are genuinely shared across
more than one assignment policy or algorithm, so they don't belong under
`joint_routing_assignment/`, `nearest_open/`, or `benders/` specifically:

- `aggregate_od_route_column_objective_coefficient`: used by `joint_routing_assignment/columns.jl`,
  `opt/objectives/aggregate_od_route/core.jl`, and the Benders objective expressions.

(`add_fixed_open_station_constraints!`, used only by `RouteCoveringProblem` builds, was
removed along with `AggregateODRouteProblem` -- `RouteCoveringProblem` is currently
unwired, see `StationSelection.jl`'s include comments.)
"""

export aggregate_od_route_column_objective_coefficient

aggregate_od_route_column_objective_coefficient(
    route_regularization_weight::Real,
    repositioning_time::Real,
    column::AggregateODRouteColumn,
) = Float64(route_regularization_weight) * (column.tau + Float64(repositioning_time))
