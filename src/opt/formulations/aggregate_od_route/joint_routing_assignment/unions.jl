"""
Unions over the `AggregateODRoute` formulation types, so shared-engine functions dispatch
once instead of repeating a method per formulation. Loaded last of the family (see
`shared.jl`) because every member has to exist first.
"""

export AnyAggregateODRouteFormulation
export AnyJointRoutingAssignmentFormulation

"""
    AnyJointRoutingAssignmentFormulation

The joint routing+assignment family: the monolith plus the two halves it decomposes into
under `BendersSolver` (see `shared.jl` for the table). All three describe the same model
and carry the same six encoding fields, which is what lets one derive from another and
what lets the column-cost/pool machinery
(`joint_routing_assignment_column_cost`, `add_joint_routing_assignment_column!`) be
shared verbatim by the monolith's build and the subproblem's.
"""
const AnyJointRoutingAssignmentFormulation = Union{
    AggregateODRouteJointRoutingAssignmentFormulation,
    AggregateODRouteJointRoutingAssignmentMasterFormulation,
    AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation,
}

"""
    AnyAggregateODRouteFormulation

Every `StationSelectionProblem`-paired aggregate-OD-route formulation that carries the
identical encoding-detail field set (see `AggregateODRouteBaseFormulation`'s docstring),
so shared-engine functions (`create_aggregate_od_route_map`,
`enumerate_aggregate_od_route_columns`) dispatch on this rather than repeating themselves
per formulation. Note that no member of `AnyJointRoutingAssignmentFormulation` carries an
`allow_walk_only` field (see the monolith's own docstring) despite matching this Union's
field set otherwise -- `create_aggregate_od_route_map` resolves that one field via
`_aggregate_od_route_allow_walk_only` instead of direct field access. Mirrors
`AnyAggregateODRouteProblem` (`opt/problems/route_covering.jl`) for the same reason.
"""
const AnyAggregateODRouteFormulation = Union{
    AggregateODRouteBaseFormulation,
    AnyJointRoutingAssignmentFormulation,
}
