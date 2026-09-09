"""
The encoding-field set every member of the joint routing+assignment formulation family
shares, and the derivation helper that copies it from one member to another.

The family has three members, all describing the SAME model:

| Formulation | Solvers | Carries |
| --- | --- | --- |
| `AggregateODRouteJointRoutingAssignmentFormulation` (`monolithic.jl`) | `DirectMIPSolver`, `CGSolver` | the whole model: `y`, `x_walk`, `theta` |
| `AggregateODRouteJointRoutingAssignmentMasterFormulation` (`master.jl`) | `BendersSolver` | the first stage: `y` + cut placeholders `Theta` |
| `AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation` (`benders_subproblem.jl`) | `BendersSolver` (inner) | one scenario's second stage: `x_walk`, `theta`, `y` fixed |

Together the master and the subproblem partition the monolith's rows exactly -- the master
takes the pure-`y` rows (station budget, endpoint feasibility), the subproblem takes
everything else, one scenario at a time -- so every block in
`opt/{variables,constraints,objectives}/` is used by whichever of the three needs it, and
nothing is reimplemented per decomposition. See
`optimize/aggregate_od_route/benders/build_master.jl` and `build_subproblem.jl` for the
per-member block lists.

# Why the derivation helper exists

The three must agree on the route universe (`max_stops`, `max_wait_time`,
`detour_factor`) and on the cost weights (`route_regularization_weight`,
`walk_cost_weight`, `repositioning_time`), because a Benders cut derived from a subproblem
priced at one `detour_factor` is simply not a valid underestimator of a master built at
another -- and nothing anywhere would raise: the run converges, reports `OPTIMAL`, and the
number is wrong. So the derived types are NOT independently constructible from loose
keywords in the normal path: each takes a parent formulation and copies this field set off
it, which makes the inconsistent combination unrepresentable rather than merely discouraged.

`max_stops` is the one field a derivation may deliberately narrow (see
`benders_subproblem.jl`): the subproblem's `:direct_enumeration` oracle enumerates the
whole route universe up front, which is only tractable at small `max_stops`. That
narrowing is an explicit argument, is validated to be a restriction and never an
expansion, and is reported in the result's metadata -- unlike silent drift, it is a
recorded scope reduction on the optimality claim.
"""

"""
    _joint_routing_assignment_shared_fields(formulation) -> NamedTuple

The six encoding fields every member of the family carries, as a splattable
`NamedTuple`. Any member can be the source: the master and the subproblem carry the same
six as the monolith, so a subproblem can equally be derived from a master (which is what
`build_model(problem, ::MasterFormulation, ::BendersSolver)` does -- it never sees the
original monolith).
"""
function _joint_routing_assignment_shared_fields(formulation)
    return (
        route_regularization_weight = formulation.route_regularization_weight,
        walk_cost_weight = formulation.walk_cost_weight,
        repositioning_time = formulation.repositioning_time,
        max_wait_time = formulation.max_wait_time,
        detour_factor = formulation.detour_factor,
        max_stops = formulation.max_stops,
    )
end
