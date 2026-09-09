"""
The one piece of model state every member of `AnyJointRoutingAssignmentFormulation` must
stash identically: the scalars and pool containers that the column machinery reads back
off `m`.

`joint_routing_assignment_column_cost`, `add_joint_routing_assignment_column!` and
`set_joint_routing_assignment_objective!` all take their cost weights from `m[...]` rather
than from a `formulation` argument, because `add_columns!` is a fixed-signature `CGSolver`
hook that never sees one. That is fine while one build path exists; with three (the
monolith's, the Benders master's, the Benders subproblem's) it is a live drift hazard --
a subproblem that stashed a different `route_regularization_weight` than the master would
price columns at one cost and cut the master at another, converging happily on a wrong
number. So the stash is a function, called by every build, rather than a block of
assignments copied per build.
"""

"""
    _stash_joint_routing_assignment_cost_parameters!(m, data, formulation; relax_integrality)

Stash on `m` everything the shared column/objective machinery reads, and initialize the
(empty) column pool containers.

Deliberately NOT included here: `:joint_routing_assignment_l`, the pricing mode, the
precomputed node list / travel-cost table, and every relaxed-cluster key. Those belong to
`CGSolver`'s master specifically (`_build_joint_routing_assignment_model`) -- the Benders
master carries no columns at all, and the Benders subproblem never prices -- so stashing
them everywhere would assert a pricing capability the model does not have.
"""
function _stash_joint_routing_assignment_cost_parameters!(
        m::Model,
        data::StationSelectionData,
        formulation::AnyJointRoutingAssignmentFormulation;
        relax_integrality::Bool,
    )
    m[:aggregate_od_route_formulation] = formulation
    m[:joint_routing_assignment_data] = data
    m[:joint_routing_assignment_relax_integrality] = relax_integrality
    m[:joint_routing_assignment_route_regularization_weight] = formulation.route_regularization_weight
    m[:joint_routing_assignment_repositioning_time] = formulation.repositioning_time
    m[:joint_routing_assignment_walk_cost_weight] = formulation.walk_cost_weight
    m[:joint_routing_assignment_max_wait_time] = formulation.max_wait_time
    m[:joint_routing_assignment_max_stops] = formulation.max_stops
    m[:joint_routing_assignment_detour_factor] = formulation.detour_factor
    # Empty pool containers: real entries arrive from whichever seeding path this build
    # uses (the two-stop seed, exhaustive enumeration, or a CG iteration's `add_columns!`).
    m[:joint_routing_assignment_theta] = Dict{Int, VariableRef}()
    m[:joint_routing_assignment_columns] = Dict{Int, JointRoutingAssignmentRouteColumn}()
    m[:joint_routing_assignment_column_signatures] = Dict{Any, Int}()
    return nothing
end
