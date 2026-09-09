"""
`BendersSolver` hook dispatch for `AggregateODRouteMap`-based masters.

Same two-level pattern as `column_generation/dispatch.jl`, and for the same reason:
`BendersSolver`'s hooks are fixed-signature `(build_result::BuildResult, mapping,
m::JuMP.Model, ...)` calls, and every aggregate-OD-route formulation shares one
`mapping::AggregateODRouteMap` type -- so a method per formulation on that signature would
collide (Julia would silently keep whichever loaded last). Each hook below is therefore a
one-line dispatcher reading the formulation stashed on the model at build time
(`m[:aggregate_od_route_formulation]`) and re-dispatching on *its* type, with the real
logic in `_aggregate_od_route_benders_*` methods.

Only `AggregateODRouteJointRoutingAssignmentMasterFormulation` implements them today. A
second Benders decomposition (say of `AggregateODRouteBaseFormulation`) adds its own
`_aggregate_od_route_benders_*` methods and needs nothing changed here -- which is the
point of paying for the indirection now rather than after the collision.
"""

function extract_incumbent(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model)
    return _aggregate_od_route_benders_extract_incumbent(
        m[:aggregate_od_route_formulation], build_result, mapping, m,
    )
end

function solve_subproblem(build_result::BuildResult, mapping::AggregateODRouteMap,
        m::JuMP.Model, incumbent, solver::BendersSolver)
    return _aggregate_od_route_benders_solve_subproblem(
        m[:aggregate_od_route_formulation], build_result, mapping, m, incumbent, solver,
    )
end

function add_benders_cut!(build_result::BuildResult, mapping::AggregateODRouteMap,
        m::JuMP.Model, subproblem_result, solver::BendersSolver)::Int
    return _aggregate_od_route_benders_add_cut!(
        m[:aggregate_od_route_formulation], build_result, mapping, m, subproblem_result, solver,
    )
end

"""
    _aggregate_od_route_benders_extract_incumbent(::MasterFormulation, ...) -> Vector{Float64}

The master's `y`, rounded to exactly 0/1.

Rounded, not passed through raw: `y` is binary in the master, but a MIP solution can
report `0.9999999` or `-1e-11`, and those values go on to be `JuMP.fix`ed as the
subproblem's linking right-hand sides. A right-hand side of `0.9999999` makes the
subproblem's optimum very slightly wrong (and its duals correspondingly), which the
strong-duality check would not flag because the subproblem is internally consistent -- it
is simply not the second stage of the `y` the master chose.
"""
function _aggregate_od_route_benders_extract_incumbent(
        ::AggregateODRouteJointRoutingAssignmentMasterFormulation,
        build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model,
    )::Vector{Float64}
    return Float64[v > 0.5 ? 1.0 : 0.0 for v in JuMP.value.(m[:y])]
end

function _aggregate_od_route_benders_solve_subproblem(
        ::AggregateODRouteJointRoutingAssignmentMasterFormulation,
        build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model,
        incumbent, solver::BendersSolver,
    )
    return _solve_joint_routing_assignment_benders_subproblems(m, incumbent, solver)
end

function _aggregate_od_route_benders_add_cut!(
        formulation::AggregateODRouteJointRoutingAssignmentMasterFormulation,
        build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model,
        subproblem_result, solver::BendersSolver,
    )::Int
    return _add_joint_routing_assignment_benders_cuts!(
        m, subproblem_result, formulation.cut_mode,
    )
end
