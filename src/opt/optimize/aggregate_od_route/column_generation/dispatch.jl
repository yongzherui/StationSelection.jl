"""
`CGSolver` hook dispatch, shared by every `AggregateODRouteMap`-based CG master
(`AggregateODRouteBaseFormulation`, `AggregateODRouteJointRoutingAssignmentFormulation`).

`CGSolver`'s 4 hooks (`opt/solvers/cg_solver.jl`) are fixed-signature
`(build_result::BuildResult, mapping, m::JuMP.Model, ...)` calls, and both live
aggregate-OD-route formulations share the exact same `mapping::AggregateODRouteMap` type --
so a naive `price_columns(::BuildResult, ::AggregateODRouteMap, ::JuMP.Model, ...)` method
per formulation would collide (Julia can't have two methods with an identical signature; one
would silently overwrite the other at load time). Each hook here is instead a one-line
dispatcher that reads the formulation stashed on the model at build time
(`m[:aggregate_od_route_formulation]`, set by both
`optimize/aggregate_od_route/column_generation/build_base.jl` and
`build_joint_routing_assignment.jl`) and re-dispatches on *its* type -- genuine multiple
dispatch, just one level down from where `CGSolver` itself dispatches. The real per-formulation
logic lives in `_aggregate_od_route_{extract_duals,price_columns,add_columns!,integer_recovery_build}`
methods defined alongside each formulation's own CG machinery, not here.
"""

function extract_duals(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model)
    return _aggregate_od_route_extract_duals(m[:aggregate_od_route_formulation], build_result, mapping, m)
end

function price_columns(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver)
    return _aggregate_od_route_price_columns(m[:aggregate_od_route_formulation], build_result, mapping, m, duals, solver)
end

function add_columns!(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model, columns)::Int
    return _aggregate_od_route_add_columns!(m[:aggregate_od_route_formulation], build_result, mapping, m, columns)
end

function integer_recovery_build(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model)::BuildResult
    return _aggregate_od_route_integer_recovery_build(m[:aggregate_od_route_formulation], build_result, mapping, m)
end
