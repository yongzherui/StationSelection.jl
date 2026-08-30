"""
`CGSolver` hook dispatch, shared by every `AggregateODRouteMap`-based CG master
(`AggregateODRouteBaseFormulation`, `AggregateODRouteJointRoutingAssignmentFormulation`).

`CGSolver`'s 4 hooks (`opt/solvers/cg_solver.jl`) are fixed-signature
`(build_result::BuildResult, mapping, m::JuMP.Model, ...)` calls, and both live
aggregate-OD-route formulations share the exact same `mapping::AggregateODRouteMap` type --
so a naive `extract_duals(::BuildResult, ::AggregateODRouteMap, ::JuMP.Model)` method per
formulation would collide (Julia can't have two methods with an identical signature; one
would silently overwrite the other at load time). `extract_duals`/`add_columns!`/
`integer_recovery_build` below are each a one-line dispatcher that reads the formulation
stashed on the model at build time (`m[:aggregate_od_route_formulation]`, set by both
`optimize/aggregate_od_route/column_generation/build_base.jl` and
`build_joint_routing_assignment.jl`) and re-dispatches on *its* type -- genuine multiple
dispatch, just one level down from where `CGSolver` itself dispatches. The real
per-formulation logic for those three lives in
`_aggregate_od_route_{extract_duals,add_columns!,integer_recovery_build}` methods defined
alongside each formulation's own CG machinery, not here.

`price_columns` doesn't need that redispatch trick: `_run_pricing_round`
(`label_setting/round.jl`) is a single formulation-agnostic function -- there is
only one method of it, ever, so calling it directly here can't collide with
anything. Formulation-specific pricing behavior is hooks dispatched *inside*
`_run_pricing_round`, not separate top-level methods of `price_columns` itself.
"""

function extract_duals(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model)
    return _aggregate_od_route_extract_duals(m[:aggregate_od_route_formulation], build_result, mapping, m)
end

price_columns(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model, duals,
              solver::CGSolver; time_limit_sec::Real=solver.pricing_time_limit_sec) =
    _run_pricing_round(
        m[:aggregate_od_route_formulation], mapping, m, duals, solver;
        time_limit=time_limit_sec,
    )

function add_columns!(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model, columns)::Int
    return _aggregate_od_route_add_columns!(m[:aggregate_od_route_formulation], build_result, mapping, m, columns)
end

function integer_recovery_build(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model)::BuildResult
    return _aggregate_od_route_integer_recovery_build(m[:aggregate_od_route_formulation], build_result, mapping, m)
end
