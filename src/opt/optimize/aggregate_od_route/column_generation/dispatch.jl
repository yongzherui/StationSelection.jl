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

# `CGSolver.warm_start_pricing_mode` support. Only the joint formulation has a selectable
# pricer, so these read/write `m[:joint_routing_assignment_pricing_mode]` -- the same slot
# `_pricing_build_scenario_context` consults on every pricing call, which is what makes a
# mid-solve switch take effect without rebuilding anything. `AggregateODRouteBaseFormulation`
# falls through to `cg_pricing_mode`'s `nothing` default (no selectable pricer), so asking
# it for a warm start is rejected rather than silently ignored.
cg_pricing_mode(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model) =
    get(m.obj_dict, :joint_routing_assignment_pricing_mode, nothing)

function set_cg_pricing_mode!(build_result::BuildResult, mapping::AggregateODRouteMap,
        m::JuMP.Model, mode::Symbol)
    haskey(m.obj_dict, :joint_routing_assignment_pricing_mode) || throw(ArgumentError(
        "this AggregateODRoute model has no selectable pricer to switch " *
        "(no :joint_routing_assignment_pricing_mode on the model)",
    ))
    m[:joint_routing_assignment_pricing_mode] = mode
    return mode
end

# `CGSolver.certification_pricing_mode` support -- the relaxation-based certify-first
# path, structurally parallel to the warm-start pair above but answering a different
# question: not "which pricer finds columns" but "can we prove none exist without running
# one at all". Only the joint formulation has a relaxation
# (`label_setting/joint_routing_assignment/relaxed_cluster/`), and only when its build
# stashed a station partition -- `AggregateODRouteBaseFormulation`, or a joint model built
# without `relaxed_cluster_count`, falls through to `cg_certification_supported`'s `false`
# default, so asking for certification is rejected rather than silently ignored.
function cg_certification_supported(build_result::BuildResult, mapping::AggregateODRouteMap,
        m::JuMP.Model, mode::Symbol)
    mode in (:relaxed_cluster, :relaxed_cluster_nogood) || return false
    m[:aggregate_od_route_formulation] isa AggregateODRouteJointRoutingAssignmentFormulation ||
        return false
    return haskey(m.obj_dict, :joint_routing_assignment_station_clustering)
end

function cg_certification_round(build_result::BuildResult, mapping::AggregateODRouteMap,
        m::JuMP.Model, duals, solver::CGSolver, mode::Symbol; time_limit_sec::Real)
    if mode === :relaxed_cluster_nogood
        # Same contract, different loop: cut a barren cluster support and retry instead of
        # giving up the moment the relaxation finds an improving cluster route.
        return _run_relaxed_cluster_nogood_certification_round(
            m[:aggregate_od_route_formulation], mapping, m, duals, solver;
            time_limit=Float64(time_limit_sec),
        )
    end
    mode === :relaxed_cluster || throw(ArgumentError(
        "unknown certification_pricing_mode $(repr(mode)) for an AggregateODRoute model -- " *
        "expected :relaxed_cluster or :relaxed_cluster_nogood",
    ))
    return _run_relaxed_cluster_certification_round(
        m[:aggregate_od_route_formulation], mapping, m, duals, solver;
        time_limit=Float64(time_limit_sec),
    )
end
