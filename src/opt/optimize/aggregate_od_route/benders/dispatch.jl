"""
Top-level `run_opt` dispatch for `BendersSolver` over `AggregateODRouteModel`, routing
to the appropriate decomposition-specific `_run_aggregate_od_route_*` function based on
`solver.decomposition` and whether the model uses the NearestOpen assignment policy.
"""

function _warn_if_uncertified_standard_cut(solver::BendersSolver)::Bool
    if solver.cut_derivation == :standard && !solver.reprice_subproblem &&
       solver.decomposition isa Union{BendersY, BendersYZ, BendersYZH}
        @warn "Benders cut_derivation=:standard with reprice_subproblem=false is not a correctness-certified solve; use this combination for diagnostics only. Use the default cut_derivation=:zero_completion or enable repricing."
        return true
    end
    return false
end

function run_opt(
    instance::StationSelectionData,
    formulation::AggregateODRouteModel,
    solver::BendersSolver,
)
    _warn_if_uncertified_standard_cut(solver)
    if formulation.assignment_policy isa NearestOpenAggregateODAssignmentPolicy
        solver.decomposition isa BendersY &&
            return _run_aggregate_od_route_nearest_open_benders_y(instance, formulation, solver)
        solver.decomposition isa BendersXY &&
            return _run_aggregate_od_route_nearest_open_benders_xy(instance, formulation, solver)
        solver.decomposition isa BendersYZ &&
            return _run_aggregate_od_route_nearest_open_benders_yz(instance, formulation, solver)
        solver.decomposition isa BendersYZH &&
            return _run_aggregate_od_route_nearest_open_benders_yzh(instance, formulation, solver)
        throw(ArgumentError("unsupported Benders decomposition $(typeof(solver.decomposition))"))
    end
    solver.decomposition isa BendersY &&
        throw(ArgumentError("AggregateODRouteModel free assignment Benders supports BendersXY only; BendersY is unsupported"))
    solver.decomposition isa BendersXY &&
        return _run_aggregate_od_route_free_benders_xy(instance, formulation, solver)
    throw(ArgumentError("unsupported Benders decomposition $(typeof(solver.decomposition))"))
end
