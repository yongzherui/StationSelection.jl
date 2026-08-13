"""
`AggregateODRouteBendersYXFormulation`'s per-scenario subproblem: for `y` fixed to the
master's current incumbent `y_hat`, reuses `AggregateODRouteBaseFormulation`'s own
coverage/linking/objective code verbatim (`base/assignment.jl`, `base/coverage.jl`,
`base/linking.jl`, `objectives/aggregate_od_route/base/assembly.jl`) -- the only
structural difference is `y` being a fixed variable
(`add_fixed_station_selection_variables!`) instead of a free binary, and `x`/`theta`
relaxed to continuous so the LP has valid duals off `y`'s fixing constraints.

Solved one scenario at a time (never all scenarios jointly) so `MultiCut` can read a
separate objective/dual set per scenario without needing per-scenario fixed-`y` copies
inside one shared model; `SingleCut` just sums however many of these solves its cut
group spans.
"""

"""
    _build_benders_yx_subproblem(data, mapping, formulation, y_hat, scenario) -> (m::Model, y::Vector{VariableRef}, fix_cons::Dict{Int,ConstraintRef})

Builds (but does not solve) the fixed-`y`, single-scenario LP relaxation. `Method=1`
(dual simplex) / `Presolve=0` mirror the historical Benders subproblem's own Gurobi
settings for reliable duals off a possibly-degenerate covering LP. Reuses the
pre-existing `add_fixed_station_selection_variables!`
(`variables/aggregate_od_route/benders/subproblem.jl`, kept live but otherwise unwired
since the historical `BendersY`/`XY`/`YZ`/`YZH` decompositions were archived) rather than
declaring a second fixed-`y` builder -- it fixes `y` via an explicit equality
`ConstraintRef` (not `JuMP.fix`), which is what `add_benders_cut!` reads its dual off.
"""
function _build_benders_yx_subproblem(
    data::StationSelectionData,
    mapping::AggregateODRouteMap,
    formulation::AggregateODRouteBendersYXFormulation,
    y_hat::Vector{Float64},
    scenario::Int,
)::Tuple{Model, Vector{VariableRef}, Dict{Int, ConstraintRef}}
    sub = Model(() -> Gurobi.Optimizer())
    set_silent(sub)
    set_optimizer_attribute(sub, "Method", 1)
    set_optimizer_attribute(sub, "Presolve", 0)

    y, fix_cons = add_fixed_station_selection_variables!(sub, data, y_hat)
    x = add_aggregate_od_route_base_assignment_variables!(
        sub, data, mapping; scenarios=[scenario], relax_integrality=true,
    )
    add_aggregate_od_route_theta_variables!(
        sub, data, mapping; relax_integrality=true, scenarios=[scenario],
    )
    theta = sub[:theta_compat]
    add_aggregate_od_route_base_coverage_constraints!(sub, data, mapping, x; scenarios=[scenario])
    add_aggregate_od_route_base_station_linking_constraints!(sub, x, y)
    add_aggregate_od_route_base_route_linking_constraints!(sub, mapping, x, theta)
    set_aggregate_od_route_base_objective!(
        sub, data, mapping, x, theta,
        formulation.walk_cost_weight, formulation.route_regularization_weight, formulation.repositioning_time,
    )
    return sub, y, fix_cons
end
