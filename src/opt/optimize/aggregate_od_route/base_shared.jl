"""
Shared master-construction core for `AggregateODRouteBaseFormulation`, used by both
`direct/build_base.jl` (`DirectMIPSolver`, exhaustive column pool known before `m` exists)
and `column_generation/build_base.jl` (`CGSolver`, empty pool grown via
`add_aggregate_od_route_base_column!`). Neither solver strategy owns this file -- it lives
here, a sibling of `direct/` and `column_generation/`, for the same reason
`column_generation/dispatch.jl` isn't inside either formulation's own subtree.

What's actually shared: `y`/`x`/`x_walk` variable creation and the
coverage/station-linking/route-link/station-limit/objective wiring -- all identical between
the two builds regardless of `theta`'s own state. `theta` itself is deliberately *not*
created here, only consumed (as a plain argument): Direct wants it in one closed-form batch
(`add_route_variables!`, `variables/routes.jl`), fully populated *before* `route_link` is
built, so that constraint's sum-of-theta is written once from a complete pool. CG wants
`route_link` built against an *empty* `theta`, so every row degrades to `x <= 0` and gets
patched up later, one column at a time, via `add_aggregate_od_route_base_column!`'s
`set_normalized_coefficient` calls. Both are valid inputs to the exact same constraint-
building code -- `add_aggregate_od_route_base_route_linking_constraints!` only ever reads
whatever `theta`/`mapping.columns_by_pair` already contain -- so each caller creates
`theta` however its own strategy requires and passes it in already in that state.
"""

"""
    _aggregate_od_route_base_master_core!(m, data, mapping, l, formulation, theta;
        relax_integrality=false) -> (y, x, x_walk, route_link, variable_counts, constraint_counts)

`theta::Dict{Tuple{Int,Int}, VariableRef}` is supplied by the caller, already in whatever
state its own build strategy requires at this point (fully populated for Direct, empty for
CG) -- see this file's module docstring for why creating it here isn't an option.
`relax_integrality` threads through to every variable family the same way in both callers.

Returns the raw pieces (not a `BuildResult`): each caller still has its own follow-up work
after this -- Direct is done (theta was already complete); CG stashes extra `m[...]`
bookkeeping and runs its seed loop through `route_link`.
"""
function _aggregate_od_route_base_master_core!(
        m::Model,
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        l::Int,
        formulation::AggregateODRouteBaseFormulation,
        theta::Dict{Tuple{Int, Int}, VariableRef};
        relax_integrality::Bool=false,
    )
    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] =
        add_station_selection_variables!(m, data; relax_integrality=relax_integrality)
    y = m[:y]

    x = add_assignment_variables!(m, data, mapping; relax_integrality=relax_integrality)
    variable_counts["x"] = length(x)
    x_walk = add_walk_variables!(m, data, mapping; relax_integrality=relax_integrality)
    variable_counts["x_walk"] = length(x_walk)

    constraint_counts = Dict{String, Int}()
    coverage = add_aggregate_od_route_base_coverage_constraints!(m, data, mapping, x, x_walk)
    constraint_counts["coverage"] = length(coverage)
    pickup_link, dropoff_link = add_aggregate_od_route_base_station_linking_constraints!(m, x, y)
    constraint_counts["pickup_link"] = length(pickup_link)
    constraint_counts["dropoff_link"] = length(dropoff_link)
    route_link = add_aggregate_od_route_base_route_linking_constraints!(m, mapping, x, theta)
    constraint_counts["route_link"] = length(route_link)
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, l; equality=true)
    endpoint_feasibility = add_aggregate_od_route_endpoint_feasibility_constraints!(m, data, mapping, y)
    constraint_counts["endpoint_feasibility"] = length(endpoint_feasibility)

    set_aggregate_od_route_base_objective!(
        m, data, mapping, x, x_walk, theta,
        formulation.walk_cost_weight, formulation.route_regularization_weight, formulation.repositioning_time,
    )

    return y, x, x_walk, route_link, variable_counts, constraint_counts
end
