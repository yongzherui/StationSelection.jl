"""
`build_model` for `StationSelectionProblem` paired with `AggregateODRouteBaseFormulation`
-- the `y`/`x`/`θ` master: `y` (station), `x` (OD-to-station-pair assignment, decoupled
from routing), `θ` (route activation, from an exhaustively enumerated column pool, `θ`
reused from the sibling non-joint aggregate-OD-route engine's own
`add_aggregate_od_route_theta_variables!`/`AggregateODRouteColumn`). Solved directly as
one MIP (`DirectMIPSolver`), no iterative pricing loop -- see
`opt/optimize/aggregate_od_route/enumeration.jl` for how the `θ` pool is built up front.
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteBaseFormulation,
                solver::DirectMIPSolver) -> BuildResult

Free assignment only -- the coverage/linking helpers here assume that semantics (matches
`AggregateODRouteJointRoutingAssignmentFormulation`).
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteBaseFormulation,
        solver::DirectMIPSolver,
    )::BuildResult
    data = problem.data
    columns = enumerate_aggregate_od_route_columns(problem, formulation, data)
    mapping = create_aggregate_od_route_map(problem, formulation, data; initial_columns=columns)

    m = Model(() -> Gurobi.Optimizer())

    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    y = m[:y]

    variable_counts["theta"] = add_aggregate_od_route_theta_variables!(m, data, mapping)
    theta = m[:theta_compat]

    x = add_aggregate_od_route_base_assignment_variables!(m, data, mapping)
    variable_counts["x"] = length(x)

    constraint_counts = Dict{String, Int}()
    coverage = add_aggregate_od_route_base_coverage_constraints!(m, data, mapping, x)
    constraint_counts["coverage"] = length(coverage)
    pickup_link, dropoff_link = add_aggregate_od_route_base_station_linking_constraints!(m, x, y)
    constraint_counts["pickup_link"] = length(pickup_link)
    constraint_counts["dropoff_link"] = length(dropoff_link)
    constraint_counts["route_link"] =
        length(add_aggregate_od_route_base_route_linking_constraints!(m, mapping, x, theta))
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.l; equality = true)

    set_aggregate_od_route_base_objective!(
        m, data, mapping, x, theta,
        formulation.walk_cost_weight, formulation.route_regularization_weight, formulation.repositioning_time,
    )

    extra_counts = Dict{String, Int}("routes_enumerated" => length(columns))
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
