"""
`build_model` for `StationSelectionProblem` paired with `AggregateODRouteBaseFormulation`
-- the `y`/`x`/`θ` master: `y` (station), `x` (OD-to-station-pair assignment, decoupled
from routing), `x_walk` (direct-walk, station-free), `θ` (route activation, from an
exhaustively enumerated column pool, built with the generic, reusable
`add_route_variables!`/`AggregateODRouteColumn` (`variables/routes.jl`)). No unserved-demand
slack: `aggregate_od_route_validate_feasible_coverage`
(`data/maps/aggregate_od_route_map.jl`) proves every demand group has a real coverage
option before any variable is built. Solved directly as one MIP (`DirectMIPSolver`), no
iterative pricing loop -- see `opt/optimize/aggregate_od_route/enumeration.jl` for how the
`θ` pool is built up front. `y`/`x`/`x_walk`/coverage/station-linking/route-link/objective
wiring is shared with `CGSolver`'s own build for this formulation via
`_aggregate_od_route_base_master_core!` (`optimize/aggregate_od_route/base_shared.jl`).
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
    # ---- 1. Parameters ----
    # Column pool and mapping are built up front here (no CGSolver hooks need scalars
    # stashed on `m` to reconstruct anything later, unlike the joint routing+assignment
    # master), so unlike that file's own "Parameters" section this is entirely
    # pre-`m`: the enumerated `θ` pool, the resulting `AggregateODRouteMap`, and the
    # feasibility check that every demand group has a real coverage option.
    data = problem.data
    columns = enumerate_aggregate_od_route_columns(problem, formulation, data)
    mapping = create_aggregate_od_route_map(problem, formulation, data; initial_columns=columns)
    aggregate_od_route_validate_feasible_coverage(data, mapping)

    m = Model(() -> Gurobi.Optimizer())

    # ---- 2. Variables ----
    # `theta` is batch-created (not empty) *before* the shared core below, so
    # `_aggregate_od_route_base_master_core!`'s call into
    # `add_aggregate_od_route_base_route_linking_constraints!` writes `route_link` closed-form
    # over the complete pool -- see `optimize/aggregate_od_route/base_shared.jl`'s module
    # docstring for why theta creation itself isn't part of that shared function.
    theta_count = add_route_variables!(m, data, mapping)
    theta = m[:route_theta]

    _, _, _, _, variable_counts, constraint_counts =
        _aggregate_od_route_base_master_core!(m, data, mapping, problem.k, formulation, theta)
    variable_counts["theta"] = theta_count

    extra_counts = Dict{String, Int}("routes_enumerated" => length(columns))
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
