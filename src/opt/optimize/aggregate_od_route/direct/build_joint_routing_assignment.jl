"""
`build_model` for `StationSelectionProblem` paired with
`AggregateODRouteJointRoutingAssignmentFormulation` and `DirectMIPSolver` -- the same
`y`/`x_walk`/`θ` master `CGSolver`'s build (`column_generation/build_joint_routing_assignment.jl`)
solves, but seeded with the *exhaustive* column pool
(`enumerate_joint_routing_assignment_columns`, `label_setting/joint_routing_assignment/exact/enumeration.jl`)
instead of the two-stop seed, and solved directly with no CG loop -- this formulation's
counterpart to `AggregateODRouteBaseFormulation`'s own `direct/build_base.jl`.

Reuses `_build_joint_routing_assignment_model` (the same shared master-construction body
`CGSolver`'s build and its own integer-recovery rebuild already share) rather than
building a third copy of the `y`/`x_walk`/`θ` wiring -- the only thing genuinely new here
is which columns seed `theta` and that `relax_integrality` is caller-controlled instead of
always `true`.
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteJointRoutingAssignmentFormulation,
                solver::DirectMIPSolver; relax_integrality=false,
                max_routes=10_000, time_limit_sec=30.0) -> BuildResult

`relax_integrality=true` declares every variable family continuous instead of binary, over
the same exhaustively enumerated column pool -- a genuine LP relaxation of the direct-solve
master, for benchmarks that want this formulation's own true direct-solve LP/IP pair (see
`benchmarks/study1_formulation_lp_ip_gap`) without going through `CGSolver` at all.

`max_routes`/`time_limit_sec` pass straight through to `enumerate_joint_routing_assignment_columns`
-- exposed here (unlike `AggregateODRouteBaseFormulation`'s own `DirectMIPSolver` build,
whose pool has stayed comfortably under the enumerator's own defaults) because even the
maximal, elementarity-preserving expansion this enumerator does can exceed 10_000 at small
`max_stops` (measured: 16,320 columns at `max_stops=4` on a 10-station/8-pair instance).
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteJointRoutingAssignmentFormulation,
        solver::DirectMIPSolver;
        relax_integrality::Bool=false,
        max_routes::Int=10_000,
        time_limit_sec::Float64=30.0,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)
    aggregate_od_route_validate_feasible_coverage(data, mapping)

    columns = enumerate_joint_routing_assignment_columns(
        problem, formulation, data; max_routes=max_routes, time_limit_sec=time_limit_sec,
    )
    return _build_joint_routing_assignment_model(
        data, mapping, problem.k, formulation;
        relax_integrality=relax_integrality, initial_columns=columns,
    )
end
