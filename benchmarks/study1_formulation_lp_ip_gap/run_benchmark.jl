"""
`run_benchmark.jl <job_line>` -- one Study 1 job: build/reuse a fixed representative
instance, run one (formulation, operating-setting) cell from `config/jobs.tsv` (see
`generate_jobs.jl`), and append one result row to this job's output CSV under
`../../experiments/<date>_study1_formulation_lp_ip_gap/` (see `../README.md` for the
output-location convention).

Current-API shape to follow (confirmed against `test/opt/test_aggregate_od_route_base_cg.jl`
and `test/opt/test_cg_solver_integer_recovery.jl` -- this is the "blessed example," not
anything under `../../scripts/`, which is stale):

```julia
data        = create_station_selection_data(stations, requests, walking_costs; routing_costs, scenarios)
problem     = StationSelectionProblem(data, k; max_walking_distance=800.0)

# Base arm:
formulation = AggregateODRouteBaseFormulation(; max_stops=...)
solver      = DirectMIPSolver(; config=SolverOptions(silent=true))
result      = run_opt(problem, formulation, solver)
z_lp = <relax the enumerated pool's LP -- TODO: confirm the exact call shape for this;
        DirectMIPSolver may need its own LP-relaxation entry point, unlike CGSolver's
        recover_integer_solution path below>
z_ip = result.objective_value

# Joint arm -- recover_integer_solution=true is required, it's the only way
# OptResult.metadata["cg_lp_objective_value"] gets populated:
formulation = AggregateODRouteJointRoutingAssignmentFormulation(; max_stops=..., max_wait_time=..., detour_factor=...)
solver      = CGSolver(; config=SolverOptions(silent=true), recover_integer_solution=true)
result      = run_opt(problem, formulation, solver)
z_lp = result.metadata["cg_lp_objective_value"]
z_ip = result.objective_value
```

Output row: `{instance, formulation, setting, z_lp, z_ip, gap, termination_status,
runtime_sec}` -- see `README.md`'s Metrics section.

TODO: not implemented -- confirm the Base-arm LP-relaxation call shape (DirectMIPSolver
doesn't go through `recover_integer_solution`; check whether relaxing the built JuMP
model directly, e.g. via `JuMP.relax_integrality!`, is the right approach, or whether
there's an existing helper), then implement the two-arm solve + row-write body.
"""

# TODO: implement.
