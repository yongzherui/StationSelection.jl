"""
`run_benchmark.jl <job_line>` -- one Study 5 cell: build an instance at the job's
`(|P|, |J|, |S|, max_stops)` point, run three methods against it under a wall-clock time
limit, and write one row per method to this job's output under
`../../experiments/<date>_study5_scaling_vs_enumeration/` (see `../README.md`).

Instance generation (current, reused as-is via relative `include`):

```julia
include("../../scripts/generate_zhuzhou_instance.jl")
data, meta = generate_zhuzhou_data(data_dir, n_stations, n_pairs; n_scenarios=n_scenarios, seed=seed)
problem = StationSelectionProblem(data, k; max_walking_distance=800.0)
```

Three methods per cell:

1. **CG / `exact/`**: `run_opt(problem,
   AggregateODRouteJointRoutingAssignmentFormulation(; max_stops=...), CGSolver(;
   config=SolverOptions(silent=true, time_limit_sec=...)))` -- time the call, read
   `result.runtime_sec`/`result.termination_status`.
2. **`darp/`**: build `JointRoutingAssignmentDarpPricingData`, call
   `joint_routing_assignment_pricing_by_darp_label_setting(...; time_limit=...)`
   directly (bypassing the CG outer loop -- this study measures pricing itself, not a
   full solve) -- time the call, read the returned `exhausted` flag.
3. **Enumeration**: BLOCKED on `enumerate_joint_routing_assignment_columns` not existing
   yet (see `README.md`'s Prerequisite section) -- once it exists, call it the same way
   `../../src/opt/optimize/aggregate_od_route/direct/build_base.jl` calls
   `enumerate_aggregate_od_route_columns`, under a wall-clock `time_limit_sec`, and
   catch/record the `ArgumentError` it raises on timeout (see that function's own
   contract) as this cell's "timed out" status rather than a hard failure.

Output row (one per method): `{n_stations, n_pairs, n_scenarios, max_stops, method
("cg"/"darp"/"enumeration"), wall_sec, status ("exhausted"/"timed_out"/"oom"/"error"),
objective_or_min_reduced_cost}`.

TODO: not implemented. The CG and darp/ arms have no code prerequisite and can be built
now; the enumeration arm is blocked (see README.md) -- write this file so the
enumeration branch is a clearly isolated `# TODO: needs enumerate_...` block, not a
reason to hold up the other two arms.
"""

# TODO: implement (CG + darp/ arms are unblocked; enumeration arm needs
# enumerate_joint_routing_assignment_columns first -- see README.md).
