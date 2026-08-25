"""
`generate_jobs.jl` -- expands Study 1's (formulation, operating-setting) grid into
`config/jobs.tsv`, the job list `run_benchmark.jl` reads one row of per array task (see
`README.md` for the grid and `submit_benchmark.sh` for how a row gets selected via
`SLURM_ARRAY_TASK_ID`).

Grid (see README.md "Method"):
  - AggregateODRouteBaseFormulation / DirectMIPSolver / baseline
  - AggregateODRouteJointRoutingAssignmentFormulation / CGSolver / baseline
  - AggregateODRouteJointRoutingAssignmentFormulation / CGSolver / short max_wait_time
  - AggregateODRouteJointRoutingAssignmentFormulation / CGSolver / tight detour_factor
  - AggregateODRouteJointRoutingAssignmentFormulation / CGSolver / max_stops=3

crossed with whatever fixed representative instance(s) the study settles on (one row of
`config/jobs.tsv` per (instance, formulation, setting) combination).

Output: `config/jobs.tsv`, tab-separated, header row + one row per job -- same
convention as `../../scripts/generate_zhuzhou_job_list.jl`'s job lists (header at row 0,
`submit_benchmark.sh` selects row `SLURM_ARRAY_TASK_ID + 1` via `sed`).

TODO: not implemented -- pick the fixed instance(s) (likely one or two calls to
`generate_zhuzhou_data`, see Study 5's `run_benchmark.jl` for the current-API shape),
decide the concrete short-max_wait_time / tight-detour_factor parameter values, then
write the TSV.
"""

# TODO: implement.
