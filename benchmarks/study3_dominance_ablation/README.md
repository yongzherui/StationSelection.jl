# Study 3 — Compensated-dominance ablation

## Question

How much does compensated dominance reduce the end-to-end work of exact Joint Routing
Assignment column generation relative to plain subset dominance?

The only experimental switch is
`AggregateODRouteJointRoutingAssignmentFormulation(compensated_dominance=...)`:

- `true`: the compensated/catch-up dominance rule;
- `false`: plain subset dominance.

Both arms use `pricing_mode=:exact` and solve the same master problem. Certified paired
objectives must agree; `analyze.jl` raises an error if they do not.

## Larger instance grid

The grid carries twice Study 2's demand so compensation has room to matter, and sweeps
the station count so that effect can be read against instance size rather than at a
single point:

- Zhuzhou top 15, 20, and 25 stations;
- 16 OD pairs and one scenario;
- seeds 42–51;
- `max_stops=10`;
- 900-second per-scenario pricing limit.

Every `(instance, dominance mode)` pair is a separate Julia process, producing 60 jobs
(3 station counts x 10 seeds x 2 arms). `n_pairs` is held fixed across the sweep so the
size axis isolates the candidate-station count `|J|` rather than moving demand and
stations together; Study 2 sweeps the same three station counts at `p=8`, so the two
studies' station axes line up and demand is the only instance difference between them.
The remaining settings match Study 2: `k = ceil(n/2)` (8, 10, and 13), 600-second
walking cap, route/walk weights 10.0/0.1, 20-second repositioning, 900-second pickup
horizon, and detour factor 2.0.

`n=25` at `max_stops=10` is the least certain cell in this grid: it is the largest
instance either ablation attempts, and uncertified runs are excluded from the summaries
by design. If it fails to certify, the `n=15`/`n=20` cells still stand on their own.

## Metrics

The headline paired metrics are runtime, total labels generated, and peak live labels.
The raw rows also retain termination and pricing certification, CG iterations, columns,
dominance rejection/removal counts, maximum frontier size, and the LP/IP bounds.

Both arms run with `recover_integer_solution=true`, so `runtime_sec` covers the CG loop
**and** the integer-recovery MIP solve, and each job records `z_lp` (the converged CG LP
bound), `z_ip` (the recovered integer objective), and `gap = (z_ip - z_lp) / |z_ip|`.

Run to exhaustion both dominance rules must reach the **same `z_lp`** -- both are exact,
so `analyze.jl` errors on a certified disagreement there. They may legitimately reach
**different `z_ip`**: integer recovery is a restricted-master heuristic over whichever
column pool each arm generated, so a divergence is an observation about pool quality,
not a bug, and is warned rather than failed.

Label statistics are accumulated across every pricing search in the full CG run. Runtime
is `OptResult.runtime_sec`; it is not an outer process timer. Summaries use certified
runs (`cg_converged=true` and `cg_pricing_exhausted=true`) while retaining every raw row,
and are grouped **per `(n_stations, dominance arm)` cell** — means pool the 10 seeds
only, never across station counts.

## Running

```bash
julia --project=. benchmarks/study3_dominance_ablation/generate_jobs.jl
mkdir -p benchmarks/study3_dominance_ablation/slurm_logs
sbatch --array=1-60 benchmarks/study3_dominance_ablation/submit_benchmark.sh
```

`STUDY3_DATA_DIR` and `STUDY3_OUTPUT_DIR` override the defaults. Aggregate with:

```bash
julia --project=. benchmarks/study3_dominance_ablation/analyze.jl \
    benchmarks/experiments/YYYY-MM-DD_study3_dominance_ablation
```

Outputs are `case_results.csv`, `paired_comparison.csv`, `variant_summary.csv`, and
`slides_results.tex`. The LaTeX row macros are per-cell — `\StudyThreeCompensatedRowNFifteen`,
`\StudyThreePlainRowNTwentyFive`, and so on — each expanding to
`n_stations & n_certified & runtime & labels & max_live`.


## Per-iteration log

Every job additionally writes `iterations/job_NNNN_<arm>_iterations.csv` -- one row per
CG iteration, carrying the job's identity keys plus `master_sec`, `pricing_sec`,
`add_columns_sec`, `columns_added`, `cumulative_columns_added`, `master_objective`, and
`master_status` (see `CGSolver`'s docstring for the underlying
`metadata["cg_iteration_log"]` schema). The three-way time split is what makes a
master-bound run distinguishable from a pricing-bound one without a profiler.

These live in a subdirectory so the top-level `*.csv` glob stays one summary row per
job. The summary row also carries scalar roll-ups (`mean_columns_per_iteration`,
`mean_iteration_sec`, `total_pricing_sec`, `pricing_share_of_loop`, ...) plus
`lp_loop_sec` and `integer_recovery_sec`, which separate the CG loop from the recovery
MIP that `runtime_sec` lumps together.

`analyze.jl` concatenates them into `iteration_log.csv` (every row) and
`iteration_profile.csv` (mean columns/timings per `(n_stations, arm, iteration)`).