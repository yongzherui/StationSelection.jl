# Study 2 — DARP-style versus specialized exact pricing

## Question

How much does the explicit DARP-style label representation cost relative to
the specialized exact running-maximum representation for the uncapacitated
joint-routing-assignment problem?

- `pricing_mode=:darp` explicitly boards `(p,j,k)` commitments, carries an
  onboard/open-request set and ride clocks, and must discharge those requests.
- `pricing_mode=:exact` certifies assignment opportunities from route visits
  and retains each passenger's best reward through reward layers.

Both are exact for the same master problem when pricing exhausts. This is a
representation/search comparison, not a capacity ablation: both modes are
uncapacitated.

## Independent-job design

Every `(generated instance, pricing_mode)` pair is a separate row in
`config/jobs.tsv` and therefore a separate SLURM job and Julia process. Exact
and DARP never run sequentially in one process. This prevents one mode from
benefiting from the other's JIT compilation, caches, or memory state.

The grid in `generate_jobs.jl` is:

- Zhuzhou top 10, 15, and 20 stations;
- 8 OD pairs, one scenario;
- seeds 42–51;
- `max_stops=10`;
- modes `exact` and `darp`;
- 1800-second limit for each scenario's label search.

This produces 60 jobs (3 station counts x 10 seeds x 2 modes). `n_pairs` is held
fixed across the sweep so the size axis isolates the candidate-station count `|J|`
rather than moving demand and stations together. Note that `k = ceil(n/2)` still
tracks `n`, so the `n=20` cells build 10 stations for the same 8 OD pairs and may
turn out easier rather than harder — read that cell with the station/demand ratio in
mind. Edit the constants in `generate_jobs.jl` to expand the grid further.

Both modes use the production `CGSolver`, normal two-stop seed columns, and
the same formulation parameters: `k=ceil(n/2)` (5, 8, and 10), 600-second
walking cap, route/walk weights 10.0/0.1, 20-second repositioning, 900-second
pickup horizon, and detour factor 2.0. Note the 900 s pickup horizon
(`max_wait_time`) is a *model* parameter and is unrelated to the 1800 s pricing
budget above.

## Runtime definition

The benchmark's `runtime_sec` is **only** `OptResult.runtime_sec` returned by:

```julia
result = run_opt(problem, formulation, solver)
runtime_sec = result.runtime_sec
```

It does not use `@elapsed`, `time()`, scheduler duration, data-generation
time, or model-build time. Under `CGSolver`, this measures the repeated RMP
optimization, pricing, and column-addition loop implemented by `run_opt`.

Both arms run with `recover_integer_solution=true`, so `runtime_sec` covers the CG
loop **and** the integer-recovery MIP solve. It is therefore not comparable to a
run made without recovery.

## LP/IP gap

Each job records `z_lp` (the converged CG LP bound, `"cg_lp_objective_value"`),
`z_ip` (the recovered integer objective), and `gap = (z_ip - z_lp) / |z_ip|`.

Run to exhaustion both pricing modes must reach the **same `z_lp`** -- they are both
exact for the same master, so `analyze.jl` treats a certified disagreement there as a
correctness failure and errors. They may legitimately reach **different `z_ip`**:
integer recovery is a restricted-master heuristic over whichever column pool each
pricer happened to generate, so a divergence is a real observation about pool quality,
not a bug. `analyze.jl` records it and warns rather than failing.

## Running

Generate and submit the grid:

```bash
julia --project=. benchmarks/study2_passenger_max_ablation/generate_jobs.jl
mkdir -p benchmarks/study2_passenger_max_ablation/slurm_logs
sbatch --array=1-60 benchmarks/study2_passenger_max_ablation/submit_benchmark.sh
```

`STUDY2_DATA_DIR` and `STUDY2_OUTPUT_DIR` override the default data and raw
output locations. Aggregate completed jobs with:

```bash
julia --project=. benchmarks/study2_passenger_max_ablation/analyze.jl \
    benchmarks/experiments/YYYY-MM-DD_study2_passenger_max_ablation
```

## Outputs and interpretation

Each job writes one CSV containing its mode, instance parameters, LP and IP
termination status, `z_lp`/`z_ip`/`gap`, `run_opt` runtime, CG iterations,
convergence, pricing exhaustion, column count, and seed-column count.

`analyze.jl` produces `case_results.csv`, `paired_comparison.csv`,
`variant_summary.csv`, and `slides_results.tex`. Because the grid sweeps `n`,
`variant_summary.csv` and the LaTeX row macros are emitted **per `(n_stations,
pricing_mode)` cell** — means pool the 10 seeds only, never across station counts,
since averaging runtimes over instances of different difficulty is meaningless. The
macros are correspondingly named `\StudyTwoExactRowNFifteen`,
`\StudyTwoDarpRowNTwentyFive`, and so on, each expanding to
`n_stations & n_certified & runtime & iterations & columns`.

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

Only rows with both `cg_converged=true` and `cg_pricing_exhausted=true` count
as certified. A master may have `termination_status=OPTIMAL` even when a
pricing search times out; such a result is deliberately not treated as a
pricing certificate or objective-equivalence observation.


## Run history

Raw output goes to `benchmarks/experiments/<date>_<slug>/`, curated output to
`benchmarks/results/<date>_<slug>/`; both directories are stamped with the date the run
was *submitted*, so a re-run never overwrites its predecessor. Note that `.gitignore`
carries `*.csv` and `experiments/`, so **only `slides_results.tex` is committed** -- every
CSV under `results/` exists solely on the filesystem that produced it.

| Date | Array | Tasks | Config at that revision | Outcome |
| --- | --- | --- | --- | --- |
| 2026-08-25 | `21243070` (re-run) | 60 | n {15,20,25}, 900 s pricing, 1 h walltime | archived `..._SUPERSEDED_pre_dominance_fix`; 40/60 certified, 20 `darp` clock-2 losses |
| 2026-08-29 | `21570513` | 60 | n {10,15,20}, 1800 s pricing, 2 h walltime, 4 CPU / 8G | in flight |

`de5d56b` raised the pricing cap 900 -> 1800 s and moved the grid off `n=25` because the
corrected dominance rule is provably weaker; re-running the old configuration unchanged
risked certifying *fewer* cells than 2026-08-25, not more. The two runs' cells therefore
overlap only at n=15 and n=20, and even there the pricing budget differs -- do not pool them.
