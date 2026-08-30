# Study 5 — Exact CG scalability: serial vs. parallel scenario pricing

## Objective

Measure how the Joint formulation's exact, dual-certified CG solver scales with candidate
stations, passengers, and scenarios — and whether **parallelising scenario pricing lets it
certify cells a serial run cannot within the same budget.**

## The comparison, and why it is apples to apples

Both arms are held to the **same 300 s wall budget per pricing round** and the same 6 h
total solve budget. The single difference is `CGSolver(parallel_scenario_pricing=...)`:

| Arm | CPUs / threads | Per-scenario search inside a 300 s round |
| --- | --- | --- |
| `serial` | 1 | `300 / n_scenarios` — searches run back to back, so the round's wall is their **sum** |
| `parallel` | `n_scenarios` | **300 s each** — searches overlap, so the round's wall is their **max** |

So the parallel arm does up to `n_scenarios ×` more label search *for the same round wall*.
That is the mechanism under test: parallelism is not merely finishing sooner, it is fitting
enough search into the budget to **certify**. Gurobi is pinned to one thread on both arms,
so scenario pricing is the only thing that parallelises.

Measured on a 3-scenario pilot at s=3 (20 s rounds, 240 s total): serial's longest single
search was 8.25 s (its `20/3` slice) against parallel's 20.82 s (the full round); total
search work 220 s vs 610 s — a **2.77× work ratio** — for 235 s vs 251 s of wall, yielding
3496 vs 7476 columns and a better objective.

## Grid

One axis swept at a time, both arms on every cell:

- stations: `n = 10, 20, 30` at `p=16`, `s=3`  (**n=40 dropped**, see below);
- passengers: `p = 8, 16, 24, 32` at `n=20`, `s=3`;
- scenarios: `s = 3, 6, 9, 12` at `n=20`, `p=16` per scenario;
- `max_stops = 10`, seeds 42–51.

**220 jobs** (stations 3 values, passengers and scenarios 4 each, × 10 seeds × 2 arms), one
process each. Within each `config/*_jobs.tsv` the arm varies slowest — stations is rows
1–30 serial / 31–60 parallel; passengers and scenarios are 1–40 / 41–80 — and within an arm
the axis value varies before the seed.

### Why n=40 was dropped

The archived serial run
(`experiments/2026-08-30_study5_scaling_exact_cg_SUPERSEDED_serial_only`) gave each
scenario 300 s — **exactly what this study's parallel arm gives each scenario** — so it is
a direct forecast of what the parallel arm can certify. It certified **0/10 at n=40**, with
9/10 not even writing a row; every other value certified at least partially (stations 30:
1/10, passengers 24/32: 5/10 and 2/10, scenarios 6/9/12: 9, 7 and 4 of 10). n=40 is
therefore out of reach for both arms and buys only queue time.

The remaining frontier cells are deliberately **kept**: they are exactly where the parallel
arm has something to prove, because the serial arm gets only `300/n_scenarios` per scenario
there. Dropping `scenarios=12` in particular would remove the study's headline cell — the
12-thread one, where parallel does 12× the search of serial for the same round wall.

The stations and passengers axes hold `s=3`, so their parallel arm shows a fixed ~3×; the
**scenarios axis is the headline**, since the parallel arm's thread count grows with `s`.

## Budgets

| Setting | Value | Scope |
| --- | --- | --- |
| `time_limit_sec` | 300 s | wall for one pricing round, **both arms** |
| `certifying_time_limit_sec` | 3600 s | wall for one certifying round |
| `total_time_limit_sec` | 21600 s (6 h) | the CG loop; on expiry a row is still written, flagged uncertified |
| `SolverOptions.time_limit_sec` | 300 s | each master LP/IP solve |
| SLURM `--time` | 6 h 30 m | budget + recovery MIP + final master re-solve |
| Memory | 24 GB | per task, **both arms** |

Memory is 24 GB on *both* arms deliberately. The parallel arm holds `n_scenarios` label
search frontiers live where serial holds one, and the archived run already peaked at
6.1–10.8 GB of 16 GB at s=12. Giving the arms different limits would confound the
measurement: Julia sizes its GC heap against the cgroup limit, so GC time — which lands in
the wall clock being compared — would differ for reasons unrelated to threading.

The round wall bound holds only while each scenario has its own thread. With fewer threads
the searches run in waves and a round can reach `ceil(s / nthreads) ×` the budget, so
`run_benchmark.jl` **hard-fails** a `parallel` row given fewer threads than its table asks
for — a silently-serial "parallel" run would look like a null result rather than a
misconfiguration. `submit_benchmark.sh` derives `JULIA_NUM_THREADS` from
`SLURM_CPUS_PER_TASK`, so the allocation and Julia can never disagree.

## Running

`--cpus-per-task` is fixed per submission, so the parallel arm needs one submission per
distinct `n_scenarios`. From this directory:

```bash
julia --project=../.. generate_jobs.jl
mkdir -p slurm_logs

# --- serial arm (1 CPU). stations has 3 axis values, the others 4 ---
sbatch --job-name=study5_stations_serial   --array=1-30 --cpus-per-task=1 submit_benchmark.sh stations
sbatch --job-name=study5_passengers_serial --array=1-40 --cpus-per-task=1 submit_benchmark.sh passengers
sbatch --job-name=study5_scenarios_serial  --array=1-40 --cpus-per-task=1 submit_benchmark.sh scenarios

# --- parallel arm on the s=3 axes (3 CPUs) ---
sbatch --job-name=study5_stations_parallel   --array=31-60 --cpus-per-task=3 submit_benchmark.sh stations
sbatch --job-name=study5_passengers_parallel --array=41-80 --cpus-per-task=3 submit_benchmark.sh passengers

# --- parallel arm, scenarios axis: one submission per s (threads must match) ---
sbatch --job-name=study5_scen_par_s3  --array=41-50 --cpus-per-task=3  submit_benchmark.sh scenarios
sbatch --job-name=study5_scen_par_s6  --array=51-60 --cpus-per-task=6  submit_benchmark.sh scenarios
sbatch --job-name=study5_scen_par_s9  --array=61-70 --cpus-per-task=9  submit_benchmark.sh scenarios
sbatch --job-name=study5_scen_par_s12 --array=71-80 --cpus-per-task=12 submit_benchmark.sh scenarios
```

Roughly 4,030 CPU-hours if every job runs its full 6 h 30 m; substantially less in
practice, since the easy cells (n=10/20, p=8/16, s=3) converged in minutes on the archived
run. The 12-CPU tasks will queue slowest. `STUDY5_DATA_DIR` /
`STUDY5_OUTPUT_DIR` override the defaults. Aggregate with:

```bash
julia --project=../.. analyze.jl ../experiments/YYYY-MM-DD_study5_scaling_exact_cg
```

## Outputs

`case_comparison.csv` (all 240 jobs; un-run ones as `missing_result` placeholders with
blank measurements), `variant_summary.csv` (per substudy × axis × arm),
`arm_comparison.csv` (**the headline**: certified counts per arm, `certified_delta`, and
`search_work_ratio` = parallel search work ÷ serial), `missing_results.csv`,
`censored_cells.csv` (which limit bound each non-certified cell), and `slides_results.tex`.

Each job also writes `iterations/job_NNNN_<arm>_iterations.csv`, and the summary row carries
`scenario_search_sec_{sum,max,min}` over every `(iteration × scenario)` label search — the
sum-versus-max that distinguishes the arms.

## Status of previous runs

| Date | Design | Result |
| --- | --- | --- |
| 2026-08-25 | pre-dominance-fix | archived `..._SUPERSEDED_pre_dominance_fix` |
| 2026-08-29 | 1800 s per-scenario pricing, 2 h wall, no total budget | 56/120 certified, 60 lost to walltime |
| 2026-08-30 | 6 h total budget, serial only, per-scenario budgets | 78/120 certified — archived `..._SUPERSEDED_serial_only`; **used as the pilot for this grid** |
| — | **this two-arm rewrite** | not yet run |

The 2026-08-30 run remains the evidence base for the n≥30 frontier and the budget-clamp
defect; it is **not comparable** to this rewrite, which uses per-round rather than
per-scenario budgets. Full settings and caveats for every run:
[`../notes/2026-08-30_compute_budgets_of_record.md`](../notes/2026-08-30_compute_budgets_of_record.md).
