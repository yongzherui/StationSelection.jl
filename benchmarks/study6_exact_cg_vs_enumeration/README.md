> **Corrected 2026-08-25.** The `cg_exact` arm now runs
> `AggregateODRouteJointRoutingAssignmentFormulation` under `CGSolver` -- the production CG
> path -- against `AggregateODRouteBaseFormulation` under `DirectMIPSolver` for
> `enumeration`. It previously ran *Base* under `CGSolver`, which compared a solver against
> itself on a formulation nobody uses that way. Study 1 shows Base and Joint reach the same
> integer optimum, so the comparison isolates the solve algorithm. The pre-correction runs
> are archived under `benchmarks/experiments/2026-08-25_study6_basecg_SUPERSEDED/`; they are
> also affected by the Base+CG livelock fixed the same day
> (`notes/2026-08-25_study6_cg_livelock_stale_tau_columns.md`).

# Study 6 — Exact column generation versus exhaustive enumeration

This study compares two solution strategies for the same
`AggregateODRouteBaseFormulation`:

- `cg_exact`: exact, dual-certified column generation with integer recovery;
- `enumeration`: exhaustive route enumeration followed by `DirectMIPSolver`.

Both use `max_stops=4`, `p=16` OD pairs per scenario, `s=3` scenarios, and one CPU.
The station sweep is `n = 10, 15, 20`, with seeds 42–51. Each method runs in a separate
process, producing 60 jobs. Each task has a two-hour SLURM limit; enumeration also has a
1800-second/two-million-route guard. Missing
rows after scheduler OOMs remain visible in the analyzer output.

The runner is idempotent: if its deterministic `job_<id>_<method>.csv` already exists,
it exits successfully before generating data or starting a solver. Resubmitting the full
array therefore preserves completed results and runs only missing jobs.

```bash
julia --project=. benchmarks/study6_exact_cg_vs_enumeration/generate_jobs.jl
cd benchmarks/study6_exact_cg_vs_enumeration
mkdir -p slurm_logs
sbatch --array=1-60 submit_benchmark.sh
```

Raw results default to
`benchmarks/experiments/YYYY-MM-DD_study6_exact_cg_vs_enumeration/`. Aggregate with:

```bash
cd ../..
julia --project=. benchmarks/study6_exact_cg_vs_enumeration/analyze.jl \
    benchmarks/experiments/YYYY-MM-DD_study6_exact_cg_vs_enumeration
```


## Run history

Raw output goes to `benchmarks/experiments/<date>_<slug>/`, curated output to
`benchmarks/results/<date>_<slug>/`; both directories are stamped with the date the run
was *submitted*, so a re-run never overwrites its predecessor. Note that `.gitignore`
carries `*.csv` and `experiments/`, so **only `slides_results.tex` is committed** -- every
CSV under `results/` exists solely on the filesystem that produced it.

| Date | Array | Tasks | Config at that revision | Outcome |
| --- | --- | --- | --- | --- |
| 2026-08-25 | (Base+CG) | 60 | superseded arm definition | archived `2026-08-25_study6_basecg_SUPERSEDED` |
| 2026-08-25 | `21243072` (re-run) | 60 | Joint+CG vs Base+Direct, 900 s, 2 h walltime | archived `..._SUPERSEDED_pre_dominance_fix`; 55/60 certified; 5 livelock + 5 infeasible-master |
| 2026-08-29 | `21570519` | 60 | 1800 s pricing, 2 h walltime, 1 CPU / 16G | in flight |

Two solver bugs blocked cells in the 2026-08-25 run and are fixed for 2026-08-29: the
stale-tau CG livelock (`6f8db0b`, 5 cells spinning to `max_iterations=1000`) and the
scenario-blind column dedup (`0bb2d43`, 5 cells returning `INFEASIBLE` on iteration 1).
Neither was a budget problem, so neither should have been reported as a timeout.
`de5d56b` also added the per-iteration `iterations/job_NNNN_*.csv` log.
**This runner is idempotent** -- it exits early if its `job_<id>_<method>.csv` already
exists, so resubmitting the full array re-runs only missing jobs.
