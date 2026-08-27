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
process, producing 60 jobs. Each task has a one-hour SLURM limit; enumeration also has a
900-second/two-million-route guard. Missing
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
