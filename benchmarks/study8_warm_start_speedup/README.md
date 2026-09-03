# Study 8 — Does the station-simple warm start pay, and when does it hand off?

Two questions, one arm:

1. **Is elementary-first faster end to end?** `CGSolver(warm_start_pricing_mode =
   :station_simple)` versus the plain exact pricer.
2. **How fast does the elementary phase exhaust?** i.e. how much of the run is phase 1,
   and how much work is left for phase 2 afterwards.

Both come from the same 30 runs: the handoff is instrumented
(`cg_warm_start_sec`, `cg_warm_start_iterations`), and `cg_iteration_log` carries
`pricing_mode` per row so phase 1 and phase 2 are costed separately without a second run.

## The baseline is Study 7, not a re-run

Study 8 runs only the `warm_start` arm. The `exact` baseline is
`benchmarks/experiments/2026-09-02_study7_route_elementarity`, whose runs used this grid,
these budgets, this formulation and this allocation. `cell_id` is formatted identically so
`analyze.jl` joins the two studies directly.

| Setting | Value |
| --- | --- |
| `n_stations` / `n_scenarios` / `max_stops` | 20 / 3 / 10 |
| `n_pairs` | 8, 16, 24 |
| `seed` | 42–51 (10 per regime, 30 jobs) |
| CG budget | 300 s round / 3600 s certifying / 14400 s total |
| Allocation | 3 CPUs, 24 G, parallel scenario pricing, Gurobi 1 thread |
| `recover_integer_solution` | `true` |

**Two confounds this carries**, from comparing across studies rather than within one run:

- *Build drift.* Study 7's `exact` runs predate recording assignment positions in route
  replay. That work is in `_replay_joint_routing_assignment_route`, which runs once per
  accepted column rather than per label, and the dominance scan is ~90% of pricing wall —
  so the delta should sit well below run-to-run noise, but it is not zero.
- *Node variance.* Both studies run on `mit_preemptable`; cells land on whatever hardware
  is free. Per-cell paired ratios absorb instance difficulty but not hardware, which is
  why `analyze.jl` reports the *distribution* of per-cell speedups rather than one pooled
  ratio of totals.

Neither is correctable without re-running the baseline, which was a deliberate call to
save ~23 h of queue time.

## Correctness gate before any timing

Warm start changes only which pricer runs first; phase 2 is the full pricer either way, so
both arms must certify the **same optimum** on a cell. `analyze.jl` checks that first and
excludes any mismatching cell from the speedup — a mismatch is a bug, not a result. It
also asserts every warm run ended with
`cg_optimality_scope == "full_route_universe"`; a run still sitting in
`"elementary_routes_only"` never handed off, and its number would mean something else.

Only cells where **both** arms certified enter the speedup. A budget-stopped run's wall
measures the budget, not the work.

## Why p=24 is kept despite the queue cost

Study 7 measured **88.9%** elementary optima at p=24 against **100%** at p≤16. That makes
p=24 the regime where the elementary universe most likely exhausts with real work still
outstanding — precisely where a speedup could fail to appear, and therefore the most
informative cell for both questions.

## Files

```
generate_jobs.jl      -> config/jobs.tsv   (30 rows)
submit_benchmark.sh   sbatch --array=1-30 submit_benchmark.sh
run_benchmark.jl      one job; writes the metrics row + per-iteration log
analyze.jl            joins to Study 7 and answers Q1/Q2
```

Outputs land in `benchmarks/experiments/<date>_study8_warm_start_speedup/`:
`job_NNNN.csv` (one row per job, including `warm_start_sec`, `phase{1,2}_*`) and
`iterations/job_NNNN_iterations.csv` (per-iteration, with `pricing_mode`).
