# `benchmarks/`

Solver-internals benchmark studies backing the "Result Placeholder" frames in
`../../VBS-Location-Manuscripts/Presentation/main.tex`. Each study calls `run_opt`/the
label-setting pricers directly and reports bounds, timings, and label counts — none of
them touch the simulator, so none of them use the top-level project's
`experiments/`/`scripts/create_study.jl` selection→simulation pipeline (that pipeline is
untouched, and still targets a different, older model API — see its own `CLAUDE.md`).

Benchmark code and output now live together under this directory. Generated directories
are dated when a study is actually run:

- `experiments/<date>_studyN_<name>/` — raw per-run CSVs (gitignored).
- `results/<date>_studyN_<name>/` — curated case-level + variant-summary CSVs
  (git-tracked), plus a `slides_results.tex` of `\newcommand{\RowName}{col & col & ...}`
  macros ready to `\input` into the manuscript, following
  `../experiments/2026-07-15_restricted_pricing_report/slides_results.tex`'s convention.
- `notes/<date>_studyN_<name>_results.md` — benchmark write-ups.

Instance generation reuses `../scripts/generate_zhuzhou_instance.jl`'s
`generate_zhuzhou_data` (current, unmodified) via relative `include`, and each
`submit_benchmark.sh` sources `../scripts/lib/slurm_modules.sh` /
`slurm_array_task_env.sh` (also current, unmodified) rather than duplicating them.

End-to-end CG studies share `lib/cg_benchmark.jl` for Zhuzhou problem loading,
baseline parameters, solver construction, output locations, and standard CG and
label-search metrics. Per-search label statistics are retained in
`OptResult.metadata["cg_pricing_stats"]`, including across integer recovery.

## Status

| # | Directory | Objective | Status |
| - | --- | --- | --- |
| 1 | `study1_formulation_lp_ip_gap/` | Base vs. Joint and operating-condition LP/IP gaps | implemented — four sub-studies, 110 Zhuzhou jobs |
| 2 | `study2_passenger_max_ablation/` | `exact` running-max pricing vs. explicit DARP-style pricing | implemented — 60-job Zhuzhou grid, n ∈ {15, 20, 25} at p=8 |
| 3 | `study3_dominance_ablation/` | `compensated_dominance` true vs. false | implemented — 60-job grid, n ∈ {15, 20, 25} at p=16 |
| 4 | `study4_heuristic_local_search/` | Heuristic pricing frontier (provisional name `local_search`) | placeholder — no design yet, see study README |
| 5 | `study5_scaling_vs_enumeration/` | Exact-CG runtime vs. `\|P\|`/`\|J\|`/`\|S\|` | implemented — three sub-studies, 120 single-threaded jobs |
| 6 | `study6_exact_cg_vs_enumeration/` | Joint CG vs. Base exhaustive enumeration at `max_stops=4` | implemented — 60 single-threaded jobs; re-run 2026-08-25 after the arm correction |

Studies 1–3 and 5–6 are executable end to end. Study 4 remains a placeholder as
described above. `submit_benchmark.sh` files are pure SLURM plumbing.

## Compute budgets (what "timed out" means)

Three independent clocks bound a benchmark job. Only the first is a global cap, so it is
the one that produces a censored, reportable "timed out" outcome:

1. **SLURM walltime** (`#SBATCH --time` in each `submit_benchmark.sh`) — the *only*
   global budget on a run. The CG solver itself has no overall wall budget: it runs to
   `max_iterations=1_000` (`lib/cg_benchmark.jl`'s `benchmark_cg_solver`). Exceeding it
   kills the process, so **no CSV row is written at all** — a job that hit this limit is
   identifiable solely by its `TIMEOUT` state in `sacct` and by a missing
   `experiments/<date>_studyN_*/job_NNNN_*.csv`.
2. **`pricing_time_limit_sec` / `time_limit_sec`** (900.0 s in every `config/*.tsv`) —
   for CG arms, the per-scenario, per-iteration label-search budget. Hitting it makes the
   search return without proving no negative-reduced-cost column remains, so CG cannot
   certify: the row *is* written, with `cg_pricing_exhausted=false` and
   `status="incomplete"`. For Study 6's `enumeration` arm the same field is instead
   Gurobi's budget for the one direct MIP solve.
3. **`SolverOptions.time_limit_sec`** (300.0 s, set in `benchmark_cg_solver`) — Gurobi's
   budget for each individual master LP/IP solve inside the CG loop.
4. **`max_iterations`** (1_000, set in `benchmark_cg_solver`) — not a clock at all, but it
   censors the same way. A run that tails off adds columns without closing the bound and
   stops at iteration 1000 having spent very little wall time. It is distinguishable from
   clock 2 only by `cg_iterations == 1000` in the CSV: the tell is a *short* wall time
   paired with a non-certified status.

So an absent CSV with `sacct` `TIMEOUT` means clock 1 bound. A written row with
`status="incomplete"` means clock 2, 3, or the iteration cap bound — **check
`cg_iterations` before attributing it to time**: 1000 means the iteration cap, and neither
more wall time nor a larger pricing budget will change the outcome. All are legitimate
censored outcomes, but they are different limits and imply different remedies.

### Walltime granted per study

`--time` is doubled on the `sbatch` command line when re-running jobs that hit clock 1
(`sbatch --time=... --array=<timed-out tasks> submit_benchmark.sh`); the committed
`submit_benchmark.sh` files always retain the *original* budget, so the table below is
the record of what each attempt actually received.

| # | Study | CPUs / mem | Original `--time` | Doubled retry | Pricing limit |
| - | --- | --- | --- | --- | --- |
| 1 | `study1_formulation_lp_ip_gap` | 8 / 8G | 30 min | not needed (110/110 completed) | 900 s |
| 2 | `study2_passenger_max_ablation` | 4 / 8G | 30 min | **1 h** | 900 s |
| 3 | `study3_dominance_ablation` | 4 / 8G | 30 min | **1 h** | 900 s |
| 4 | `study4_heuristic_local_search` | 4 / 8G | 1 h | — (placeholder) | — |
| 5 | `study5_scaling_vs_enumeration` | 1 / 16G | 1 h | **2 h** | 900 s |
| 6 | `study6_exact_cg_vs_enumeration` | 1 / 16G | 1 h | **2 h** | 900 s |

All studies run on `mit_preemptable` except Study 4 (`mit_normal`). Studies 5 and 6 pin
`JULIA_NUM_THREADS=1` so their timings are single-threaded and comparable.

**Reporting a job as timed out** requires naming the largest budget it was given — after
the 2026-08-25 re-runs that is 1 h (Studies 2–3) or 2 h (Studies 5–6), *not* the 30 min /
1 h in the committed scripts. Note that Study 5's `scenarios` arm can exceed even the
doubled budget without clock 1 being the real constraint: at `n_scenarios=12`, a single
CG iteration can consume up to 12 × 900 s = 3 h of label search, so those cells are
bounded by clock 2 accumulating past clock 1 rather than by slow convergence.

### Re-run history

| Date | Study | Array | Tasks | Walltime |
| --- | --- | --- | --- | --- |
| 2026-08-25 | 2 | `21243070` | 10 | 1 h |
| 2026-08-25 | 3 | `21243071` | 15 | 1 h |
| 2026-08-25 | 5 `passengers` | `21243078` | 17 | 2 h |
| 2026-08-25 | 5 `scenarios` | `21243080` | 2 | 2 h |
| 2026-08-25 | 5 `stations` | `21243081` | 19 | 2 h |
| 2026-08-25 | 6 | `21243072` | 1 | 2 h |
| 2026-08-25 | 5 `passengers` | `21243293` | 1 | 2 h (conditional) |
| 2026-08-25 | 5 `scenarios` | `21243294` | 1 | 2 h (conditional) |
| 2026-08-25 | 5 `stations` | `21243295` | 1 | 2 h (conditional) |

Re-runs pass `--export=ALL,STUDY<N>_OUTPUT_DIR=<original experiments dir>` so late-
finishing jobs write beside the first attempt instead of opening a new date directory.

### Censored cells after the doubled-budget re-runs

Each analysed study carries a `censored_cells.csv` in its `results/` directory recording
every cell that did not certify, which clock bound it, and the evidence. Counts as of
2026-08-25 (Study 5 still running):

| # | Grid | Certified | Walltime-censored (clock 1) | Pricing-censored (clock 2) |
| - | --- | --- | --- | --- |
| 2 | 60 | 40 | 0 | 20 (all `darp`: 4 at n=15, 8 at n=20, 8 at n=25) |
| 3 | 60 | 53 | 7 (all n=25, `21243071` re-run, 1 h) | 0 |
| 5 | 120 | 52 | 36 (2 h; n=30/40 stations, p=24/32) | 0 -- its 32 split 29 **infeasible-master bug** + 3 iteration-cap |
| 6 | 60 | 55 | 0 | 0 -- its 5 were **iteration-cap** (pre-correction; see below) |

Studies 5 and 6 were both finalised on 2026-08-26; Study 6's row above describes the
**superseded** Base+CG run. Its corrected Joint+CG re-run certifies 55/60 (enumeration
30/30, CG 25/30) with 0 walltime losses: 5 infeasible-master and 2 early-stop. Per-study
`censored_cells.csv` files carry the cell-level detail for all four.

**The infeasible-master bug is now the dominant non-time censoring cause across the
suite** (29 in Study 5, 5 in Study 6): the restricted master is `INFEASIBLE` on iteration
1 after 6-8 s, so CG has no duals and cannot price its way out. It is *not* a budget
problem and must not be reported as a timeout. Its rate tracks scenario count in Study 5
(1/9 at s=3, 7/10 at s=6, 7/10 at s=9, 8/9 at s=12) but it also fires at s=3 in Study 6,
so scenario count aggravates rather than causes it. Likely an initial-column-pool
coverage gap (no artificial/big-M columns guaranteeing a feasible start). **Study 5's
scenarios axis is uninterpretable until this is fixed** -- its apparent certification
collapse past s=3 is this bug, not a scaling limit; among runs that did certify, runtime
grows only ~4.3x for 4x the scenarios.

Study 3's seven are the only true walltime casualties: they hit clock 1 twice (30 min,
then 1 h) and write no CSV, so they are reportable as "did not certify within 1 h".

Study 2's twenty are genuinely clock-2 bound: their longest label search lands on
900.0-905.3 s (exactly the cap) after only 1-4 CG iterations, and the whole job finishes
in 900-1810 s of its 3600 s wall budget. Raising `pricing_time_limit_sec` is the correct
remedy for these, and there is wall headroom to do it.

Study 6's five are **not** time-censored -- all five stop at `cg_iterations == 1000`, the
`max_iterations` cap, after just 32 s (n=10 seed44), 228-243 s (n=15) and 1406-1717 s
(n=20). This has since been root-caused as a **livelock, not tailing-off**: the pricer
re-finds one already-pooled column every iteration, `add_columns!` de-duplicates it away,
and the loop never notices because it breaks only on an empty pricing result. See
`../notes/2026-08-25_study6_cg_livelock_stale_tau_columns.md`; reproducers live in
`diagnostics/`. Raising any time budget or `max_iterations` changes nothing -- these five
cells should be treated as **blocked on a solver bug**, not reported as censored, and
re-run once it is fixed.

**Survivorship bias warning for `variant_summary.csv`.** Its `mean_runtime_sec` pools only
certified runs, so an arm that certifies rarely reports a *flattering* mean over its
easiest seeds. Study 2's `darp` at n=25 certifies 2/10 and Study 3's `plain` at n=25
certifies 5/10 for that reason. Use `paired_comparison.csv` (`both_certified` /
`both_exhausted`) for any head-to-head claim -- it compares the two arms on identical
instances and reverses the sign of the n=25 dominance conclusion that the pooled means
suggest.

The three "conditional" rows above were queued while their first attempts were still
running on the original 1 h budget, using
`--dependency=afternotok:<jobid> --kill-on-invalid-dep=yes`. They launch only if the
first attempt times out, is preempted, or errors, and are cancelled automatically if it
succeeds -- which avoids both wasting a slot on a duplicate and racing two processes onto
the same `job_NNNN_*.csv` path. This is the preferred pattern for topping up a budget on
a job that is still in flight, since cancelling a running job throws away work that may
yet finish.
