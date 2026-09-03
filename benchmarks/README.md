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
- `results/<date>_studyN_<name>/` — curated case-level + variant-summary CSVs. Note
  `.gitignore` carries `*.csv`, so these are **not** committed: only the
  `slides_results.tex` in each directory is. Plus a `slides_results.tex` of `\newcommand{\RowName}{col & col & ...}`
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
| 2 | `study2_passenger_max_ablation/` | `exact` running-max pricing vs. explicit DARP-style pricing | implemented — 60-job Zhuzhou grid, n ∈ {10, 15, 20} at p=8 |
| 3 | `study3_dominance_ablation/` | `compensated_dominance` true vs. false | implemented — 60-job grid, n ∈ {10, 15, 20} at p=16 |
| 4 | `study4_heuristic_local_search/` | Heuristic pricing frontier (provisional name `local_search`) | placeholder — no design yet, see study README |
| 5 | `study5_scaling_vs_enumeration/` | Exact-CG runtime vs. `\|P\|`/`\|J\|`/`\|S\|` | implemented — three sub-studies, 120 single-threaded jobs |
| 6 | `study6_exact_cg_vs_enumeration/` | Joint CG vs. Base exhaustive enumeration at `max_stops=4` | implemented — 60 single-threaded jobs; re-run 2026-08-25 after the arm correction |
| 7 | `study7_route_elementarity/` | Are the route columns in a certified optimum elementary in their station set? | implemented — 30 jobs at n=20, p ∈ {8,16,24}; first study to export the solution itself |
| 8 | `study8_warm_start_speedup/` | Does `warm_start_pricing_mode=:station_simple` pay, and when does phase 1 exhaust? | implemented — 30 jobs, warm_start arm only; `exact` baseline is Study 7's completed runs |
| 9 | `study9_relaxed_cluster_certification/` | Does the relaxed-cluster relaxation ever certify (skipping the expensive certifying round), and at which cluster count `K`? | implemented — 30 jobs, baseline + K ∈ {3,6,9,12,15} at n=15/p=16/s=3; a fast probe, not a measurement grid |

Studies 1–3 and 5–6 are executable end to end. Study 4 remains a placeholder as
described above. `submit_benchmark.sh` files are pure SLURM plumbing.

## Compute budgets (what "timed out" means)

> **The authoritative record of what each run was granted and what it actually used is
> [`notes/2026-08-30_compute_budgets_of_record.md`](notes/2026-08-30_compute_budgets_of_record.md).**
> Its numbers are read back from `sacct`/`scontrol` and the written result rows, not from
> the submission scripts (which have been overridden on the `sbatch` command line in the
> past). Cite that file for any runtime claim; the sections below give the concepts.

Five independent limits bound a benchmark job. They are not interchangeable and imply
different remedies, so a censored cell must always name the one that bound it. Only the
first loses data:

1. **SLURM walltime** (`#SBATCH --time` in each `submit_benchmark.sh`, or an
   `sbatch --time=` override) — the only limit that *loses data*. Exceeding it kills the
   process, so **no CSV row is written at all**: a job that hit it is identifiable solely
   by its `TIMEOUT` state in `sacct` and by a missing
   `experiments/<date>_studyN_*/job_NNNN_*.csv`.
2. **`total_time_limit_sec`** (Study 5 only: 21600 s = 6 h; `Inf`, i.e. unset, everywhere
   else) — a strict budget on the CG loop, added 2026-08-30. On expiry the loop stops and
   the row **is** written, with `status="budget_exhausted"`,
   `cg_stop_reason="total_budget"` and `cg_converged=false`: a feasible incumbent whose
   optimality is not certified and whose `z_lp` is not a valid bound. This clock exists so
   a job that runs out of time still yields an interpretable data point rather than
   vanishing under clock 1.
3. **`pricing_time_limit_sec` / `certifying_pricing_time_limit_sec`** — the budget for
   **one whole pricing round**, divided *equally* across scenarios (each gets
   `limit / n_scenarios`). Hitting it makes the search return without proving no
   negative-reduced-cost column remains, so CG cannot certify. For Study 6's `enumeration`
   arm the same config field is instead Gurobi's budget for the one direct MIP solve.

   > **Changed 2026-08-30, after every run currently on record.** These limits were
   > previously **per scenario**, so a round cost up to `n_scenarios ×` the stated value.
   > All recorded results ran under the old semantics — read their numbers with the
   > multiplier, and see the compute-budgets record §1 and §3c. Committed config values
   > were chosen under the old meaning and are 3× smaller at s=3 when read the new way.
4. **`SolverOptions.time_limit_sec`** (300.0 s, set in `benchmark_cg_solver`) — Gurobi's
   budget for each individual master LP/IP solve inside the CG loop.
5. **`max_iterations`** (1_000, set in `benchmark_cg_solver`) — not a clock at all, but it
   censors the same way. A run that tails off adds columns without closing the bound and
   stops at iteration 1000 having spent very little wall time. It is distinguishable from
   the time limits only by `cg_iterations == 1000` in the CSV: the tell is a *short* wall
   time paired with a non-certified status.

Reading an outcome back:

- absent CSV + `sacct` `TIMEOUT` → **clock 1**. The only case where data is lost.
- `status="budget_exhausted"` (`cg_stop_reason="total_budget"`) → **clock 2**. Report as
  "did not certify within the solve budget", quoting the budget, not the walltime.
- `status="incomplete"` → clock 3, 4, or the iteration cap. **Check `cg_iterations` and
  `cg_stop_reason` before attributing it to time**: 1000 means the iteration cap, and
  neither more wall time nor a larger pricing budget will change it;
  `"pricing_inconclusive"` means the loop stopped with budget still remaining.

All are legitimate censored outcomes, but they are different limits and imply different
remedies.

### Walltime granted per study

`--time` can be raised on the `sbatch` command line when re-running jobs that hit clock 1
(`sbatch --time=... --array=<timed-out tasks> submit_benchmark.sh`). For the 2026-08-25
attempts the committed scripts retained the *original* budget and the doubling lived only
on the command line; `de5d56b` has since folded 2 h into the committed
`submit_benchmark.sh` for Studies 2/3/5/6, so no command-line override was needed on
2026-08-29.

As committed today (after `de5d56b`), which is what the 2026-08-29 re-run used:

| # | Study | CPUs / mem | `--time` | Pricing limit | Changed by `de5d56b`? |
| - | --- | --- | --- | --- | --- |
| 1 | `study1_formulation_lp_ip_gap` | 8 / 8G | 30 min | 900 s | no — deliberately left as-is |
| 2 | `study2_passenger_max_ablation` | 4 / 8G | **2 h** | **1800 s** | yes, + grid n {15,20,25}→{10,15,20} |
| 3 | `study3_dominance_ablation` | 4 / 8G | **2 h** | **1800 s** | yes, + grid n {15,20,25}→{10,15,20} |
| 4 | `study4_heuristic_local_search` | 4 / 8G | 1 h | — (placeholder) | — |
| 5 | `study5_scaling_vs_enumeration` | 1 / 16G | **6 h 30 m** | **300 s regular / 3600 s certifying**, 6 h total budget | yes; budgets revised again 2026-08-30 |
| 6 | `study6_exact_cg_vs_enumeration` | 1 / 16G | **2 h** | **1800 s** | yes |
| 7 | `study7_route_elementarity` | 3 / 24G | **4 h 30 m** | **300 s regular / 3600 s certifying**, 4 h total budget | yes; total budget sized from Study 5's measured walls on the same cells |
| 8 | `study8_warm_start_speedup` | 3 / 24G | **4 h 30 m** | **300 s regular / 3600 s certifying**, 4 h total budget | matched to Study 7 exactly, since its runs are this study's baseline |
| 9 | `study9_relaxed_cluster_certification` | 3 / 8G | **45 m** | **120 s regular / 600 s certifying**, 30 min total budget, **60 s per certification attempt** | sized off Study 3's measured n=15/p=16 walls (7–154 s), not inherited from Studies 7/8 |

The 2026-08-25 attempts ran at 900 s pricing with 30 min / 1 h original walltimes and
needed doubled-budget retries; see the re-run history below. Those budgets are the
*superseded* ones and no longer match any committed script.

All studies run on `mit_preemptable` except Study 4 (`mit_normal`). Studies 5 and 6 pin
`JULIA_NUM_THREADS=1` so their timings are single-threaded and comparable.

**Reporting a job as timed out** requires naming the largest budget it was actually
given. For the 2026-08-29 run that is the committed 2 h (Studies 2/3/5/6) or 30 min
(Study 1). For the archived 2026-08-25 run it was 1 h (Studies 2–3) or 2 h (Studies 5–6)
via command-line override, *not* the 30 min / 1 h those scripts then carried.

Note that Study 5's `scenarios` arm can exceed even the 2 h budget without clock 1 being
the real constraint: at `n_scenarios=12` a single CG iteration can now consume up to
12 × 1800 s = 6 h of label search (was 3 h at the 900 s cap), so those cells are bounded
by clock 2 accumulating past clock 1 rather than by slow convergence. Raising the pricing
cap makes this *more* likely, not less — read those cells accordingly.

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
| **2026-08-29** | **1** (4 sub-studies) | `21570508` `21570509` `21570510` `21570511` | 110 | 30 min (unchanged) |
| **2026-08-29** | **2** | `21570513` | 60 | 2 h (committed) |
| **2026-08-29** | **3** | `21570514` | 60 | 2 h (committed) |
| **2026-08-29** | **5** (3 sub-studies) | `21570516` `21570517` `21570518` | 120 | 2 h (committed) |
| **2026-08-29** | **6** | `21570519` | 60 | 2 h (committed) |
| **2026-08-30** | **5** (3 sub-studies, re-run) | `21589496` `21589497` `21589498` | 120 | 6 h 30 m (6 h solve budget) |

The 2026-08-29 rows are a **full 410-job re-run of every implemented study** on the
corrected dominance rules, not a top-up of censored cells. The 2026-08-25 output was
archived to `experiments/`/`results/2026-08-25_*_SUPERSEDED_pre_dominance_fix/` first;
each archived directory carries a `README_SUPERSEDED.txt` naming the superseding commits.

Re-runs pass `--export=ALL,STUDY<N>_OUTPUT_DIR=<original experiments dir>` so late-
finishing jobs write beside the first attempt instead of opening a new date directory.

### Censored cells after the doubled-budget re-runs (2026-08-25 run — SUPERSEDED)

> Everything in this subsection describes the **2026-08-25** results, now archived under
> `experiments/`/`results/2026-08-25_*_SUPERSEDED_pre_dominance_fix/`. Those runs predate
> the dominance-soundness corrections (`f644a7c`, `b38d46b`), the CG livelock fix
> (`6f8db0b`) and the column-dedup fix (`0bb2d43`). Retained as the analysis of record for
> *that* run and as the rationale for the `de5d56b` budget/grid changes — the 2026-08-29
> re-run's own censoring analysis replaces it once its `analyze.jl` pass lands.

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
