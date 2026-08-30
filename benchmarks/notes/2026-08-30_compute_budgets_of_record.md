# Compute budgets of record

**This file is the authoritative answer to "how long was each experiment allowed to run,
and how long did it actually run."** Every number below was read back from SLURM
(`sacct`/`scontrol`) or from the written result rows — not from the submission scripts,
which can be overridden on the `sbatch` command line and have been in the past. Where a
run's committed script and its granted budget differ, the granted budget is what is
recorded here.

Last verified: 2026-08-30, after all arrays reached a terminal state (`squeue` empty).

## 1. The five limits, and which one a result hit

A benchmark job is bounded by five independent limits (matching the numbering in
`../README.md`). They are not interchangeable and
imply different remedies, so a censored cell must always name the one that bound it.

| # | Limit | Set by | On expiry |
| - | --- | --- | --- |
| 1 | **SLURM walltime** | `#SBATCH --time`, or an `sbatch --time=` override | Process killed. **No row is written.** Visible only as `TIMEOUT` in `sacct` and a missing `job_NNNN_*.csv`. |
| 2 | **Total solve budget** (`total_time_limit_sec`) | `CGSolver`, from `config/*.tsv` | CG loop stops; a row **is** written with `status="budget_exhausted"`, `cg_stop_reason="total_budget"`, `cg_converged=false`. Feasible incumbent, optimality **not** certified. |
| 3 | **Pricing budget** (`pricing_time_limit_sec` regular / `certifying_pricing_time_limit_sec`) | `CGSolver`, from `config/*.tsv` | The label search returns without proving no improving column remains. Cannot certify. |
| 4 | **Gurobi per-solve limit** (`SolverOptions.time_limit_sec`, 300 s) | `lib/cg_benchmark.jl` | Bounds each individual master LP/IP solve inside the loop. |
| 5 | **`max_iterations`** (1000) | `lib/cg_benchmark.jl` | Not a clock, but censors the same way: the tell is `cg_iterations == 1000` paired with a *short* wall time. |

Clock 2 exists specifically so that a job which runs out of time still produces a
recorded, interpretable data point instead of vanishing. **Only clock 1 loses data.**

### Critical semantics of clock 3 — AS RUN (superseded in code, see below)

> **The code changed after these runs.** As of 2026-08-30, *after* every run recorded in
> §2 completed, `pricing_time_limit_sec` was redefined as a **per-round** budget divided
> equally across scenarios (§3c). Everything in this subsection describes the semantics
> the recorded runs actually executed under, and remains the correct way to read their
> numbers. Do not apply it to any run made after that change.

**As run**, `pricing_time_limit_sec` and `certifying_pricing_time_limit_sec` were **per
scenario, per CG iteration** — a fresh budget for each `(scenario, iteration)` pair, not cumulative.
Verified in code: `CGSolver` calls `price_columns` once per iteration
(`optimize/aggregate_od_route/column_generation/dispatch.jl`), which passes the same value
into `_run_label_setting` inside a loop over scenarios (`label_setting/round.jl`).

**Therefore one pricing round can consume up to `n_scenarios × the limit`.** For Study 5's
`stations`/`passengers` sub-studies (s=3) a 3600 s certifying limit permits a single round
of up to 3 h. Measured against that contract the pricer is well behaved: across 2423
logged rounds the maximum ratio of round time to `n_scenarios × limit` was **1.12**, with
one borderline case. Any statement of the form "a pricing round is capped at X seconds" is
wrong unless it multiplies by the scenario count.

## 2. Runs of record

All on `mit_preemptable`. `TimeLimit` verified via `sacct`.

### 2026-08-30 — Study 5 serial-only run (SUPERSEDED by the two-arm rewrite)

> Archived as `experiments/`/`results/2026-08-30_study5_scaling_exact_cg_SUPERSEDED_serial_only/`.
> Study 5 has since been rewritten in place as a **two-arm** study (serial vs parallel
> scenario pricing). This run remains the evidence base for the n>=30 frontier (SS3b) and
> the budget-clamp defect (SS3), and its numbers are valid for what they measured, but it
> is **not comparable** to the rewrite: it used **per-scenario** pricing budgets, whereas
> the rewrite uses a **per-round** wall budget. The rewrite had not been run at the time of
> writing.

| Sub-study | Array | Tasks | TimeLimit (verified) |
| --- | --- | --- | --- |
| `stations` | `21589496` | 40 | `06:30:00` |
| `passengers` | `21589497` | 40 | `06:30:00` |
| `scenarios` | `21589498` | 40 | `06:30:00` |

Solver budgets, from `study5_scaling_vs_enumeration/config/*_jobs.tsv`:

| Setting | Value | Scope |
| --- | --- | --- |
| `time_limit_sec` (regular pricing) | **300 s** | per scenario, per iteration |
| `certifying_time_limit_sec` | **3600 s** | per scenario, per certifying round |
| `total_time_limit_sec` | **21600 s (6 h)** | the CG loop, whole job |
| `SolverOptions.time_limit_sec` | 300 s | each master LP/IP solve |
| `max_iterations` | 1000 | CG iterations |
| Resources | 1 CPU, 16 GB, `JULIA_NUM_THREADS=1`, Gurobi `Threads=1` | per task |

The 6 h budget bounds the **loop**. Data generation and `build_model` run before it, and a
final master re-solve plus the integer-recovery MIP run after it (each ≤ 300 s). Measured
overhead outside the loop was 7–206 s (median 10–40 s), which is why the walltime was set
30 min above the budget.

**Two-tier pricing.** Ordinary iterations price at 300 s. When a regular round returns no
columns *without* exhausting its frontier, that result is inconclusive, so the loop
re-prices the same duals at 3600 s; only that certifying round can set
`cg_converged=true`. A regular round that returns empty **and** exhausted certifies
directly with no escalation. Escalation fired in 7 of the first 78 rows (~9%).

### Observed runtimes, 2026-08-30

| Sub-study | Completed | Timed out (clock 1) | Elapsed min–max (completed) |
| --- | --- | --- | --- |
| `stations` | 29 | 11 | 00:17:12 – 06:27:37 |
| `passengers` | 40 | 0 | 00:09:39 – 06:18:23 |
| `scenarios` | 40 | 0 | 00:29:58 – 06:29:14 |

### Outcomes by cell, 2026-08-30

| axis | value | certified | budget-bound (clock 2) | inconclusive | no row (clock 1) | median wall (h) |
|---|---|---|---|---|---|---|
| stations | 10 | 10 | 0 | 0 | 0 | 0.01 |
| stations | 20 | 10 | 0 | 0 | 0 | 0.37 |
| stations | 30 | 1 | 5 | 2 | 2 | 6.10 |
| stations | 40 | 0 | 1 | 0 | 9 | 6.14 |
| passengers | 8 | 10 | 0 | 0 | 0 | 0.02 |
| passengers | 16 | 10 | 0 | 0 | 0 | 0.62 |
| passengers | 24 | 5 | 5 | 0 | 0 | 6.03 |
| passengers | 32 | 2 | 8 | 0 | 0 | 6.10 |
| scenarios | 3 | 10 | 0 | 0 | 0 | 0.56 |
| scenarios | 6 | 9 | 1 | 0 | 0 | 3.37 |
| scenarios | 9 | 7 | 3 | 0 | 0 | 4.82 |
| scenarios | 12 | 4 | 6 | 0 | 0 | 6.02 |

**Totals: 78 certified, 29 budget-bound, 2 inconclusive, 11 with no row. 120 tasks.**

### 2026-08-29 — Studies 1, 2, 3, 6 (results of record for those studies)

| Study | Array(s) | Tasks | TimeLimit (verified) | Pricing budget | Total budget |
| --- | --- | --- | --- | --- | --- |
| 1 | `21570508` `21570509` `21570510` `21570511` | 110 | `00:30:00` | 900 s single-tier | none |
| 2 | `21570513` | 60 | `02:00:00` | 1800 s single-tier | none |
| 3 | `21570514` | 60 | `02:00:00` | 1800 s single-tier | none |
| 6 | `21570519` | 60 | `02:00:00` | 1800 s single-tier | none |

Resources: Study 1 is 8 CPU / 8 GB; Studies 2–3 are 4 CPU / 8 GB; Study 6 is 1 CPU / 16 GB
single-threaded. Outcomes: Study 1 110/110 certified; Study 2 46/60 (`exact` 30/30,
`darp` 16/30); Study 3 60/60; Study 6 59/60 rows (1 clock-1 loss), all written rows valid.

**These four ran single-tier**, before the two-tier pricing and total budget existed. See
§4 — their committed configs no longer reproduce them.

### Curated results (analysed 2026-08-30)

`analyze.jl` was run for Studies 1, 2, 3 and 6 against their 2026-08-29 raw directories,
writing into `results/2026-08-29_studyN_*` — **dated to the run, not to the analysis day**,
so a results directory always names the run that produced it. Headline numbers:

| Study | Result |
| --- | --- |
| 1 | 110/110 certified. **Base's LP/IP gap averages 0.581; Joint's is 0.000** — the Joint formulation's relaxation is tight where Base's is not. Operating conditions (`max_stops`, `max_wait_time`, `detour_factor`) leave the gap at ~1e-17, i.e. zero. |
| 2 | `exact` beats `darp` on every paired cell: 4.1 s vs 83.6 s (n=10), 6.1 vs 57.0 (n=15), 59.9 vs 311.3 (n=20). `darp` certifies 7/6/3 of 10 pairs against `exact`'s 10/10/10. |
| 3 | Compensated dominance's advantage **grows with size**: runtime ratio (plain ÷ compensated) 0.99 → 1.20 → **1.78** at n=10/15/20, with 10/10 pairs certified at every size and **0 LP-objective disagreements**. |
| 6 | CG overtakes enumeration between n=15 and n=20: enum÷CG runtime 0.42 → 0.70 → **2.69**, while enumeration's pool explodes (3.3k → 20k → 72k columns) against CG's (0.8k → 2.9k → 5.3k). |

Study 6's `analyze.jl` gained a `slides_results.tex` writer on 2026-08-30 (it previously
emitted none), following Studies 1–3's macro convention.

### 2026-08-29 — Study 5 first attempt (superseded)

Arrays `21570516`/`21570517`/`21570518`, 120 tasks, `TimeLimit=02:00:00`, 1800 s
single-tier pricing, no total budget. Result: 56 certified, **60 lost to clock 1 with no
row**. Superseded by the 2026-08-30 run above; retained for the budget comparison only.

### 2026-08-25 — all studies (archived)

Archived under `experiments/`/`results/2026-08-25_*_SUPERSEDED_pre_dominance_fix/`, each
with a `README_SUPERSEDED.txt`. Ran at 900 s pricing with 30 min/1 h committed walltimes
and command-line doubled retries to 1 h/2 h. Predates the dominance-soundness corrections;
**not comparable to anything above** and not to be cited for runtime.

## 3. Known defect affecting 11 tasks on 2026-08-30

The 11 `stations` tasks that hit clock 1 (9× n=40, 2× n=30) did so because of a defect in
how `CGSolver` clamps the total budget, not because 6 h was consumed by useful work.

`cg_solver.jl` clamps the **per-scenario** pricing limit to the **total** remaining budget.
Because a round then spends up to `n_scenarios ×` that value (§1), a certifying round
starting with 3600 s remaining can run 3 × 3600 = 10800 s at s=3 — up to 2 h past a budget
documented as strict. That overshoot exceeded the 30 min margin and the scheduler killed
the job before clock 2 could stop it and write a row.

Scope of the impact, stated precisely:

- It cost **11 rows** (data completeness). It did **not** alter any written result.
- It is **not** why n≥30 fails to certify. Of the 8 n=30 tasks that ran under a correctly
  enforced budget, only **1 certified**; 5 exhausted the full 6 h and 2 stopped
  inconclusive. n=30 is a genuine limit of the exact method at this budget.
- **Fixed 2026-08-30 (after the runs above) by §3c.** Recorded here so the 11 missing rows
  are not mistaken for a scaling result.

Related observation, not acted on: at n≥30 certifying rounds consumed 62–84% of total
pricing time (individual rounds up to 9241 s), while the single n=30 task that certified
spent only 24% there. Escalation at that size tends to either resolve quickly or starve
the regular iterations.

## 3b. Scenario pricing is serial — the frontier is a single-threaded frontier

Every reported runtime is **single-threaded, with scenarios priced sequentially**. A
pricing round costs the *sum* of its per-scenario label searches, not the max. This is the
same multiplier described in §1: it is why a 3600 s per-scenario limit permits a 3 h round
at s=3, and it applies to every round in every cell.

Two independent gates keep it serial (`label_setting/round.jl:90`):

```julia
parallel = _pricing_parallel_scenarios(formulation) && length(scenarios) > 1 && Threads.nthreads() > 1
```

1. Study 5's `submit_benchmark.sh` exports `JULIA_NUM_THREADS=1` (and Gurobi `Threads=1`),
   so the third clause is false.
2. `_pricing_parallel_scenarios` has **only its default method, `= false`**
   (`round.jl:273`); no formulation overrides it. The threaded branch is therefore
   unreachable for every live formulation regardless of thread count.

Raising `JULIA_NUM_THREADS` alone would not parallelize anything. Both gates would have to
open.

**Consequence for how the frontier is stated.** At n≥30 pricing is 62–84% of the budget
(§3), all of it three sequential searches per round. A concurrent implementation bounds
wall time below by the *max* rather than the *sum*, i.e. up to ~3x for the `stations` and
`passengers` sub-studies (s=3 throughout) and up to ~12x at s=12 — an upper bound, before
load imbalance and the serial cross-scenario merge. That is not measured here: per-scenario
timings live in `OptResult.metadata["cg_pricing_stats"]`, which the Study 5 row does not
write out.

So the defensible claim is **"exact CG does not certify n=30 within a 6 h single-threaded
solve budget"**, not "n=30 is beyond exact CG". Two of the eight correctly-bounded n=30
tasks stopped inconclusive at 3.8 h and 4.6 h with budget remaining, so the headroom
question is open.

Note that parallelizing scenarios is clean for the `stations`/`passengers` axes, where s=3
is constant, but would **confound the `scenarios` axis**, whose purpose is to measure how
cost grows with s. Spreading s scenarios across s cores would convert that curve into a
parallel-efficiency measurement. The single-threaded pin is what makes the three axes
comparable to each other and is deliberate — see the study README.

## 3c. Pricing budget redefined as per-round (2026-08-30, post-run)

`pricing_time_limit_sec` / `certifying_pricing_time_limit_sec` are now the budget for **one
whole pricing round**, divided **equally** across scenarios: each gets
`limit / n_scenarios` (`label_setting/round.jl`). This replaces the per-scenario semantics
of §1.

Two reasons, both load-bearing:

1. **It makes the total budget enforceable.** Under per-scenario semantics a round cost
   `s x` its stated budget, so clamping the per-scenario limit to the remaining total
   budget still let a round overshoot by `(s-1) x` — the §3 defect. With a per-round
   budget the clamp is exact.
2. **It stops scenarios starving each other.** The obvious alternative — a shared deadline
   consumed in scenario order — would let scenario 1 spend the whole round. Because the
   scenario order is stable across iterations, the *same* scenarios would be starved every
   round, so pricing would keep improving one scenario's coverage while the others never
   advanced. An equal split guarantees every scenario progresses each round.

**Serial reallocation.** The serial path re-divides the *remaining* round budget before
each scenario, over the scenarios still to run: slack left by one that exhausts early (or
has nothing to price) passes to those after it instead of being wasted. When every scenario
spends its full slice this is exactly the equal split, since after `k` of `s` scenarios have
each spent `T/s` the next slice is `(T - kT/s)/(s - k) = T/s`. So reallocation can only
help; it can never let one scenario claim more than its fair share of what remains.

**Parallel does not reallocate**, because all scenarios start together and are fixed at
`T/s` up front. A serial round can therefore search longer in aggregate than a parallel one
whenever a scenario finishes under its slice — the two are identical only when none does.
**A serial-vs-parallel study must control for this**, by equalizing work or reporting it;
it is not purely a wall-clock difference.

`CGSolver(parallel_scenario_pricing=true)` opts a run into `Threads.@threads` scenario
pricing without needing a formulation-level override (the previous
`_pricing_parallel_scenarios` hook still works and is OR-ed with it).

**Thread-safety audited 2026-08-30.** Phase 1 (`_pricing_build_scenario_context`) only
*reads* the JuMP model's object dictionary and column pool and allocates fresh per-scenario
data. Phase 2's `best_pool_tau`/`scored`/`accept!` are allocated per scenario inside
`_prepare_pricing_scenario`, so no state is shared between scenarios. Both phases write
only to preallocated `Vector`s at their own index. The stats append to
`m[:label_setting_pricing_stats]` happens *after* the loop, single-threaded. There is no
global mutable state in `label_setting/`, and `round.jl` holds the only `Threads.@threads`
in `src/`. Verified empirically at `JULIA_NUM_THREADS=4` on a 3-scenario instance: serial
and parallel agreed exactly on objective, LP bound, iteration count (3) and column pool
(145), across interleaved repeat runs. **No speedup was measurable there** — the instance
prices in ~0 s and the apparent 13.7 s → 0.1 s difference was first-run JIT, reproducible
by swapping the arm order. A speedup measurement needs a large instance and one process per
arm.

Regression test: `test/opt/test_joint_routing_assignment_column_dedup.jl` asserts a
2-scenario run respects `total_time_limit_sec` and still returns a usable incumbent.

**Consequence for config values.** Study 5's committed `time_limit_sec=300` /
`certifying=3600` were *per-scenario* when it ran. Read as *per-round* they are 3x smaller
at s=3 (100 s and 1200 s per scenario). **The committed Study 5 configs therefore no longer
mean what they meant on 2026-08-30** — to reproduce that run's effective search budget they
would need `900` / `10800`. Decide this deliberately before any Study 5 re-run.

**Consequence for the scenarios axis.** Under a fixed per-round budget, *serial* execution
gives each scenario `limit/s`, so raising `s` squeezes every scenario — whereas the old
per-scenario semantics gave the round more total time as `s` grew. The scenarios sub-study
is therefore not comparable across the change.

This is precisely what the two-arm rewrite exists to exploit rather than merely suffer.
Under **parallel** execution the scenarios overlap, so the round's wall is their max rather
than their sum and each scenario gets the *full* round budget. Both arms obey the same
round wall; the parallel one fits up to `s x` more label search inside it. The hypothesis
under test is that this converts into certification: cells the serial arm cannot certify
within 6 h, the parallel arm can, with runtime staying roughly flat in `s`. Measured on a
pilot at s=3 (20 s rounds): longest single search 8.25 s serial vs 20.82 s parallel, total
search work 220 s vs 610 s (**2.77x**), for comparable wall (235 s vs 251 s).

## 4. Reproducibility hazard

`benchmark_cg_solver` (`lib/cg_benchmark.jl`) now defaults to
`certifying_pricing_time_limit_sec=3600.0`, so **two-tier pricing is on by default for
every study**. Studies 1, 2, 3 and 6 produced their 2026-08-29 results *single-tier*.
Re-running their committed configs today would exercise escalation and would **not**
reproduce those numbers. To reproduce them, pass
`certifying_pricing_time_limit_sec == pricing_time_limit_sec`, which disables escalation.

`total_time_limit_sec` still defaults to `Inf`, so those studies remain unbounded by
clock 2 unless their configs are updated.

## 5. How to state a runtime claim

- **"Certified in X"** — only for rows with `status="exhausted"`; cite `wall_sec`.
- **"Did not certify within 6 h"** — for `status="budget_exhausted"`. The 6 h is the
  *solve* budget (clock 2), not the 6 h 30 m walltime. A feasible incumbent exists; its
  `z_lp` is **not** a valid bound on the unrestricted optimum.
- **"Stopped inconclusive"** — for `status="incomplete"` with
  `cg_stop_reason="pricing_inconclusive"`: the certifying round could not resolve and the
  loop stopped with budget remaining. Report the wall time reached, not the budget.
- **"No result within 6 h 30 m"** — only for the 11 clock-1 tasks, and §3 must be cited
  alongside, since the walltime was not reached by useful computation.
- Never quote a committed `#SBATCH --time` as evidence of what a run received; quote the
  `sacct` `Timelimit` recorded in §2.
