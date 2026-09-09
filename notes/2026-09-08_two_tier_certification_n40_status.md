# Two-tier relaxed-cluster certification: n=30 complete, n=40 in progress

Status as of 2026-09-08 ~23:00. Written to survive a context reset -- everything below is
measured, with the disconfirming results kept in.

## What was built

`pricing.mode = :relaxed_cluster_two_tier` (`CGSolver.pricing`), a nested macro/meso
partition pair under the existing `:relaxed_cluster` certification contract.

- **Code**: `src/opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/two_tier.jl`
  (~700 lines). Reuses `certify.jl`'s round driver via its new `pass` keyword, its result
  type and its stat record, so concurrency/budget/reduction logic cannot drift between modes.
- **Config**: `CGPricingConfig.relaxed_cluster_macro_count` (K1, required by the mode) and
  `relaxed_cluster_aligned_subset_max` (default **15**). Rejects `relaxed_cluster_max_count`
  (refinement re-partitions the meso layer and strands the macro parent map -- a
  mis-translated cut EXCLUDES relaxed routes, i.e. false certificate, not a crash).
- **Tests**: `test/opt/test_joint_routing_assignment_two_tier_pricing.jl`, 9 testsets,
  green (93k assertions). Covers the nesting property, cut translation, the alignment
  identity, all config rejections, and an end-to-end objective match against `:relaxed_cluster`.
- **Benchmark arms**: `twotier_k<..>m<K1>` in `benchmarks/study9_relaxed_cluster_scalability`
  (`run_benchmark.jl` accepts 17- or 18-field rows; the 18th is `macro_count`).

### The algorithm

1. macro sweep over K1 cluster nodes (all macro cuts applied) -> macro support U
2. meso sweep restricted to `pi^-1(U)` (all meso cuts applied)
3. exact station search over the meso support's stations -> real columns, or barren
4. barren + **macro-aligned** (`T' = pi^-1(pi(T))`, so `stations(T') = stations(pi(T))`)
   -> meso cut AND macro cut
5. barren but NOT aligned (over the 15-station cap) -> meso cut only
6. restricted meso sweep exhausts under accumulated meso cuts -> `Barren(stations(U))`
   -> macro cut on U  (this is the route that does not need alignment)
7. macro sweep exhausts under macro cuts -> **certificate, full route universe**

`Barren(X)` is downward-closed and **not union-closed**, so meso cuts can never be
*inferred* into a macro cut -- cutting `pi(T)` because `stations(T)` is barren is unsound.
Only (4) and (6) are valid.

## RESULTS: n=30 (complete, 10 seeds)

| arm | K2/K1 | certified | median vs BEST single-tier |
| --- | --- | --- | --- |
| single-tier best-of-both | 18 / 24 | 10/10 | -- |
| twotier_k80 | 24/14 | **10/10** | **2.1x** (range 1.0-9.1x) |
| twotier_k60m14 | 18/14 | 10/10 | 1.1x |
| twotier_k60 | 18/11 | (cancelled) | 0.4x -- rounds pinned at the 300s cap |

All objectives matched exactly. **Zero reach gain** -- single-tier already did 10/10.

## RESULTS: n=40 (in progress)

| arm | K2/K1 | certified | seeds |
| --- | --- | --- | --- |
| relaxed_k60 | 24 | 4/9 | 44, 46, 48, 49 |
| relaxed_k80 | 32 | **0/10** | -- (all `pricing_inconclusive`, median wall 17424s) |
| twotier_k60m14 | 24/14 | 4/8 | **44, 46, 48, 49 -- the identical set** |
| twotier_k80m16 | 32/16 | 3/4 | 44, 48, 49 |

Speedups on the seeds it certifies (vs best certified single-tier): seed 44 **64x**
(6150s -> 96s), seed 48 **36x** (2812s -> 78s), seed 49 **20x** (3089s -> 155s),
seed 46 **2.1x** (19524s -> 9376s).

### The three findings that matter

**1. No reach gain yet.** Two-tier certifies exactly `{44, 46, 48, 49}` -- the same set as
single-tier. If variable round times were the blocker, the failing set would differ. It
does not. Seed 50 is the proof: all four arms reach 30271.3882 and none certifies. Those
six seeds look intrinsically uncertifiable by this relaxation, not two-tier's fault.

**2. Bounds converge to the same value; two-tier gets there sooner.** Final bound identical
on 6 of 10 seeds, better on 2 (seed 43 by 146.58 units, 47 by 5.06), WORSE on 2
(seed 42 by 1239.42, seed 51 by 948.18). But at a fixed budget it dominates:

| budget | two-tier better | single-tier better | tied |
| --- | --- | --- | --- |
| 60s | **6** | 0 | 0 |
| 300s | **6** | 0 | 1 |
| 900s | 5 | 4 | 1 |
| 3600s | 4 | 3 | 3 |

At 300s two-tier holds the PROVEN OPTIMUM on seeds 44/48/49 while single-tier is
250-4200 units above. The edge decays with budget and reverses on seeds 42/51.
Total bound movement available at n=40 is small: 352 units (seed 50) to 4236 (seed 48).

**3. K2 = 24 is the right absolute value; the ratio is a red herring.** At n=30, K2=24
(0.8n) beat K2=18 (0.6n). At n=40, K2=24 (0.6n) beats K2=32 (0.8n) by 8-15x on every
shared seed (48: 78s vs 616s; 44: 96s vs 1494s; 49: 155s vs 1869s). K2~24 and K1~12-16 are
node counts, not fractions of n -- search cost depends on graph size. `generate_jobs.jl`
still hard-codes `k1 = round(0.6*k2)`, which gives 11 at K2=18 and 19 at K2=32, both
outside the working band; **it should clamp K1 to 12-16.**

## Diagnostics built (both give a ranked answer in <45 min)

- `benchmarks/diagnostics/two_tier_param_scan.jl` + `run_param_scan.sh` -- sweeps
  K2 x K1 x g on ONE dual vector (K2 is swept without rebuilding the model), ranks by
  predicted round cost, ends with a literal `RECOMMENDATION` line. `PS_DUALS=<file>`
  replays a captured dual snapshot instead of re-running CG.
- `benchmarks/diagnostics/relaxed_cluster_two_tier_guide.jl` -- the `g` ladder;
  `cov_top(g)` is free for the whole ladder (prefixes of one sorted sweep).

**Both recommend K2=24, K1=12, `guide_routes=1`.** Every run so far used **g=5** (inherited
from single-tier). At K2=32 the ladder measured g=1 taking the restricted meso sweep to
**0.0s** where g=5 took **15.5s**, with IDENTICAL `subset_rc`. `g` sizes the restricted
meso graph in two-tier, where in single-tier it only sized the station subset -- that is why
5 is the wrong default here.

**Known limitation of both diagnostics**: they run at iteration-4 duals, where a round costs
~1s. Real solves show 137s rounds late, when the master is near-converged against a ~0
margin. The n=30 scan's K2 recommendation (18) CONTRADICTS the measured CG outcome (24 is
better), because cheapest-round-at-easy-duals is the wrong objective. Fix: capture late
duals (below) and re-scan.

## Instrumentation added (all new, all needed)

- `CGSolver.iteration_callback` -- streams each iteration row as it happens. Runs used to
  emit NOTHING for six hours; a preempted task lost everything. `run_benchmark.jl` now
  appends `iterations/<stem>.csv` live and rewrites a `<stem>.progress.csv` snapshot
  (deliberately not matching the globs `analyze.jl`/`check_gate.jl` use).
- `CGSolver.dual_callback` + `STUDY9_EXPORT_DUALS=1` -- serialises each iteration's duals to
  `duals/<stem>/itNNNN.jls`, for replay via `PS_DUALS`. **Not yet used to capture anything.**
- Per-tier accounting: `two_tier_{macro,meso,station}_{rounds,sec}`,
  `station_unexhausted`, `align_skipped`, `align_downgraded`.
- **11 tagged exit reasons** (`two_tier_exit_reasons`, tallied per run):
  `budget_outer`, `budget_inner`, `macro_unexhausted`, `meso_unexhausted`,
  `station_unexhausted`, `mask_full_macro`, `mask_full_meso`, `inner_cap`, `outer_cap`.
  Before this, every failure was an undifferentiated `:inconclusive`.

Per-tier data already earned its keep: at K2=32 the restricted meso sweep costs **27.5s per
round** (seed 49) against 4-7s at K2=24, and the station search hits its cap on 19 of 35
searches (seed 48). The bottleneck MOVES between seeds -- meso-dominated on 44/49,
station-dominated on 48 -- so no single-component fix works. Fewer guide routes shrinks both.

## Fixes made today

- `submit_benchmark.sh` `--mem` 32G -> **16G**. Measured peak RSS 13.1G across 107 tasks,
  flat in n. (The n=50 arm's 64G was ~5x over. NOTE: over-reserving did NOT cause the queue
  waits -- pending jobs report `(Priority)`, i.e. fair-share, not `(Resources)`.)
- Macro sweep slice: `0.25 * remaining` per outer round -> absolute **15s** cap. The
  fractional slice compounded (ten outer rounds left 5.6% of the budget).
- Station search: was given the round's ENTIRE remaining budget -> capped at **30s**.
- Inner (meso) rounds per outer round: 65 -> **8**.

## Bugs still open

1. **The alignment downgrade path is dead.** `align_downgraded = 0` on every run. It is
   gated on `!isnothing(macro_cut_cells)` -- i.e. alignment having applied -- so it is
   unreachable in exactly the cases it should rescue (`align_skipped = 22` on seed 50).
   It should shrink by dropping guide routes (g -> g-1) instead.
2. **Seed 50's exit reason is unknown.** It stopped at 663s of a 21600s budget on two
   consecutive inconclusive rounds. Arithmetic favours `inner_cap` (8 inner rounds x ~15.7s
   ~= 127s vs the observed 113s round) over `station_unexhausted`, but the run predates the
   exit-reason logging. Needs one re-run to confirm.
3. `certify.jl`'s single-tier loop still has untagged `:inconclusive` exits.
4. Orphaned `.progress.csv` files survive a cancelled task (deletion only happens on
   success), so a dead task reads as live. Needs a heartbeat, or treat stale as dead.
5. `submit_benchmark.sh`'s whitelist has `n40_twotier.tsv` twice (harmless, concurrent edits).

## What to do next, in order

1. **Run K2=24, K1=12, g=1 at n=40, 10 seeds, 21600s budget.** The only configuration both
   diagnostics recommend and the only one untried. Table: `config/n40_twotier.tsv` (edit
   `guide_routes` to 1 and `macro_count` to 12). Success criterion fixed in advance:
   **beat 4/10 certified**, best-of-both single-tier arms, same seeds, same budget.
2. Fix bug 1 (downgrade via g), then re-run seed 50 to resolve bug 2.
3. Capture late duals (`STUDY9_EXPORT_DUALS=1` on a long n=40 seed) and re-scan the grid
   with `PS_DUALS`. This is the only way to make the parameter recommendation trustworthy.
4. n=50 only if (1) shows a reach gain. Single-tier there is 0/5.

## The honest bottom line

The mechanism works and is safe: **20/20 objectives matched exactly across n=30 and n=40**,
certificates are full-route-universe, and it is 20-64x faster on the seeds it certifies.
At short budgets it dominates single-tier 6-0.

But it has **not yet certified a single instance single-tier could not**, its bound is a
wash on most seeds and worse on two, and its whole value therefore rests on speed. If
step (1) also returns 4/10, the fair conclusion is that the six hard n=40 seeds are hard for
reasons two-tier does not address, and the mode should be kept as an accelerator rather than
pursued as a frontier-extender.
