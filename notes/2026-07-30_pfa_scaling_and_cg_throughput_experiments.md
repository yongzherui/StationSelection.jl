# Passenger free-assignment: scaling grid and CG throughput experiments

Live experiment log, written to survive a context reset. Companion to
`2026-07-30_passenger_pricing_label_search_optimizations.md`, which covers the
label-search optimizations themselves; this note covers how the resulting pricer
behaves inside the full CG loop, and the two open questions about it.

Status as of 2026-07-30 evening: the n=10..30 grid is **complete**; the n=40/50
grid and a 2x3 throughput experiment are **still running** (job ids below).

## 1. Scaling grid (COMPLETE for n=10..30)

`scripts/sbatch_pfa_scaling_grid.sh`, one array task per (n, p) cell so each gets
its own 3h CG budget. 3 scenarios, seed 42, unbounded `max_stops`,
station-budget cap off. Results under
`experiments/2026-07-30_pfa_scaling_grid/n<N>_p<P>/`.

Aggregate with `julia --project=. scripts/analyze_pfa_scaling_grid.jl`.

Wall time (s), `*` = hit the 3h budget so the number is a floor:

```
 n\p          8         16         24         32
  10        6.5        8.1       10.5       17.8
  15        8.2       56.0      157.7     1177.9
  20       15.8      553.3     1388.6   10833.9*
  25      423.9   10803.1*   10803.2*   10875.8*
  30     1734.0   10803.4*   10805.7*   10804.1*
```

13/20 cells certified optimality. Power-law fits (certified cells only):

| fixed | exponent in n | | fixed | exponent in p |
| --- | --- | --- | --- | --- |
| p=8 | 5.18 (R^2=0.78) | | n=10 | 0.66 (R^2=0.84) |
| p=16 | 6.01 (R^2=0.98) | | n=15 | 3.36 (R^2=0.95) |
| p=24 | 7.03 (R^2=1.00) | | n=20 | 4.19 (R^2=0.97) |

**The exponents are not separable -- each grows with the other.** p=8->24 takes
the n-exponent from 5.2 to 7.0; n=10->20 takes the p-exponent from 0.66 to 4.19.
Cost is not `n^a * p^b`; the tractable frontier runs diagonally (n=30 is fine at
p=8, hopeless at p=16).

LP-MIP gap is <= 0.27% on every certified cell.

**Reading the gap column:** truncated cells sometimes show a *negative* gap
(-0.239% at n=30/p=32). That is not a bug. For an uncertified cell `lp_bound` is
not a valid lower bound -- pricing never proved no improving column remains -- so
it can exceed the MIP objective. Only trust `lp_mip_gap_pct` where
`cg_stop_reason == optimality_proven`.

## 2. Iteration counts: the cost is per-iteration, not iteration count

```
 n\p          8   16   24   32
  10         10   27   46   88
  15         13   40   50  127
  20          8   33   79  149*
  25          8   54*  56*  71*
  30         14   38*  50*  54*
```

The 3h cells did only **38-71 iterations** -- n=30/p=32 spent 3 hours on 57.

This matters because it rules out seeding as the main lever. On n=30/p=16 the
first six iterations cost 0.2s, 17s, 0.2s, 0.2s, 0.5s, 37s -- about **56 seconds
out of 10,803**. Perfect seeding saves under 1% of the run.

Worse, pricing gets *harder* as duals converge: labels generated per iteration
goes 43k (iter 1) -> 1.38M (iter 22), a 32x increase. Early iterations are cheap
precisely *because* the duals are far from optimal and few `(p,j,k)` are
attractive. Seeding to good duals starts you in the expensive regime rather than
skipping it, so it may be a wash or worse. Do not invest in seeding heuristics
without first measuring against this.

## 3. Open question A -- harvest per search (RUNNING)

Inside those iterations, on n=30/p=16:

```
early_return  38 iters  3897s pricing   715 cols -> 5.5 s/col  (32/38 hit the 120s cap)
certification  5 iters  6900s pricing  3908 cols -> 1.8 s/col
```

Certification is 3x more productive per second than the phase designed to be the
fast one. Individual calls are starker: one certification pulled **3,706 columns
from a single search** while nearby early-return calls burned the full 120s for
**0-5 columns**.

Hypothesis: a pricing call's cost is dominated by *exploring* the label space,
not by harvesting columns out of it, and `n_candidates=20` (x3 scenarios = 60)
discards most of each search. Both failure modes are visible in the trace --
early iterations return 58-60 columns (harvest cap binding outright), later ones
return almost nothing (120s too short to find pool-novel columns at that pool
size).

Supporting magnitude: a single search at n=10..12 yields **400-1300 distinct
columns**; we currently keep 20.

## 4. Open question B -- is compensated dominance net-positive? (RUNNING)

Compensated layer dominance made pricing 2.5-3.9x faster, but candidates come
from *popped labels*, so a label killed by dominance never becomes a column.
Measured on the n=10/12 brute-force oracle runs:

| case | columns before -> after | labels before -> after |
| --- | --- | --- |
| n=10 s1 | 1167 -> 801 (-31%) | 2175 -> 2054 (-6%) |
| n=10 s2 | 968 -> 408 (-58%) | 1825 -> 1423 (-22%) |
| n=10 s3 | 1401 -> 636 (-55%) | 3099 -> 2660 (-14%) |
| n=12 s1 | 2404 -> 1309 (-46%) | 6108 -> 5234 (-14%) |
| n=12 s2 | 1759 -> 764 (-57%) | 3632 -> 2782 (-23%) |
| n=12 s3 | 1110 -> 586 (-47%) | 5859 -> 5504 (-6%) |

Columns fell 2-8x faster than labels. `best_rc` is unchanged (the oracle still
matches brute force), so the *best* column survives -- but ~half the distinct
candidate routes no longer exist.

**The original label benchmark could not see this**: it ran with
`max_new_columns=1`, measuring time-to-exhaustive-search at fixed `best_rc`.
That is the right metric for "did pruning stay exact" and the wrong one for CG,
which cares about columns harvested per second. The 2.5-3.9x speedup is real but
it is a speedup on a search returning half as many columns; the net CG effect was
never established. The benchmark now harvests everything and reports the count.

Toggle: `compensated_dominance` on the pricing data, `PFA_COMPENSATED_DOMINANCE`
on `scripts/passenger_free_assignment_cg_scaling.jl`, `HARVEST_COMPDOM` on the
experiment script. Off restores the plain `A_a subseteq A_b` rule.

## 5. What is running, and what to do when it lands

### Large scaling grid -- array 19318746, 8 tasks
n=40 and n=50, p in {8,16,24,32}. `experiments/2026-07-30_pfa_scaling_grid/`.
n=60 (tasks 2,5,8,11) was **cancelled to free memory** and produced no data --
`run_one` only writes its CSV when a case returns, so killed cells leave nothing.
Resubmit with `sbatch --array=2,5,8,11 scripts/sbatch_pfa_scaling_grid_large.sh`
if wanted.

Re-run the analyzer when done; it reads per-case `results/*.csv` (written as each
case returns) rather than `combined_results.csv` (written only after a whole job
finishes), so a killed job does not silently drop its longest cells.

### Throughput experiment -- 2x3 factorial at n=30/p=16
All under `experiments/2026-07-30_pfa_harvest/`.

| | cand=20 pt=120 | cand=500 pt=600 | cand=2000 pt=900 |
| --- | --- | --- | --- |
| compensated **on** | 19320226 | 19319280 | 19319281 |
| compensated **off** | 19320227 | -- | 19320228 |

Both factors at both levels on the same cell, so the dominance effect and the
harvest effect can be separated rather than confounded.

`19320226` (cand=20, pt=120, compdom on) is also a **regression check**: it should
reproduce the grid's original n=30/p=16 result -- 10803s, 43 iters,
lp_bound=39688.7 -- confirming the refactors did not change behavior. If it does
not, trust nothing else in the table until that is explained.

Metric for all of them: does the cell certify inside 3h, and if not how far
`lp_bound` got. Compare against the grid baseline (never certified; lp reached
39688.7 in 43 iterations).

## 6. Decision rules set in advance

- If **high harvest certifies** n=30/p=16 within 3h: the harvest cap was the
  throttle. Raise `n_candidates`/`pricing_time_limit` defaults and re-run the
  scaling grid before touching anything else.
- If **compdom off beats compdom on** end to end at equal harvest: column
  diversity outweighs pricing speed, and compensated dominance should become
  opt-in rather than default -- despite winning the pricing microbenchmark.
- If **high harvest makes the dominance choice irrelevant**: keep compensated
  dominance on (it is strictly faster) and treat the diversity loss as
  compensated for by harvesting.
- Only after both are settled is seeding worth revisiting, and section 2 says
  start by measuring what it could possibly save.

## Related

- `2026-07-30_passenger_pricing_label_search_optimizations.md` -- the label-search
  optimizations, what worked and what did not, and the profiling that showed the
  dominance scan is ~90% of pricing wall time.
