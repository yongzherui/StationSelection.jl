# Widening the PRICED set instead of lowering the duals: rotate, then cover

2026-09-10. `benchmarks/diagnostics/benders_rotating_activated_set.jl`. Zhuzhou p=8, s=3,
seed 42, **max_stops 10** (the formulation's own, not the 4 earlier diagnostics forced).
Arrays 22503817 (n=10/15 heuristic grid), 22510828 (n=15/20/30 cover), plus reruns.

Follow-on from `2026-09-10_activated_dual_completion_verified_and_why_weak.md`, which
established that the closed-form completion's coefficients are FORCED (alpha cannot be
redistributed) and ~8x the true dual's mass. Since the duals cannot be fixed, the lever left
is to widen the set pricing is allowed to search.

## The construction, and why it needs no new soundness claim

The completion's validity argument never mentions WHICH set was searched -- only that the
search exhausted over the priced set and the closed form covers everything outside it. So
`S union T` substitutes for `S` verbatim, for ANY `T`. Two schedules:

- **rotate** -- one block of `r` per scenario per iteration, chosen by a heuristic.
- **cover**  -- `U` partitioned into disjoint blocks, ALL priced, one cut each. Every
  unbuilt station gets an honest coefficient within a single iteration.

Implementation: `y` stays fixed at the TRUE incumbent (so `Q_s` is real) while the
completion is handed a PSEUDO-incumbent with `T` marked built. The completion then skips
`T`, its candidates stay above the pricer's `rho > 0` filter, and the search covers
`S union T`. `T`'s `gamma` comes from the LP's own linking duals, i.e. the honest value.
Blocks within a scenario share that scenario's pool, so block 2 onward starts warm for free.

## 1. The heuristic barely matters; `r` does

n=10, s=3, six rules x r in 1..3. At r=3: `last` 8 iterations, `sum_all` 9, **`random` 9**,
`max_all` 9, `min_all` 12, `incidence` 12. At n=15 the ordering is similar (`sum_all` best,
`random` third). **A random ranking ties the best rule.** The `min_all` rule derived from
first principles (the family's least over-promising statement) is among the WORST.

Do not spend more effort on the selection rule. Spend it on `r`.

## 2. Lower bound: the cuts are dramatically tighter early

n=10, LB as % of the optimum:

| arm | it1 | it2 | it3 | it5 | it10 | it->90% | it->99% |
| --- | --- | --- | --- | --- | --- | --- | --- |
| plain (all n) | 0.0% | 99.7% | 99.9% | -- | -- | 2 | 2 |
| r=0 (today) | 0.0% | 0.0% | 0.0% | 4.7% | 16.9% | 16 | 25 |
| r=2 | 0.0% | 96.5% | 97.5% | 97.9% | 99.2% | 2 | 8-10 |
| r=3 | 0.0% | 97.5% | 98.2% | 99.3% | 99.9% | 2 | 4-5 |

`r=0` sits at exactly 0% for three iterations; `r>=2` is at 96-97% by iteration 2, against
plain's 99.7%. Nearly all the tightening lands in the first two iterations.

## 3. There is an INTERIOR optimum in `r`

n=15 (|U| = 7 unbuilt), all like-for-like inside one array:

| arm | iters | cuts | wall |
| --- | --- | --- | --- |
| cover r=3 | 49 | 441 | 18.5 s |
| **cover r=5** | **13** | **78** | **18.4 s** |
| cover r=8 == full pricing | 4 | 12 | 47.7 s |

**`r=5` converges 2.6x faster in wall than full pricing.** Note wall is FLAT between r=3 and
r=5 while iterations fall 3.8x and cuts 5.7x -- the per-iteration cost rises exactly as fast
as the iteration count falls, so in this range `r` is nearly free and should be chosen for
cut economy.

`r=8` at n=15 is one block holding all 7 unbuilt stations, i.e. full pricing under another
name, and it reproduced plain's 4 iterations / 12 cuts exactly. **That is the plumbing
check**: cover with `r >= |U|` must equal plain, and it does.

n=20 (|U| = 10), all like-for-like in one array:

| arm | iters | cuts | gap | wall | vs plain |
| --- | --- | --- | --- | --- | --- |
| plain | 5 | 15 | 0 | 902.5 s | -- |
| cover r=3 (runt [3,3,3,1]) | 300 cap | 3600 | 531.5 | 797.8 s | fails |
| cover r=3 (balanced [3,3,2,2]) | 300 cap | 3600 | 1473.6 | 1141.6 s | fails, worse |
| **cover r=5** | 138 | 828 | 0 | **701.6 s** | **1.29x faster** |
| cover r=8 | 32 | 192 | 0 | 936.5 s | 1.04x SLOWER |

**The best `r` is ~5 in ABSOLUTE terms at both sizes, not a fixed fraction of |U|** (5 of 7
at n=15, 5 of 10 at n=20). I predicted the opposite -- that r=8 of 10 would be the good
setting at n=20 by analogy with 5 of 7 at n=15. It was not: r=8 cut iterations 4.3x (138 ->
32) and was still slower in wall than plain, because per-block search cost grew faster than
the iteration count fell. So per-block cost, bounded by `k + r`, is what governs.

## The scaling verdict -- the direction is CLOSED

| n | plain | best cover | verdict |
| --- | --- | --- | --- |
| 10 | 8.9 s / 4 it | r=2, 0.3 s / 6 it | **30x faster** |
| 15 | 44.4 s / 4 it | r=5, 18.4 s / 13 it | **2.4x faster** |
| 20 | 902.5 s / 5 it | r=5, 701.6 s / 138 it | **1.29x faster** |
| 30 | **586 s / 5 it, converged** | r=5, 3024 s / 158 it, gap 471.9 | **>5x slower, FAILS** |

n=30 is s=3 seed 42 ms=10 throughout, and **both arms use `:relaxed_cluster` K=18** -- plain's
figure is the archived `benders_lpo` run, which also used K=18, so the n=30 row is a fair
like-for-like. K must be read against `n`, not against the block: the k-medoids partition is
built once at build time over all `n` stations.

**`:exact` does NOT work at n=30 s=3, and I claimed otherwise.** I read the archived n=30
table, saw plain converging in 5 iterations, and concluded the documented pricing frontier
(n<=20 all scenarios, n=25 to <=5, n=30 only s=1) did not apply to Benders subproblems --
then cancelled two running cells on that basis and called the earlier cancellation an error.
It was not. Those archived runs were all `:relaxed_cluster` K=18; I never checked the pricer
behind the numbers. A confirming run of plain at n=30 s=3 under `:exact` (22521820_5) managed
**zero iterations in 90 minutes**, hung in the first subproblem solve. The frontier holds.

Cover found the right incumbent at n=30 (UB 24744.36, matching the archived optimum exactly)
and spent 1422 cuts failing to prove it, stalling at 98.1% of the bound.

**Block size trades against certifiability.** r=8 at n=30 died at iteration 8 with
`certification_inconclusive` on a 7-station block; r=5 ran 158 iterations with no
certification failure. Fewer live stations makes the relaxation's job easier, but bigger
blocks make the cuts better -- at n=30 both ends lose.

## What a GOOD cut looks like (plain, measured per iteration)

From `results/plain_trajectory/` (array 22521820), the signature is not sparsity or
flatness -- it is **where the coefficient mass sits**.

n=20 plain, share of each cut's mass on UNBUILT stations:

| iteration | mass/const | mass on unbuilt |
| --- | --- | --- |
| 1 | 0.10-0.51 | 81%, 100%, 94% |
| 2 | 0.10-0.13 | 42%, 0%, 0% |
| 4 | 0.07-0.10 | **0%, 0%, 0%** |
| 5 | 0.07-0.10 | **0%, 0%, 0%** |

The converged cuts put **zero** mass on unbuilt stations (e.g. `3B:5 11B:56 18B:371
19B:755`); the first cut puts 81-100% there. LB moves 0.0% -> 82.3% -> 98.2% -> 98.6% ->
100%, so the second cut round does nearly all the work.

**This corrects a framing used earlier in this note and in conversation:** `gamma_j > 0` on
an unbuilt station is NOT inherently fiction. At a bad incumbent it is the truth -- building
that station really would help by that much, and plain's first cut says so correctly. The
completion's error is narrower: it charges heavily on unbuilt stations *when the truth is
zero*, which happens at GOOD incumbents -- exactly the late cuts that close the gap. Its
`mass/const` of 1.80 against plain's converged 0.07-0.16 is a 10-25x overcharge, not the 8x
quoted from the n=10 aggregate.

**Harness defect affecting every cut count here:** this diagnostic calls
`add_benders_optimality_cut!` directly and bypasses
`_add_joint_routing_assignment_benders_cuts!`'s signature dedup, so identical cuts are added
twice (visible at n=20 iterations 4 and 5, n=15 iterations 3 and 4). The real solver drops
them and reports `cut_repeated`. Cut COUNTS above are inflated; iterations, bounds and wall
times are not.

## 4. Two mistakes of mine, recorded so they are not repeated

**I "fixed" the block partition and made it worse.** `ranked[i:i+r-1]` leaves the remainder
as a runt ([3,3,3,1] at |U|=10, r=3), and I assumed that singleton's cut -- honest about 1 of
10 unbuilt stations -- was clogging the master, ~900 of the 3600 cuts. Balancing to [3,3,2,2]
made the gap **2.8x WORSE**: 1473.6 against 531.5, slower (1141.6s vs 797.8s), at the same
3600 cuts (array 22518097).

The mechanism runs the other way. The master takes the MAX over cuts, so a weak cut costs a
row and dilutes nothing; what matters is how many blocks are at FULL size `r`, since that is
how many stations each cut is honest about. [3,3,3,1] has three full blocks, [3,3,2,2] has
two. **Pack full-size blocks and accept the runt.** Reverted.

**Every arm used the `:exact` pricer.** `BendersSubproblemConfig(...)` left `pricing` at its
default, which resolves to the formulation's `:exact`, whose measured exhaustion frontier is
n<=20 all scenarios, n=25 to <=5, **n=30 only s=1**. All four n=30 cells were run at s=3,
so `pricing_inconclusive` was the guaranteed outcome and says nothing about the cover scheme.
`:relaxed_cluster` is licensed for Benders cuts precisely for this. Rerun 22518096 uses it
(K=12, 600s/round).

**And an infrastructure one:** the 12-task array was submitted with `CS_COPY_DEPOT=0` on the
argument that an unchanged package makes every task a pure reader of a warm shared depot.
Wrong -- five tasks hit missing cache entries, tried to recompile into shared `~/.julia`
concurrently, and blocked on each other's precompile locks for the full 90-minute wall
(stderr: "Module Markdown ... is missing from the cache"). That is exactly what the per-job
depot copy exists to prevent. **Array tasks copy the depot; the rule has no exceptions worth
taking.**

## Open

- Does the good `r` scale with `|U|`? 5 of 7 at n=15 (71%); at n=20 r=5 of 10 (50%) works but
  needs 138 iterations. If the answer is "a fixed large fraction", this is a constant
  discount on full pricing, not a scaling win. n=20 r=8 (8 of 10) decides it.
- n=30 under a certifying pricer: untested until 22518096 reports.
- The master runs multi-threaded here, so cut counts carry tie-break noise (the repo
  measured 9 vs 12 on identical n=20 settings). Wall time is the reliable column.
