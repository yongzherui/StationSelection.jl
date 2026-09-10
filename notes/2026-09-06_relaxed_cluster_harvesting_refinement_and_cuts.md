# Relaxed-cluster certification: harvesting, parallelism, refinement, cut management

> **Naming note (added 2026-09-10).** The outcome this note calls `:refuted` was renamed
> TWICE on 2026-09-10, so the full chain is:
>
> | | outcome symbol | metadata counter / CSV column |
> | --- | --- | --- |
> | until 2026-09-10 | `:refuted` | `cg_certification_refuted_rounds` |
> | 2026-09-10, briefly | `:negative_rc_column_found` | `cg_certification_negative_rc_column_rounds` |
> | 2026-09-10 onward | **`:column_found`** | **`cg_certification_column_found_rounds`** |
>
> The whole chain is given because a reader with an older checkout or an older results CSV
> needs it to place what they are looking at; note that no run ever wrote the middle name,
> so result files carry either the first or the last. The reason for the rename is that
> "refuted" read as a failure and was repeatedly misread as one: the relaxed-cluster mode
> **prices first and certifies second**, so an attempt that finds an improving real column
> is that iteration's pricing round doing its job, not a failed certification. Read every
> "refuted" below as "priced a column, so CG iterates again". Full argument in
> `notes/2026-09-10_certification_outcome_naming.md`.

**2026-09-06.** Follow-on to `2026-09-05_relaxed_cluster_certification_and_guiding.md`.

Four changes to `:relaxed_cluster_nogood` certification were built and measured. Two are
ready to ship, one was vindicated only after the experiment was corrected, and one is
correct but inert. Everything below is measured on the Study 10 grid (Zhuzhou, p=16, s=3,
seeds 42-46) unless marked otherwise.

**TL;DR**

| change | verdict | evidence |
| --- | --- | --- |
| column harvesting + parallel certification round | **SHIP** | 8-13x vs plain exact CG; certifies n=30 where baseline certifies 0/5 |
| witness-guided cluster refinement | **PROMISING, opt-in** | at equal final K: 2-3.8x faster at K=12; at K=16 fixed never converges and refined always does |
| cut management (drop dominated cuts) | sound, benefit **unmeasured** | 60% of cuts are dominated; no A/B run |
| barren-support cache | correct but **inert** | 0 hits in 18 runs |

---

## 1. Column harvesting + parallel certification round — SHIP

### What it does

A no-good certification round proves "no improving column exists" by, for each spurious
cluster route, searching its cluster support's stations **exhaustively with the real exact
pricer**. Two things were wrong with how that was used:

- when that search *refuted* (found an improving real column), the columns were **discarded**
  -- measured, 753 of 788 attempts refuted, each throwing away a completed pricing search;
- the round walked scenarios **serially** while the pricing round it displaced was threaded.

Now: refuted attempts harvest their columns and hand them to CG as the iteration's pricing
result (skipping the regular round entirely), and scenarios run concurrently with no early
exit -- every scenario is searched, because a refuted scenario is now a pricing round rather
than wasted work.

### Results (Study 10, 60/60 jobs, correctness gate passes on all 45 paired rows)

Paired speedup vs plain `:exact` CG, same cell, same run:

| n | K/n=0.4 | K/n=0.6 | K/n=0.8 |
| --- | --- | --- | --- |
| 20 | 2.79x | 7.81x | **12.81x** |
| 25 | 1.66x | 4.78x | **8.41x** |

The same table **pre-harvest** read 1.02x / 1.17x / 0.84x at n=20 -- i.e. break-even. The
gain is essentially all from these two changes.

**n=30 is a capability change, not a speedup.** Plain exact CG certified **0 of 5** cells,
each burning its full ~6 h budget and returning FEASIBLE:

| arm | certified | wall of certified runs |
| --- | --- | --- |
| baseline `:exact` | 0/5 | 18621-21617 s, all FEASIBLE |
| K/n=0.4 | 1/5 | 16039 s |
| **K/n=0.6** | **4/5** | 357-5365 s, **median 652 s** |
| K/n=0.8 | 3/5 | 2457-18106 s |

8 `arm only` wins across n=25 and n=30 -- cells proven optimal that baseline cannot prove
at any budget.

### Also established, across every run at n=12/15/20/25/30

`certified_at_round_1 = 0` across every attempt measured (thousands). A round-1
certification is exactly what the one-shot `:relaxed_cluster` mode does, so **the no-good
cuts are the entire mechanism** -- this reproduces Study 9's 0/31 at four further sizes.

`K/n = 0.4` is the wrong operating point at every size in every configuration. **Use
K/n in [0.6, 0.8].**

### Code

- `src/opt/label_setting/joint_routing_assignment/relaxed_cluster/nogood_certify.jl`
  -- harvesting (`_pricing_accept_closure` on the subset search), concurrent scenario pass,
  no early exit.
- `src/opt/label_setting/round.jl` -- `_materialize_pricing_columns`, extracted so a
  harvested column goes through the identical id-allocation and `_pricing_verify_column`
  path as a priced one.
- `src/opt/solvers/cg_solver.jl` -- takes harvested columns as the iteration's pricing
  result; `cg_certification_harvested_columns` metadata.

---

## 2. Witness-guided cluster refinement — PROMISING (opt-in, default off)

### What it does

`rho_bar(p,C,D) = max` over real station pairs lets **every passenger independently pick its
own best in-cell station** -- the dominant source of relaxation slack. When a spurious
cluster route's support is barren, replaying it reveals which real `(j,k)` each `rho_bar`
maximum came from. If two passengers credited at ONE cluster needed DIFFERENT stations, that
cell manufactured the fiction, and it is split (2-medoids seeded on the two witnesses).

Partitions are **per scenario** (each scenario prices its own duals, so a cell that is a
fiction under one may be tight under another), and refinement is monotone, which a K sweep
is not: independent k-medoids runs at different K need not be nested, so a larger K can give
a *looser* bound, whereas every split strictly tightens.

### The first experiment was wrong, and its negative result should be ignored

The original A/B compared **fixed K=12** against **start at 12, refine up to 20**. Those do
not end at the same cluster count -- the refine arm was running a larger relaxed graph by the
end -- so it measured graph size, not partition quality, and reported refinement as "57%
slower". That number is meaningless for the hypothesis.

### The corrected experiment: equal final K, n=20, p=16, s=3, ms=7

| target K | seed | fixed (k-medoids at K) | refined (guided, up to K) | verdict |
| --- | --- | --- | --- | --- |
| 12 | 42 | OPTIMAL 1233 s | OPTIMAL **364 s** | 3.39x |
| 12 | 43 | OPTIMAL 687 s | OPTIMAL **332 s** | 2.07x |
| 12 | 44 | OPTIMAL 1611 s | OPTIMAL **419 s** | 3.84x |
| 16 | 42 | **FEASIBLE** @1800 s, obj 16337.65 | OPTIMAL **612 s**, obj 11376.12 | fixed never converged |
| 16 | 43 | **FEASIBLE** @1800 s, obj 10809.35 | OPTIMAL **488 s**, obj 10802.86 | fixed never converged |
| 16 | 44 | **FEASIBLE** @1800 s, obj 12293.18 | OPTIMAL **580 s**, obj 10192.89 | fixed never converged |

Objectives match exactly on every converged pair. At K=12 refinement is 2-3.8x faster; at
K=16 it is the difference between converging and not.

The mechanism shows in the secondary columns: CG iterations 73->15, 29->16, 60->18, and cuts
placed 855->61, 215->72, 1038->212 (4x-14x fewer). If the partitions were merely equivalent
you would expect comparable cut counts; needing an order of magnitude fewer says the guided
partition is genuinely **tighter at the same cell count**.

Note also **fixed K=16 is worse than fixed K=12** (12 converges, 16 does not) -- more cells
is not better, because the relaxed graph grows. But *refined* to 16 is best of all. Same
size, opposite outcome: partition quality, not size, is what matters.

### The one thing that made refinement work

Splits originally **cleared all accumulated cuts** (`empty!(cluster_sets)`), so a split
tightened the bound and reset all progress toward the certificate in the same move. Cuts are
now **rewritten** instead: splitting cell `c` into `c` and `c_new` leaves `stations(T)`
unchanged, so a `T` already proven barren stays proven -- it just needs `c_new` added
wherever `c` appears (`rewrite_cut_sets_for_split`).

### The measured ceiling, which still stands

`census_empty` -- barren rounds where **no** cluster showed witness disagreement, so
splitting provably cannot help -- is **64-90% at n=15 and 92-99% at n=20** (2235 of 2291
pooled). In those rounds the slack came from travel optimism or the independently-taken
ride-limit maximum, not from `rho_bar` aggregation. So refinement acts on roughly 1 barren
round in 40 at n=20. It is evidently enough (see the table above), but it caps how far this
particular witness can be pushed.

### Code

- `relaxed_cluster/refine.jl` -- census (`relaxed_cluster_split_candidates`), split
  (`refine_station_clustering`), trigger (`_relaxed_cluster_refine!`),
  `rewrite_cut_sets_for_split`.
- `relaxed_cluster/data.jl` -- records `reward_witness` (the `rho_bar` argmax), re-keyed onto
  routed node ids so intra-cluster credits resolve after replay.
- `optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl` --
  per-scenario clusterings / disagreement counters / split counts / refine stats.
- `formulations/aggregate_od_route/joint_routing_assignment.jl` --
  `relaxed_cluster_max_count` (default `nothing` = off), `relaxed_cluster_refine_recurrence`.
- Reporting: `cg_relaxed_cluster_final_counts`, `_splits`, `_refine_stats`.

---

## 3. Cut management — sound, benefit unmeasured

Dropping cuts a later one subsumes: for `T_old` subset of `T_new`, `Cut(T_new)` implies
`Cut(T_old)`, so the older cut excludes nothing further while still holding a bit of the
`UInt64` mask and **doubling the `(current, satisfied)` state space** -- labels differing
only in a dead bit can never dominate one another. Same failure mode as the earlier
`bounded_max_stops` finding.

**Measured (n=15 census, 861 cuts):** 1365 nested pairs, **0 duplicates**, **515 (60%)
dominated**. Duplicates are structurally impossible -- a surviving route must escape every
existing cut, so its support is never contained in one -- which is why only the
`old subset of new` direction needs pruning.

**Not measured:** no arm was run with it disabled, so its speed contribution is inference
from the dominance argument, not data. It was active in the refinement comparison above, so
its effect is folded into those numbers rather than isolated.

Code: `nogood_certify.jl`, `filter!(t -> !issubset(t, support), cluster_sets)` before the
cut is appended.

---

## 4. Barren-support cache — correct but inert

If `T` is barren, `T` subset of `T'`, and every cluster in `T'` not in `T` is
**reward-free** (holds no positive-reward candidate endpoint), then `T'` is barren too --
delete the extra stops from any route over `stations(T')` to get a route over `stations(T)`
that is no worse (travel does not increase, arrivals get earlier so no window or ride limit
is harder, no reward is lost). So the exhaustive subset search can be skipped.

**Verified sound** by independent audit: direction, recomputation across refinement, service
nodes, `WALK_ONLY_PAIR`, and the `j != k` exclusion that guarantees the shortened route
exists. It also requires a **complete** travel matrix, not merely metric -- deleting a stop
needs the shortcut arc to exist -- which is now checked (`_relaxed_cluster_travel_is_complete`)
and **disables** the cache rather than throwing.

**Measured: 0 hits in all 18 runs**, including ones placing 607, 726 and 923 cuts. The
completeness gate is not the cause (it returns true). The cause is density: only **0-1 of 12
clusters is reward-free**, so a hit needs the new support to be exactly a proven one plus
that single cluster. It needs `K << stations-carrying-demand-endpoints` to have anything to
work with.

Note barren-ness is **downward**-closed, never upward -- knowing `stations(T)` is barren says
nothing about a superset that adds reward-carrying stations. The downward direction is
already fully exploited by the cut itself, which kills every route confined to any subset
of `T`.

Code: `nogood_certify.jl` -- `_relaxed_cluster_reward_free`,
`_relaxed_cluster_barren_by_cache`, `_relaxed_cluster_travel_is_complete`;
`nogood_barren_cache_hits` on the recorded stats.

---

## What still needs doing

1. **Bound the escalation change before shipping it.** `certification_pricing_mode` now makes
   the two-tier certifying round unreachable: an empty-but-not-exhausted pricing round
   re-runs the **certifier** at `certifying_pricing_time_limit_sec` (3600 s) instead of
   re-pricing all n stations. Rationale: the relaxed search runs on K nodes rather than n,
   and the exact pricer's cost is super-linear in node count. **But it is unmeasured, and
   there is a specific risk:** a 300 s certification budget was measured to *starve* the CG
   loop at n=20/ms=10 (4-6 CG iterations in 30 minutes, `cert_sec` 67-99% of wall, zero cuts
   placed). 3600 s could be far worse than the round it replaces. Measured cost of removing
   the fallback: 4 of 45 arm runs (serial) and 1 of 35 (parallel) reached OPTIMAL only via
   the two-tier round, and all but one were K/n=0.4.

2. **The certification-budget tension is unresolved.** Short budgets keep the loop moving but
   rarely certify (30 s: 0/12 relaxation certificates at n=20); long budgets certify but
   starve the loop (300 s: 4-6 iterations in 30 min). Study 10 used 600-1200 s and did
   certify. Nobody has swept this knob deliberately, and it is probably the highest-value
   remaining measurement.

3. **A/B cut management** with it disabled, to convert the 60% domination rate into an actual
   speed number.

4. **Refinement at n=25 and n=30.** The equal-K result is n=20 only. n=30 is where
   certification is the only path to a proof, and where a tighter partition should matter
   most.

5. **`relaxed_cluster_refine_recurrence` was never swept.** All refinement results use 1
   (split on first sight). With `census_empty` at 92-99% the opportunities are rare enough
   that a higher threshold may simply never fire, but that is untested.

6. **Test coverage gaps** (none blocking): no test exercises a barren-cache hit *inside* the
   loop (only the predicate in isolation); the service-node witness path is verified by
   inspection only; the refine-counter identity
   `barren_rounds = blocked_ceiling + census_empty + census_nonempty` is unasserted.

7. **Update `CLAUDE.md`** -- `relaxed_cluster_max_count` and
   `relaxed_cluster_refine_recurrence` are absent from the AggregateODRoute parameter list,
   and the "cells must be identical across every CG iteration" rationale for
   `relaxed_cluster_count` is now conditional on refinement being off.

## Reproducing

```bash
# Study 10 (3 sizes x 4 arms x 5 seeds = 60 jobs)
cd benchmarks/study10_nogood_certification_scaling
julia --project=../.. generate_jobs.jl          # prints the --array range per size
export STUDY10_RUN_DATE=$(date +%F)             # MUST be set in the submitting shell
STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=1-20 --time=04:45:00 --mem=24G submit_benchmark.sh
STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=21-40 submit_benchmark.sh
STUDY10_RUN_DATE=$STUDY10_RUN_DATE sbatch --array=41-60 --time=05:45:00 --mem=28G submit_benchmark.sh
julia --project=../.. analyze.jl <run dir>      # pass the dir; _newest sorts alphabetically
```

Runs live in `benchmarks/experiments/`:
`2026-09-04_...` (pre-harvest), `2026-09-05_...` (harvest, serial),
`2026-09-05b-parallel_...` (harvest + parallel; the shipping numbers).

Diagnostics: `benchmarks/diagnostics/nogood_cut_nesting_probe.jl` (cut containment census).
**Both probes must be given enough budget to CONVERGE** -- deep no-good loops only occur near
convergence, and a truncated run yields one cut per attempt, zero pairs, and a census of
trivially zero. That mistake cost two runs here.
