# Relaxed-cluster n=50 frontier: bounded harvesting and fixed-K experiments

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

## Goal

Push the confirmed `:relaxed_cluster` pricing pipeline from occasional certification at
`n=40` toward reliable certification at `n=50`, without relying on the currently
unconfirmed barren-cache, cut-management, or cluster-refinement features.

This is an experiment/design handoff. The main hypothesis is that the current bottleneck
is exhaustive harvesting inside a *productive* exact subset search, not the number of
no-good cuts handled by relaxed label setting.

**Amended after writing:** that hypothesis was acted on. The barren cache and cut
management were *removed* from `:relaxed_cluster` outright, since the cut load this note
measures below is far too small for either to pay for itself. Every
`barren_cache = false` / `cut_management = false` line in the plan below is therefore no
longer a setting to pass -- it is simply how the pricer now works. Both are written up as
possible future work in
`src/opt/label_setting/joint_routing_assignment/relaxed_cluster/README.md`.

## Current benchmark and evidence

Study scaffold:

- `benchmarks/study9_relaxed_cluster_scalability/`
- current clean output:
  `benchmarks/experiments/2026-09-08_rc_scale_v3_study9_relaxed_cluster_scalability/`
- workload: Zhuzhou, `p=16`, `s=3`, `max_stops=10`, parallel scenario pricing with
  three Julia threads
- current arms: `K/n = 0.6` and `K/n = 0.8`
- per-round regular budget: 300 s
- escalated certification budget: 3,600 s
- total n>=30 CG budget: 21,600 s (six hours)
- unconfirmed features explicitly disabled:
  `relaxed_cluster_max_count=nothing`, `relaxed_cluster_barren_cache=false`, and
  `relaxed_cluster_cut_management=false`

At the time of this note, all written n=30/40 rows certified correctly. The completed
sample is still right-censored, so runtime means must not be treated as final sweep
statistics.

Observed cut load is small when measured at the level that matters to label setting:

| size / arm | scenario attempts | cuts | pooled cuts per attempt | maximum inner rounds |
|---|---:|---:|---:|---:|
| n=30, K=18 (k60) | 378 | 283 | 0.75 | 11 |
| n=30, K=24 (k80) | 219 | 114 | 0.52 | 6 |
| n=40, K=24 (k60) | 84 | 45 | 0.54 | 3 |

Cuts are scenario-local and are discarded after each `(CG iteration, scenario)` attempt.
Thus the typical relaxed label-setting call sees zero or about one cut. The deepest
observed searches correspond to at most roughly 10 active cuts for n=30 k60, 5 for n=30
k80, and 2 for the completed n=40 cases. This does not support cut representation or
barren caching as the primary n=50 intervention.

In contrast, productive certification attempts can harvest very large pools. Examples
among completed n=30 k60 cases harvested roughly 4,500--6,500 accepted columns. Almost all
wall time is reported as certification time. The first two completed n=40 k60 cases took
46.9 and 51.5 minutes while harvesting 1,167 and 574 columns respectively.

## Primary hypothesis: productive subset searches run too long

The relevant implementation is:

- `src/opt/label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/certify.jl`
- `_relaxed_cluster_nogood_loop`, specifically the exact subset-pricing call
- `src/opt/solvers/cg_solver.jl`, which materializes and accepts certification candidates

The exact search over `stations(T)` currently constructs its acceptance closure with an
effectively unlimited candidate count:

```julia
accept! = _pricing_accept_closure(
    sub_ctx, s, sub_best_pool_tau, harvested, solver, typemax(Int) ÷ 2,
)
```

Consequently, a support that already contains a verified negative-reduced-cost column can
continue searching exhaustively and harvest a very large batch.

Exhaustion is required only along the barren branch:

- If no improving real column is found, the subset search must exhaust before `T` can be
  declared barren and a sound no-good cut can be added.
- If at least one verified improving real column is found, the attempt is already safely
  `:refuted`. It may stop early, return those columns, add no cut for `T`, and reoptimize
  the master. No optimality claim is made on that branch.

Therefore bounded harvesting is correctness-preserving as long as early termination is
interpreted as `:refuted`, never as barren or certified.

## Proposed parameter and behavior

Add an explicitly experimental pricing parameter, tentatively:

```julia
relaxed_cluster_harvest_limit::Union{Nothing,Int} = nothing
```

Semantics:

- `nothing` preserves current exhaustive-harvest behavior.
- A positive integer stops a productive subset search after that many accepted improving
  candidates have been collected for the scenario attempt.
- Reaching the cap returns `:refuted`, even though the subset search is not exhausted.
- Zero accepted candidates still requires genuine subset exhaustion before adding a cut.
- A timeout before either finding a candidate or proving exhaustion remains
  `:inconclusive`.
- Reject zero and negative values during config validation.

The implementation must preserve the existing final reduced-cost verification and pool
deduplication path. Prefer using the existing `_pricing_accept_closure` stopping mechanism
rather than introducing a second candidate filter.

Suggested initial limits: 16, 32, and 64. A limit of 32 is the leading first pilot.

Record enough metadata to distinguish generated candidates from accepted master columns:

- configured harvest limit
- candidates accepted per scenario attempt
- whether subset search stopped because the harvest cap fired
- subset-search exhaustion flag
- time split between relaxed guide search and real exact subset search

The last time split is important: current `certification_sec` combines both pieces, so the
existing output cannot conclusively attribute the n=40 cost.

## Secondary hypothesis: K should not grow as a fixed fraction of n

The current sweep makes relaxed search larger with station count:

- n=30: K=18 or 24
- n=40: K=24 or 32
- n=50: K=30 or 40

The small observed cut load leaves room to use a coarser relaxation in exchange for a few
more no-good rounds. Test fixed absolute values at n=50:

```text
K in {18, 20, 24}
```

Do not assume `K/n` should remain constant. In particular, K=24 already has evidence at
both n=30 (k80) and n=40 (k60), whereas allowing K to reach 30 or 40 may expand the
relaxed label-setting state space sharply.

## Secondary hypothesis: avoid eagerly unioning five guide routes

Current `relaxed_cluster_guide_routes=5` unions the supports of the five best relaxed
routes before launching one real exact subset search. Consider progressive guidance:

1. Search the best route's support.
2. If barren, add its sound cut and request another relaxed route.
3. Enlarge to two or more guide supports only when single-route supports are repeatedly
   barren or unproductive.

An easier first ablation is `guide_routes in {1, 2, 5}`. This may exchange additional
small cuts for substantially cheaper exact subset searches. It is secondary because the
completed n=40 cases already report median guided subsets of only 7--8 stations, but an
exhaustive elementary search with `max_stops=10` can still be expensive at that size.

## Recommended experiment sequence

Use matched seeds and keep all other Study 9 settings identical.

### Stage A: isolate bounded harvesting at n=40

Run at least five matched seeds with K=24 and:

```text
current exhaustive harvest
harvest_limit = 16
harvest_limit = 32
harvest_limit = 64
```

This establishes whether lower per-iteration work compensates for any increase in CG
iterations. Compare certified LP objectives on matched seeds, not only recovered integer
objectives.

### Stage B: n=50 fixed-K pilot

Take the best harvest policy from Stage A and run:

```text
K in {18, 20, 24}
guide_routes = 5 initially
```

The leading candidate is:

```text
K = 20 or 24
harvest_limit = 32
guide_routes = 5
barren_cache = false
cut_management = false
cluster refinement disabled
```

### Stage C: guide-support ablation

If exact subset time remains dominant, test `guide_routes in {1, 2, 5}` with the best
fixed K and harvest cap. Prefer progressive support growth over a permanent large union if
implementing an adaptive version.

### Stage D: only if needed

Consider parallel searches of individual guide supports within a scenario. Scenario-level
parallelism already uses three Julia threads; this stage would require deliberate core
allocation and cancellation behavior and is more invasive than the first three stages.

## Acceptance criteria

Correctness invariants:

- Every claimed certificate has `cg_converged=true`,
  `cg_stop_reason="converged_by_certification"`, and
  `cg_optimality_scope="full_route_universe"`.
- Matched certified arms agree on the final LP objective within numerical tolerance.
- A capped, non-exhausted subset search can only yield `:refuted`; it must never add a
  barren cut or certify.
- An empty non-exhausted or timed-out subset search remains `:inconclusive`.
- Existing experimental cache, cut-management, and refinement flags remain false in this
  study.

Performance metrics:

- certification rate within the same six-hour CG budget (headline)
- wall time and certification time, including right-censored cases
- CG iterations and certification attempts
- relaxed-search versus exact-subset-search seconds
- harvested accepted columns per refuted attempt
- master pool size and master LP time
- cuts per scenario attempt and maximum active cuts
- fraction of subset searches stopped by the harvest cap

The frontier criterion used by Study 9 is strictly greater than 90%. With ten seeds, an
arm must certify 10/10; 9/10 does not clear the frontier.

## Likely implementation tests

Add focused tests near
`test/opt/test_joint_routing_assignment_relaxed_cluster_pricing.jl`:

1. A productive subset with a cap returns `:refuted`, candidates are nonempty, and
   `subset_exhausted=false` is allowed on that branch.
2. The cap is respected after verification/deduplication, not merely after raw route
   generation.
3. A capped search that finds no column cannot add a cut unless it exhausted.
4. A capped run and the uncapped baseline certify the same LP objective on a small matched
   instance.
5. `nothing` reproduces current behavior and invalid limits are rejected.

## Caution for interpreting existing results

Completed-only runtime summaries are optimistically biased while hard jobs remain active.
Report terminal success rates only after every task in an arm is terminal, and treat the
six-hour `total_budget` rows as right-censored observations rather than ordinary runtimes.

