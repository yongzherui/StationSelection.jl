# Relaxed-cluster pricing and certification

This directory holds the `:relaxed_cluster` pricing mode for
`AggregateODRouteJointRoutingAssignmentFormulation` — the only mode that can **certify**
column-generation optimality over the full route universe.

```text
clustering.jl  relaxation.jl  data.jl          # the relaxed graph (K cluster nodes)
cuts.jl  types.jl  seed.jl  extend.jl          # the cut-aware pricer that runs on it
         context.jl  hooks.jl
utils/
├── certification/certify.jl                   # the no-good-cut loop
├── guiding/guide.jl                           # station-subset selection (warm start)
└── refinement/refine.jl                       # witness-guided cell splitting (opt-in)
```

Read `relaxation.jl` for what the relaxation is and why its minimum reduced cost
lower-bounds the real one, `cuts.jl` for the cut's exact form and the soundness argument,
then `utils/certification/certify.jl` for the loop that ties them together. The parent
`../../README.md` explains why two different searches run on this one graph.

## The loop, in one box

```text
1. relaxed search, respecting every cut so far  ->  best improving cluster route
2. nothing improving, search exhausted          ->  CERTIFIED (full route universe)
3. T := clusters the best few relaxed routes visit
4. exact search over stations(T), exhaustively
     found an improving real column  ->  REFUTED (harvest it; this was a pricing round)
     nothing                         ->  T is BARREN: cut it, go to 1
```

The cuts are the mechanism, not an optimization on top of it: a cut-free round is exactly
round 1, and round 1 certified 0 times across ~1130 measured attempts at every size and
every `K` (`notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`). A
converged master's exact minimum reduced cost is exactly 0 while the relaxation's slack is
10²–10³, so the relaxation essentially always names *something* improving on round 1.

## Configuration

Everything here is selected through `CGSolver.pricing`, a `CGPricingConfig`:

| Field | Role |
| --- | --- |
| `mode = :relaxed_cluster` | run the no-good-cut loop as the pricing round |
| `relaxed_cluster_count` = K | **required**; sizes the k-medoids partition, built once at build time |
| `relaxed_cluster_guide_routes` | how many improving relaxed routes union their support into step 3 (default 5) |
| `relaxed_cluster_max_count` | opt-in ceiling that turns the partition into a starting point for `utils/refinement/` |
| `warm_start_mode = :cluster_guide` | warm-start-only use of the cut-free search via `utils/guiding/` |

## Possible improvements, currently unnecessary

Two optimizations of the cut loop were implemented, measured, and then **removed** from
this pathway. Both were sound; neither had anything to win, because the measured cut load
per certification attempt is tiny. On the current frontier workload (Zhuzhou, `p=16`,
`s=3`, `max_stops=10`, `notes/2026-09-08_relaxed_cluster_n50_frontier_handoff.md`):

| size / arm | scenario attempts | cuts | cuts per attempt | max inner rounds |
| --- | ---: | ---: | ---: | ---: |
| n=30, K=18 | 378 | 283 | 0.75 | 11 |
| n=30, K=24 | 219 | 114 | 0.52 | 6 |
| n=40, K=24 | 84 | 45 | 0.54 | 3 |

Most attempts add fewer than one cut, and the deepest observed loop used 11 of the 64
available mask bits. Both optimizations pay off only in the opposite regime — many cuts per
attempt — so they are worth revisiting if a future workload pushes attempts toward the
`RELAXED_CLUSTER_MAX_CUTS` cap, and not before.

### Barren-support cache

**Idea.** If `T` is barren, `T ⊆ T'`, and every cluster in `T' \ T` is *reward-free* (holds
no candidate endpoint at these duals), then `T'` is barren too — so step 4 can be skipped
entirely and the cut added with no exact search at all.

**Why it is sound.** Take any route `R` over `stations(T')`. Delete its stops in the extra
clusters to get `R'`. Travel does not increase (triangle inequality), every arrival is
earlier so no pickup window or ride limit is harder to meet, and no reward is lost —
a reward-free cluster anchors no certification. So `rc(R) >= rc(R') >= -tol` because `R'`
lies in the barren `stations(T)`.

**What it needs.** A travel matrix that is both metric and *complete*: deleting `n2` from
`n1 -> n2 -> n3` requires the arc `n1 -> n3` to exist, and only finite arcs are stored. The
old implementation checked both once per attempt and disabled itself otherwise. The
reward-free test also cannot be weakened to "the witnessing route gained nothing there" —
the exact search ranges over *all* routes in `stations(T')`, so the condition has to be a
property of the candidate set, not of one route. And the inference runs in this direction
only: barren-ness is downward-closed, so a barren `T` says nothing about a superset that
adds reward-carrying stations.

**Why it was dropped.** It saves at most one exact subset search per cut, on the subset of
cuts whose support grows only by reward-free cells. At 0.5–0.75 cuts per attempt that
ceiling is well under one search per attempt, before accounting for the O(n³) metric check
each attempt paid up front.

### Active-cut subsumption pruning

**Idea.** `Cut(T_new)` implies `Cut(T_old)` whenever `T_old ⊆ T_new`, so the older cut then
excludes nothing further while still holding a bit of the `UInt64` mask and doubling the
`(current, satisfied)` state space — which weakens dominance in the cut-aware search.
Dropping subsumed cuts as each new one is proved keeps the mask narrow.

**Measured nesting.** At n=15, 60% of cuts were dominated this way (515 of 861), with 0
exact duplicates ever — which is why only this one direction would need pruning.
`benchmarks/diagnostics/nogood_cut_nesting_probe.jl` measures it from a run's recorded
`nogood_supports`, which `utils/certification/certify.jl` still emits for exactly that
purpose.

**Care required if reinstated.** Pruning acts on the *active cut set* only. A proof must
never be discarded along with its cut: a smaller barren support that a larger cut subsumes
for search purposes can still be the premise a barren-support cache needs later, so the
two structures have to be kept separate (the removed implementation carried
`cluster_sets` and `barren_supports` side by side). Under refinement, both need
`rewrite_cut_sets_for_split` applied on every split.

**Why it was dropped.** The pruning only matters once a single attempt carries enough
simultaneous cuts for the mask to hurt dominance. With a maximum of 11 active cuts observed
at n=30 and 3 at n=40, the state-space penalty it removes is not measurable.
