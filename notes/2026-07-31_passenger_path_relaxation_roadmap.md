# Passenger-path relaxation roadmap

## Current production default

`station_simple_warm_start=true` is now the default. Column generation first
exhausts the elementary station-simple universe, then switches to the exact
revisit-tolerant pricer; only the latter certifies the unrestricted PFA pricing
problem. The n=20/p=16 comparison reached the same certified LP/MIP objective in
198.4s versus 513.1s for pure exact pricing (2.59x). Set
`station_simple_warm_start=false` to request pure revisit pricing for controlled
experiments.

The immediate production experiment is an opt-in `reward_coarsening_levels=2`
early-return harvester. It retains the revisit-tolerant physical route search,
rounds each passenger's reward upward to at most two ladder levels, and replays
every returned route against exact pricing data. Exhaustive certification is
always exact. Compare it against both the exact revisit harvester and the
station-simple harvester under identical duals, candidate limits, and time
limits.

## Acceptance gate for every increment

1. Prove the relaxation direction and test it route-by-route.
2. Reject any false certificate or accepted non-improving exact column.
3. Benchmark identical saved CG dual snapshots, excluding compilation.
4. Report wall time, labels, columns/second, best exact replayed reduced cost,
   overlap with the exact optimum, and timeout/exhaustion status.
5. Retain a feature only when its isolated speed improvement repeats across
   n=10/15/20 and does not materially degrade column quality or CG convergence.
6. Relaxed methods may harvest columns; only the exact revisit-tolerant pass may
   certify the unrestricted PFA pricing problem.

## Incremental hierarchy

1. **L=2 reward-coarsened harvesting.** Implemented opt-in. Compare directly
   with station-simple and exact revisit harvesting.
2. **Passenger-Lagrangian route relaxation.** Preserve one physical route but
   dualize one-assignment-per-passenger. First measure whether the relaxed route
   actually repeats passenger reward; do not build a multiplier optimizer until
   this census shows that uniqueness is a material gap source. Try 5-10 projected
   subgradient iterations, warm-started between CG iterations.

   Initial experimental API: `passenger_free_assignment_lagrangian_bound` in
   `pricing/passenger/lagrangian_relaxation.jl`. It maps every assignment to a
   pseudo-passenger, uses the existing label search as the single-route inner
   oracle, reports repeated-passenger multiplicity, and applies projected
   subgradient updates. It is intentionally not connected to CG pending the
   saved-dual benchmark.

   **Measured 2026-07-31: reject the plain Lagrangian bound.** On the first
   seeded-RMP dual snapshot at n=10/p=16, the relaxed best route collected as
   many as 4-6 alternatives for one passenger, confirming that uniqueness is a
   material missing constraint. The response to five projected-subgradient
   rounds was nevertheless weak: across the three scenarios the exact pricing
   optima were -1359/-1473/-1183, while the bounds were still
   -3333/-3184/-2746 (gaps 1974/1710/1563). After compilation, five rounds cost
   about 6-10x one exact search, and their best routes replayed at only
   -660/-670/-613. Validity was clean, including randomized tests, but this is
   neither a useful certificate nor a competitive harvester. Keep the prototype
   as a diagnostic; do not wire it into CG or add a bundle method unless a later
   formulation makes each inner solve much cheaper.
3. **Passenger-specific DSSR.** Track a small critical set exactly; add a
   passenger only when the relaxed best route exploits inconsistent alternatives
   or timing. Start with already committed, high-reward, low-slack passengers.

   First experiment: noncritical assignments become independent pseudo-passengers;
   critical passengers retain the exact max-reward ladder. After each exhaustive
   relaxed search, promote every real passenger collected through multiple
   alternatives by its best route. This directly repairs the measured Lagrangian
   failure without solving several multiplier subproblems at the same state.

   **Measured 2026-07-31: correct and finite, but reject this direct form for
   acceleration.** Randomized validity/monotonicity tests pass. On the same
   n=10/p=16 seeded-dual snapshot, starting with no exact passengers reached the
   exact optimum in 4-5 rounds after promoting 6-7 passengers, but generated
   20k-34k labels versus 3k-5k for one exact search. Starting with the top four
   reward passengers reduced this to three rounds, yet still generated
   10k-17k labels and took about 1.4-2.5x exact wall after warm-up. Starting
   with eight passengers was effectively the exact pricer (all eight active
   passengers were already exact): one round, essentially identical label count
   and wall. Thus refinement correctly identifies the missing state, but the
   relaxed passes are pure overhead and the useful critical set is almost the
   full active set. Keep the implementation as a correctness/reference oracle;
   do not wire it into CG.
4. **Post-pickup-cutoff completion.** Build a destination-only suffix bound after
   `max_wait_time`, first as a memoized exact micro-oracle on small instances,
   then test time buckets and critical-passenger state.

   Initial oracle: `passenger_free_assignment_post_w_completion`. It enumerates
   an elementary destination-only suffix from a concrete post-W label and
   returns the exact best completion under the metric-travel assumption. It is
   intentionally outside the hot search until state-count and cache-reuse
   measurements establish that querying it can save more work than it costs.

   **First n=10/p=16 result.** Ten sampled post-W labels per scenario needed only
   2.4-3.3 suffix states and 2-3 microseconds per exact completion. Relative to
   the existing remaining-reward bound, mean tightening was 0.0, 18.7, and 9.0
   reduced-cost units. Using the oracle directly as queue priority preserved the
   exact optimum and exhaustion in every scenario. Excluding the first/JIT-
   contaminated comparison, wall changed from 0.015s to 0.016s (neutral) and
   from 0.050s to 0.030s (1.64x faster); labels changed only 3120->3120 and
   5220->5207. The oracle consumed just 1-3ms across 413-917 calls, so it is
   cheap enough to test at n=15/20, but a speed win is not established yet.

5. **Short exact fragments.** Preserve exact local pickup/dropoff consequences
   inside 2-3 transition fragments and relax memory only between fragments.
6. **Single-path time-space relaxation.** Keep a single vehicle-flow backbone;
   attach at-most-one passenger rewards to that path. Prototype only on saved
   hard tail snapshots because the earlier MCF LP was 200x slower.
7. **Assignment incompatibility cuts.** Add passenger cliques and pairwise
   timing conflicts only as tightening for the single-path model.

Generic station ng-route memory and generic MCF cuts are lower priority: the
observed gap is passenger assignment/timing consistency, not primarily station
elementarity.

## Implementation order

Each item gets its own switch, unit validity tests, snapshot benchmark columns,
and decision note. Never combine two unproven features in the first benchmark;
composition is tested only after each feature wins independently.
