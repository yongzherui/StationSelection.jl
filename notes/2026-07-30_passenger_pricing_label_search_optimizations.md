# Passenger free-assignment label search: what actually made it faster

Evaluation of eight proposed pruning/dominance improvements to
`src/opt/optimize/aggregate_od_route/pricing/passenger/`, each implemented and
measured separately rather than adopted as a batch.

## Measurement setup

`scripts/bench_passenger_free_assignment_labels.jl` runs the label search to
**genuine exhaustion** (`n_candidates` effectively unbounded) on the zhuzhou
n_pairs=16 seed=42 family, and reports both a correctness invariant (`best_rc`,
the most negative reduced cost found, which every exact change must leave
bit-identical) and cost metrics (`wall`, `labels`, `max_live`).

The pre-existing `diag_passenger_free_assignment_pricing_scale.jl` is not usable
for this: it stops at `n_candidates=5`, so it measures "time to five columns",
which moves whenever acceptance *order* changes even if nothing got faster.

Variants are submitted through `scripts/submit_pfa_bench_snapshot.sh`, which
copies the tree to `$HOME/.pfa_bench_snapshots/<variant>/` before submitting.
Benchmark jobs queue for minutes and Julia reads the source when the job
*starts*, so editing `src/` while a job is queued silently benchmarks the new
code under the old variant's name. Snapshots make variants pipelineable.

Note when comparing runs: node-to-node variance on `mit_normal` is around 8%
(the same code measured 33.0s and 35.7s on two different nodes), so anything
under ~10% is noise.

## The profile that redirected everything

Before optimizing, the profile counters showed:

```
n=15 ms=6 s=3:  wall=33.0s   dominance=30.3s   queue=0.20s  candidates=0.10s  extension=0.53s
```

**~90% of wall time is the dominance scan.** `t_queue` includes the
`remaining_reward_bound` call (`label_priority` is evaluated inside that timer),
so the bound is ~0.6% of runtime. `search.jl`'s docstring claiming the bound is
"the single biggest cost in the search" is stale -- it was true before the
earlier `O(|opportunities|)` -> `O(#live opportunities + n_nodes)` rewrite, and
has not been true since.

Consequence: bound-tightening proposals can only help *indirectly*, by shrinking
the label population so the buckets the dominance scan walks get smaller. The
dominance scan is linear in bucket size per insertion, so halving live labels
quarters the work -- which is why the change that halved `max_live` was worth
far more than the change that tightened the bound.

## Results

Each variant is cumulative on the one before it.

| variant | change | n=15 ms=6 s=1 | n=20 ms=5 s=1 | verdict |
| --- | --- | --- | --- | --- |
| baseline | -- | 21.2s | 104.1s | -- |
| v1 | reuse popped priority | 20.8s | -- | neutral, kept |
| v2 | travel-discounted bound | 22.1s | -- | no effect, kept on principle |
| v3 | compensated layer dominance | 6.57s | 33.0s | **2.5-3.9x, kept** |
| v4 | inline label into bucket entry | 5.91s | 31.2s | 1.1-1.15x, kept |
| v5 | `Vector` bucket instead of `SortedDict` | 4.38s | 20.7s | 1.3-1.5x, kept |

End to end, on the standard grid:

| case | baseline | final | speedup |
| --- | --- | --- | --- |
| n=15 ms=5 s=1 | 4.78s | 2.53s | 1.9x |
| n=15 ms=5 s=2 | 2.65s | 0.99s | 2.7x |
| n=15 ms=5 s=3 | 5.89s | 2.11s | 2.8x |
| n=15 ms=6 s=1 | 21.2s | 4.38s | 4.8x |
| n=15 ms=6 s=2 | 17.0s | 2.88s | 5.9x |
| n=15 ms=6 s=3 | 33.0s | 8.43s | 3.9x |
| n=20 ms=5 s=1 | 104.1s | 20.7s | 5.0x |
| n=20 ms=5 s=2 | 46.2s | 12.6s | 3.7x |
| n=20 ms=5 s=3 | 219.7s | 34.1s | **6.4x** |

`best_rc` was bit-identical to baseline in every case at every variant; the
focused unit tests (460 assertions) passed at v3, v4 and v5; and
`diag_passenger_free_assignment_vs_direct.jl` still matched brute-force
enumeration on every n=10/n=12 scenario at v2 and v5.

Dominance is still ~85-90% of the remaining wall time, so that scan is where any
further work belongs.

### v1 -- reuse the popped priority (neutral, kept)

`label_priority` was computed once at enqueue and again at pop. Labels and their
bitsets are immutable after insertion, so `dequeue_pair!` can just hand the
value back. Semantics-identical; worth ~0.05s of a 33s run, i.e. nothing, because
the bound is not the bottleneck. Kept because it is strictly less work.

### v2 -- travel-discounted remaining-reward bound (no measured effect)

The bound returned the raw reward of everything still reachable, pretending the
travel to collect it were free. It now books each still-attainable layer against
the node that must be reached to collect it (destination for a live origin's
opportunity, the origin itself for a refreshable one), walks nodes in increasing
distance from `current` via a precomputed order, and takes the running maximum of
`R(prefix) - beta * travel(current, node)`. Valid under the triangle inequality,
which `_passenger_free_assignment_age_is_useful` already relies on.

It is exact -- unit tests and the brute-force oracle both pass -- but it changed
label counts by under 0.2%. The reason is scale: at cold-start duals the bound
sums ~10+ passenger rewards of ~5000 each while one hop costs
`beta * travel ~ 4000`, so a discount of one hop almost never flips the pruning
test.

**This is expected to matter under converged CG duals**, where most `rho_pjk` are
near zero and `beta * travel` is comparable to the whole remaining reward. That
regime is not reproducible by rescaling `BASE_VALUE` in this benchmark (lowering
it shrinks every reward uniformly, rather than making a few large and the rest
zero), so it was not demonstrated here either way.

### v3 -- compensated reward-layer dominance (2.5-3.9x, kept)

The dominance rule required `A_a subseteq A_b`. It now requires

    rc_a + w(A_a \ A_b) <= rc_b

with the old subset rule recovered as the `w = 0` case, so this only ever adds
dominations.

**The direction matters and the original proposal had it backwards.** It
proposed charging `w(A_b \ A_a)`. Deriving it: for a suffix `sigma` feasible from
`b`, `a`'s no-worse ages give `reach_b(sigma) subseteq reach_a(sigma)`, and
`reach_b \ A_b subseteq (reach_a \ A_a) union (A_a \ A_b)`, so
`reward_b(sigma) <= reward_a(sigma) + w(A_a \ A_b)`. It is the layers **`a`
holds and `b` lacks** that must be paid for -- reward `b` can still bank off the
shared suffix while `a`, having already banked it, cannot re-earn it. Charging
`w(A_b \ A_a)` instead is unsound: with `A_a = {big}`, `A_b = {}`,
`rc_a = rc_b - epsilon`, it declares domination even though a suffix certifying
`big` lets `b` overtake `a`.

Because the compensation is non-negative it still implies `rc_a <= rc_b`, so
`_add_passenger_free_assignment_label_to_bucket!`'s reduced-cost-ordered walk
(scan below the new label's rc for dominators, above it for dominatees) stays
valid unchanged.

Cost control: `issubset` is tried first (word-wise, and it is the single most
common case since it is exactly the old rule), and the element-wise scan bails as
soon as the running weight exceeds the reduced-cost budget -- which is usually
after one layer, since layer weights are large relative to typical rc gaps.

Effect: `max_live` roughly halves (58,260 -> 21,917 at n=15/ms=6/s=1;
117,950 -> 52,461 at n=20/ms=5/s=1), and wall time falls superlinearly with it.
Speedup grows with instance size, which is the desirable direction.

### v4 -- inline the label into the bucket entry (1.1-1.15x, kept)

The scan read `live_labels[existing_id]` and `label_bitsets[existing_id]` for
every entry it visited: two hash probes per entry, on a loop that runs tens of
millions of times. Bucket entries now carry the label and its bitsets directly
(`PassengerFreeAssignmentBucketEntry`), which also let the separate
`label_bitsets` dictionary be deleted outright -- after v1 the only remaining
read of it was already dead.

Labels and `max_live` are bit-identical, as they must be for a pure data-layout
change. Worth 10-15%, less than the two-hash-probe estimate suggested, which is
what pointed at v5.

### v5 -- `Vector` dominance bucket instead of `SortedDict` (1.3-1.5x, kept)

Backing out the arithmetic after v4: ~69M predicate evaluations in 5.26s is
~76ns each, far too slow for a predicate that is mostly short-circuiting scalar
comparisons. The cost was the container, not the test -- `SortedDict` iteration
is a balanced-tree traversal, so every step is a pointer chase into an unrelated
cache line.

Buckets are now sorted `Vector`s, walked contiguously; eviction collects
ascending indices during the scan and applies one `deleteat!`, and insertion is a
`searchsortedfirst` plus `insert!`. Insertion/eviction become `O(bucket)`
memmoves rather than `O(log bucket)` tree surgery, which is a good trade: there
is one insertion per label against a full-bucket scan, and a memmove of a few
thousand small structs runs at memory bandwidth.

Note when copying this pattern: `searchsortedfirst(v, x; by=f)` applies `by` to
the search value too, so `x` must be an *entry*, not a bare key tuple.

## Proposals rejected on inspection

### "Useful-stop elimination, especially after W" -- already implemented

`_passenger_free_assignment_candidate_next_nodes` (`labels.jl`) already enforces
exactly this. Past the pickup cutoff it offers *only* destinations, and only
those where a specific live-origin opportunity both has an inactive layer and
satisfies `origin_age + travel(current, dest) <= ride_limit` -- i.e. every
post-`W` visit must immediately activate a layer. Pre-`W` origin offers are
filtered by `origin_layer_mask` plus cutoff reachability, and
`_has_useful_live_passenger_free_assignment_origin` kills dead-end labels
outright. Nothing left to add.

### "At most L distinct service stations" -- IP-valid only, and not exactly enforceable on a label

Two independent problems.

*It invalidates the LP bound.* The master links `theta_r <= y_j` per `(p, j)`
with `sum(y) == l` and `y in [0,1]` relaxed (`master.jl`). A route using `m > l`
distinct stations is not LP-infeasible; it is merely capped at
`theta_r <= l/m > 0`. Excluding such columns restricts the LP, which for this
minimisation *raises* the RMP objective -- so `lp_bound` would stop being a
lower bound on the true LP, which is precisely what
`cg_stop_reason == :optimality_proven` certifies.

*The label cannot enforce it exactly.* Only stations carrying an *assignment*
need to be open, and which those are is resolved at route replay, not during
expansion. A label's visited-station set (or its certified-station set)
over-approximates the final assignment-station set, so pruning on
`|U| > l` can discard routes that would have been feasible. An exact cap needs a
lower bound on the final count, which the label does not carry.

Both are surmountable only by making it an opt-in, IP-only option with the
`lp_bound` certificate explicitly disabled. Not adopted.

### "Compatibility components" -- measured, and the graph does not split

`scripts/diag_passenger_free_assignment_components.jl` builds the positive-reward
station graph (an edge `j -- k` per opportunity `(p, j, k)`) and reports its
connected components. Result across every scenario at n=10, 15 and 20:

```
components=1   largest_share=1.000   mean_stations_per_passenger=4.8-9.6 of 10-20
```

One component, every time, holding 100% of opportunities -- against the "worth
doing only below ~80%" threshold from the design discussion. Passengers can reach
roughly half the stations within the walking radius, so the graph is one blob.
Decomposition is dead here, and so is the adaptive station core (#7), which needs
the same structure plus a subset bound to certify the omitted region.

### "Used-station-set dominance `U_a subseteq U_b`" -- a slowdown on its own

Adding a required condition makes dominance fire *less* often. It is only
meaningful as the correctness companion to the `L`-cap above (without which `U`
is not a consumed resource at all), so it is one package with a proposal that was
not adopted, not an independent win.

## Related

- `2026-07-29_passenger_free_assignment_pricing.md` -- the pricer's design
  rationale (reward layers, replay, signatures).
