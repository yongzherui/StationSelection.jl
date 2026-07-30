# Passenger free-assignment label search: what actually made it faster

Evaluation of eight proposed pruning/dominance improvements to
`src/opt/optimize/aggregate_od_route/pricing/passenger/`, each implemented and
measured separately rather than adopted as a batch.

## Verdict summary

Net result: **4-6x faster exhaustive pricing**, essentially all of it from one
change.

| change | result | status |
| --- | --- | --- |
| compensated layer dominance | **2.5-3.9x** -- halves `max_live` | kept, on |
| `Vector` dominance buckets instead of `SortedDict` | **1.3-1.5x** | kept, on |
| label + bitsets inlined into the bucket entry | **1.1-1.15x** | kept, on |
| reuse the popped priority | no effect (bound is ~0.6% of runtime) | kept, on |
| travel-discounted remaining-reward bound | no effect at cold-start duals | kept, on |
| station-budget cap at `l` (+ `U_a ⊆ U_b`) | slower; LP bound did not move | kept, **off** |
| useful-stop / post-`W` elimination | already implemented | n/a |
| compatibility-component decomposition | graph is 1 component, 100% of opportunities | not built |
| adaptive station core | needs the decomposition above | not built |

The one thing to take away before optimizing this further: **the dominance scan
is ~85-90% of wall time** and everything else together is under 2%. Every change
that did not shrink the live-label population or cheapen the bucket walk measured
as a no-op, regardless of how sound it looked on paper.

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
| v6 | station-budget cap at `l` (+ `U` dominance) | -- | -- | valid but slower; **default off**, see below |

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
focused unit tests passed at every variant (460 assertions through v5, 471 once
the station-budget cap added its own); and
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

### "At most L distinct service stations" (+ `U_a subseteq U_b` dominance) -- valid, implemented, but does not pay for itself

**This section previously rejected the cap on two grounds. Both were wrong**, and
the corrected analysis is recorded here because the wrong version is the
intuitive one.

*It does not invalidate the LP bound; it tightens it.* The mistake was treating
the restriction as arbitrary. It is not: with `P' = {r : |A_r| <= l}`, an integer
`theta_r >= 1` forces `y_j = 1` at every assignment-carrying station of `r`, so
`|A_r| <= sum(y) = l`. The removed columns are unusable by *any* integer
solution, hence

    IP(P') = IP(P),   LP(P') <= IP(P') = IP(P),   LP(P') >= LP(P)

-- still a valid lower bound on the IP, and no weaker than before.

*The label can enforce it, on visited stations.* The worry was that a route may
visit a station carrying no assignment after replay, so the visited set
over-approximates `A_r`. True, but such a station can always be deleted from the
route: by the triangle inequality removing `k` lowers `tau` and makes every later
arrival earlier, which only relaxes ride limits and opens more pickup clocks, so
reward cannot fall. Hence every column with `|A_r| <= l` has a counterpart with
reduced cost no worse whose route visits only assignment-carrying stations, and
pricing over `{|visited| <= l}` attains the same minimum as over `{|A_r| <= l}`.

Implemented as `max_distinct_stations` on the pricing data (a `UInt64`
`visited_mask` per label, disabled above 64 stations), with `station_budget_cap`
on the CG entry point wiring it to `master_data.l`. Its soundness companion --
the `U_a subseteq U_b` condition in dominance, without which a label that has
spent more station budget could stand in for one that has spent less -- is gated
on the cap being active, so it costs nothing when the cap is off.

**Measured, with `max_stops` unbounded (the regime the cap exists for):**

| case | no cap | cap at `l` | labels | max_live |
| --- | --- | --- | --- | --- |
| n=15 s=1 (l=8) | 7.07s | 6.93s | 137k -> 271k | 23k -> 62k |
| n=15 s=2 (l=8) | 3.58s | 6.15s | 110k -> 238k | 19k -> 61k |
| n=15 s=3 (l=8) | 11.73s | 9.78s | 215k -> 339k | 34k -> 74k |
| n=10 s=1 (l=5) | 1.53s | 1.60s | 11.9k -> 10.2k | 3.4k -> 4.4k |

Neutral to 1.7x *slower*. The domination rate falls from 80.7% to 74.4% because
of `U_a subseteq U_b`, so more labels survive, buckets grow, and the dominance
scan -- still ~90% of runtime -- costs more than the branch pruning saves.

It binds weakly for a structural reason: ride limits and the pickup window
already hold the best routes to 6-8 distinct stations at n=15, so `l = 8`
constrains almost nothing. It bites hardest at n=10/`l=5`, which is also the only
place label counts actually fell.

`best_rc` changes where the cap binds (n=10 s=1: `-53591.86 -> -53578.05`). That
is the cap working, not a bug -- the excluded route used 6 distinct stations
against `l = 5`. Because of this the feature cannot be validated against the
unrestricted optimum, so it carries its own brute-force test asserting it attains
the best reduced cost *over the routes the cap permits*.

**The metric that should have decided it was bound quality, not pricing speed.**
The point of `LP(P') >= LP(P)` is a tighter bound, and the motivating failure mode
was `project_lp_mip_gap_hub_routes`: "CG picks broad hub routes for dual credit;
MIP pays full travel cost for unused certified pairs; 21.6% gap" -- broad hub
routes being exactly the many-station columns the cap forbids. Measured end to
end through the CG loop (`scripts/passenger_free_assignment_cg_scaling.jl`,
`PFA_STATION_BUDGET_CAP=1` vs `=0`, unbounded stops, 1 scenario, both reaching
`optimality_proven`):

| n | lp_bound (off) | lp_bound (on) | gap% off | gap% on | wall off | wall on |
| --- | --- | --- | --- | --- | --- | --- |
| 10 | 18974.94480755 | 18974.94480755 | 0.524 | 0.525 | 7.55s | 7.97s |
| 15 | 16107.42403670 | 16107.42403670 | 0.000 | 0.000 | 5.73s | 7.86s |

**The LP bound is identical to ten decimal places.** The LP optimum never wanted a
column spanning more than `l` stations, so the cap removes nothing that was
binding -- while still costing 5-37% more wall time and generating *more* columns
(436 -> 453, 459 -> 537).

Why the motivating pathology does not appear here: the hub-route gap was measured
on the **aggregate** pricer, whose reward sums over independently-certified
station pairs, so a route can buy extra dual credit merely by touching more
stations. The passenger free-assignment reward is a per-passenger *maximum*
(that is what the layer encoding enforces), so touching more stations for the
same passenger earns nothing extra. The formulation is structurally immune to the
gap the cap was meant to close, which is also why its LP-IP gap here is already
0.0-0.5%.

**Verdict: kept, default off.** Exact, tested, and correct -- worth having for
instances where feasibility does *not* already bound distinct stations below `l`
(short ride limits and a wide pickup window would be the case to try) -- but inert
and mildly costly on everything measured here. Do not enable it expecting a
better bound without re-measuring `lp_bound` on the target instance.


## Related

- `2026-07-29_passenger_free_assignment_pricing.md` -- the pricer's design
  rationale (reward layers, replay, signatures).
