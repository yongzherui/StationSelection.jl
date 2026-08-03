# PFA pricers: mechanical optimisation pass (2026-08-03)

A pass over the passenger free-assignment label-setting pricers that changes **no
pruning rule**: condition order, memory layout, one fused bitset walk, and
allocation removal. Every case in
`scripts/bench_passenger_free_assignment_labels.jl` returns bit-identical
`best_rc`, `labels`, `rejected`, `removed`, `max_live`, `columns` and winning
route; the full suite (2456 tests) passes.

Report with interactive before/after flame graphs:
<https://claude.ai/code/artifact/75310c3a-57da-4d40-ad4f-d83af19b4073>

## Result

| pricer | wall speedup | dominance-region speedup |
| --- | --- | --- |
| revisit-tolerant | 1.67-9.17x (grows with n) | 6.2-17.2x |
| station-simple | 1.57-2.26x | 1.6-2.0x |

Benchmark total 128.62s -> 38.05s (**3.38x**). Baseline job 19577466, new job
19578799. The revisit pricer gains most because the dominance scan was ~90% of its
wall; station-simple's four instrumented regions are under 20% of its wall, so its
gains came from the harvest path and allocation pressure instead.

## Why the condition order matters (the measurement that drove it)

`scripts/audit_pfa_dominance_conditions.jl` (new) counts which condition is the
*first* to reject each tested pair. Over 456M pairs:

| # | condition | cost | n15/ms6/s3 | n20/ms5/s3 |
| --- | --- | --- | --- | --- |
| 1 | time | 1 compare | 49.4% | 47.9% |
| 3 | live-clock support mask | 1 AND-NOT | 39.5% | 43.6% |
| 2 | support size | 2 loads | 9.1% | 6.6% |
| 7 | station ages | O(#live) walk | 1.1% | 0.7% |
| 4 | route length | 1 compare | 0.7% | 0.6% |
| 8 | reward compensation | bitset walk | 0.2% | 0.5% |
| - | dominates | - | 0.10% | 0.06% |
| 5 | reduced cost | 1 compare | **0** | **0** |
| 6 | visited mask | 1 AND-NOT | **0** | **0** |

Three scalar tests dispose of ~96% of pairs; the two conditions that touch heap
data run on under 2%. `reduced_cost` records zero rejections because the bucket
walk splits on exactly that comparison before calling the predicate -- it is kept
only as a guard for the non-bucket caller and because the compensation's early
bail is defined against a non-negative budget. `visited_mask` is zero because the
station budget is off by default (and is then compiled out of the predicate).

Instrumentation is a type-parameter specialization
(`PassengerFreeAssignmentDominanceRules{..., Instrumented}`), so a production
search compiles the counters out entirely. The census run is therefore a
differently compiled pricer: use it for rejection distribution, never for wall
clock.

## Changes

1. **Conditions reordered** cheapest-and-likeliest first. The reward compensation
   ran ahead of the station-age merge walk, so every pair the ages were going to
   reject paid a bitset traversal first. (`labels.jl`,
   `_dominates_passenger_free_assignment_in_bucket`; same in
   `station_simple.jl`.)
2. **`age_mask`**: `dom(age_b) subseteq dom(age_a)` as one `UInt64` AND-NOT in
   front of the walk that used to establish it element by element. Station indices
   fold mod 64, which above 64 nodes can only *weaken* the filter (the walk still
   decides), never break it. Alone this rejects 39-44% of pairs.
3. **Compensation fused into one word-wise pass** over `BitSet` chunks, instead of
   `issubset` followed by an element walk that re-probed membership per integer.
   The subset case costs what `issubset` cost; the non-subset case stops
   traversing twice. (`_passenger_free_assignment_compensation`.)
4. **Scalar dominance state inlined into the bucket entry**
   (`PassengerFreeAssignmentDominanceFilters`, 40 bytes; entry 24 -> 64 bytes =
   one cache line). A rejected entry never dereferences `label` or `bitsets`.
5. **Dominance switches moved into the type**
   (`PassengerFreeAssignmentDominanceRules`), so `bounded_max_stops` /
   `bounded_distinct_stations` -- both off by default -- compile away instead of
   costing a branch per scanned entry. One dispatch per label insertion, against a
   scan hundreds of entries long.
6. **Four allocations per label removed**: the per-label mirror insertion-sorts in
   place instead of `sortperm` + two gathers, and aliases the reward-layer `BitSet`
   rather than copying it (nothing mutates it -- see the docstring for what breaks
   if that changes); extension returns its single child directly instead of a
   one-element `Vector`; `live_labels` is an array indexed by label id instead of
   a `Dict`; `best_by_signature` and the station-simple bucket `Dict` are
   concretely typed instead of `Any`-keyed.
7. **Route replay stopped rebuilding a `Dict` per stop** -- absolute pickup times
   with `age = elapsed - pickup_time[j]` instead of a per-stop aged copy. Runs once
   per harvested column.
8. **`string(route)` lifted out of a sort comparator.** `sort!(...; by=f)` calls
   `f` inside the comparison, so the route's string form was built once per
   *comparison* -- ~17x more at 1e5 harvested columns, and 15% of the
   station-simple pricer's working time in array-show machinery
   (`_typeinfo_implicit`). Decorate-sort-undecorate; ordering unchanged.

Items 7 and 8 were found by the flame graph, not by reading the code -- both sit
outside every hand-instrumented region, which is exactly why the old `PROFILE`
counters could not see them.

## Flame-graph harness (new)

`scripts/profile_pfa_flamegraph.jl` + `scripts/sbatch_profile_pfa_flamegraph.sh
<label>` run a pricer under the sampling and allocation profilers and write a
**self-contained interactive flame graph** (click to zoom, search to highlight),
the Brendan-Gregg folded stacks, and `SELFTIME`/`ALLOC` lines to stdout for
grepping out of a SLURM log. No new package dependency -- the profile is folded
and rendered in the script.

Two things it does that matter for reading the output:

- **idle worker threads are dropped.** The sampler fires on every thread and this
  is a single-threaded search, so idle threads park in `poptask` and contributed
  exactly 50% of every profile, rescaling all real frames by the thread count.
  `idle_dropped` reports how many were discarded.
- **GC is kept** as a single collapsed frame rather than dropped with the other C
  frames, so allocation pressure shows up rather than being redistributed onto
  whatever Julia frame triggered the collection.

## What's next

The station-simple pricer is now **GC at 33% of its samples** -- the largest
single item in its profile, and no longer hidden behind anything else. Each label
allocates a route vector, a station-age `Dict`, a visited `BitSet` and two age
arrays; at 473k labels that is the whole story. Cutting it means changing how a
label is represented (parent-pointer routes, pooled age buffers), which is a
design change, not a mechanical one.

For the revisit pricer the profile is now flat -- no frame above 15%, with GC,
the bucket memmoves (`unsafe_copyto!` from `deleteat!`/`insert!`) and the
dominance predicate's own arithmetic in roughly equal parts. Further gains there
need fewer or shorter bucket scans, i.e. an algorithmic change, not a mechanical
one.
