# Station-simple (elementary-route) passenger free-assignment pricer

An opt-in elementary variant of the passenger free-assignment (PFA) label search
that strictly forbids station revisits, mirroring the aggregate pair-based
`pricing/station_simple.jl` for the passenger reward model. Lives in
`src/opt/optimize/aggregate_od_route/pricing/passenger/station_simple.jl`;
selected by `use_station_simple=true` on
`run_passenger_free_assignment_column_generation`.

## Why

The revisit-tolerant PFA pricer prices columns over routes that may visit a
station more than once. Its dominance scan is ~85-90% of wall time (see
`2026-07-30_passenger_pricing_label_search_optimizations.md`), and it must carry a
full `station_age` Dict because a revisit can re-open a pickup clock. Forbidding
revisits is expected to be faster on two levers, both aimed at that scan:

1. **Fewer extensions** — candidate generation drops already-visited nodes, so the
   branching factor shrinks as a route grows.
2. **Tiny dominance buckets** — buckets are keyed on the exact `(current, visited)`
   pair rather than on `current` alone. Two elementary labels with different
   visited sets have different forbidden futures and can never dominate each
   other, so they never share a bucket, and the visited comparison drops out of
   the per-pair dominance predicate entirely.

## What elementarity does and does not simplify

Only the *route universe* changes. The reward contract is unchanged: a visit to
origin `j` within `max_wait_time` opens a live clock; a later visit to `k`
certifies `(p, j, k)` when the clock survives the passenger-specific `ride_limit`;
and each passenger banks only its single best certified reward, tracked
incrementally via `activated_reward_layers`.

Crucially, the per-passenger *maximum* reward means elementarity does **not** let
us drop the layer/age bookkeeping the way the aggregate pair-based
`station_simple.jl` collapses `station_age` to `live_origin_age` (there a served
pair settles permanently on first visit). Reaching a strictly better dropoff later
still activates incremental layers, so a live clock stays useful even after it has
already certified something. What elementarity removes is clock *resets*: a
station is visited exactly once, so a clock only ages and is never reopened. The
label therefore keeps `activated_reward_layers` and the live-clock Dict, and adds
an authoritative `visited::Set{Int}` (no 64-station `UInt64` ceiling, unlike the
revisit-tolerant `visited_mask` budget bitset it replaces).

## Dominance soundness (exact-visited signature)

Buckets are keyed on `(current, visited)`, so any two labels compared for
dominance already share both. With identical visited sets the two labels have
exactly the same set of reachable futures, so `a` dominates `b` iff:

- `time_a <= time_b`;
- the **compensated** reward-layer budget `rc_a + w(A_a \ A_b) <= rc_b` (the same
  test the revisit-tolerant pricer uses — see `_passenger_free_assignment_compensation`
  and the dominance docstring in `labels.jl`; a plain `A_a ⊆ A_b` subset test is
  the special case `w(A_a \ A_b) = 0` and is unsound in general);
- `age_a(j) <= age_b(j)` for every live origin `j` (sparse merge walk).

Two resources present in the revisit-tolerant dominance are **dropped** here as
redundant, not weakened: `route_length` (equal within a bucket, since
`route_length == |visited|`) and the visited-subset resource (equality is
stronger than subset). This is a strict special case of the revisit-tolerant
compensated dominance, so it never claims an unsound domination.

## Correctness caveat — this restricts the column universe

Pricing over elementary routes only prices over a **subset** of the master's
column universe. Where the model's optimum genuinely wants a revisiting route,
this pricer is a *heuristic*: it can terminate CG with a weaker `lp_bound` or miss
improving columns. The aggregate pair-based `use_station_simple` did exactly this
on some instance families (grid family 70–76% worse objective under invalid
elementary cuts; see memory `project_station_simple_cut_validity`). It is
therefore opt-in, off by default, and its `lp_bound` must be validated against the
revisit-tolerant pricer before it is relied on.

A concrete revisit-beneficial instance (also a unit test): on two stations with
passenger p1 wanting 1→2 and p2 wanting 2→1, serving both needs `1→2→1`, which
revisits 1. The revisit-tolerant pricer collects both (reward 20); every
elementary route serves at most one (reward 10), so station-simple's best reduced
cost is strictly worse.

## Reuse

Shares `PassengerFreeAssignmentPricingData` (no new data struct) and the
`_passenger_free_assignment_travel`, `_certify_passenger_free_assignment_layers_at_node`,
`_passenger_free_assignment_age_is_useful`, `_has_useful_live_passenger_free_assignment_origin`,
`_passenger_free_assignment_compensation`, the shared
`_passenger_free_assignment_remaining_reward_bound` (its `label`/`label_bs`
annotations were loosened so both label types can call it), and the
`PassengerFreeAssignmentSearchIndex`/`PassengerFreeAssignmentBoundWorkspace`
scaffolding. Columns are emitted via the identical route-replay path
(`_passenger_free_assignment_column_from_route`), since replay is agnostic to how
the physical route was found and replays an elementary route unchanged — so the
station-simple driver differs from the revisit-tolerant driver only in which
enumeration it calls.

## Measurement

`scripts/bench_passenger_free_assignment_labels.jl` now runs both pricers on the
same `pricing_data` and emits, per (n, max_stops, scenario), a `COMPARE` line with
`speedup` (rev wall / ss wall), `live_ratio` (rev max_live / ss max_live), and
`rc_gap` (relative `best_rc` divergence — 0 when the optimum is elementary,
positive when station-simple is restricted). Expected signature: `speedup > 1` and
`live_ratio > 1` where the case is nontrivial, with `rc_gap` either ~0 (exact) or
positive (restriction visible).

<!-- MEASURED: fill in from a bench run once the cluster grid frees up:
     scripts/bench_passenger_free_assignment_labels.jl --cases 15:5,15:6,20:5 -->
```
(pending bench run)
```
