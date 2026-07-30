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
2. **Stronger, subset-visited dominance** — buckets are keyed on `current` alone,
   and dominance adds the elementary resource `U_a ⊆ U_b`: a label that has visited
   a subset of another's stations has fewer forbidden futures, so it dominates
   whenever otherwise no worse. This prunes the "wandered" labels the
   revisit-tolerant pricer (which ignores `visited` when uncapped) would carry.
   (An earlier iteration keyed on the exact `(current, visited)` pair — tiny
   buckets, but labels with differing visited sets never compared, which measured
   letting the live population balloon 3-6x. See the measured section.)

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

## Dominance soundness (subset-visited)

Buckets are keyed on `current`. `a` dominates `b` iff:

- `U_a ⊆ U_b` (visited-subset). For an elementary route `visited` is the set of
  forbidden future stations, so a subset means every station `b` may still visit,
  `a` may visit too — every completion feasible for `b` is feasible from `a`.
- `time_a <= time_b`;
- the **compensated** reward-layer budget `rc_a + w(A_a \ A_b) <= rc_b` (the same
  test the revisit-tolerant pricer uses — see `_passenger_free_assignment_compensation`
  and the dominance docstring in `labels.jl`; a plain `A_a ⊆ A_b` subset test on
  *layers* is the special case `w(A_a \ A_b) = 0` and is unsound in general);
- `age_a(j) <= age_b(j)` for every live origin `j`, i.e. `a` holds every live clock
  `b` does at no larger age (sparse merge walk; an extra clock in `a` is fine).

`route_length` needs no separate condition: `U_a ⊆ U_b` implies
`route_length_a <= route_length_b`, so the `max_stops` resource is subsumed. Every
condition is necessary, so this never claims an unsound domination; and it is
strictly stronger than the exact-`(current, visited)` rule (which is the special
case `U_a = U_b`), so it prunes at least as many labels.

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

### MEASURED (A) — exact-visited dominance, SUPERSEDED (2026-07-30)

This is the first-cut dominance (bucket on `(current, visited)`), kept here as the
motivation for switching to subset-visited. Zhuzhou grid, n ∈ {10,15,20}, ms=5,
p=16, 3 scenarios. Both pricers run to genuine exhaustion on identical `pricing_data`
(`scripts/bench_passenger_free_assignment_labels.jl --cases 10:5,15:5,20:5`, one
SLURM array task per n). `speedup = rev_wall / ss_wall`,
`live_ratio = rev_maxlive / ss_maxlive`, `rc_gap` = relative worsening of the best
reduced cost (0 = exact match, positive = station-simple is restricted).

| n  | s | speedup | live_ratio | rc_gap  | rev max_live | ss max_live |
|----|---|---------|------------|---------|--------------|-------------|
| 10 | 1 | 9.11\*  | 0.31       | 0.0442  | 2,090        | 6,807       |
| 10 | 2 | 0.58    | 0.30       | 0.0201  | 1,129        | 3,756       |
| 10 | 3 | 0.50    | 0.33       | 0.0000  | 1,989        | 6,025       |
| 15 | 1 | 1.26    | 0.20       | 0.0210  | 13,269       | 66,429      |
| 15 | 2 | 0.59    | 0.20       | 0.0169  | 11,715       | 59,596      |
| 15 | 3 | 0.95    | 0.26       | 0.0834  | 19,095       | 73,079      |
| 20 | 1 | 1.69    | 0.17       | 0.0009  | 52,461       | 301,059     |
| 20 | 2 | 1.51    | 0.18       | 0.0000  | 41,761       | 237,402     |
| 20 | 3 | 2.84    | 0.23       | 0.0000  | 76,683       | 337,846     |

\* n=10 walls are sub-second, so that ratio is timing noise, not signal.

**Three findings, none matching the optimistic premise:**

1. **The exact-visited signature inflates the label population 3–6×** (`live_ratio`
   0.17–0.33, i.e. station-simple's `max_live` is 3–6× the revisit-tolerant
   pricer's). Keying buckets on `(current, visited)` destroys the cross-domination
   the revisit-tolerant pricer gets from keying on `current` alone and ignoring
   `visited` when uncapped — labels with different visited sets never compare, so
   far fewer are pruned. This is the opposite of the "tiny buckets shrink the live
   set" hypothesis in the header.

2. **Speed is inconsistent and only reliably positive at n=20** (1.5–2.8×). At
   n≤15 it is a wash-to-slower (0.5–1.3×). The n=20 win comes purely from cheaper
   per-insertion scans (buckets are tiny even though there are 3–6× more of them);
   it grows with n, so the tiny-bucket lever may eventually dominate, but not in the
   measured range.

3. **It is NOT exact on this family** — `rc_gap` is nonzero in 5 of 9 cases, up to
   **8.3%** (n=15/s3). The Zhuzhou optimum frequently wants a revisiting route, so
   elementary pricing genuinely misses the best column. This is the
   column-universe restriction, exactly as warned above, and it is why the feature
   is off by default.

**Why this motivated the switch.** The label-count blow-up is entirely a dominance
artifact: bucketing on exact `(current, visited)` prevents a lean route from
dominating a wandered one. The subset-visited rule (now the implementation) keys on
`current` and adds `U_a ⊆ U_b`, restoring that cross-domination. The `rc_gap`
column is a property of the *route universe*, not the dominance rule, so it is
unchanged by the switch — the Zhuzhou optimum still frequently wants a revisit, and
the feature stays off by default for that reason.

### MEASURED (B) — subset-visited dominance (the shipped rule)

Same grid, re-run after switching the dominance rule. `live_ratio` should climb
toward / above 1 (label count no longer inflated) and `speedup` improve, while
`rc_gap` stays as in (A) (same elementary universe).

<!-- MEASURED: fill from the subset-visited re-run:
     sbatch --array=0-2 tmp_ss_bench/sbatch_ss_grid_array.sh -->
```
(subset-visited bench re-run pending)
```
