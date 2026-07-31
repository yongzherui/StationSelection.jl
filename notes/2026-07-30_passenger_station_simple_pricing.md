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

### MEASURED (B) — 3-way, BitSet-`visited` label, exact vs subset (2026-07-31)

`visited` was moved from `Set{Int}` to `BitSet` and the hot-path mirror was slimmed
to ages-only (dominance now reads `visited`/`activated_reward_layers` straight off
the label). A `dominance_mode` toggle was added so one grid measures both rules on
the identical optimized label. `speedup` and `live_ratio` are vs the revisit-tolerant
pricer (higher = better).

| n  | s | speedup :exact | speedup :subset | exact_over_subset | subset_live/exact |
|----|---|----------------|-----------------|-------------------|-------------------|
| 15 | 1 | **1.61**       | 1.14            | 1.40              | 2.44              |
| 15 | 2 | 0.64           | 0.45            | 1.42              | 2.33              |
| 15 | 3 | **1.24**       | 0.65            | 1.91              | 2.30              |
| 20 | 1 | **1.84**       | 0.28            | 6.64              | 2.10              |
| 20 | 2 | **1.60**       | 0.44            | 3.67              | 2.50              |
| 20 | 3 | **3.51**       | 0.30            | 11.78             | 2.09              |

(n=10 omitted — sub-second, pure timing noise.) Absolute n=20 walls (s1/s2/s3):
revisit 19.2/12.1/41.7s, **ss_exact 10.4/7.6/11.9s**, ss_subset 69.0/27.8/140.0s.

**The subset detour is refuted; `:exact` is the default.**

1. **`:exact` (fine `(current, visited)` buckets) wins outright** — fastest of all
   three, beating the revisit-tolerant pricer 1.6-3.5x at n=20.
2. **`:subset` keeps ~2x fewer labels yet runs 1.4-11.8x slower than `:exact`.** Its
   coarse `current`-only buckets grow huge and the O(bucket) per-insertion scan
   (~85-90% of wall time) dominates; halving the labels does not pay for it. This is
   the decisive lesson: **bucket granularity, not domination power, sets wall time.**
3. **BitSet-`visited` + slim label helped `:exact` further** — its n=20 speedup rose
   from 1.69/1.51/2.84 (MEASURED A, `Set`-based) to 1.84/1.60/3.51 here, a further
   ~8-25%, from removing the per-label `visited_bits` rebuild and `activated` copy.
4. **`rc_gap` is unchanged** (identical between modes and vs A — same elementary
   universe), so the Zhuzhou restriction (up to 8.3%) stands and the feature stays
   off by default in CG.

**Takeaway.** Ship `:exact`. The elementary pricer, with fine BitSet-keyed buckets,
is now genuinely faster than the revisit-tolerant pricer at n=20 — but remains a
heuristic on instance families whose optimum wants a revisit, so it is opt-in.

## End-to-end CG: objective gap and the two-phase warm start (2026-07-31)

`scripts/diag_passenger_station_simple_vs_revisit_objective.jl` runs the FULL
passenger CG three ways on the same model+data (p=16, 3 scenarios, ms=5, seed 42):
`station_simple` (`use_station_simple=true`), `revisit` (exact), and `warm_start`
(the new `station_simple_warm_start=true` — elementary pricing until the elementary
universe is exhausted, then switch to the exact pricer and certify).

**How far station-simple alone takes us (objective gap = ss − revisit, minimisation):**

| n  | ss LP / MIP        | revisit LP / MIP   | LP gap  | MIP gap | ss wall | rev wall |
|----|--------------------|--------------------|---------|---------|---------|----------|
| 10 | 7434.91 / 7434.91  | 7434.91 / 7434.91  | 0.000%  | 0.000%  | 6.6s    | 1.8s     |
| 15 | 6254.00 / 6258.29  | 6247.27 / 6258.29  | 0.108%  | 0.000%  | 20.5s   | 23.4s    |
| 20 | 5545.00 / 5550.70  | 5521.84 / 5527.54  | 0.419%  | 0.419%  | 100.7s  | 515.4s   |

Elementary columns alone reach the exact MIP at n=10/15 and within **0.42%** at
n=20 — and at n=20 the elementary CG certifies in **99s vs 515s** for the full
revisit CG (5.2x), because the exhaustive certification pass over the full revisit
universe is what makes the exact CG expensive. (At n=10 station-simple is slower —
the revisit CG is already 1.8s, so the elementary phase is pure overhead.)

**Two-phase warm start — same certified optimum, faster at scale:**

| n  | warm_start wall | rev wall | ws_speedup | ws_matches_rev |
|----|-----------------|----------|------------|----------------|
| 10 | 2.4s            | 1.8s     | 0.76x      | true           |
| 15 | 31.3s           | 23.4s    | 0.75x      | true           |
| 20 | **197.9s**      | 515.4s   | **2.60x**  | true           |

Warm start certifies the IDENTICAL LP and MIP as pure revisit at every n
(`ws_matches_rev=true`; the correctness test in
`test/opt/test_passenger_free_assignment_seeding.jl` asserts this and that both
pricer phases run). At n=20 it is **2.6x faster**: Phase A reaches 0.42%-of-optimal
in ~100s, then Phase B closes the gap and certifies in ~98s — vs 515s from scratch,
because the warm elementary pool leaves the exact phase little to find. At small n
it is a mild loss (~0.75x): the elementary phase is overhead the cheap exact CG
does not need.

**Takeaway.** `station_simple_warm_start=true` is the way to use the elementary
pricer inside CG: it keeps the exact optimum and pays off (2.6x at n=20) precisely
where the exact CG is expensive. Default off; worth enabling as instances grow.
