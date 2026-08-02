# Passenger free-assignment (PFA) pricing: the label-setting algorithm

Reference note for the pricer in
`src/opt/optimize/aggregate_od_route/pricing/passenger/`. Covers the subproblem
it solves, the label state, extension, every pruning rule, the dominance rule,
and the heuristics layered on top (two-stop seeding, station-simple warm start,
parallelism, harvest caps, reward coarsening).

Companion notes, not repeated here:

- `2026-07-29_passenger_free_assignment_pricing.md` — why reward layers encode a
  per-passenger maximum (the correctness argument for the reward encoding).
- `2026-07-30_passenger_pricing_label_search_optimizations.md` — the measurement
  log behind the data-structure choices.
- `2026-07-30_passenger_station_simple_pricing.md` — the elementary pricer.
- `2026-07-31_two_stop_seeding_and_bound_trajectory.md` — seeding measurements.
- `2026-07-31_pfa_equals_direct_enumeration_verified.md` — end-to-end check that
  CG over this pricer reproduces direct route enumeration.

Code map:

| file | contents |
| --- | --- |
| `types.jl` | label, pricing data, dominance-bucket containers |
| `data.jl` | reward-layer preprocessing, certification, age-usefulness test |
| `labels.jl` | initial labels, candidate nodes, extension, dominance |
| `search_data.jl` | per-search index + bound scratch buffers |
| `search.jl` | the frontier loop, route replay, top-level driver |
| `station_simple.jl` | elementary (no-revisit) variant of the same search |
| `column_generation.jl` | CG loop, seeding, warm start, parallel scenarios |
| `master.jl` | RMP, duals, candidate construction, two-stop seeds |

---

## 1. The subproblem

One pricing call handles **one scenario**. Input from the master:

- a passenger set `P_s`, a station set `N`, a travel matrix `travel(u,v)`;
- for each passenger `p`, a set of feasible assignments `(j, k)` with a
  passenger-specific **ride limit** `R_pjk` and a **reward**

  ```
  rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - walk_weight * w_pjk
  ```

  built in `passenger_free_assignment_pricing_candidates` (`master.jl`). Only
  `rho_pjk > 0` candidates are represented — a non-positive one can never help a
  route's reduced cost.

A **column** is a physical station route `r = (n_1, ..., n_m)` starting at
`t = 0`, unlimited capacity, synchronized start. Its rules:

- visiting `j` at elapsed time `<= W` (`max_wait_time`) **opens a pickup clock**
  at `j` (age 0); visiting past `W` opens nothing;
- arriving at `k` **certifies** `(p, j, k)` if `j`'s clock age on arrival is
  `<= R_pjk`;
- a passenger banks only its **single best** certified reward, not the sum.

The pricing objective is

```
rc(r) = beta * (tau(r) + repositioning_time) - sum_p max{ rho_pjk : (p,j,k) certified by r }
```

with `beta = route_regularization_weight` and `tau(r)` the route's travel time.
Pricing looks for `rc(r) < -tol`.

**Requirement: the travel matrix must be metric.** Age pruning, the remaining-
reward bound, and the station-budget argument all use
`travel(a,b) <= travel(a,x) + travel(x,b)`. A non-metric matrix does not merely
loosen bounds, it makes them wrong — and surfaces as a misleading reward-layer
assertion failure in route replay.

### Reward layers (the per-passenger max, as a bitset)

Per passenger, sort the distinct positive rewards `0 = v_0 < v_1 < ... < v_m`
and create **incremental layers** `delta_h = v_h - v_{h-1}`, each a global bit.
Certifying an assignment worth `v_q` activates the **prefix**
`{(p,1),...,(p,q)}` (`_build_passenger_reward_layers`, `data.jl`).

Because per-passenger masks are nested prefixes:

- OR-ing a better assignment's mask in adds exactly the layers between the old
  and the new level — the running weight equals the new max, never a sum;
- OR-ing a worse one in adds nothing (subset of what's active);
- several origins certifying the same passenger at the same destination can be
  unioned in one batch before diffing against `activated`
  (`_certify_passenger_free_assignment_layers_at_node`), with no double credit.

So `sum of activated layer weights == sum_p (best certified reward for p)`,
maintained by bitset union alone. This is what lets labels compare and dominate
on reward without carrying a dense per-passenger reward vector.

---

## 2. Label state

`PassengerFreeAssignmentPricingLabel` (`types.jl`):

| field | meaning |
| --- | --- |
| `current` | station the vehicle is at |
| `route` | the physical node sequence so far |
| `time` | elapsed time since `t = 0` (drives the `W` pickup cutoff) |
| `station_age` | `station => age`, the **live pickup clocks** only |
| `activated_reward_layers` | `BitSet` of activated layers = the per-passenger running maxima |
| `tau` | accumulated travel time |
| `reduced_cost` | `beta * (tau + repositioning_time) - w(activated)` |
| `route_length` | `length(route)` |
| `visited_mask` | `UInt64` of distinct stations, only meaningful under the station-budget cap |

Two things deliberately **not** in the state:

- **no onboard-passenger / capacity resource.** Capacity is unbounded and
  passengers do not interact, so certifying `p` never consumes anything another
  passenger needs. There is nothing to branch on beyond the physical route.
- **no concrete `(j, k)` per passenger.** Only reward *levels* are tracked.
  The actual pairs are recovered by route replay, and only for finished,
  accepted routes (§6) — not for every intermediate label.

`PassengerFreeAssignmentLabelBitsets` mirrors the pruning-relevant part
(`activated_bits` plus `station_age` as **parallel sorted arrays**
`age_idx`/`age_val`) so the dominance test is an `O(#live)` merge walk instead of
an `O(n_nodes)` dense scan or a Dict traversal. Age pruning keeps `#live` tiny,
which is what makes the sparse form pay.

**Initial labels** (`initial_passenger_free_assignment_pricing_labels`): one per
station that is an origin or destination of some positive-reward opportunity,
with `time = 0`, its own clock open, `rc = beta * repositioning_time`.

---

## 3. Extension

`extend_passenger_free_assignment_pricing_label(label, next_node, data)` produces
exactly one child (there is no drop-off subset to enumerate):

1. `travel_time = travel(current, next_node)`; `time`, `tau`, `route` advance.
2. **Certify** at `next_node`: scan `assignments_by_destination[next_node]`
   only, union the layer masks of every opportunity whose origin clock survives
   `origin_age + travel_time <= R_pjk`, diff once against `activated`, and take
   the diff's weight as the reward.
3. **Age, reset, prune in one pass**:
   - every other live clock ages by `travel_time`, and is kept only if it is
     still *useful* (§4.1);
   - if `arrival_time <= W`, `next_node` gets a **fresh clock at age 0** (still
     subject to the usefulness test);
   - if past `W`, `next_node`'s existing clock (if any) just ages like the rest —
     the visit opens nothing.
4. `rc_child = rc_parent + beta * travel_time - reward`.

Steps 2–3 were deliberately fused into a single traversal; the earlier
"build aged dict, then build pruned dict" form allocated twice per extension on
the hottest path.

---

## 4. Pruning

Six independent mechanisms, in the order they bite.

### 4.1 Live-clock (age) pruning — exact

`_passenger_free_assignment_age_is_useful(station, age, activated, data, current)`:
a clock is kept only if **some** opportunity `(p, station, k)` can both

- still activate a layer not already active (`_has_inactive_layer`), and
- still be reached in time: `age + travel(current, k) <= R_pjk`.

`travel(current, k)` is the *minimum* additional time to reach `k` (metricity —
detouring only adds), so this is exact, not heuristic. Dropping a clock costs
nothing because it represents a *potential* certification, not an irrevocable
pickup commitment — precisely because there is no capacity resource.

This is the single most important state-shrinker: it keeps `#live` at a handful,
which in turn is what makes the sparse age representation and the `O(#live)`
dominance age test viable.

### 4.2 Candidate-node filtering

`_passenger_free_assignment_candidate_next_nodes` builds the successor set
instead of extending to all of `N`:

- **hard stop:** if `time > W` *and* no live clock is useful
  (`_has_useful_live_passenger_free_assignment_origin`), return `[]` — the label
  can never earn anything again.
- **useful origins** (only while `time <= W`): `j` is a candidate if
  `origin_layer_mask[j]` has an inactive layer and `j` is reachable before the
  cutoff. `origin_layer_mask` is the union of everything reachable from `j`, so
  this is an *optimistic* pre-filter; real feasibility is re-checked once that
  clock actually goes live.
- **useful destinations:** driven **from the label's live clocks**, not from
  `assignments_by_destination`. Only a live origin can make a destination useful,
  and `#live` is small, whereas iterating destination groups touches all
  `~P * n^2` opportunities regardless of label state.
- **`max_visits_per_node`**: drop nodes already visited that many times.
- **station budget** (§4.5), when enabled.

### 4.3 Reduced-cost bound pruning at pop — exact, and the search order

`_passenger_free_assignment_remaining_reward_bound` returns an admissible bound
on the additional **net** gain (reward minus the travel needed to collect it)
still available, so

```
priority = rc(label) - bound(label)
```

is a lower bound on the reduced cost of every completion. The frontier is a
`PriorityQueue` on that value, i.e. the search is **best-first over completion
lower bounds**. A popped label with `priority >= -tol` cannot complete into an
improving column and is not extended.

The bound has two sources of future reward, handled differently:

- **live origins** — age-dependent, so still tested per opportunity, but only
  over opportunities of origins that are *actually live*. Booked against the
  destination `k` that would certify them.
- **refreshable origins** — while `time <= W`, any origin reachable before the
  cutoff could open a fresh clock. This condition depends only on the origin, so
  the whole `origin_union_mask[j]` is OR'd in at once. Booked against `j`.

Then the **travel discount**: collecting the layers booked against node set `S`
costs at least `max_{x in S} travel(current, x)`. Walking nodes in increasing
distance (`nodes_by_travel`, precomputed once per search) and taking the running
max of `w(prefix) - beta * travel(current, x)` maximises over all `S` in one
pass, with `max(0, ...)` covering "stop here". `w(prefix)` accumulates the
**union** of masks minus `activated`, so a passenger certifiable at several
destinations is counted once.

Two measurement facts that keep being rediscovered the hard way:

- This bound was rewritten from `O(|opportunities|)` per label to
  `O(#live origins' opportunities + n_nodes)`. That rewrite mattered a lot:
  `|opportunities| ~ P * n^2`, so the old form drove measured time to
  `~n^5.5-7.6` while label counts grew only `~n^3.4`.
- Since the rewrite the bound is **~0.6% of wall time**. Tightening it further
  for *speed* is wasted effort. The travel discount measured as a no-op at
  cold-start duals (rewards ~5000 vs one hop ~4000, so discounting one hop never
  flips the test); it is retained because it is exact and strictly tighter, and
  should bite under converged duals where most `rho` are near zero.

### 4.4 `max_stops`

`route_length >= max_stops` → no extension. Resolved through
`_resolve_aggregate_od_route_pricing_max_stops` against `max_visits_per_node`
and `|N|`. Cost blows up super-linearly in this cap (measured cap 3→7 at n=20),
so it is the main dial for tractability, at the price of restricting the column
universe.

### 4.5 Station budget `l` — valid, and **off by default**

`_passenger_free_assignment_station_budget_allows`: cap distinct visited
stations at `l = max_distinct_stations` (revisits are free). Validity:

- only assignment-carrying stations need `y_j = 1`, and `theta_r >= 1` forces
  those, so `|A_r| <= sum(y) = l` — wider columns are unusable by any integer
  solution, so dropping them leaves the IP optimum untouched and cannot loosen
  the LP bound;
- capping *visited* rather than *carrying* stations is stronger but still
  lossless: a visited station carrying nothing can be deleted from the route, and
  by the triangle inequality that lowers `tau` and makes every later arrival
  earlier, so reward cannot fall and `rc` cannot rise.

**Measured and left off.** Pricing is neutral-to-1.7x *slower* (the required
`U_a ⊆ U_b` dominance companion drops the domination rate 80.7% → 74.4%, so
buckets grow), and `lp_bound` was identical to ten decimals at n=10 and n=15.
Two structural reasons it binds weakly: ride limits + the pickup window already
hold good routes to 6–8 distinct stations at n=15, and the per-passenger *max*
reward structure means a route cannot buy extra dual credit by touching more
stations (unlike the aggregate pricer, whose summed per-pair reward is what
produced the hub-route LP–IP gap this cap was designed to close). Also limited
to `|N| <= 64` (`visited_mask` is a `UInt64`), with a warning above that.

### 4.6 Post-`W` completion oracle — off by default, measurement tool

`post_w_completion.jl`. Once `time >= W`, no visit can open or reset a clock, so
with metric travel the optimal suffix is an **elementary destination path** over
fixed origins and can be enumerated exactly. When
`use_post_w_completion_bound=true`, an exhausted completion replaces the
heuristic priority with the *exact* completion value. Currently an oracle for
assessing cheaper memoized bounds, not a production hot-loop bound.

---

## 5. Dominance

The hot loop: **~85–90% of wall time**, and the main thing keeping an unbounded
search finite. Anything that does not shrink the live-label population or make
the bucket walk cheaper has consistently measured as a no-op here.

### The rule

`a` dominates `b` (`_dominates_passenger_free_assignment_label`) iff all of:

1. `a.current == b.current` (the bucket signature);
2. `a.route_length <= b.route_length`, **only when `max_stops` is bounded**;
3. `U_a ⊆ U_b` (`visited_mask`), **only when the station budget is capped** —
   budget is a consumed resource, so a label that spent more cannot stand in for
   one that spent less;
4. `a.time <= b.time`;
5. **compensated reward test**: `rc_a + w(A_a \ A_b) <= rc_b`;
6. `dom(age_b) ⊆ dom(age_a)` and `age_a(j) <= age_b(j)` on `dom(age_b)` — i.e.
   `a`'s clocks are a superset and no older. Implemented as a single merge walk
   over the two sorted `age_idx` arrays; exactly equivalent to the dense
   "missing = Inf" rule at `O(#live)` instead of `O(n_nodes)`.

### Why compensation, and why that direction

For a suffix `sigma` feasible from `b`, conditions 4 and 6 give
`reach_b ⊆ reach_a`, and since
`reach_b \ A_b ⊆ (reach_a \ A_a) ∪ (A_a \ A_b)`:

```
reward_b(sigma) <= reward_a(sigma) + w(A_a \ A_b)
```

Both completions pay the same travel, so `a` finishes no worse whenever
`rc_a + w(A_a \ A_b) <= rc_b`.

The direction matters: `a` pays for the layers **`a`** holds and `b` lacks —
reward `b` can still bank off the shared suffix while `a`, having already banked
it, gets nothing more. Charging `w(A_b \ A_a)` instead is **unsound**: it would
let a label that already banked a large layer dominate one that has not, even
though the shared suffix can pay that layer out to the second label and overtake
the first.

The old rule `A_a ⊆ A_b` is the special case `w(A_a \ A_b) = 0`, so compensation
only ever *adds* dominations, and since compensation is non-negative it never
weakens the `rc_a <= rc_b` precondition — which is what keeps the reduced-cost-
ordered bucket scan valid.

**Measured: the single biggest win in this pricer, 2.5–3.9x.** `max_live` roughly
halves (58,260 → 21,917 at n=15/ms=6; 117,950 → 52,461 at n=20), and because the
scan is linear in bucket size *per insertion*, halving the population quarters
the work; the speedup grows with instance size.

Cost control is part of the win: `_passenger_free_assignment_compensation` tries
the word-wise `issubset` first (the most common case, and exactly the old rule)
and bails the element scan as soon as the running weight exceeds the budget.
Removing either guard gives the speedup back.

**Toggle `compensated_dominance=false`** restores the plain subset rule. It
exists because compensation trades **column diversity** for speed: ~50% fewer
distinct columns harvested per search. Which side wins for CG is an end-to-end
question, not a pricing-speed one.

### Bucket mechanics

`_add_passenger_free_assignment_label_to_bucket!`. Buckets are keyed on
`current` and held as a **`Vector` sorted by `(rc, time, route_length, id)`**
with the label and its bitsets **inlined in the entry**.

Since domination in either direction requires `rc_dominator <= rc_dominated`,
the walk splits at the new label's `rc`: below it, only an incumbent can dominate
the newcomer (and finding one ends the walk); above it, only the newcomer can
dominate incumbents. Dominated entries are collected as ascending indices and
removed in one `deleteat!`, so nothing mutates during the scan.

Measured layout wins (both bit-identical in labels and `max_live` — pure data
layout):

| change | result |
| --- | --- |
| `Vector` bucket instead of `SortedDict` | **1.3–1.5x** — the scan walks contiguous memory; a tree was ~76ns/entry of pointer chasing |
| entry inlines label + bitsets | **1.1–1.15x** — removes two hash probes per entry |

The `Vector` trade is `O(bucket)` memmoves on insert/evict vs `O(log bucket)`
tree surgery, against one insertion per *full-bucket scan* — a memmove of a few
thousand small structs runs at memory bandwidth, so it is not close.

Evicted labels are deleted from `live_labels`; their queue entries become
**stale pops** (counted in stats) and are skipped.

---

## 6. From labels to columns

The search returns the best label per **layer signature**
(`activated_reward_layers`) — a cheap bookkeeping key, deliberately *not* the
column identity.

`_replay_passenger_free_assignment_route` then replays each finished route from
`t = 0`, independently of the label's (dominance-pruned) clock history, and takes
each passenger's argmax certified assignment, ties broken lexicographically on
`(origin, destination)` for determinism. **This is the only place concrete
`(j, k)` pairs are materialized.**

`_passenger_free_assignment_column_from_route` recomputes `rc` directly from the
selected assignments and **asserts** it matches the label's `rc` to 1e-6 — on
every finished route, not just in tests. Replay is cheap next to the search that
produced the route.

Acceptance in `passenger_free_assignment_pricing_by_label_setting`:

- `rc < -tol`, at least one assignment;
- **pool novelty on the real assignment signature**: accept only if `tau` beats
  the pool's best `tau` for that signature. Two different physical routes that
  reach the same running per-passenger maxima are still different columns here,
  which is why the layer signature cannot be used for this.
- keep the best `(rc, tau)` per signature; sort by `(rc, tau, route)` and return
  at most `max_new_columns`.

One more guard outside the pricer: `verify_reduced_costs` (default on)
re-derives each accepted column's reduced cost from the master's own duals and
objective coefficient, catching any drift between pricer and master (wrong dual
sign, missing linking row, one-sided walk weight) at the moment it happens rather
than as a silently wrong LP bound.

---

## 7. Heuristics and accelerations in the CG loop

All in `run_passenger_free_assignment_column_generation` (`column_generation.jl`).

### 7.1 Two-stop seeding — **default ON**

`passenger_free_assignment_two_stop_seed_columns` (`master.jl`) enumerates every
two-stop route `[j, k]`, one column per `(scenario, j, k)`, before the first LP.

Why: from an empty pool the `v[p]` slack makes the RMP feasible, but the first
several iterations are not improving routing cost at all — they are hunting for
*any* covering set while the LP objective is dominated by
`unserved_penalty * sum_p v[p]`. Measured on the 2026-07-30 grid: the iteration-1
LP is **39x–131x** the final value, essentially all big-M draining out; genuine
CG improvement from the first covering iterate is only 1.8%–50%.

Two-stop routes are exactly what the big-M was standing in for
(`_default_unserved_penalty`'s own derivation: serving one passenger never needs
more than a direct `[j, k]`). Coverage claim: `R_pjk = detour_factor *
travel(j,k)` and replaying `[j,k]` gives the pickup an age of exactly
`travel(j,k)` at `k`, so with `detour_factor >= 1` every feasible `(p,j,k)` is
certified by its own two-stop route. The age test is applied explicitly anyway,
so a `detour_factor < 1` config drops uncertifiable assignments rather than
building an invalid column.

One column per `(s,j,k)`, not per `(p,j,k)`: the same route carries every
passenger of that scenario whose pair it certifies. Seed count is bounded by
`n_scenarios * n * (n-1)`.

Measured end-to-end: proven optimum-preserving; **2.0x at n30_p8** and **+6.3%
better incumbent at n30_p16**, but roughly a wash at `n <= 20` — the benefit is
size-dependent.

### 7.2 Station-simple warm start — **default ON**

`station_simple_warm_start=true` runs a **two-phase** scheme:

1. price with the **elementary** (no-revisit) search in `station_simple.jl` until
   it *proves* no improving elementary column remains — an exhausted, empty
   certification pass (or a pass that only re-finds pooled columns);
2. then flip permanently to the exact revisit-tolerant pricer to close the
   remaining gap and certify.

The switch is sound as a certificate path: it happens precisely because nothing
was added, so the LP is unchanged and the next round re-prices the *same* duals
with the exact pricer, which then runs to its own exhaustion. The final
`lp_bound` remains a genuine certificate.

Why the elementary phase is fast: (a) candidate generation drops visited nodes,
so the branching factor shrinks as routes grow; (b) buckets are keyed on the
exact `(current, visited)` pair, so each bucket is tiny — and since the scan is
`O(bucket)` per insertion, bucket **granularity** is the whole game. 1.6–3.5x
faster than the revisit-tolerant pricer at n=20.

(A `:subset` bucket mode — key on `current`, add `U_a ⊆ U_b` — is strictly
stronger dominance and keeps ~2x fewer live labels, yet is **1.4–6.6x slower**
because coarse buckets grow to tens of thousands of entries. Research only.)

Measured: **2.6x faster at n=20 with the same certified optimum.** Note the
elementarity caveat — `use_station_simple=true` (elementary for the *whole* run,
default off) is a heuristic that restricts the column universe and can weaken
`lp_bound`; the aggregate pair-based station-simple pricer produced 70–76% worse
objectives on the grid family through invalid cuts. The warm start avoids that by
always finishing with the exact pricer.

### 7.3 Parallel scenarios — **default ON**, exact and deterministic

The subproblem separates exactly by scenario: a column belongs to one scenario
and its reduced cost touches only that scenario's duals; all cross-scenario
coupling (`theta <= y_j`, `sum y = L`) lives in the master. So
`_price_passenger_scenarios` runs the per-scenario searches under
`Threads.@threads`.

Safety and determinism:

- `_price_one_passenger_scenario` only *reads* the master and dual dicts and
  allocates everything else itself; pricing touches no Gurobi at all (pure Julia
  label setting), so there is no solver thread-safety question;
- the pool is grouped by scenario **once** before the parallel region (was
  `O(S * pool)` per iteration — ~160k filter ops at 15 scenarios with a 10.7k
  pool);
- results go into preallocated per-scenario slots, are concatenated in sorted
  scenario order, and **only then** get sequential column ids — so ids do not
  depend on completion order (they would with a shared counter);
- `verify_reduced_costs` runs *outside* the parallel region so a mismatch raises
  a clean deterministic error rather than a `TaskFailedException` whose reported
  column depends on which thread failed first.

No-op with one thread or one scenario.

### 7.4 Harvest caps and early return

Two phases per round:

- **early-return pricing**: `n_candidates` / `max_new_columns` (default 20). The
  search's `stop_if` hook accepts finished routes *as they are popped* and
  aborts the search once `n_candidates` distinct accepted signatures exist. Loop
  until an iteration adds nothing.
- **exhaustive certification**: same code with the caps effectively removed
  (`typemax(Int) ÷ 2`) and its own `certification_time_limit_sec`. Only an
  `exhausted` *and* empty result is an optimality certificate; a timeout is not,
  and is reported as such.

Known open behaviour (see `project_pfa_cg_throughput_open_questions.md`): cost is
per-iteration, not iteration count; the bound is a step function that flatlines
for 40–64% of the budget; harvested routes are short (median 4 stops) even with
`max_stops` unbounded.

### 7.5 Reward coarsening — opt-in relaxation harvester (default 0 = off)

`coarsen_passenger_assignment_rewards(candidates, levels)` rounds each reward
**up** to one of at most `levels` retained values per passenger (max always
retained), collapsing reward layers. Since transformed rewards are never smaller,
`relaxed_rc <= exact_rc` route by route, so the relaxed search is a valid
*relaxation* — and a fast one, since layer count drives label count.

Rules that make it safe:

- every candidate route is **replayed against exact pricing data** before being
  admitted, and `relaxed_rc <= exact_rc` is asserted;
- the relaxed search runs against an **empty pool** and pool novelty is
  re-applied on exact signatures afterwards — a relaxed signature can differ from
  the exact one (coarsening changes replay's argmax tie-break), so a spurious
  match would silently discard a genuinely new column;
- `exhausted` is only propagated when the *relaxed* search itself came back
  empty (then no route has `relaxed_rc < -tol`, hence none has
  `exact_rc < -tol`). If it returned routes that all failed exact replay, nothing
  is proven — the returned set is dominance-pruned under relaxed rewards. That is
  the normal case at convergence (16/16 certifiable scenarios at n=20 returned a
  non-empty exhausted relaxed set);
- the CG driver pins the certification pass to level 0 regardless, and refuses to
  combine coarsening with station-simple modes.

Measured: **2.29x faster pricing at 0.94% shortfall** — a harvester, not a
certificate. `levels = 2` is the best quality/speed compromise measured. The
census confirmed `m_p` is 5–17x collapsible, but *clocks*, not reward layers,
drive label count, which caps the upside.

### 7.6 Pricing-aware dual selection — opt-in (`dual_selector`, default off)

`dual_selection.jl`. Each iteration, re-optimize the duals **over the RMP-optimal
face** before pricing:

```
min  mu * sum |rho_pjk - rho_bar_pjk|  +  eps * sum max(0, rho_pjk)
```

- the stabilization term damps bang-bang dual oscillation;
- the positive-reward term shrinks how many `(p,j,k)` look attractive — which
  matters *directly* here, since pricing cost scales with the number of
  positive-`rho` opportunities (`~P * n^2`), so it is a smaller label search, not
  merely fewer iterations.

Exactness: any RMP-optimal dual that prices out certifies full-LP optimality, so
substituting one cannot invalidate the certificate. Unlike naive dual smoothing
(`pi = a*pi_prev + (1-a)*pi_new`), which lands *off* the optimal face and needs
mis-pricing safeguards. The selector certifies nothing on its own.

### 7.7 Things measured and rejected

| idea | verdict |
| --- | --- |
| MCF relaxed pricer (`mcf_relaxation.jl`) | 0/5 certification rate, **200x slower** than the label search; knobs measured to go the wrong way |
| treap dominance index | removed (commit `95eba58`) |
| station-budget cap at `l` | slower, LP bound unmoved (§4.5) |
| reuse popped priority instead of recomputing | no effect — the bound is 0.6% of runtime |
| travel-discounted reward bound | no effect at cold-start duals; kept for being exact |
| compatibility-component decomposition | not built — the reward graph is one component holding 100% of opportunities |
| station-subset B&B pricer | bound-limited (ub ~1.9–2.2x optimum → ~zero pruning, ~1M nodes); loses to the direct pricer at every `max_stops` cap |

---

## 8. Invariants worth protecting

1. **Metric travel matrix.** §1.
2. **Replay `rc` == label `rc`** to 1e-6, asserted on every finished route.
3. **Pricer `rc` == master dual-implied `rc`** (`verify_reduced_costs`).
4. **Dominance compensation direction** is `w(A_a \ A_b)`, charged to `a`.
   Reversing it is unsound.
5. **`exhausted` means "no improving column exists."** It is what the driver
   turns into `:optimality_proven`; any relaxation that cannot support that claim
   must not propagate it (§7.5).
6. **Benchmark `best_rc` must stay bit-identical** across exact optimizations
   (`scripts/bench_passenger_free_assignment_labels.jl`). The station-budget cap
   is the one deliberate exception, since it restricts the column set on purpose.
