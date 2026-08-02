# A last-contributing-pickup split for PFA pricing (design + gating census)

Proposal: decompose every priced route at its **last contributing pickup** into a
forward pickup phase and a backward delivery suffix, solve the suffix with a
shared backward DP instead of re-exploring it under every forward label, and join.

Status: **CLOSED, not pursued** (2026-08-01, job 19454069). The scheme is sound and
fully constructible -- see "Backward labels are constructible" below -- but it
optimizes a dimension that is not binding in this pricer. Verdict and the five
reasons are at the end of this note.

Nothing is wired into production pricing; no join machinery was written. The only
production change is an optional, default-off `label_observer` hook in
`_enumerate_passenger_free_assignment_pricing_labels`, kept because it is the
reusable instrument for the deferred clock-quantization axis.

Background: `2026-08-01_pfa_label_setting_algorithm_reference.md` (how the pricer
works today), `2026-07-31_pfa_state_space_relaxation_design.md` (where the state
actually is).

## Why not split at `W`

`post_w_completion.jl` already establishes the useful lemma:

> Once `label.time >= max_wait_time`, no station visit can open or reset a pickup
> clock. With metric travel times [...] an optimal suffix does not revisit a
> suffix station. The residual problem can therefore be enumerated as an
> elementary destination path.

That makes `t > W` a tempting split. It is the wrong one, for two reasons.

1. **The post-`W` suffix is a subset of the true delivery suffix.** A route can
   finish picking up long before `W` and spend the rest of its life delivering.
   Every stop after `W` is after the last clock opening, but not conversely.
2. **If `W` is generous relative to route duration, the split never fires.** A
   4-stop route `j1 -> j2 -> k1 -> k2` completing inside the pickup window has no
   post-`W` labels at all, yet still has a clean 2-stop delivery suffix. Given
   that routes here are short (median 4 stops; 6-8 distinct stations at n=15),
   this is the expected regime, not a corner case.

Splitting at the last contributing pickup captures a strictly larger suffix and
does not depend on `W` binding. Q1/Q3 of the census measure exactly this gap.

## What the split costs: it becomes elective, not detectable

`time > W` is a property of a label -- free to test. "This is my last pickup" is
not; it is a **commitment the search elects to make** at any label. So the scheme
is not "labels cross a threshold" but "at any forward label, declare the pickup
phase closed and hand the clock state to the backward DP."

### Exactness

Enumerate pairs (forward label `f`, delivery suffix `sigma`) and value `f (+) sigma`
by walking `sigma` while certifying **only from clocks already live at `f`** --
ignoring any clock that `sigma`'s own visits would open.

- **No spurious columns.** Ignoring certifications can only lower reward, so the
  computed `rc` is `>=` the physical route's true `rc`. Nothing can beat the
  optimum by being mis-valued upward.
- **The optimum is found exactly.** For the optimal route `r*`, split at the last
  stop whose opened clock still certifies an assignment `r*` actually banks. By
  construction no ignored clock contributed, so the computed value equals the
  true value.
- Over-enumeration (the same route reachable at several split points) costs time,
  not correctness.

### The suffix is elementary

The `post_w_completion.jl` argument transfers verbatim once the origins are
frozen: a second visit to a suffix station is later, so certifies nothing more
from fixed clocks, and deleting the intervening cycle is no more expensive and
reaches every later station no later. Requires the metric travel matrix the
pricer already requires. So the backward search is over **elementary** paths even
though the parent pricer is revisit-tolerant.

## Backward labels are constructible (an earlier draft of this note said otherwise)

An earlier version claimed suffix reward is a function of the forward clock vector
and therefore that reward-carrying backward labels cannot be shared. That is
wrong, and it made the general construction look impossible when it is not.

**Cross-coupling is one-directional at any split point**, because time flows
forward: a clock opened in the suffix can only ever be certified by a destination
in the suffix. So suffix reward splits cleanly:

- **self-contained** -- suffix-origin x suffix-destination pairs. Forward-independent.
  An ordinary layer bitset `A_b`.
- **cross** -- forward clocks x suffix destinations. Needs only the suffix's
  *geometry*: arrival offsets `{(k, delta_k)}` from the split.

A backward label is therefore `(start node, A_b, offset profile, travel_b)`, all
forward-independent, and the join is

```
reward = w( A_f  u  A_b  u  Cross(clocks_f, profile_b) )
```

Because layer masks are nested per-passenger prefixes, that union *is* the
per-passenger maximum -- no double count. Backward dominance: `A_a superseteq A_b`,
offsets pointwise `<=` on a superset of destinations, `travel_a <= travel_b`.

Nothing here is blocked. The design fails for reasons of payoff, not feasibility;
see the verdict below.

### Proposed resolution: shard by live-clock support

> **This is the variant that was measured, and it is the one Q2 kills.** It is not
> the only possible construction -- the forward-independent backward label above
> needs no sharding at all -- but sharding was the route to a *strong* backward
> dominance rule, and its degeneracy is what first exposed the deeper problem the
> verdict describes. Read it as the tested proposal, not as live guidance.

Key the backward DP on `(start station, support(station_age))` -- the *set* of
live origins, not their ages. Within a shard:

- dominance only has to consider destinations relevant to that support, which is
  far stronger than the global geometric rule;
- the DP is shared by every forward label with the same support but different
  ages, which is the amortization the design exists for;
- ages enter only at join time, evaluated against the offset profile.

This is plausible because the pricer prunes clocks hard
(`_passenger_free_assignment_age_is_useful`), so `#live` is small. Whether the
number of *distinct supports* is correspondingly small is an empirical question
-- and it is the one that decides the design (Q2).

Post-split, the clock vector is frozen: all clocks age uniformly, so each origin
`j` has a fixed absolute pickup time and certifying `(p,j,k)` becomes "reach `k`
before `pickup_time_j + R_pjk`". The suffix problem is therefore a
**prize-collecting elementary path with absolute deadlines**, with per-passenger
max prizes.

## What the amortization buys, if it holds -- RESOLVED: it does not

Today `use_post_w_completion_bound` re-solves the completion **per past-cutoff
label**, from scratch, inside `label_priority` (`search.jl`). A sharded backward
DP computes the same exact oracle once per `(station, support)` instead of once
per label. The win is exactly the mean labels-per-shard ratio.

Measured: **2.51-3.04**. Against a bar of 10, and against the cost of a join plus
a second label type, that is not a win.

## Risks stated in advance, and how they landed

- **Depth.** Bidirectional labeling pays in proportion to route depth; these routes
  are short. -> **Materialized, and it is the decisive one.** Mean route length
  4.57-5.03, flat in `max_stops`. See verdict reason 1.
- **The label explosion is not depth-driven.** Label count is driven by
  `activated_reward_layers` and the clocks -- 43k -> 1.38M across a CG run -- and a
  split on the time axis does not attack either. -> **Materialized.** See verdict
  reason 2. (The parenthetical this bullet originally carried, that the suffix
  "escapes the layer state entirely", is wrong: the suffix has its own
  self-contained layer set `A_b`. What the suffix escapes is *clock creation*,
  which is the real asymmetry and the subject of verdict reason 3.)
- **Join cost** `|F_a| x |B_a|` with an `O(#live)` evaluation per pair. -> Never
  reached; no join was built.
- **Shard degeneracy.** If supports are near-unique per label the DP is rebuilt per
  label. -> **Materialized** (median 2 labels per shard), though the verdict treats
  this as a symptom rather than the root cause.

## The gating census

`scripts/diag_passenger_split_census.jl` (+ `sbatch_diag_passenger_split_census.sh`),
on a seeded-RMP dual snapshot, swept over **n in {15, 20, 30} x max_stops in
{7, 10, uncapped}**.

Sweeping `max_stops` is not incidental. The case for this split rests entirely on
route depth, so censusing only at `max_stops = 7` -- a cap that forces short routes
-- would answer the question for the wrong regime and could kill the idea on
evidence from a setting it was never aimed at. Deep search is also where the
current pricer hurts most (cost blows up super-linearly in `max_stops`), i.e. the
regime a bidirectional method exists to rescue.

**Prior that cuts the other way, and must be checked against Q1:** the
`max_stops` convergence sweep found the pricing optimum *saturates* from cap 4
onward at n=20 (4558 at cap 3, 7898 at cap 4+). If routes are intrinsically short
because ride limits and the pickup window bind -- not because the cap binds --
then raising `max_stops` lengthens the *search* without lengthening the *routes*,
and the depth argument for bidirectionality does not recover. Q1 measured across
the `max_stops` sweep settles this directly: if `mean_route_length` and
`mean_suffix_pickup_frac` are flat in `max_stops`, depth is not the lever.

| | question | kill condition |
| --- | --- | --- |
| **Q1** | share of route stops after the last contributing pickup, vs after `W` | suffix is a small share of the route -> nothing to amortize |
| **Q2** | mean/median labels per `(current, support)` shard | `~1`, or a high singleton fraction -> DP degenerates to per-label, **design dies** |
| **Q3** | fraction of labels past `W` (control) | near zero confirms the `W` split was never viable |

Q2 is the make-or-break number. Bars set in advance, in the style of the
state-space note:

- **Q2 mean labels/shard `>= 10` and singleton fraction `<= 40%`** -> proceed to a
  backward-DP prototype.
- **Q2 mean `< 3` or singletons `> 70%`** -> stop; record the negative result.
- In between -> the win is marginal; only proceed if Q1 shows the delivery suffix
  is a majority of the route.

Q1 additionally settles the `W`-vs-last-pickup question directly:
`frac_routes_w_never_fires` is the fraction of harvested routes on which the
post-`W` decomposition would have produced no suffix at all.

### Instrumentation

The census needs the live-label population, which only the search loop sees. It
is collected via an optional `label_observer` kwarg on
`_enumerate_passenger_free_assignment_pricing_labels`, called once per label that
survives dominance and enters the frontier. Default `nothing`; production pricing
never sets it and pays one branch per insertion. This is deliberately a hook
rather than a duplicated copy of the loop, which would drift from the original.

Q1 is computed post-hoc from harvested routes by `split_indices`, which replays
the route (reusing `_replay_passenger_free_assignment_route`) and, for each banked
assignment, finds the earliest certifying destination visit and the clock-open
position feeding it. The max over banked assignments is the split point.

---

# Results (measured 2026-08-01, job 19454069)

7 of 8 tasks; n=30/ms=7 was still in post-processing and is not needed for the
verdict (n=30/ms=10 covers that size). All searches exhausted except n=30, which
hit the 600s budget.

| n | max_stops | exhausted | labels | mean route len | suffix after last pickup | suffix after W | W never fires | **mean labels/shard** | median | singletons |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 15 | 7 | yes | 15,856 | 4.58 | 42.0% | 22.4% | 22.4% | **3.00** | 2.0 | 36.5% |
| 15 | 10 | yes | 16,234 | 4.59 | 42.0% | 22.4% | 22.4% | **3.04** | 2.0 | 36.2% |
| 15 | uncapped | yes | 13,749 | 4.89 | 41.9% | 21.0% | 24.3% | **2.74** | 2.0 | 42.4% |
| 20 | 7 | yes | 45,329 | 4.72 | 36.5% | 16.5% | 30.2% | **2.61** | 2.0 | 41.7% |
| 20 | 10 | yes | 46,420 | 4.73 | 36.4% | 16.5% | 30.2% | **2.66** | 2.0 | 41.7% |
| 20 | uncapped | yes | 39,131 | 5.03 | 36.3% | 15.8% | 31.0% | **2.51** | 2.0 | 45.8% |
| 30 | 10 | no | 267,875 | 4.80 | 40.6% | 9.7% | 57.4% | **2.60** | 2.0 | 44.6% |

## Q2: the sharded variant is dead

Pre-registered bar: mean labels/shard `>= 10` and singletons `<= 40%` to proceed;
mean `< 3` or singletons `> 70%` to stop.

**Measured mean is 2.51-3.04, median 2.0 in every single configuration** -- a
factor of 3-4x short of the proceed bar, and at or under the stop line in 5 of 7
runs. The `(current, support)` backward DP would be shared by two or three labels
before being rebuilt. That is not amortization; it is today's per-label
`post_w_completion` with extra bookkeeping.

The decisive part is the **stability**. Mean labels/shard sits at 2.5-3.0 across a
20x range in label count (13.7k at n=15 to 268k at n=30) and across every
`max_stops` setting. Sharding does not concentrate as instances grow, so this is
not a small-instance artifact that a bigger case would fix. Live-clock supports
are close to unique per label because the support is a *set*, and the ages that
distinguish labels within a support are precisely what the join would have to
re-evaluate anyway.

Note what this does and does not establish. It kills the **sharded** variant on its
pre-registered bar. It does not by itself kill bidirectional labeling here -- the
forward-independent backward label needs no shards at all. The reason that variant
is not worth building either is structural rather than empirical, and is the
subject of the verdict section below; the shard number is what pointed at it.

**Verdict: do not build the backward DP.** The `label_observer` hook is retained
(it is default-off and one branch) since it is the reusable instrument here; the
join machinery was never written.

## Q1: the last-pickup cut is the right cut, and it still is not enough

The redirection from `t > W` to "last contributing pickup" is fully vindicated:

- the last-pickup suffix is **2.0x** (n=15), **2.2x** (n=20), **4.2x** (n=30) the
  post-W suffix;
- the post-W split produces **no suffix at all** on 22% of routes at n=15, rising
  to **57.4% at n=30** -- on the majority of large-instance routes the W
  decomposition would have been a no-op;
- the advantage *widens* with n, so the W split would have looked worse and worse.

So the earlier framing was wrong in the direction identified. But a 36-42% suffix
share cannot rescue a 2.6x shard factor -- Q1 was never the binding constraint.

## Depth is not the lever (confirmed)

`mean_route_length` is 4.57-5.03 and **flat in `max_stops`**: at n=20 it is 4.72
(cap 7), 4.73 (cap 10), 5.03 (uncapped), with suffix fractions 36.5 / 36.4 / 36.3%.
`best_rc` is **bit-identical** across all three caps (-7898.0243 at n=20,
-13380.2715 at n=15), matching the earlier saturation finding (optimum flat from
cap 4 onward).

Routes are short because ride limits and the pickup window bind, not because the
cap binds. Raising the cap lengthens the *search*, not the *routes* -- which
removes the depth premise bidirectional labeling needs.

## Unrelated finding worth acting on: capping `max_stops` can cost more than it saves

Uncapping was **faster and generated fewer labels** than capping at 7, with an
identical certified optimum:

| n | cap 7 | uncapped | |
| --- | --- | --- | --- |
| 20 | 45,329 labels / 13.9s | 39,131 labels / 8.1s | **1.7x faster, same `best_rc`** |
| 15 | 15,856 labels / 1.2s | 13,749 labels / 1.3s | 13% fewer labels, time flat |

Mechanism: `bounded_max_stops` switches on an *extra dominance condition*
(`a.route_length <= b.route_length`, `labels.jl:420`). Capping therefore makes
dominance strictly harder to satisfy, so fewer labels are dominated and buckets
grow -- and since the dominance scan is ~85-90% of wall time, that costs more than
the shallower search saves at these sizes.

This is one pricing snapshot per size, not a CG run, so it is a lead rather than a
recommendation. The follow-up is to sweep `max_stops` through the full CG loop and
compare certified `lp_bound` plus total time -- if it holds there, the cap should
be off by default at these instance sizes.

---

# Verdict: not worth pursuing (2026-08-01, closed)

Not because it cannot be built -- the construction above is sound and complete --
but because it optimizes a dimension that is not binding here, at a cost to the one
that is. Five reasons, in order of how decisive they are.

## 1. The payoff premise fails: this search is not depth-bound

Bidirectional labeling is a depth-halving technique. It pays when label count grows
exponentially in route depth. Here it does not: **45,329 labels for 4.7-stop routes
over 20 nodes**, against an unpruned branching of ~20^4.7. Dominance already
flattens the growth curve, so depth is not what is being paid for, and halving it
does not halve anything that hurts.

There is also barely any depth to halve. Mean route length is 4.57-5.03 and *flat*
in `max_stops`, with `best_rc` bit-identical across caps 7 / 10 / uncapped. Routes
are short because ride limits and the pickup window bind -- the cap was never the
constraint.

## 2. The state is in the wrong half, for any split

Label count is driven by `activated_reward_layers` (a BitSet over `sum_p m_p`) and
`station_age` -- and **both are created only by pickups**. Pickups sit in the route
prefix (2.68-3.26 stops of prefix against 1.69-2.01 of suffix). So a split divides
the *stops* without dividing the *state*: whichever half owns the pickup phase
still enumerates the clock/layer combinations that actually explode.

## 3. The last-pickup cut is maximally lopsided *by construction*

It is defined as the point after which no clock-opening event occurs. That is
exactly what makes the decomposition clean -- and exactly what leaves the backward
half with no state. The measured shard degeneracy (mean 2.51-3.04 labels per
`(current, support)` shard, median 2.0, stable across a 20x label-count range) is a
symptom of this, not an independent finding.

This is a genuine structural tradeoff, not a bad choice of split point:

- **split early** -> both halves carry clocks and layers, so neither simplifies;
- **split late** -> clean one-directional coupling, but nothing left on the
  backward side.

There is no split point that gets both.

## 4. The ceiling is low and the cost is real

Best case, the backward half absorbs a **1.7-2.0 stop tail** (only 16-27% of routes
have a suffix of 3+ stops). Against that, it adds: a `|F| x |B|` join with a
per-pair reward evaluation, an offset-profile dominance far weaker than the current
rule, and a second label type with its own correctness surface. Meanwhile ~85-90%
of wall time is the dominance scan -- which bidirectionality does not touch. It adds
a second one.

## 5. Track record: this pricer yields to state-shrinking, not search-restructuring

Every measured win has come from making the state smaller or the bucket scan
cheaper -- compensated dominance 2.5-3.9x, station-simple warm start 2.6x, `Vector`
buckets 1.3-1.5x, two-stop seeding 2.0x -- and the station-simple result is explicit
that *bucket granularity, not domination power, sets wall time*. Every attempt to
restructure the search instead has lost: MCF relaxation (200x slower, 0/5
certified), the treap dominance index (removed), station-subset B&B (bound-limited,
loses at every cap).

## What to do instead

The live lead from this same experiment is in the winning family and is a config
change rather than an algorithm: **uncapping `max_stops` ran 1.7x faster with 14%
fewer labels at n=20 and a bit-identical certified optimum**, because
`bounded_max_stops` switches on an extra dominance condition (`labels.jl:420`) that
makes dominance harder to satisfy. Measured on one pricing snapshot per size, so it
needs a full-CG sweep before the default changes -- but it targets the dominance
scan, which is where the time actually is.

The deferred clock-quantization axis (`2026-07-31_pfa_state_space_relaxation_design.md`)
is the other candidate in that family, and the `label_observer` hook added here is
the instrument for it.
