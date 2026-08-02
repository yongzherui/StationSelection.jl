# A state-space relaxation for the PFA pricer (design + measurement)

2026-07-31. Successor to `notes/2026-07-31_passenger_mcf_relaxed_pricer.md`, which
rejected the time-expanded MCF LP (valid, but 0/5 certification rate and 200x
slower than the label search it was meant to replace).

**Measured 2026-07-31 (see "Results" at the bottom, and
`scripts/diag_passenger_reward_ladder_census.jl`). Verdict: this is NOT a
termination certificate -- 0% certification rate, same as the MCF -- but unlike
the MCF it is a real win as a column *harvester*: at n=20 an exhaustive relaxed
pass runs 2.3-3.9x faster than the exact one and returns columns 0.9% short of
the true pricing optimum. Recommended use is to replace the early-return pricing
phase, not the certification pass.**

## What went wrong last time, and what that implies

The MCF relaxation replaced the label search with a different algorithm, and lost
on both axes at once: it was looser *and* slower. The lesson is that any relaxed
pricer here has to **stay inside the label search** -- which is heavily tuned
(compensated dominance 2.5-3.9x, inlined bucket entries 1.1-1.15x, `Vector`
buckets 1.3-1.5x, sparse ages) and already runs in well under a second at n=10.

So: relax the *state space* the labels carry, not the algorithm.

## Where the state actually is

```julia
struct PassengerFreeAssignmentPricingLabel
    current; route; time; station_age; activated_reward_layers; tau; reduced_cost;
    route_length; visited_mask
end
```

Dominance (`labels.jl:411`) needs all of: same `current`; `route_length <=`;
`visited_mask ⊆`; `time <=`; the compensated reward test
`rc_a + w(A_a \ A_b) <= rc_b`; and the live-age test `dom(age_b) ⊆ dom(age_a)`
with `age_a(j) <= age_b(j)`.

Two components create essentially all the incomparability, and the label count
grows 43k -> 1.38M across a CG run because of them:

  1. `activated_reward_layers` -- a BitSet over **`sum_p m_p`** layers, where
     `m_p` is passenger `p`'s number of *distinct positive reward values* under
     the current duals. Since `rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - w_pjk`
     takes a generically distinct value for every feasible `(j,k)`, `m_p` is
     large -- likely dozens.
  2. `station_age` -- the live pickup clocks.

This design relaxes (1). (2) is a documented second axis at the end.

## The relaxation family: coarsen the reward *ladder*

Notation: `B(r) = sum_p max{rho_pjk : (p,j,k) certified by r}` is a route's true
reward (max over the empty set is 0), and `rc(r) = beta*(tau_r + repo) - B(r)`.

Today `_build_passenger_reward_layers` turns `p`'s distinct positive rewards
`0 < v_p1 < ... < v_p,m_p` into an incremental ladder `delta_ph = v_ph - v_p,h-1`,
and maps a candidate worth `v_pq` to the **prefix** `{(p,1)...(p,q)}`. OR-ing
prefixes takes the max; summing activated weights recovers `B(r)` exactly.

**The relaxation: retain only a subset `V_p ⊆ {v_p1..v_p,m_p}` of each
passenger's reward values, always including the largest, and build the ladder
over `V_p`. A candidate worth `v_pq` maps to the prefix ending at the smallest
retained value `>= v_pq`.**

That is: each candidate's reward is **rounded up** to the next retained level.

  - `|V_p| = m_p` -> exact, byte-identical to today.
  - `|V_p| = 1` -> one bit per passenger, credited `rho^_p = max_{(j,k)} rho_pjk`
    for *any* certification of `p`.
  - anything in between -> a tunable ladder.

### Validity

For any route `r` and any retained set `V`, write `B_V(r) = sum_p b^V_p(r)` where
`b^V_p(r)` is the smallest value in `V_p` that is `>= b_p(r)` (and `0` when `p` is
uncertified, since an uncertified passenger activates no layer either way). Then
`b^V_p(r) >= b_p(r)` by construction, so

    B_V(r) >= B(r)   ==>   rc_V(r) <= rc(r)   for every route r.

The relaxed search therefore under-estimates every route's reduced cost, and

    min_r rc(r) >= min_r rc_V(r).

**Certificate.** If the relaxed search runs to exhaustion and finds no route with
`rc_V < -tol`, then no route has `rc < -tol` -- no improving column exists. This
is exactly the certification the exhaustive exact pass currently has to establish,
obtained from a strictly smaller search.

**Monotonicity.** Adding a value to `V_p` can only lower `b^V_p`, so refining the
ladder raises `rc_V` pointwise and raises `min_r rc_V`. The bounds form a
non-decreasing chain ending at the exact value when every `V_p` is complete.

**Columns are never wrong.** Any route the relaxed search returns is replayed
through the existing `_passenger_free_assignment_column_from_route` against the
**exact** pricing data, which recomputes `rc(r)` from scratch. A route with
`rc(r) < -tol` is a genuine improving column no matter how it was found.

### Why this shrinks the search rather than just re-labelling it

  - The layer universe drops from `sum_p m_p` bits to `sum_p |V_p|`.
  - Labels that differ only in *which reward level* they reached for `p` become
    identical and merge -- this is the actual state-space collapse.
  - The compensated dominance test scans a much shorter BitSet, and it is ~90% of
    wall time.
  - `best_by_signature` is keyed on the layer BitSet, so its key space collapses too.

Crucially the **bound function stays admissible unchanged**:
`_passenger_free_assignment_remaining_reward_bound` books each layer at most once
against the node that must be reached, and each retained level is still earned at
most once. No change to `search.jl`.

## The DSSR loop

Decremental state-space relaxation, with "over-credited passenger" playing the
role that "customer appearing in a cycle" plays in VRP.

```
V := coarsest ladder (|V_p| = 1 for all p)   # or warm-started from the last CG iteration
loop:
    routes := label search on pricing data built from V     # unchanged search code
    replay every route against the EXACT data -> true rc
    if any true rc < -tol:  return those columns            # FAST PATH
    if the search was exhausted and no route had rc_V < -tol:
        return CERTIFIED (no improving column exists)
    # every route over-claimed: repair
    for the best route r*, insert each over-credited passenger's ACHIEVED reward
        value b_p(r*) into V_p
```

**Termination.** If no passenger is over-credited on `r*` then `rc_V(r*) = rc(r*)`,
so one of the two exits above already fired. Otherwise at least one value is
inserted, and `sum_p m_p` is finite; at the complete ladder the search is exact.

**Bounded downside.** The worst case is the exact search plus the wasted relaxed
passes. That is the property the MCF design lacked.

**Promotion is sharp.** Inserting the *achieved* value `b_p(r*)` (rather than
doubling `|V_p|`) makes exactly the assignment that cheated stop cheating, which
is the direct analogue of DSSR adding the specific cycled customer.

## Cost model, honestly

The fast path costs one relaxed search per CG iteration and is where the
per-iteration savings live -- ordinary iterations do have improving columns, and a
route that over-claims usually still has a genuinely negative true `rc`.

The known risk: at `|V_p| = 1` a passenger is worth `rho^_p` for *any*
certification, so the relaxed pricer prefers broad routes that touch many
passengers cheaply and claim full value for each. That is precisely the hub-route
pathology of `notes/2026-06-22_lp_mip_gap_ghost_u.md`, where the aggregate
pricer's additive reward produced 33/36 wasted pairs and a 21.6% gap. Here it is
survivable -- replay prices those routes honestly and the repair loop targets
exactly the offending passengers -- but it is the reason the starting ladder
should probably be `|V_p| = 2` or 3 rather than 1, and the reason the measurement
below leads with the promotion count.

## Second axis (designed, deferred): the clocks

`station_age` is the other incomparability source. The valid direction is
**optimism**: smaller ages certify more.

  - **Age rounding.** `age' = Delta*floor(age/Delta)`, applied at each extension.
    Younger clocks make `age + travel <= ride_limit` easier, so reward can only
    increase -- valid by the same argument as above. `label.time` must be floored
    with it so the `<= max_wait_time` reset stays consistent and optimistic.
  - The real prize is second-order: rounded ages are **integers**, so the age
    vector becomes hashable and can be folded into the dominance bucket
    signature. Buckets are currently keyed on `current` alone, so at 1.38M labels
    over ~20 nodes they run to tens of thousands of entries and every insertion
    scans the whole bucket. The station-simple measurement is explicit that
    **"bucket granularity, not domination power, sets wall time"** -- `:exact`
    buckets keep 3-6x MORE labels and still run 1.6-3.5x faster.

Do this only after the reward axis is measured; it changes `labels.jl` and the
dominance rule, which the reward axis does not.

## Implementation

**Changed** -- `pricing/passenger/data.jl`:

  - `_build_passenger_reward_layers(candidates; retained_values)` -- per passenger,
    build the ladder over `V_p` instead of all distinct values, and map each
    candidate to the prefix ending at the smallest retained value `>= its reward`.
    Default (`retained_values` empty) keeps every value, i.e. today's behaviour
    exactly.
  - `create_passenger_free_assignment_pricing_data(...; retained_reward_values::Dict{Int,Vector{Float64}}=Dict())`
    threading it through. This is the entire relaxation.

**New** -- `pricing/passenger/state_space_relaxation.jl`, included in
`src/opt/optimize.jl` after `passenger/search.jl`:

```julia
passenger_free_assignment_pricing_by_dssr(
    exact_pricing_data, candidates, existing_columns;
    next_column_id, reduced_cost_tol, max_new_columns, n_candidates,
    time_limit, initial_levels::Int=2, max_rounds::Int=..., warm_start_values=nothing,
) -> (columns, exhausted, stats)
```

matching the duck-typed `(columns, exhausted, stats)` contract so it drops into
`_price_one_passenger_scenario` next to the station-simple branch. `stats` adds
`relaxed_best_rc` (a valid lower bound on `min rc`), `n_dssr_rounds`,
`n_promoted_passengers`, `retained_values_final`, and `certified`.

It calls `passenger_free_assignment_pricing_by_label_setting` unmodified on the
relaxed data, and `_passenger_free_assignment_column_from_route(route,
exact_pricing_data; label_reduced_cost=nothing)` for replay -- the assert must be
suppressed there, because relaxed and true `rc` legitimately differ now.

**New runtime invariant**, in the spirit of `_verify_passenger_master_reduced_cost`:
every replayed route must satisfy `relaxed_rc <= true_rc + 1e-6`. It is the
relaxation property itself, it is cheap, and it catches any ladder-construction
bug on the first route rather than as a silent wrong certificate.

**Changed** -- `pricing/passenger/column_generation.jl`: a `use_dssr` kwarg (off by
default), the per-scenario level map carried across CG iterations as a warm start,
and `relaxed_best_rc` / `n_dssr_rounds` on `iteration_rows`.

## Measurement, with the bars set in advance

**Step 0 -- the layer census, before writing any search code.** Log `sum_p m_p`
and `|P|` per CG iteration at n=10/15/20. This is ~20 lines and it bounds the
entire win: if `m_p` is close to 1, coarsening the ladder cannot merge anything
and the whole reward axis is dead, so go straight to the clock axis instead. If
`m_p` is in the dozens, as the dual structure suggests, proceed.

**Step 1 -- relaxed vs exact per iteration**, mirroring
`scripts/diag_passenger_mcf_relaxation_gap.jl`: for every (iteration, scenario)
record relaxed best `rc`, true best `rc`, labels generated by each, wall of each,
DSSR rounds, and passengers promoted.

Pre-registered rules:

  1. `relaxed_rc > true_rc + 1e-6` on any route is a **validity bug**, not a
     tuning problem. Must be 0.
  2. Ship only if, on iterations where the exact pass proves nothing remains, DSSR
     certifies **>= 50%** of them at **less** wall than the exact pass. This is the
     bar the MCF failed (0%, 200x slower).
  3. On ordinary iterations, the fast path (relaxed search yields a genuinely
     improving column with no promotion round) should fire on the large majority;
     that is where the per-iteration saving comes from. If most iterations need
     several promotion rounds, the starting ladder is too coarse -- raise
     `initial_levels` before abandoning.
  4. Report worst-case overhead explicitly: total DSSR wall on iterations where it
     ended up promoting every passenger, versus the exact search alone.

Run everything through `sbatch`, one `Gurobi.Env()` per process, and via the
snapshot submitter (`scripts/submit_pfa_bench_snapshot.sh`) so a queued job cannot
silently benchmark edited source under the old variant name.

---

# Results (measured 2026-07-31)

`scripts/diag_passenger_reward_ladder_census.jl`, Zhuzhou p=16 seed=42, 3
scenarios, max_stops=5, max_visits=3, run to CG convergence with exact pricing
driving the loop and the relaxed searches measured alongside. 486 rows.

**No source changes were needed to measure this.** Rounding every candidate's
reward up to its retained rung before calling
`create_passenger_free_assignment_pricing_data` makes
`_build_passenger_reward_layers` construct exactly the coarsened ladder, because
it groups distinct reward values itself. The whole relaxation family is a
pre-transform on the candidate vector.

## Step 0: the census -- hypothesis CONFIRMED, and it improves with n

| n | mean \|P\| | mean `sum_p m_p` | ratio |
| --- | --- | --- | --- |
| 10 | 14.2 | 69.6 | 4.92 |
| 15 | 14.9 | 105.6 | 7.13 |
| 20 | 14.7 | 244.6 | **16.77** |

`m_p` is nowhere near 1 -- median 3-4, max up to 12 at n=10 -- and the collapsible
slack *grows* with n, since more stations means more feasible `(j,k)` and hence
more distinct `rho_pjk` values per passenger. The reward axis is alive.

## Validity: clean

0 validity violations (`relaxed_rc > true_rc`) and 0 false certificates across all
486 rows, at every `L`.

## Speed: real, and best at the coarsest ladder

Totals over the whole run, exact -> relaxed:

| n | L | all rows | label ratio | certification-tail rows only |
| --- | --- | --- | --- | --- |
| 10 | 1 | 2.47s -> 0.54s (**4.55x**) | 0.830 | 0.09s -> 0.05s (1.86x) |
| 15 | 1 | 21.9s -> 11.3s (1.95x) | 0.891 | 3.11s -> 1.85s (1.68x) |
| 20 | 1 | 403s -> 103s (**3.93x**) | 0.769 | 110s -> 32.5s (3.38x) |
| 20 | 2 | 403s -> 176s (2.29x) | 0.896 | 110s -> 49.7s (2.21x) |
| 20 | 3 | 403s -> 207s (1.95x) | 0.927 | 110s -> 62.3s (1.77x) |

**The predicted mechanism was wrong.** Labels only drop 8-23%, yet wall drops
2-4.5x. The win is not fewer labels -- it is a *cheaper dominance scan per label*,
because the compensated test `rc_a + w(A_a \ A_b) <= rc_b` walks a BitSet that is
5-17x shorter. That is consistent with the scan being ~90% of wall
(`notes/2026-07-30_passenger_pricing_label_search_optimizations.md`), and it means
**the reward ladder is not what drives the label count.** The clocks are the
remaining suspect, and the clock axis at the end of this note is now the prime
candidate.

## Certification: 0%, the bar is missed

0/5 (n=10), 0/10 (n=15), 0/16 (n=20). At the iterations where the exact pricer
*proves* nothing remains, the relaxed optimum still sits at -128 to -667, and the
`L` sweep shows the gap decaying far too slowly to reach 0:

    n=15 iter 11 s=1:  L=1 -335.8   L=2 -134.4   L=3 -128.4     (m_p ~ 7)

Reaching a certificate would need essentially the complete ladder, i.e. the exact
pricer. Pre-registered rule 2 (certify >= 50% of certifiable iterations) fails.

(The `L` sweep is not nested -- `V_p` at L=2 is not a subset of `V_p` at L=3 --
so the sweep is occasionally non-monotone. The monotonicity proof is about
*growing* `V_p`, which is what the DSSR loop does, and it is unaffected.)

## Column quality: the actual finding

Fraction of ordinary iterations where the relaxed route set, after honest replay,
**contains the exact pricing optimum**, and how far short it falls otherwise:

| n | L | contains the optimum | mean shortfall | worst |
| --- | --- | --- | --- | --- |
| 20 | 1 | 16.0% | 18.7% | 100% |
| 20 | 2 | 76.0% | **0.94%** | 16.5% |
| 20 | 3 | 84.0% | 0.83% | 16.5% |
| 15 | 2 | 73.7% | 4.40% | 100% |
| 10 | 2 | 69.8% | 7.06% | 100% |

The fast path (relaxed search yields a *genuinely improving* column with no
promotion round) fires on **94.7-100%** of ordinary iterations at every `L`.

Compare against what the early-return phase already ships: it is documented as
returning columns "up to ~89% short of the true pricing optimum"
(`column_generation.jl:9`). L=2 at n=20 is 0.94% short -- two orders of magnitude
better -- from an exhaustive relaxed pass costing 2.29x less than the exact one.

## Recommendation

**Do not build the DSSR certificate.** It converges to the exact pricer, which is
what it was supposed to avoid.

**Do consider the relaxed pass as the pricing phase**, with `L = 2`: it is valid,
it needs no search-code changes (only the candidate pre-transform), columns are
~1% off optimal at n=20, and it is 2.3x cheaper. The certification pass must still
run exactly, so the end-to-end saving is bounded by the early-return share of
pricing wall (~36% at the n=30/p=16 cell, per the scaling note) -- call it a
~25% overall pricing win, not a step change.

**The higher-value target is now the clock axis**, on the strength of the label
ratios above: coarsening the reward ladder by 17x moved label count by only 23%,
so `station_age` is what the state space is made of. Round the ages down (valid by
the same optimism argument) and, more importantly, fold the resulting integer age
vector into the dominance bucket signature -- buckets are currently keyed on
`current` alone, and the station-simple result is explicit that "bucket
granularity, not domination power, sets wall time".

---

# Second axis measured: clock quantization (2026-07-31) -- REJECTED

`scripts/diag_passenger_clock_quantization.jl`. Floor the travel-time matrix to a
grid `q` (as a multiple of the smallest positive travel time), then take the
metric closure. Same Zhuzhou family, run to convergence, 240 rows per n.

## A latent precondition in the exact pricer, found the hard way

The first attempt crashed inside the *relaxed* search on
`_passenger_free_assignment_column_from_route`'s own assertion: the label's
incremental reduced cost (-1143.35) disagreed with replay (-1174.32) on the same
pricing data.

Cause: **`_passenger_free_assignment_age_is_useful` silently assumes the travel
matrix satisfies the triangle inequality.** It discards a live clock once that
origin's opportunities are unreachable in time *from the current node*, which is
sound only if no detour can beat the direct arc. Flooring each arc independently
loses up to `q` per hop, so a two-hop path can undercut the direct arc; the
pruning then drops a clock that later certifies, and replay -- which keeps every
clock -- finds reward the label missed.

Taking the metric closure of the floored matrix restores the precondition and
keeps the relaxation valid (`closure(floor(d)) <= floor(d) <= d`). After that
change: 0 validity violations and 0 false certificates across every row.

**This is worth recording independently of the relaxation**: any future change to
`travel_cost` that breaks the triangle inequality will silently corrupt the
pricer's reduced costs, and the only thing that catches it is an assertion whose
message points at reward-layer accounting rather than at the metric.

## Results: worse on both axes, monotonically

Totals over the whole run, exact -> variant:

| n | variant | wall | labels |
| --- | --- | --- | --- |
| 10 | q=1x | 1.56s -> 1.14s (1.36x) | 1.113 |
| 10 | q=2x | 1.56s -> 3.28s (**0.47x**) | 1.853 |
| 10 | q=4x | 1.56s -> 7.94s (**0.20x**) | 3.142 |
| 15 | q=1x | 21.5s -> 38.8s (0.55x) | 1.404 |
| 15 | q=2x | 21.5s -> 121.9s (0.18x) | 2.433 |
| 15 | q=4x | 21.5s -> 249.9s (**0.09x**) | 3.575 |

For reference in the same run, the reward ladder alone: `L2` gives 2.33x (n=10)
and 1.51x (n=15). **The axes do not compose** -- `L2 + q=1x` drops the ladder's
win to 2.08x (n=10) and 0.85x (n=15), i.e. quantization actively cancels it.

## Why it backfires, and what the isolated experiment would be

Quantizing the travel matrix lowers travel **cost** as well as the clocks. Cheaper
travel makes every route's reduced cost more negative, so the pop-time prune
`popped_priority >= -reduced_cost_tol && continue` fires far less often and the
search explores much more. Whatever dominance collisions the coarser clocks buy
are swamped by that.

The clean version of this experiment is **clock-only quantization**: keep the true
travel matrix for `tau` (and hence for `rc`), and quantize only the `time` /
`station_age` bookkeeping. Still valid -- `tau` exact, clocks younger, so reward
can only go up and `rc' <= rc`. It cannot be done with a pre-transform, though:
`PassengerFreeAssignmentPricingData` would need a second matrix used by
`extend_passenger_free_assignment_pricing_label` and the age-usefulness test while
`travel_cost` stays for cost.

Whether that is worth doing is genuinely open. The evidence cuts both ways: it
removes the cost-inflation effect that sank this attempt, but the reward-axis
result already showed that a 17x state reduction moved label count only 23%, which
suggests label count is limited by route diversity rather than by state
granularity -- in which case no amount of clock coarsening will help either.

## Standing recommendation

Unchanged from the reward-axis section: **`L=2` reward-ladder coarsening is the
one relaxation that pays** (2.3x at n=20, columns ~1% off optimal), as a
replacement for the early-return pricing phase. Neither axis certifies. Do not
quantize travel times.

---

# Correctness audit of the shipped `reward_coarsening_levels` path (2026-07-31)

Two soundness defects found in `_price_one_passenger_scenario`, both fixed, both
now covered by `test/opt/test_passenger_reward_coarsening_pricer.jl`.

## 1. `exhausted` was propagated from the relaxed search unconditionally

The pricer contract is that `exhausted == true` with zero columns means "no
improving column exists" -- that is what the caller turns into
`:optimality_proven`. Under coarsening the old code returned the *relaxed*
search's flag straight through.

That is sound only when the relaxed search itself returned nothing: then no route
has `relaxed_rc < -tol`, and since `relaxed_rc <= exact_rc` route by route, none
has `exact_rc < -tol` either. But when the relaxed search returns routes that all
then fail exact replay, nothing is proven -- the returned set is dominance-pruned
under *relaxed* rewards, so a route with `exact_rc < -tol` can be dominated by one
with better relaxed and worse exact reduced cost and never surface.

**Reach, from the census CSVs (L=2):** the old code would have reported
`exhausted = true` alongside zero exact columns on **12.5% (n=10), 22.9% (n=15),
24.2% (n=20)** of priced scenarios. Every relaxed search in those runs was
exhausted, so the flag was load-bearing on roughly one call in four.

**It was not producing wrong LP bounds**, because
`run_passenger_free_assignment_column_generation` pins the certification pass to
`reward_coarsening_levels=0` (level 0 = exact), and `:optimality_proven` is set
only from that pass. The defect was in the public pricer contract, which anyone
calling `_price_one_passenger_scenario` or `_price_passenger_scenarios` directly
would have hit.

Fix: `certified = exhausted_s && isempty(columns_s)`.

Consequence worth stating plainly: **a run with coarsening enabled everywhere can
never certify.** That matches the measurement (0/5, 0/10, 0/16) and is why the
level-0 certification pass is the right design, not an accident.

## 2. Pool novelty was judged on relaxed assignment signatures

The relaxed search was handed the real column pool, and its internal
`try_accept_route!` rejects a route whose assignment signature is already pooled
at no worse `tau`. That check is sound for the exact pricer (a pooled column with
the same signature and smaller `tau` has smaller reduced cost, and is already in
the master at `rc >= 0`), but under coarsening the signature is computed from
*relaxed* replay -- and those genuinely differ, because collapsing two of a
passenger's reward levels changes replay's argmax tie-break to a different
`(j, k)`.

A spurious match therefore discarded routes that were new columns exactly, and it
also weakened "no columns returned", which fix 1 now relies on.

Fix: run the relaxed search against an empty pool and apply novelty afterwards on
exact signatures, which the surrounding code already did.

## Also verified

  - Coarsening changes only what an assignment is *worth*, never which `(p,j,k)`
    are certifiable: it maps `reward > tol` to a retained value `>= reward > tol`
    and copies `reward <= tol` untouched, so the opportunity set is identical.
    This is what makes "relaxed found nothing" bound the exact problem, and it is
    now asserted directly.
  - Every emitted column carries the *exact* reduced cost in metadata and
    satisfies `exact_rc < -tol`.
  - `reward_coarsening_levels = 0` is byte-identical to the untouched pricer.
  - `test/Project.toml` was missing `Combinatorics`, so
    `test_passenger_station_subset_pricing.jl` errored under `Pkg.test()`
    (declared in the top-level `Project.toml`, but the test environment is
    separate). Added.

Full suite after the fixes: **2499 passed, 0 failed, 0 errored.**
