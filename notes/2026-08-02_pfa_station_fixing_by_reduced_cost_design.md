# Excluding stations from PFA pricing by reduced-cost fixing (design)

Proposal: use master-side reduced-cost / Lagrangian variable fixing on the
build variables `y` to **permanently delete stations from every future pricing
graph** during a passenger free-assignment (PFA) CG run, shrinking the label
search where it is most expensive (the converged certification tail).

Status: **OPEN for permanent/global fixing.** This note pins the theorem, the
exact fixing threshold, and the correctness guardrails so the idea can be picked
up cleanly. It also records the one seductive-but-unsound version so we don't
rebuild on sand.

Update 2026-08-02: a weaker **iteration-only dual-slack filter** was implemented
and tested. It redistributes positive reduced-cost slack of currently closed
stations into nonnegative increments of the current linking duals, so adjusted
pricing rewards only decrease within the same RMP iteration. This is exact for
the current iteration, but it is **not** permanent fixing and it is empirically
not effective at the `n=10,15` scale tested here. Details are recorded below.

Origin: the intuition that "the good negative-reduced-cost routes in one basis,
and the stations they involve, must be related to the next basis, so we should
be able to exclude stations from the next pricing step." That intuition is
correct, but only in the bound-anchored form below -- not in the
basis-to-basis form.

Background:
- `2026-08-01_pfa_label_setting_algorithm_reference.md` -- how the pricer works.
- `2026-07-31_pfa_state_space_relaxation_design.md` -- where the state lives.
- `lagrangian_relaxation.jl` -- the valid per-iteration bound this note leans on.
- `station_subset.jl` -- the *other* station-pruning attempt (contrast below).

## What is fixed vs. what moves between pricing rounds

For the master formulation see `master.jl`. The physical graph (`nodes`,
`travel_cost`, `ride_limit`) and every column's cost `beta*(tau + repo)` are
**fixed for the whole run**. The only thing that changes iteration to iteration
is the per-assignment reward the pricer carries,

    rho_pjk = alpha_p - gamma^O_pj - gamma^D_pk - w_pjk,

driven entirely by the RMP duals (`passenger_free_assignment_pricing_candidates`
rebuilds it every iteration).

A station `j` is *usable* in an improving route **iff it is the origin or
destination of some opportunity with `rho_pjk > 0`**. Reason: `travel_cost` is a
shortest-path / metric matrix (the triangle inequality is already relied on by
the age-pruning in `data.jl` and `_passenger_free_assignment_age_is_useful`), so
an optimal route never visits a station purely as a waypoint -- every visited
station either opens a pickup clock or certifies a delivery. Hence the
per-iteration station set feeding the pricer is exactly
`{ j : exists (p,k) with rho_pjk > 0 } union { k : exists (p,j) with rho_pjk > 0 }`,
which the `rho > 1e-9` filter in `passenger_free_assignment_pricing_candidates`
already computes.

So the question is precisely: **can we predict which stations will have
`rho > 0` next time and never build them into the pricing graph?**

## The unsound version (the trap)

"Station `j` carried no negative-reduced-cost route under basis `t`, so exclude
`j` from pricing at `t+1`" is **false here**, because of dual oscillation.

Between two consecutive RMP optima the duals sit on different faces -- this
master is highly degenerate, with disaggregated `(p,j)` linking rows -- so
`alpha_p` and `gamma` can swing enough to flip `rho_pjk` from `<= 0` to `> 0`. A
station absent this round can legitimately be the endpoint of an improving column
next round. That is *why* the candidate filter recomputes `rho` from scratch
every iteration rather than caching a station set.

Any pruning keyed on "used last time" can therefore cut a real improving column
and, worse, can make a search that stopped early *look* exhausted -- breaking the
`:optimality_proven` certificate contract documented at the top of
`column_generation.jl`. Do not build on this version.

## The sound version: reduced-cost fixing on `y`

The rigorous statement anchors the excluded set to a **global bound**, not to a
basis. It uses the master's own structure.

`y` couples in only through the linking rows and the budget row `sum_j y_j = l`.
Define station `j`'s **dual worth**

    Gamma_j = sum_p ( gamma^O_pj + gamma^D_pj )   >= 0

(the total dual price the linking rows pay for `j` being open, over all
passengers, as pickup and as dropoff), and let `mu` be the dual of the budget
row `sum_j y_j = l`.

`y_j` has objective coefficient 0, a `-1` in every linking row it touches, and a
`+1` in the budget row, so its LP reduced cost is

    rc(y_j) = -Gamma_j - mu.

At the LP optimum this is the expected p-median behaviour: `-mu` is a threshold,
stations with `Gamma_j > -mu` open (`y_j = 1`), stations with `Gamma_j < -mu`
close, ties go fractional. The LP effectively selects the **top-`l` stations by
`Gamma_j`**. That threshold *is* the "station basis" the original intuition was
reaching for -- but expressed as a scalar order statistic on `Gamma`, not as a
set carried between iterations.

### Fixing theorem

Let `z_LP` be a **valid** lower bound and `z_UB` an incumbent. Standard
reduced-cost fixing on a variable at its lower bound gives:

    Gamma_j  <  -mu - (z_UB - z_LP)   ==>   y_j = 0 in every improving
                                            integer solution.

Once `y_j` is fixed to 0, the linking constraint `theta_r <= y_j = 0` forbids
*any* route through `j`, so **`j` is provably deleted from every subsequent
pricing graph**, including the exhaustive certification pass. Equivalently: a
station whose dual worth falls short of the open/close threshold by more than the
current optimality gap can never be in an optimal build set.

Because the budget is a hard cardinality (`= l`), the same order-statistic
argument fixes stations *in* as well as out (a station far above the `l`-th
largest `Gamma` by more than the gap must be open). Fixing `y_j = 1` does not
shrink the pricing graph, but it tightens the master and can trigger further
`y_j = 0` fixings.

### Which bound to use

- `z_UB`: solve the pool MIP occasionally (it is already solved once at the end;
  do it periodically in the converged tail). Any feasible integer pool solution
  is a valid `z_UB`.
- `z_LP`: **must be valid.** Use either the certified LP bound (only trustworthy
  when `cg_stop_reason == :optimality_proven`) or, better for mid-run use, the
  Lagrangian bound from `lagrangian_relaxation.jl`, which is valid *every*
  iteration. **Never** use the early-return-phase LP objective as `z_LP` -- that
  pool is knowingly incomplete (see the two-phase docstring in
  `column_generation.jl`).

## Why this is a different lever than `station_subset.jl`

The station-subset B&B pricer
(`project_pfa_station_subset_bnb_bound_quality`) loses because *its subproblem's*
reward+routing LP upper bound is ~1.9-2.2x the optimum -- it is bound-quality
limited at the leaf oracle. This proposal never touches the subproblem bound. It
prunes the station set from the **master side** using the master's Lagrangian
bound, then hands the *exact* pricer a smaller graph.

Since PFA pricing cost is per-iteration *width* (scenarios x label search, ~90%
dominance-scan; see `project_pfa_label_search_dominance_bottleneck` and
`project_pfa_cg_scaling_frontier_zhuzhou`), deleting stations shrinks the live
label population directly -- attacking the actual bottleneck without depending on
a weak leaf bound.

## When it bites, and when it does nothing

It fixes nothing until `z_UB - z_LP` is small, so it is inert early in CG when the
gap is wide. That is fine: the payoff window is the **converged tail**. The bound
here is a step function that flatlines for 40-64% of the budget
(`project_pfa_cg_throughput_open_questions`), and the exhaustive certification
pass dominates late-run wall time at n=25-30
(`project_pfa_cg_scaling_frontier_zhuzhou`). In that tail `z_LP` is stable and a
pool-MIP `z_UB` is cheap, so fixing can prune the graph precisely before the most
expensive pricing work.

Open empirical question: how many stations actually clear the threshold at, say,
n=30 with a realistic tail gap? If `Gamma_j` is flat across stations (many
near-tie candidates), few get fixed and the win is small. A cheap first
experiment is to *measure* the `Gamma_j` spread and the implied fixing count at
the point CG reaches the certification pass, before wiring any pruning in.

## Correctness guardrails (bake these in)

1. Use a **valid** `z_LP` (certified LP or Lagrangian), never the early-return
   LP objective.
2. Fixing is **monotone**: once `y_j` is fixed to 0 it stays fixed; never un-fix.
   Then the final certificate legitimately means "no improving column exists over
   the reduced station set," which is sound because every deleted route was
   proven non-optimal by the bound.
3. Recompute the excluded set only from a fresh valid `(z_LP, z_UB)` pair; do not
   carry a station's exclusion forward on stale duals.
4. Sanity assertion: no station carrying an assignment in any *pooled* column
   that participates in the incumbent may be fixed out. (A correct bound cannot
   produce this, so a violation flags a sign/threshold bug -- the analogue of
   `_verify_passenger_master_reduced_cost`.)

## Suggested next step

Phase A (measure): at the iteration CG first enters the certification pass,
solve the pool MIP for `z_UB`, take the Lagrangian `z_LP`, compute
`Gamma_j = sum_p (gamma^O_pj + gamma^D_pj)` and `mu`, and log the histogram of
`Gamma_j` and the count of stations satisfying the fixing inequality. No pricing
change yet. This answers "is there anything to fix?" for a few cents of compute.

Phase B (exploit): if Phase A shows a non-trivial fixed set, delete those
stations from the pricing graph (drop their opportunities before
`create_passenger_free_assignment_pricing_data`) and re-run the certification
pass, checking the certified optimum is unchanged and measuring the label-count /
wall-time reduction. Compare against the n=25-30 tail where certification
dominates.

## 2026-08-02 implementation result: iteration-only slack filter

A related but weaker filter was implemented in the PFA column-generation path:

- `master.jl` computes
  `passenger_free_assignment_station_reduced_cost_eliminations(...)`.
- `column_generation.jl` can enable it with
  `use_station_reduced_cost_filter=true`.
- `station_filter.jl` records per-iteration diagnostics:
  raw positive opportunities, filtered positive opportunities, raw/filtered
  positive endpoint stations, excluded stations, and slack/need ratios.

This filter is **not** the permanent fixing theorem above. It is an
iteration-level dual proof. For a closed station `j`, it treats
`rc(y_j) >= 0` as available slack in the current dual constraint and asks whether
that slack can be spent as nonnegative increments to the current
`gamma^O_pj`/`gamma^D_pj` values. If so, adjusted rewards satisfy

    rho_tilde_pjk = rho_pjk - delta^O_pj - delta^D_pk <= rho_pjk.

The implementation asserts this monotonicity. Therefore, within a fixed RMP
iteration, the number of positive opportunities cannot increase. This avoids the
buggy "full auxiliary dual selector" behaviour, where reoptimizing the whole
dual solution could move `alpha` and create new positive opportunities. That
full selector was removed from the production code path.

The implementation also uses the route-pricing structure actually present in the
master: same-station direct-walk assignments are handled by `x_same`, so the
station-elimination need is based on incident route-pricing opportunities rather
than a separate `(p,j,j)` route opportunity.

### Empirical result at n=10 and n=15

Experiment grid:

- `n_stations in {10, 15}`
- `n_scenarios in {1, 3}`
- seeds `{42, 43, 44}`
- `PFA_STATION_RC_FILTER=0/1`
- `PFA_N_PAIRS=16`
- `PFA_MAX_STOPS=4`
- `PFA_MAX_CG_ITERS=500`
- `PFA_CERT_TIME=300`
- `PFA_PRICING_TIME=30`
- `PFA_CASE_TIME=600`

All 12 matched A/B cases certified optimality with identical values:

    max |LP_filter - LP_no_filter|   = 0
    max |MIP_filter - MIP_no_filter| = 0

So the implementation appears exact on this grid. However, it is not useful at
this scale:

    cases with positive opportunity reduction = 1 / 12
    cases with excluded stations             = 1 / 12
    total label-count change                 = -179

Aggregate result by size/scenario condition:

| n | scenarios | max value diff | total label delta | mean opportunity reduction | mean station reduction | mean excluded stations |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 1 | 0 | -179 | 0.000900901 | 0.00166667 | 0.0166667 |
| 10 | 3 | 0 | 0 | 0 | 0 | 0 |
| 15 | 1 | 0 | 0 | 0 | 0 | 0 |
| 15 | 3 | 0 | 0 | 0 | 0 | 0 |

The only nonzero case was `n=10`, one scenario, seed `44`, where the filter
excluded a station in a small fraction of iterations and reduced 179 generated
labels. Every other tested case had zero opportunity and station reduction.
Runtime deltas were mixed and small, consistent with noise rather than a real
speed improvement.

Conclusion for this implemented iteration-only filter: **exact but empirically
ineffective at the `n=10,15` scale**. The measured slack/need ratios are usually
too small, especially for `n=15`, so the residual reduced-cost slack on closed
stations is not large enough to eliminate incident positive-reward opportunities.

This does not disprove the permanent fixing theorem earlier in the note, which
uses a valid global bound gap and is aimed at the converged tail. But it does
mean the cheap per-iteration slack redistribution should not be expected to
materially shrink the pricing graph for the tested small and medium cases.

## 2026-08-02 follow-up: joint LP opportunity suppression

The closed-form filter only deletes an entire station when that station's own
slack can suppress every incident positive opportunity. A stronger monotone
filter was then implemented:

    PFA_STATION_RC_FILTER=joint_lp

or programmatically:

    station_reduced_cost_filter_mode = :joint_lp

This solves a small LP over nonnegative `delta^O[p,j]` and `delta^D[p,j]`
increments, constrained by each closed station's `rc(y_j)` slack, and minimizes
the remaining positive reward mass:

    rho_tilde[p,j,k] = rho[p,j,k] - delta^O[p,j] - delta^D[p,k] <= rho[p,j,k].

Unlike the removed full auxiliary dual selector, this does not change `alpha`,
`mu`, or the base dual solution. It is still monotone in the current iteration,
so positive opportunities cannot increase. The key difference from the
closed-form station test is that it can suppress individual opportunities by
splitting reward reduction across both endpoints, even when no whole station is
deleted.

The same 12-case grid was rerun:

- `n_stations in {10, 15}`
- `n_scenarios in {1, 3}`
- seeds `{42, 43, 44}`
- compared `PFA_STATION_RC_FILTER=0`, `closed_form`, and `joint_lp`

All joint-LP cases certified equivalent values:

    max |LP_joint_lp - LP_no_filter|   = 7.28e-12
    max |MIP_joint_lp - MIP_no_filter| = 1.46e-11

Joint LP was materially stronger than closed form on opportunity reduction:

    cases where joint-LP reduced more opportunities than closed-form = 8 / 12
    cases with excluded opportunities > 0                            = 8 / 12
    total label-count change vs no-filter                            = -60,402
    total label-count change vs closed-form                          = -60,223

Aggregate result by size/scenario condition:

| n | scenarios | labels joint vs closed | mean pricing delta vs closed | mean opp reduction closed | mean opp reduction joint | mean excluded opportunities |
|---:|---:|---:|---:|---:|---:|---:|
| 10 | 1 | -2,875 | -0.043s | 0.000900901 | 0.00834139 | 0.604637 |
| 10 | 3 | -56,152 | -0.272s | 0 | 0.0168534 | 3.78689 |
| 15 | 1 | 0 | -0.007s | 0 | 0 | 0 |
| 15 | 3 | -1,196 | +0.035s | 0 | 0.00083961 | 0.294221 |

The best observed improvements were at `n=10`, three scenarios:

    seed 42: labels 151,297 -> 147,252   (-4,045)
    seed 43: labels 154,633 -> 141,242   (-13,391)
    seed 44: labels 179,889 -> 141,173   (-38,716)

Conclusion: the independent closed-form station-deletion test is mostly inert,
but the joint LP is promising enough to keep as an experimental option and test
on larger server-scale runs. It still does little on `n=15` single-scenario
instances, but it produces real opportunity and label reductions on the
multi-scenario `n=10` cases while preserving the certified value.

## 2026-08-02 scaling result: joint LP is inert at n=20/25/30

Job 19523673 (`scripts/sbatch_pfa_station_filter_scaling.sh`): the earlier
"promising, test at server scale" hope did **not** hold. Grid
`{nofilter, joint_lp} x n in {20,25,30} x scen in {1,3} x seed in {42,43}`,
p=16, ms4, warm start ON, all 24 cases `optimality_proven` in <16 min.

Effectiveness: **exactly zero** at every case.

| n | scen | mean opp reduction | mean excl opps | mean excl stations | label delta vs nofilter |
|---:|---:|---:|---:|---:|---:|
| 20 | 1 | 0 | 0 | 0 | 0 |
| 20 | 3 | 0 | 0 | 0 | 0 |
| 25 | 1 | 0 | 0 | 0 | 0 |
| 25 | 3 | 0 | 0 | 0 | 0 |
| 30 | 1 | 0 | 0 | 0 | 0 |
| 30 | 3 | 0 | 0 | 0 | 0 |

Correctness reconfirmed at scale: `max |LP diff| = 0`, `max |MIP diff| = 0`
across all 12 matched joint_lp/nofilter pairs; total label delta 0 everywhere.
Pricing-time deltas are timing noise (labels bit-identical → identical search
work); the filter's per-iteration LP is pure unmeasured overhead, net-negative.

Warm-start integration is confirmed **active but empty**: the filter LP ran on
335 `station_simple` iterations and 41 `revisit` iterations across the joint_lp
runs, excluding nothing on either pricer. So the filter *is* wired into the
elementary warm-start phase (as intended); it simply has nothing to cut.

### Why zero: closed stations sit at the selection threshold with ~0 slack

The mechanism is not "few stations are closed" -- it is that closed stations
have **no reduced-cost slack**. Per-iteration diagnostics (joint_lp iters):

| n | scen | mean closed | mean closed-with-need | mean slack/need | max slack/need |
|---:|---:|---:|---:|---:|---:|
| 20 | 1 | ~9.9 | ~9.2 | 0.0000 | 0.0000 |
| 20 | 3 | ~9.6 | ~9.6 | 0.0000 | 0.0004 |
| 25 | 1 | ~12.0 | ~11.6 | 0.0000 | 0.0000 |
| 25 | 3 | ~11.5 | ~11.5 | 0.0000 | 0.0000 |
| 30 | 1 | ~14.9 | ~14.7 | 0.0000 | 0.0000 |
| 30 | 3 | ~15.0 | ~15.0 | 0.0000 | 0.0000 |

There are `~n-l` closed stations and nearly all of them ARE endpoints of
positive-reward opportunities (closed-with-need ≈ closed), so they are candidate
eliminations. But `slack/need = rc(y_j)/required_j ≈ 0`: closed stations carry
essentially zero LP reduced cost. Elimination needs `slack/need >= 1`; it never
gets within four orders of magnitude.

Root cause is the instance regime, not the filter. With `l = ceil(n/2)`, half
the stations are open and the closed ones sit right at the p-median selection
threshold `Gamma_j ≈ -mu`, so `rc(y_j) = -Gamma_j - mu ≈ 0`. `Gamma_j` is flat
across the contested band -- exactly the "flat `Gamma_j`" failure mode this note
flagged as an open question up top. The reduced-cost signal on `y` has no
separation to exploit.

### Consequence for the *permanent* fixing theorem

This is also a caution for the global-gap version. Permanent fixing needs
`rc(y_j) = -Gamma_j - mu > (z_UB - z_LP)`. If `rc(y_j) ≈ 0` for every closed
station (as measured here), the permanent rule fixes nothing either, *regardless*
of how tight the gap gets. Station fixing of any flavour can only bite when the
LP separates "clearly-in" from "clearly-out" stations -- i.e. when `l << n` or
the geometry makes many stations plainly useless -- not at the contested `l = n/2`
balance point these instances use.

### The lever to test next is `l/n`, not iteration-only-vs-permanent

The productive follow-up is a sweep of the build budget `l` (e.g. `l = n/4`,
`n/3`, `n/2`) at fixed `n`, measuring the closed-station `rc(y_j)` spread and the
implied fixing count. The prediction: reductions appear only once `l` is small
enough that a band of stations falls well below `-mu`. If even a modest `l/n`
never opens a slack gap, station-level reduced-cost fixing is not a viable lever
for this problem family and the effort should return to the label-search state
space (`2026-07-31_pfa_state_space_relaxation_design.md`).

## 2026-08-02 l/n sweep: the filter FIRES at small l but does NOT help

Follow-up to the l=n/2 null result. Jobs 19525414 (joint_lp) + 19526161
(nofilter A/B), same script with `PFA_L_DIV` added: `l = ceil(n / div)` for
div in {4, 3} (l=n/2 already measured, all zero), n in {20,25,30}, scen in {1,3},
seed in {42,43}, ms4, p=16, warm start ON. All cases `optimality_proven`,
**0 unserved** at every l, so the small-l instances are still fully coverable.

### The lever works: separation appears as l shrinks

Max reduced-cost slack/need ratio ever observed (needs >= 1 to eliminate):

| l/n | max slack/need | iters excluding a station | per-iter excluded opps (scen 3) |
|---|---|---|---|
| n/2 | 0.0000 | 0 | 0 |
| n/3 | 0.92 | 8 | ~0.3-1.0 |
| n/4 | 2.38 | 57 (up to 2 stations) | ~3-18 |

So the note's hypothesis is confirmed: a smaller build budget pushes closed
stations below the p-median threshold `-mu`, opening the `rc(y_j)` slack the
filter needs. Firing concentrates in multi-scenario cases and at n/4.

### But the payoff is a wash-to-negative (this kills the lever)

Correctness holds when it fires: `max |LP diff| < 2e-12`, `max |MIP diff| < 8e-12`.
The problem is cost. Aggregated joint_lp vs nofilter at matched l:

| l | Σ labels Δ% | Σ pricing Δ% |
|---:|---:|---:|
| 5  (n/4, n=20) | **+29.9%** | +14.2% |
| 7  (n/3 n=20 / n/4 n=25) | -4.5% | -2.4% |
| 8  (n/4, n=30) | -0.1% | -13.4% |
| 9  (n/3, n=25) | **+6.4%** | +12.2% |
| 10 (n/3, n=30) | -0.0% | -6.0% |

No consistent win, and several cases are markedly **worse**. The worst cases line
up exactly with where the filter fires hardest -- e.g. l=5 n=20 scen3 seed43:
labels **+59.6%**, pricing **+78%**, and **+23 CG iterations**; l=7 n=20 scen3
seed43: labels +38%, +10 iterations.

### Why firing hurts: it removes LP-dual-useful, integer-useless columns

The `iΔ` column is the tell: in every case where labels ballooned, CG took MORE
iterations. The filter's proof is that a closed station's slack can absorb the
opportunity's reward, so no route through it can improve the LP **optimum**. That
is not the same as "the column does not help LP **convergence**." Those
dual-credit routes are exactly the broad hub columns CG leans on to settle the
duals ([[project_lp_mip_gap_hub_routes]]: "CG picks broad hub routes for dual
credit"). Suppressing them every iteration -- with a fresh, non-monotone excluded
set each time -- perturbs the CG trajectory and can force more iterations, whose
extra label work swamps the modest per-iteration graph shrink.

### Verdict

The iteration-only station reduced-cost filter (closed_form and joint_lp) is
**not a viable speed lever** for this problem family:
- l=n/2 (the operating regime): inert, `rc(y_j)~0`.
- l<=n/3 (where it fires): certified-exact but net wash-to-negative, because the
  non-monotone per-iteration exclusion destabilizes CG convergence.

Remaining thread, not yet tested: the **permanent** global-gap fix is *monotone*
(fix a station out once, never un-fix), so it would not oscillate the pricing
graph across iterations and might avoid the trajectory inflation seen here. But it
still needs the small-l regime to have any `rc(y_j)` slack at all, and the
label-reduction ceiling even when firing looks modest. Given that, the higher-EV
direction is back to the label-search state space
(`2026-07-31_pfa_state_space_relaxation_design.md`), not further station fixing.
