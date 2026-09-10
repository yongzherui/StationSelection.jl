# The activated dual completion: sound, but its stated proof is not — and where its slack comes from

2026-09-10. Job 22472084 (`benchmarks/diagnostics/benders_activated_completion_audit.jl`,
new here). Zhuzhou n=10 p=8 s=1 seed 42, k=5, `max_stops=4`, 16,320 enumerated columns,
86 endpoint-feasible station sets, 8 anchors spread over the Q ranking. 17/17 checks.

Two questions: is `_benders_activated_complete_duals!` correct, and why are its cuts so much
weaker than the ones full-station pricing produces.

## 1. It is correct — measured as dual feasibility itself, not as a consequence

At `max_stops=4` the enumerated pool IS the column universe, so the dual constraint set is
finite and writable, and `min_c rc(c) >= 0` at the completed duals is full-universe dual
feasibility outright — no reference to the pricer, the loop, or any argument about which
columns the restricted search covered.

| arm | min rc over 16,320 cols x 8 anchors | cut valid over 86 y | tight at anchor | Q vs exact |
| --- | --- | --- | --- | --- |
| `:column_generation_activated` | **-1.82e-12** | +1.82e-12 | 3.6e-12 | 0.0e+00 |
| `:column_generation_activated_lpo` | -3.64e-12 | +1.82e-12 | 3.6e-12 | 0.0e+00 |
| `:column_generation` (control) | -5.46e-12 | +0.00e+00 | 3.6e-12 | 0.0e+00 |

All three are dual-feasible at machine precision, and the completion never breaks the
`alpha_p <= c_walk` family either (`worst alpha - c_walk = 0.0`). The script also
reproduces the solver's own cut from the raw per-`(p,j)` duals and matches it exactly
(`|Gamma_mine - Gamma_solver| = 0.0`), so the numbers audited are the numbers the loop uses.

`restricted solve attains exact Q` passes at every anchor too: the built-only search does
reach `Q_s(yhat)` exactly, so the restriction costs nothing in VALUE. That is a separate
claim from validity and it also holds.

## 2. But the argument written down for it has a false premise

`solvers/benders/subproblem_config.jl` argued step 3 like this: for a column `c` mixing
out-of-`S` and in-`S` assignments, drop the out-of-`S` assignments, leaving `c'` with **the
same route (same tau)** and a subset of the assignments, and `c'` "lies in the searched
universe" — followed by "Note step 3 needs the route universe left UNRESTRICTED — only
candidates shrink."

**The route universe is not left unrestricted.** Candidate generation is reward-driven:
`joint_routing_assignment_pricing_candidates` drops every candidate with `rho <= 0`
(`label_setting/joint_routing_assignment/pricing_round.jl:64`), `create_joint_routing_assignment_pricing_data`
builds `assignments_by_origin`/`origin_layer_mask` from the survivors only
(`data.jl:198-209`), and both the seed (`exact/seed.jl:18-22`, restricted to candidate
origins) and the extension (`exact/extend.jl:46-79`, nodes proposed only from live origins
and their opportunities' destinations) read exactly those. So driving unbuilt-station
candidates to `rho <= 0` removes those stations from the searched ROUTE universe as well.
`c'` — same route, therefore visiting unbuilt nodes — is never searched, and the exhaustion
certificate says nothing about it directly.

The repair is the shortcut argument the same docstring says it deliberately avoids: let
`c''` be `c'` with the unbuilt visited nodes removed. Then `tau_{c''} <= tau_c` by the
triangle inequality (required package-wide already — the pricer's age pruning asserts on
violation, `project_pfa_travel_matrix_triangle_inequality`), `c''` is in the searched
universe, and

    sum_{A_in} net <= f_{c''} <= f_{c'} <= f_c - w * sum_{A_out} demand_p * walk_p
    sum_{A_out} net <= w * sum_{A_out} walk_p            (the completion, per triple)
    => rc(c) >= 0                                         (demand_p >= 1)

**The `max_wait_time` worry recorded in that same docstring is answerable, not open.** Both
route-feasibility conditions the pricer enforces are upper bounds on *elapsed* durations —
the pickup window is `label.time <= max_wait_time` and the ride limit is
`origin_age + travel <= detour_factor * routing_cost(j,k)` — and both only decrease when a
stop is removed. Nothing in this model measures wait against a fixed request clock, so
shortcutting cannot make a retained passenger wait longer. (The `A → U → A → B` case, where
a passenger is picked up at the *second* visit to `A`, is also fine: the age is measured
from the last visit to that station, so the collapsed route gives the same in-vehicle time.)

Same false premise appears twice more in the family's docs, both times as "a column touching
an unbuilt station is pinned to `theta = 0` by its own linking row": that pins a column with
an unbuilt ASSIGNMENT, not one whose ROUTE merely passes through an unbuilt node. The reason
value is still exact is again shortcutting — the shortcut column is cheaper and is in the
searched universe. The audit's `restricted solve attains exact Q` check is the empirical
version of that.

Net: no soundness bug, but the proof of record was not the proof that holds. Both were
corrected in place.

## 3. Why the cuts are weak: the completion credits 1% of what it has to cancel

Per-arm means over the 8 anchors:

| arm | `sum_j Gamma_j / constant` | nonzeros | y where cut predicts <= 0 | median rel. slack vs true Q |
| --- | --- | --- | --- | --- |
| activated (closed form) | **1.80** | 3.4 | **33.9%** | **93.6%** |
| activated + LPO separation | 0.23 | 2.8 | 0.0% | 4.6% |
| full CG (control) | 0.22 | 2.2 | 0.0% | 5.1% |

The closed form puts ~8x the coefficient mass on `y` that the true dual does, and the
resulting cut sits a median 93.6% BELOW the true `Q_s` across the first-stage space — i.e.
it says essentially nothing anywhere except at its own anchor, and at a third of the space
it is dominated by the master's own `Theta_s >= 0`.

`Gamma_j` at the same anchor makes it concrete. At the optimum `y=[1,2,3,7,9]` the true
dual charges **nothing at all** for any unbuilt station (cut is the flat
`Theta >= 12249.40`), while the completion charges 5210.3 on station 4 and 5199.0 on
station 6. At `y=[3,4,6,7,9]` it charges **12212.3 on station 1 alone — 99.7% of the cut's
own constant** — where the truth is 0. The LPO completion, by contrast, lands on the true
dual almost exactly (e.g. at `y=[1,3,5,6,7]`: 660.5/1627.0/2449.0 vs the true
650.6/1636.9/2449.0).

And here is the mechanism, in the units that matter. Per unbuilt `(p,j)` the completion has
to cancel `alpha_p`, and it credits only the walking term:

| anchor | mean `alpha_p` | `w * walk_min` (credited) | `w * demand_p * walk_min` (`:route_free`) | `beta * delta_j` (discarded) |
| --- | --- | --- | --- | --- |
| [1,2,3,7,9] | 1531 | 11.6 | 11.6 | 1376 |
| [3,4,6,7,9] | 1531 | 4.0 | 4.0 | 1746 |
| [1,3,4,7,8] | 1745 | 13.3 | 13.3 | 527 |
| [1,2,3,6,7] | 1756 | 16.7 | 16.7 | 1339 |
| [1,5,6,7,9] | 1986 | 16.8 | 16.8 | 351 |
| [1,3,5,6,7] | 2067 | 13.4 | 13.4 | 352 |
| [1,4,5,7,10] | 2067 | 17.9 | 17.9 | 755 |
| [1,6,7,8,10] | 2067 | 18.1 | 18.1 | 4 |

`delta_j = min_{a,b built} [travel(a,j) + travel(j,b) - travel(a,b)]` is the cheapest detour
any route could pay to touch `j` — route-free to compute, and the term BOTH completions throw
away.

So the completion must cancel ~1500-2000 of reward per triple and credits **4-18 of it
(0.2-1.2%)**, while the route-travel term it ignores is worth **350-1750 (25-85%)**. With
`walk_cost_weight=0.1` against `route_regularization_weight=10.0`, crediting walking is
crediting the wrong term by two orders of magnitude. `gamma_pj` therefore lands at
essentially `alpha_p`, and `Gamma_j = sum_{p: j valid} gamma_pj` reaches the cut's whole
constant on a single station.

**Why `:route_free` recovers nothing, settled:** on this instance `demand_p == 1` for every
group (the two credit columns above are identical), so `(T)` and the closed-form bound are
the same number up to letting the two endpoints split the requirement. The demand factor is
not the missing strength. It may be non-vacuous at larger `p`/scenario counts, but it was
never the lever.

## 4. There is no cheap-pricing-plus-strong-cuts corner, and that is now measured twice

The true `Gamma_j` is 0 for most unbuilt stations because *no route through them is
attractive at all* — a statement about whole routes at the current duals, not a bound on one
station's insertion cost. Only pricing over those stations discovers it, and the activated
restriction removes exactly that search. Both surviving oracles pay it back in full:

| n=30 s=3 ms10 (`results/benders_lpo`) | restricted pricing | full-universe pricing | total | iters/cuts |
| --- | --- | --- | --- | --- |
| `activated_lpo` (separation) | 6.8 s (1.3%) | 501.2 s | 508 s | 5 / 12 |
| `warm_start` | 9.4 s (1.1%) | 844.5 s | 854 s | 5 / 12 |
| plain `column_generation` | -- | 566.7 s | 567 s | 5 / 12 |

~99% of the pricing is full-universe work in every arm that produces valid strong cuts. The
built-only phase is not where the time is, so there was never much to reclaim: the
full-station grind IS the cut information. The plain activated oracle is what happens when
you decline to buy it — n=15 s=3: 657 iterations, 1971 cuts, wall-stopped at 900 s with a
4.9% gap still open (vs 6/10 converged for full CG); n=20 s=3: 460 iterations, 1380 cuts,
LB 20620.60 against UB 28735.19, **28% gap open** (vs 9/13 in 10.3 s). It converges only at
n=10, where 77 cuts is comparable to the 86 station sets that exist.

## 5. The one untried lever, and the number that decides it

Charge the discarded route term: any column visiting an unbuilt `j` pays at least
`beta * delta_j`, which the table above puts at 25-85% of `alpha_p`. It is route-free, so it
keeps the cheap pricing. The obstacle is that it is a PER-COLUMN credit and the completion is
a per-triple family — one column can serve several out-triples at the same station, so the
credit needs a share-out rule (e.g. `beta * delta_j / M` with `M` a bound on out-triples per
column) to stay sound.

Worth trying, but note the ceiling: even crediting all of `beta * delta_j` leaves
`gamma_pj > 0` where the truth is exactly 0, so this buys a constant factor on the
coefficient mass, not the sparsity that makes the true cuts work. The measured `mass/const`
would have to fall from 1.80 to ~0.22 to match, and `delta_j` alone cannot do that
(0.25-0.85 of alpha credited leaves 0.15-0.75 charged, i.e. `mass/const` ~0.3-1.4). Read
that as a factor-of-2-to-5 improvement at best, on an oracle that is currently 28% short at
n=20 — probably not enough to change the frontier, and cheap enough to measure before
believing either way.
