# BendersY Premature-Convergence / Stale-Cut Soundness Gap
*2026-07-15, follow-up to notes/2026-07-14_nearest_open_solver_alignment.md*

## Status: FIXED. `BendersSolver(reprice_subproblem=true)` closes the soundness gap; verified on
every fixture below (synthetic and real-data, `max_stops` 3/4/5). See "Repricing fix" section.

## Repricing fix (this session, after the sections below)

The two "not started" directions at the bottom of this note (verify pool relevance before
trusting a cut / do genuine pricing directly on the subproblem LP) turned out to both point at
the same missing piece: `_solve_nearest_open_y_subproblem_lp` trusted `shared_pool` outright
instead of verifying it against its *own* dual structure. `shared_pool` was proven complete only
for `_solve_fixed_route_covering_by_cg`'s narrower, fixed-assignment formulation (single pair per
request, restricted to that iteration's open stations) — not for
`_build_nearest_open_y_subproblem_lp`'s broader one (free `x` over every globally feasible pair,
all `data.n_stations` as route nodes). A pool complete for the narrower problem has no guarantee
of being complete for the broader one whose duals actually drive the cut.

**Fix:** `BendersSolver(reprice_subproblem=true, max_reprice_rounds=20)`. After each subproblem
LP solve, extract its own covering-constraint duals
(`_extract_nearest_open_y_subproblem_coverage_duals`, aggregating the LP's per-`(request,pair)`
duals into `(j,k,s)`-keyed `sigma` exactly like the main master's convention) and run genuine
label-setting pricing against them (`_solve_nearest_open_y_subproblem_lp_with_repricing`,
mirroring `generate_aggregate_od_route_columns`'s own pricing round, including its
`pricing_exhausted`/`:optimality_proven`-style completeness tracking). If pricing finds any
negative-reduced-cost column, fold it in, re-solve, repeat until pricing is genuinely exhausted.
`@warn`s every time it finds something — that itself is the diagnostic signal (a nonzero find
means the pool was *not* actually complete for this subproblem, regardless of whether the
underlying cause is genuine dual degeneracy at the LP's optimal vertex or just plain pool
incompleteness from the narrower priming CG).

**Verified fixed, not just mitigated**, on:

| fixture | max_stops | plain BendersY | BendersY + repricing | ground truth (DirectSolver/CG/BendersXY) | columns found by repricing |
|---|---|---|---|---|---|
| synthetic 5-station | 3 | 3.5 (wrong) | **0.8** ✓ | 0.8 | 3 |
| Zhuzhou `{21,40,48,158,196,202}` | 3 | 352.222, open `{21,40,48,196,202}` (wrong) | **179.712**, open `{21,40,158,196,202}` ✓ | 179.712 | 32 |
| Zhuzhou `{21,40,48,158,196,202}` | 4 | 334.250 (wrong) | **172.570** ✓ | 172.570 | 58 |
| Zhuzhou `{21,40,48,158,196,202}` | 5 | 327.094 (wrong) | **161.741** ✓ | 161.741 | 85 |

Every case: repricing lands exactly on ground truth (matches DirectSolver/standalone CG/BendersXY
to full precision), and plain BendersY converges to the *same* wrong open set
(`{21,40,48,196,202}`, closing 158 instead of 48) every time it's wrong. Columns-found grows with
`max_stops` (32→58→85) — the effect is not a `max_stops=3` artifact; a larger route search space
gives the narrower priming CG's pool more room to miss columns the broader subproblem LP needs.
Full test suite (651 passed / 2 broken pre-existing / 0 errors) passes unchanged with the new
flags added (both default `false`, so existing behavior is untouched unless opted in).

**Caveat carried forward, not resolved by this fix:** the same category of gap — "a completeness
proof for the relaxation you searched isn't a completeness proof for the harder problem you
actually care about" — applies in a different, more classical form to standalone CG itself
(`run_aggregate_od_route_column_generation`): its own exhaustive-pricing proof
(`cg_stop_reason == :optimality_proven`) only certifies LP-relaxation completeness, not IP
completeness, before the final MIP resolve on the discovered pool. It matched ground truth
exactly on every fixture tested here, but that's corroborated by DirectSolver's independent
brute-force enumeration (confirmed genuinely exhaustive, not capped: 69/231/710 routes found at
`max_stops` 3/4/5 against a 20,000-route cap) on small instances (≤6 stations, ≤13 requests), not
formally guaranteed the way BendersY-with-repricing now is. Don't assume it generalizes to larger
instances without the same kind of cross-check.

## Background

After `notes/2026-07-14_nearest_open_solver_alignment.md`'s enumerator fix (DirectSolver/CG/
BendersXY now agree on all fixtures), `BendersSolver{BendersY}` remained the one solve path that
still diverges — sometimes to a genuinely wrong `y`. This note picks that up.

## Reproduction

6-station real-data fixture (chosen because the earlier real-data test2_zone_proximity fixture no
longer reproduces the bug post-enumerator-fix): a k-medoid Zhuzhou cluster
`{196, 40, 202, 158, 48, 21}`, subset from
`../Data/real_world_test_cases/zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20/sample_09_2025-03-03_11_15_midday_low`
(low demand density: 13 orders, 5 distinct OD pairs). `AggregateODRouteModel(5;
assignment_policy=NearestOpenAggregateODAssignmentPolicy(style), max_walking_distance=500.0,
route_regularization_weight=0.1, max_stops=3, max_wait_time=3600.0, detour_factor=2.0)`.
Reproduces identically under both `:gamma_chain` and `:big_m_nearest`.

- DirectSolver / standalone CG / BendersXY: **179.712**, open `{21,40,158,196,202}`.
- BendersY: **352.222**, open `{21,40,48,196,202}` (closes 158 instead of 48) — a real, ~2x
  suboptimal answer, not just a reporting artifact. Confirmed self-consistent: an independent
  fixed-`y` exhaustive re-solve at BendersY's chosen `y` also gives exactly 352.2215153337724.

Iteration trace (3 total iterations to "convergence"):

| iter | lower_bound (master, sum θ) | true v_hat at that y_hat | cuts added |
|---|---|---|---|
| 1 | 0.0 | 1309.02 (bad y) | 1 optimality cut |
| 2 | 1304.40 | — (infeasible y_hat) | 1 feasibility cut |
| 3 | 1304.40 | **352.22** (the true optimum!) | **0 — "converged"** |

## Root cause

`_run_aggregate_od_route_nearest_open_benders_y`
(`src/opt/optimize/aggregate_od_route_covering.jl`) derives each optimality cut from
`_solve_nearest_open_y_subproblem_lp`, an LP with `y` fixed via equality to that iteration's
`y_hat`, using a column pool (`v(y)`, the *restricted*-pool covering-cost function). Standard
Benders theory says a subgradient tangent to `v` at `y_hat` is a valid global underestimator of
`v` everywhere (by convexity) — but the master needs a bound on `V(y)`, the *true* value function
using the complete route universe. Since `v(y) >= V(y)` always (fewer columns ⇒ cost can't
decrease), and equality only holds at the specific `y_hat` where CG just proved the pool
complete, a cut valid for `v` is **not** thereby valid for `V` at any other `y`. Concretely: at
`y_hat_3`, iteration 1's stale cut (derived from a 7-column pool that was never asked to consider
`y_hat_3`'s stations at all) evaluates to `1304.40`, which is **greater than** the independently-
verified true optimum `V(y_hat_3) = 352.22` — a lower bound exceeding the true value it's
supposed to underestimate is a direct, checkable proof the cut is invalid. The stopping check
(`cuts_added_this_iteration == 0` ⟺ `theta_hat[cut_id] >= v_hat - tol`) treats this stale,
invalid cut as proof of optimality and returns immediately.

Cross-referenced `../../exploration/BendersStationSelection.jl` (the reference package this
Benders design was adapted from) at the user's request: it uses the **same** structural
stopping check (`converged = theta_pred >= v_hat - benders_tol` in
`solve_nearest_open_benders_y`, `src/opt/optimize/nearest_open_benders_y.jl`) — it also computes
and *logs* `outer_gap_abs = best_ub - theta_pred` as a diagnostic but does **not** gate
convergence on it. So this is not something StationSelection.jl's port uniquely broke; the
reference design has the same latent structural vulnerability, as far as can be told from
reading its code (not independently reproduced there).

**Important clarification from this session's back-and-forth (do not re-litigate without
re-deriving):** reframing the stopping rule as an explicit `lower_bound (master's global
objective) >= best_ub (best incumbent)` check was considered and is **not**, by itself, a fix —
it is mathematically equivalent to the current per-cut-group check when there's one cut group
(`theta_hat` at the master's own chosen `y_hat` **is** `objective_value(master)` by construction,
since the master minimizes `sum(theta)`). The real defect is that the "LB" side is corrupted by
an unsound cut, not that the wrong comparison is being made. A genuine fix has to make cuts
*valid* at derivation time, not just reformulate how validity gets checked.

## What's landed (mitigations, not fixes)

1. **Shared/growing column pool** (`shared_pool` in
   `_run_aggregate_od_route_nearest_open_benders_y`, threaded through
   `_solve_fixed_route_covering_by_cg` via a new `seed_columns` kwarg): mirrors
   `BendersStationSelection.jl`'s persistent `CompatibilitySetPool`. Verified via trace that it
   genuinely accumulates (7→10 columns across 3 iterations) and makes each iteration's *own*
   `v_hat` honest/current. Does **not** fix the bug above, because old cuts, once added, are
   never re-derived even after the pool that could correct them has grown — confirmed via the
   iteration trace (pool did grow, wrong answer persisted regardless).
2. **Runtime correctness assertions** (added per explicit user request, not just documented
   reasoning — all confirmed passing, i.e. none of these was actually the bug):
   - `_assert_x_matches_nearest_open` (`aggregate_od_route_covering.jl`): after solving
     `_solve_nearest_open_y_subproblem_lp`, verifies exactly one `x` per request is ≈1 and
     matches the independently-computed nearest-open pair from `_fixed_assignments_from_y`.
   - `assert_endpoint_chain_near_binary` (`src/opt/constraints/aggregate_od_route.jl`, exported):
     generic check that every `zp`/`zd` endpoint-chain (`:big_m_nearest`) indicator is within
     `atol` of 0/1, reading `m[:nearest_endpoint_chain_cache]` (the cache key shared by
     `_endpoint_chain_variable!` and `_master_endpoint_chain_variable!`, so one helper works
     everywhere). Wired into **all four** places these variables get solved: `_run_opt_impl`
     (DirectSolver), CG's `final_m` resolve, BendersXY's master, and — after adding a new
     `:big_m_nearest` branch to `_build_nearest_open_y_subproblem_lp` (previously it *always*
     silently used the `:gamma_chain` encoding regardless of `feasibility_cut_style`, via the new
     `_nearest_open_y_subproblem_endpoint_chain_variable!`, a continuous-relaxation counterpart
     to the master's binary chain variable, needed since this LP must stay a pure LP to extract
     duals) — BendersY's own subproblem LP.
   - Verified end-to-end under `:big_m_nearest` on the same fixture: zero assertion failures
     anywhere, and results match `:gamma_chain` exactly (179.712/179.712/352.222/179.712) —
     confirms the bug is unrelated to chain-style encoding and is specifically about column-pool
     completeness across different `y_hat`'s.
3. Full test suite: 651 passed / 2 broken (both pre-existing, expected `BendersY` `@test_broken`
   on the *synthetic* fixture only) / 0 errors / 0 failures after all of the above.

## Open question / next step — RESOLVED, see "Repricing fix" section above

This section is kept for history. Of the two directions discussed, the second ("do genuine
pricing directly on the subproblem LP") is what got implemented, as
`BendersSolver(reprice_subproblem=true)` — see above for the fix, verification table, and the
remaining standalone-CG caveat it does *not* resolve.

Until `reprice_subproblem=true` is made the default (not done — it's opt-in due to the extra
pricing cost per iteration), continue to not trust `BendersY`'s objective on any new fixture
without either passing `reprice_subproblem=true` or cross-checking against `BendersXY`/CG first.
