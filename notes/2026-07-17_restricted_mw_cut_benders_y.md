# Restricted (fixed-pricing-dual) Magnanti-Wong Cut for BendersY

*2026-07-17*

## Status: implemented, gated behind `BendersSolver(cut_derivation=:restricted_mw_fixed_pi)`.
Default remains `cut_derivation=:standard`, byte-identical to pre-existing behavior. A third
mode, `:zero_completion`, solves the same completion LP with a zero objective, as a baseline
for comparison.

This is **not** a full Magnanti-Wong procedure over the entire full-subproblem dual optimal
face — it fixes the route-covering dual block `pi` at the vector certified by exact pricing on
the *restricted*, fixed-assignment route-covering problem `R(x_bar)`, then only optimizes the
*remaining* dual blocks (z/x duals) against a core point. It is **not claimed to be globally
Pareto-optimal**. Referred to in code/logs as `restricted_MW_fixed_pi`.

## A. Audit: mapping the spec's math to the actual JuMP code

Everything below is scoped to `AggregateODRouteModel` + `NearestOpenAggregateODAssignmentPolicy`
under `BendersSolver(decomposition=BendersY())`, which is the only path this feature targets.
`:pair_chain` and `allow_walk_only=true` are explicitly unsupported by the new cut mode (see
"Scope restrictions" below) — the existing `:standard` cut is unaffected and still works for
those cases.

### y-only structural master region `Y`

The permanent, iteration-independent structural region is:

```
sum_j y_j = l                                              -- @constraint(master, sum(y) == model.l)
0 <= y_j <= 1                                               -- @variable(master, y[1:n], Bin) (LP relax bounds)
sum_{j in candidates(endpoint, side)} y_j >= 1               -- NOT eagerly built by the master;
                                                                 in production this is only added
                                                                 lazily, as a feasibility cut, by
                                                                 _add_endpoint_open_feasibility_cut!
                                                                 the first time infeasibility is
                                                                 hit for that endpoint.
```

`candidates(endpoint, side) = _nearest_open_endpoint_candidates(data, endpoint, max_walking_distance, side)`
(`src/opt/constraints/aggregate_od_route.jl:357`) depends only on precomputed walking-cost data,
not on any Benders iterate — so, unlike the lazily-discovered feasibility cuts the running
BendersY loop adds, this row set is fully derivable up front from the request list. The core-point
construction (Section B) builds this row set **eagerly** for every distinct physical
`(side, endpoint)` touched by the request set being cut, rather than relying on whatever
feasibility cuts happen to have been discovered so far — this is what makes it a genuinely
*permanent* structural restriction rather than an artifact of solve history, matching the spec's
intent ("any other permanent station-location constraints").

### Nearest-open subproblem primal (the LP whose dual we complete)

For `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`, one physical endpoint role
`(side, endpoint)` gets one shared selector chain via `_endpoint_big_m_variable!`
(`aggregate_od_route.jl:173`), **cached and shared** by content key
`_endpoint_chain_key(side, sorted_candidates, sorted_costs)` — i.e. shared across every
request/scenario whose endpoint has the identical candidate/cost profile, not duplicated
per request or per scenario. Concretely per chain `c` (candidates sorted by cost, ties by id):

```
sum_idx z[idx] == 1                                          -- row (5), dual lambda[c] free
z[idx] <= y[station_idx]                                      -- row (6), dual mu[c,idx] >= 0
selected_cost <= cost[idx] + M[idx]*(1 - y[station_idx])       -- row (7), dual nu[c,idx] >= 0
  where selected_cost = sum_idx' cost[idx']*z[idx'], M[idx] = max_cost - cost[idx]
```

This is exactly the spec's `sum_j h[l,j] z[l,j] <= h[l,q] + M[l,q](1 - y_bar[q])` family, with
`l` = chain (a *role-qualified* endpoint, i.e. `(side, endpoint)` — pickup and dropoff chains
for the same physical station are always distinct variables, never shared).

Per-request coupling (`_add_nearest_open_endpoint_linked_x!`, `aggregate_od_route.jl:469`), one
copy per request `p = (s, o, d)` (**x IS duplicated by scenario** — every `(s,o,d)` with positive
demand gets its own `x` row, even if `(o,d)` repeats across scenarios) and per real (non-walk)
feasible pair `(j,k)`:

```
sum_pair x[p,pair] == 1                     -- row (1), dual alpha[p] free
x[p,(j,k)] <= zp[pickup_rank[j]]            -- row (2), dual rhoO[p,(j,k)] >= 0
x[p,(j,k)] <= zd[dropoff_rank[k]]           -- row (3), dual rhoD[p,(j,k)] >= 0
x[p,(j,k)] >= zp[..] + zd[..] - 1           -- row (4), dual sigma[p,(j,k)] >= 0
```

Coverage (`add_aggregate_od_route_coverage_constraints!` / the BendersY subproblem's own
`cover_cons`), one row per `(request, pair)` (**not** pre-aggregated by `(j,k,s)` — aggregation
into `sigma[(j,k,s)]` happens downstream in `extract_aggregate_od_route_coverage_duals`):

```
sum_r a[r,p,jk] theta[r] >= x[p,jk]         -- row (8), dual pi[p,jk] >= 0
```

### Audit checklist (spec Section A items)

1. **z duplicated by scenario?** No — shared by `(side, sorted-candidates, sorted-costs)` content
   key, independent of scenario or even of which physical endpoint asked for it (two distinct
   physical endpoints with an identical candidate/cost profile would share the *same* chain
   variables). The completion LP (Section E/G below) replicates this exact grouping via the same
   `_endpoint_chain_key` helper, rather than building one chain per physical endpoint — building
   the wrong (finer) grouping would make the completed dual infeasible for the *actual* shared
   primal, silently invalidating the cut.
2. **x duplicated by scenario?** Yes, one row set per `(s, o, d)` request tuple.
3. **Direct walking (`WALK_ONLY_PAIR`) present?** Only if `allow_walk_only=true`; this cut mode
   throws `ArgumentError` if so (see below) rather than modeling the extra `x_walk >= zp+zd-1`
   row, since it has no coverage row and a different completion structure the spec doesn't cover.
4. **Ties in nearest-station distance deterministically broken?** Yes —
   `sortperm(...; by=i->(costs[i], endpoints[i]))` in both `_endpoint_chain_variable!` and
   `_endpoint_big_m_variable!`, i.e. lexicographic (cost, station id).
5. **Zero-RHS route-covering rows removed before solving?** Yes, structurally: `RouteCoveringProblem`
   narrows `valid_jk_pairs[(o,d)]` to the single fixed pair
   (`_apply_route_covering_assignments!`, `aggregate_od_route_map.jl:172`), so only one coverage
   row per request exists in `R(x_bar)`'s build — the other, non-selected pairs' rows simply never
   get created (not pruned post hoc). Section D's zero-extension is exactly filling those back in
   as `pi_full = 0`.
6. **Any rows removed by explicit code rather than solver presolve?** Yes, same as (5) — this is
   `AggregateODRouteMap`/`RouteCoveringProblem` construction-time pruning, not presolve.

### Scope restrictions (deliberate, documented)

- Only `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)` is supported by the new cut modes.
  `:pair_chain`/`:endpoint_chain` have a different (or, for `:endpoint_chain`, differently-shaped
  but same-cost) selector structure; only `:big_m_nearest`'s explicit `M[idx]` Big-M form was
  audited and derived against the spec's row family. Requesting `:zero_completion` or
  `:restricted_mw_fixed_pi` with any other style throws `ArgumentError`.
- `model.allow_walk_only=true` is rejected for the same reason walk-only pairs have no coverage
  row and a different (collision) linking constraint not covered by the derivation below.
- The route-covering CG in Section C requires `solver.inner_solver isa ColumnGenerationSolver`
  (the `DirectSolver` inner-solver branch of `_solve_fixed_route_covering_by_cg` never produces
  LP duals at all, only a MIP solve over enumerated columns).

## B. Core point

`_y_master_core_point` (new file, Section B) builds the eager structural region described above
for the *full* request set (union of every cut group), then:

- **B1** solves `max s_i(y)` for every row (`y_j`'s lower/upper bound slack, each endpoint row's
  slack) over `Y_LP`; rows whose max slack is `<= affine_hull_tolerance` are recorded as
  always-tight (and, for bound rows, as a structurally-fixed `y_j`) and are *not* included in the
  B2 normalization.
- **B2** solves one LP: `maximize delta s.t. y in Y_LP, s_i(y) >= delta * s_i^max` for every row
  with positive max slack, `0 <= delta <= 1`. Diagnostics (`delta`, min normalized slack,
  fixed-variable list, always-tight row list) are returned alongside `y_core`.

This is computed once per `_run_aggregate_od_route_nearest_open_benders_y` call (cached across
iterations, per spec — the structural region does not depend on `y_hat`), not per iteration. B3's
dynamic `lambda_core` blending update is **not implemented** — the spec explicitly allows starting
with the static point only ("Start with the static max-min-slack core point for debugging").

## C/D. Certified route-covering duals and zero-extension

`_certified_route_covering_pi` runs `run_aggregate_od_route_column_generation` on the
`RouteCoveringProblem` induced by `(y_hat, assignments)` to `cg_stop_reason == :optimality_proven`,
then re-solves the LP relaxation once more on the final pool to extract per-`(request,pair)` raw
covering duals (not the `(j,k,s)`-aggregated `sigma` — the per-request row is what the spec's
`pi[p,j,k]` indexes), and runs one further exact label-setting pricing pass against those duals
(mirroring `_solve_nearest_open_y_subproblem_lp_with_repricing`'s certification pattern) to confirm
`min_r rc(r) >= -pricing_tolerance` before accepting them. `pi_full` zero-extends this over every
`(request, pair)` in `feasible_pairs[request]`, not just the retained (assigned) one.

## E-H. Completion LP, Phi, and the cut

Signs were re-derived independently from the primal rows above (not copied from the prompt) —
see inline derivation comments in `_restricted_mw_completion_lp` in the implementation file. They
match the spec's given equations exactly once the Big-M row is written in `>=` form with
`-selected_cost` on the left (the natural direction after negating the code's `<=`).

`_restricted_mw_optimality_cut` (Section H) verifies cut tightness at `y_hat` against `Q_bar` and,
for `objective_mode=:maximize_core`, also solves the `:zero_completion` baseline (a second,
separate small LP solve reusing the same `pi_full`/`Q_bar`) purely to verify the MW completion's
`Phi(y_core)` is not worse than the baseline's before returning the cut. If the completion LP
comes back infeasible or `INFEASIBLE_OR_UNBOUNDED`, the caller
(`_run_aggregate_od_route_nearest_open_benders_y`) falls back to the `:standard` subgradient cut
for that `(iteration, cut_id)` and logs a warning — this keeps the outer BendersY loop making
progress even on the iterations where the restricted completion doesn't happen to be feasible,
which the spec anticipates as a real possibility ("If the completion LP is infeasible...").

## Empirical validation on real Zhuzhou data (10- and 15-station samples)

Two bugs were found and fixed only by testing on real data beyond the tiny synthetic fixture
(`scripts/check_zhuzhou10_restricted_mw_convergence.jl`,
`scripts/check_zhuzhou_restricted_mw_timing.jl`, `scripts/check_zhuzhou_dual_value_study.jl`,
`zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20/sample_09_...`):

1. **Gating bug**: the `theta_hat < v_hat - tol` decision (is a new cut even needed?) used
   `v_hat` from the possibly pool-incomplete, un-repriced `_solve_nearest_open_y_subproblem_lp`.
   An incomplete pool can only *inflate* `v_hat`, never deflate it, so an inflated `v_hat` made
   the master believe it had already converged before the restricted-completion cut-derivation
   code ever ran — reproducing the *exact* pre-existing `reprice_subproblem=false` premature-
   convergence bug bit-for-bit (696.672462535125, identical `y`, identical iteration count) even
   under `cut_derivation=:restricted_mw_fixed_pi`. Fixed by `_certified_qbar`: for non-`:standard`
   modes, `v_hat` is tightened to `min(v_hat, Q_bar)` using Section C's independently-certified
   `R(x_bar)` value *before* the gating decision, not only when actually deriving a cut.
2. **Double-counted walking cost**: `Q_bar` was computed as
   `sum(c_walk*x_bar) + certified.r_value`, but `certified.r_value` (the `RouteCoveringProblem`
   LP's own objective value) *already* includes the walking-cost terms — `set_aggregate_od_route_objective!`
   is the same objective builder every `AggregateODRouteModel` build uses, and `x` is fixed
   in `RouteCoveringProblem`, not omitted from the objective. Adding the walking-cost sum again
   double-counted it (`Q_bar` came out ~1.8x too high on the 15-station fixture: 1272.53 instead
   of 696.67). Invisible on the tiny synthetic test fixture because that fixture's optimal
   assignment happens to have zero walking cost on both requests. Fixed: `Q_bar = certified.r_value`
   directly.

With both fixes, on `sample_09_2025-03-03_11_15_midday_low` subsetted to `n_stations ∈ {10, 15}`,
`max_stops ∈ {3,4,5}`, `allow_walk_only=false`: **`:zero_completion` and `:restricted_mw_fixed_pi`,
both with `reprice_subproblem=false`, converge to the exact repriced-ground-truth objective and
`y`** on every case tried (whereas `:standard` without repricing reproduces the known-bad,
premature-convergence answer every time, as expected — that mode is intentionally untouched).

Wall time and iteration count (`max_stops=4`, `l=5`/`l=7`):

| fixture | mode | objective | iterations | wall time |
|---|---|---|---|---|
| n=10 | standard + repricing | 222.9085 | 12 | 21.5s |
| n=10 | zero_completion, no repricing | 222.9085 (match) | 12 | 8.2s (0.38x) |
| n=10 | restricted_mw_fixed_pi, no repricing | 222.9085 (match) | 12 | 0.77s (0.036x) |
| n=15 | standard + repricing | 1403.067 | 75 | 130.7s |
| n=15 | zero_completion, no repricing | 1403.067 (match) | 117 | 84.2s (0.64x) |
| n=15 | restricted_mw_fixed_pi, no repricing | 1403.067 (match) | **53** | 18.4s (0.14x) |

`restricted_mw_fixed_pi` is both the fastest *and* (on n=15) needs *fewer* Benders iterations
than repriced `:standard` — consistent with classic Magnanti-Wong intuition: maximizing the
completed cut at a representative core point picks a better point on the dual optimal face than
whatever vertex a plain LP solve happens to return, giving a tighter cut per iteration.
`zero_completion` (any feasible completion, no core-point optimization) is correct but needs
*more* iterations than repriced `:standard` on n=15 (117 vs 75) — it fixes the *soundness* gap
(certified `pi`, valid weak-duality cut) but not the *quality* gap a smarter completion closes.

On n=15, Section C's own CG occasionally still needed a fallback: `_certified_route_covering_pi`'s
post-hoc certification pricing pass found columns beyond the CG-converged pool on ~15 of ~150
combined iterations across the two non-standard modes (`cg_stop_reason==:optimality_proven` from
`run_aggregate_od_route_column_generation` is *not* a perfectly reliable completeness guarantee
at this size — consistent with the pre-existing caveat in
notes/2026-07-14_nearest_open_solver_alignment.md that CG's own exhaustive-pricing claim is only
independently cross-checked against `DirectSolver` on small fixtures, not proven in general).
The `certification_already_failed` short-circuit (added after the first, unoptimized timing run
logged the same CG failure twice per occurrence — once from the gating step's own
`_certified_qbar` call, once again from `_restricted_mw_optimality_cut`'s redundant retry) avoids
paying for the doomed CG solve twice; either way, on these iterations the loop safely falls back
to the `:standard` cut and the run still reaches the correct final answer.

### Dual-value study: how different are the x_{p,j,k} duals, and does a shared (j,k) matter?

`scripts/check_zhuzhou_dual_value_study.jl`, same n=15 fixture at its optimal `y`, compares the
raw per-`(request,pair)` coverage-row duals from (a) the plain (un-repriced) nearest-open
subproblem LP, (b) the repriced/certified nearest-open subproblem LP, and (c) `R(x_bar)`'s own
certified per-request duals (zero-extended) -- all three give the *same* optimal objective value
(1403.067) but are duals of different LPs (a/b are the broader joint LP with free `x`; c is the
narrower fixed-assignment LP), so they need not (and generally don't) coincide:

- **At the active pair (`x_bar=1`)**: reasonably close but *not* identical --
  max`|plain-Rxbar|`=18.66, max`|repriced-Rxbar|`=8.63 across 15 active rows. Repricing narrows
  the gap to R(x_bar)'s value but doesn't eliminate it (still genuine LP dual degeneracy).
- **At inactive pairs (`x_bar=0`)**: wildly different -- max`|plain-Rxbar|`=248.25,
  max`|repriced-Rxbar|`=36.43 across 302 inactive rows. Mechanism: when a pair's column pool has
  *zero* routes serving it, its coverage row `sum_r a[r,jk] theta[r] >= x[..]=0` degenerates to
  `0 >= 0` -- trivially tight with both sides zero, so the row is a *degenerate* binding
  constraint and the solver can report an essentially arbitrary nonzero dual there without
  violating complementary slackness or changing the primal optimum. `pi_full`'s zero-extension
  (Section D) is not "one dual estimate among several" here -- it is the only value consistent
  with the row not existing at all in `R(x_bar)`'s actual build (that specific `(request,pair)`
  row is never constructed there), so using anything else would be unjustified, not just
  imprecise.
- **Shared `(s,j,k)` assignments** (two fixtures found: one triple, one pair): the dual credit
  went entirely to *one* request in the group (e.g. `(1,4,13)` got 11.00/15.44/21.53 across the
  three methods while `(1,14,13)` and `(1,5,13)`, sharing the same `(4,13)` pair, got exactly
  `0.0` in *all three* computations) -- a "winner take all" split, consistent (not
  arbitrarily different) across plain/repriced/R(x_bar), and its sum always exactly reproduces
  the `(j,k,s)`-aggregated `sigma` pricing already relies on. **This does not need special
  handling and does not affect validity**: every dual variable in the completion LP (`alpha`,
  `rhoO`, `rhoD`, `sigma`, and the fixed `pi_full`) is indexed per `(request, pair)`, never
  aggregated by `(j,k,s)`, so each request's own x-dual-feasibility constraint
  (`alpha[p] - rhoO - rhoD + sigma - pi_full[p,pair] <= c_walk[p,pair]`) is satisfied
  independently of how the credit happened to split among co-assigned requests elsewhere. The
  *pricing* aggregation (`extract_aggregate_od_route_coverage_duals`'s `sigma[(j,k,s)]`, summed
  over all requests sharing that pair) is the one place the split does need to sum correctly --
  and it does, by construction (`aggregate_od_route_coverage_sigma` sums the raw duals it's
  handed, regardless of how a solver chose to split them).

  Concretely verified this "winner take all" split does not threaten completion-LP feasibility
  (not just argued from the constraint's free-variable structure): for the `(1,4,13)` /
  `(1,14,13)` / `(1,5,13)` group above, `pi_full` gives `21.528` / `0.0` / `0.0`, yet the solved
  completion LP's own `alpha[p]` came out `99.17` / `172.51` / `185.81` -- the two requests with
  *zero* `pi` credit got the *largest* `alpha`, because `alpha[p]`'s only constraint is
  `alpha[p] <= c_walk[p,pair] + rhoO[p,pair] + rhoD[p,pair] - sigma[p,pair] + pi_full[p,pair]`,
  and `rhoO`/`rhoD`/`sigma` are separately-chosen free/nonnegative variables per request,
  independent of how `pi` split -- `alpha[p]` is free (no lower bound), so a smaller
  `pi_full[p]` can only tighten an otherwise-unconstrained upper bound, never cause infeasibility.
  Tightness held to `6.8e-13` with this exact split in play. The split is *valid* but not
  necessarily *optimal* -- a fairer split could in principle let the maximization find an even
  larger `Phi(y_core)` (a tighter cut); not rebalanced here since the empirical results (exact
  match to ground truth, fewer iterations than repricing on the 15-station case) already look
  strong without it. A candidate future refinement, not implemented.

## Known limitations / not implemented

- B3 dynamic core-point blending.
- Multi-cut groups whose requests reference zero endpoints (shouldn't happen in practice, but not
  specially handled beyond "empty sums are zero").
- Full Section J iteration-level stat table is implemented in `benders_rows` best-effort (the cut
  mode, completion status, Phi values, and pricing-call count during completion are logged; not
  every single named diagnostic field in the spec has a dedicated CSV column).
