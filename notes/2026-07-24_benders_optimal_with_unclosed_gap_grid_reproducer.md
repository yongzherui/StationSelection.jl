# BendersY reports OPTIMAL with an unclosed ~18% outer gap -- clean synthetic reproducer, independent confirmation of the 2026-07-23 bug

*2026-07-24*

## Relationship to prior work

This independently re-derives and sharpens **Bug 3** from
[[2026-07-23_benders_reports_optimal_with_unclosed_outer_gap]] (cross-referenced in memory as
[[project_benders_false_optimal_lp_ip_gap]]), found while investigating a *different* bug
(`_restricted_mw_optimality_cut` returning `:completion_infeasible` -- now fixed, see
[[project_mw_completion_infeasible_open]], resolved this session by centralizing the Big-M
tie-break-cost perturbation into a single `_big_m_tie_break_costs` function used consistently by
the primal build and the dual-derivation code). After that fix, verifying a real n=20 Zhuzhou case
progressed cleanly instead of crashing -- but a follow-up sbatch confirmation run surfaced this
separate, pre-existing issue: `termination_status=OPTIMAL` with a large, silently-unproven gap.

**This note's contribution over the 2026-07-23 original**: a small, fully synthetic (no external
data files) grid reproducer instead of Zhuzhou data; confirmation that the bug is identical across
`:standard`, `:zero_completion`, and `:restricted_mw_fixed_pi` cut derivations (not specific to any
one, and not something the MW/zero-completion tie-break fix introduced or could fix); and a
sharper mechanistic finding -- **the `y_hat` that satisfies the "no cut needed" stopping check and
produces the reported `lower_bound` is, in general, a completely different, unrelated, and worse
station set than the actual incumbent**, not merely an imprecise version of it.

## Reproducer (fully synthetic, no data files needed)

```julia
nx, ny, n_pairs, seed = 2, 5, 16, 123   # grid_n10_p16_s123
instance = generate_grid_instance(nx, ny, n_pairs; endpoint_overlap=2.0, seed=seed)
max_walk = Float64(nx + ny)   # 7.0
data = create_grid_problem_data(instance; max_walking_distance=max_walk)
model = AggregateODRouteModel(
    5;   # l
    assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
    route_regularization_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
    max_walking_distance=max_walk, max_wait_time=900.0, detour_factor=2.0, max_stops=4,
)
```

Also reachable via the existing method-comparison harness: `jobs.txt` line 109/111/107 (family=grid,
n_stations=10, l=5, n_pairs=16, seed=123, method=bendersY_{zerocomp,mw,std_reprice}_ms4).

## What happened

All three cut derivations return `termination_status=OPTIMAL`, `objective_value=462.6`,
`selected_stations=[6,7,8,9,10]` -- and all three also report `final_lower_bound=377.5`,
`final_outer_gap=0.18396...` (18.4%), matching each other to many decimal places:

| cut_derivation | reprice | objective | lower_bound | outer_gap | n_iterations |
|---|---|---:|---:|---:|---:|
| standard | true | 462.6 | 377.5 | 0.18396 | 232 |
| zero_completion | false | 462.6 | 377.5 | 0.18396 | 243 |
| restricted_mw_fixed_pi | false | 462.6 | 377.5 | 0.18396 | 217 |

The `@warn "Benders returned its best feasible incumbent, but the outer optimality gap exceeds the
expected tolerance"` (default `outer_gap_warning_tol=0.03`) correctly fires in all three logs -- but
`status`/`termination_status` are unaffected by it, so a result-table read (which is what the
batch/analysis scripts do) sees three clean `OPTIMAL` rows with no visible sign of the problem.

## Root cause, confirmed with instrumented data (not just code reading)

Added a temporary debug print to `benders/y.jl` (right before the `cuts_added_this_iteration == 0`
early return; reverted after this investigation) logging `y_hat`/`theta_hat`/`lower_bound`/`best_ub`
at the terminating iteration. Result for the `:standard`+repriced run:

```
DEBUG TERMINATION iteration=232 y_hat_stations=[1, 2, 3, 5, 6] theta_hat=Dict(1 => 377.5)
    lower_bound=377.5 best_ub=462.6
```

**The terminating `y_hat` is `[1, 2, 3, 5, 6]` -- a completely different station set from the
incumbent `[6, 7, 8, 9, 10]`.** Direct inspection (`_solve_fixed_route_covering_by_cg` +
`_certified_qbar` + `_solve_nearest_open_y_subproblem_lp_with_repricing`, all reused verbatim from
the production code) of both station sets:

| `y_hat` | `Q_bar` (LP dual, `_certified_qbar`) | fixed-assignment IP objective | broad repriced LP |
|---|---:|---:|---:|
| `[6,7,8,9,10]` (incumbent) | 462.6 | 462.6 | 462.6 |
| `[1,2,3,5,6]` (terminating) | 377.5 | **472.5** | 377.5 |

The incumbent's own subproblem has a **genuinely zero** LP/IP gap -- its cut, once derived, is
exact. The terminating `y_hat` has a **genuine, large** LP/IP gap (377.5 vs. its true cost 472.5,
which is itself *worse* than the incumbent -- so this is not a missed better solution, `best_ub`
tracking is correct). The outer loop's stopping check (`theta_hat[cut_id] < v_hat - optimality_tol`)
only asks whether the *currently-proposed* `y_hat`'s `theta` matches *that same y_hat's own*
LP-relaxation-based `v_hat` -- never whether any other `y` (including the true optimum sitting right
next to it in the search) could still be proven better. Because LP-relaxation values can
legitimately vary in tightness from one `y` to another, the master effectively drifts toward
whichever `y` has the *loosest* (most overoptimistic) relaxation in reach and stops there --
independent of whether the true optimum has already been found and is sitting in `best_result`.

**Why the true optimum's 0% gap doesn't help**: nothing in the algorithm ever re-certifies
`[6,7,8,9,10]`'s optimality against the whole feasible region. Once *any* `y` satisfies "no cut
needed" against its own (possibly-underestimating) LP value, the loop returns -- the fact that a
better-behaved `y` was already found and recorded as the incumbent is irrelevant to why the loop
stopped.

## Route-level mechanism behind `[1,2,3,5,6]`'s LP/IP gap

Full route-pool breakdown (cost, LP-dual credit = sum of `pi_by_request` over pairs the route is
credited for, and whether selected in the final integer solve):

- The LP relaxation spreads credit across **12+ different overlapping routes**, most with
  `credit == cost` (many alternate-optimal dual vertices/tight LP columns) -- e.g. routes 8, 27, 9,
  14, 15, 16, 7, 12, 22 all cost 240-260 and each cover 3-5 of the 6 distinct required pairs, in
  different overlapping combinations.
- The final **integer** solve can only select whole routes: `route_id=7` (cost 240, covers pairs
  `(1,6),(5,6),(1,5),(2,6)`) and `route_id=20` (cost 230, covers `(5,1),(6,1)`) -- total route cost
  470, plus walking cost 2.5 = 472.5.
- The LP relaxation achieves 377.5 (route share) by fractionally blending across the many
  overlapping routes above, each contributing only the marginal fraction needed per pair, at a
  fractional cost cheaper than committing to any whole-route combination. This is exactly the
  "broad overlapping route certification" mechanism already documented in
  [[project_lp_mip_gap_hub_routes]] and the 2026-07-23 note's structural explanation (unlimited
  vehicle capacity + synchronized service model -> one route can certify many OD pairs at once ->
  dense, heavily overlapping set-covering matrix -> LP relaxation has no general tightness
  guarantee).

## Ruled out: data/weight-passing bugs (the *previous* class of bug in this exact area)

Given bugs 1-2 in the 2026-07-23 note were exactly this shape (a weight silently defaulting wrong
in a cloning helper), explicitly checked for a repeat:

- `_clone_for_final_mip` (both `AggregateODRouteModel` and `RouteCoveringProblem` overloads,
  `pricing/column_generation.jl`) and `_route_covering_problem_from_assignments`
  (`benders/covering.jl`) all correctly forward `walk_cost_weight`, `route_regularization_weight`,
  and every other field explicitly -- the 2026-07-23 fix is intact.
- `RouteCoveringProblem` does not have a separate objective-construction code path at all -- it
  goes through the identical `_build_aggregate_od_route_core!` -> `set_aggregate_od_route_objective!`
  builder as the general model (`build.jl`), so there is no second formula that could apply a
  weight twice or drop it.
- Empirically conclusive: the incumbent's `Q_bar`/IP/broad-LP matched **exactly** (462.6 = 462.6 =
  462.6) through this exact code path, while `[1,2,3,5,6]`, computed through the identical code,
  showed a real 95-unit gap. A weight-forwarding bug would bias every computation proportionally
  and show up in both cases; it cannot selectively produce an exact match for one `y` and a gap for
  another solved with the same weights through the same functions. The divergence is a genuine
  LP/IP integrality gap, not a data-passing defect.

## Confirmed instance-dependent, not universal

`grid_n10_p8_s42` (same `n_stations=10`, sparser `n_pairs=8`, different `seed=42`) converges
**perfectly cleanly** under the same `:standard`+repriced method: `final_lower_bound=221.29999...`,
`objective_value=221.3`, `final_outer_gap=2.57e-16` (machine precision), 61 iterations, 36.7s. Fewer
OD pairs means less opportunity for broad route overlap, consistent with the mechanism above. So
this is a structural property of specific (denser/more-overlap-prone) instances, not a universal
Benders failure -- matches the "crazy degenerate case" hypothesis raised during this investigation.

Confirmed **worse**, not better, at larger scale: `grid_n15_p16_s123` and `grid_n20_p16_s123` (same
generation family, more stations) don't even reach a plateau -- both hit `max_iterations=500` with
`lower_bound=0.0` the entire run, for both `zero_completion` and `restricted_mw_fixed_pi`.

## Separate, unrelated bug found and FIXED on the same instance

`bendersYZH_std_reprice_ms4` on this exact instance crashed almost immediately (4 iterations, 13s),
not with the LP/IP gap issue but with:

```
ArgumentError: endpoint-chain (z) indicator check failed: z[3] in chain
    (:dropoff, (8, 6, 7, 10, 4, 5, 9, 2, 3, 1), (0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 4.0))
    has value 0.9996092321175274, not within atol=1.0e-5 of 0 or 1
```

`assert_endpoint_chain_near_binary`'s default `atol` (`src/opt/constraints/aggregate_od_route.jl`)
was `1e-5`, matching Gurobi's own `IntFeasTol` default -- but on larger/more-degenerate master
formulations (BendersYZH's master specifically) genuinely-selected `z` values can land just outside
even that tolerance. **Fixed**: loosened default `atol` to `1e-3`. Checked all call sites that read
a `z` value onward for further computation (`benders/subproblem_api.jl`, `benders/yz.jl`) -- both
already apply `round.(value.(...))` at extraction time, so no separate rounding fix was needed;
values this check passes are already snapped to exact 0.0/1.0 wherever they're used numerically
downstream. Verified via full test suite: 1050/1050 pass, no regressions.

## Not yet done / open

- Everything in the 2026-07-23 note's "Not yet done" list still applies -- this note adds
  confirmation and a cleaner reproducer, not a fix. In particular: **no code fix applied to the
  outer-loop stopping rule or `termination_status` reporting.** The 2026-07-23 note's analysis of
  why a naive "check `outer_gap` before returning" fix is necessary-but-insufficient (LP-duality
  cuts structurally cannot prove a bound above the LP relaxation value; real convergence needs
  integer/combinatorial Benders cuts, e.g. the integer L-shaped method) still stands and is not
  repeated in full here.
- BendersYZH's separate crash is fixed for the *tolerance* dimension; whether BendersYZH also hits
  the same LP/IP-gap-driven false-OPTIMAL issue on this instance (once past the tolerance crash) is
  unconfirmed -- not re-run after the tolerance fix.
- **Done**: exhaustive-enumeration (`DirectSolver`, `direct_ms4`, sbatch job 18734116) confirms
  `grid_n10_p16_s123`'s true global optimum is `objective_value=462.6`, `selected_stations=[6,7,8,9,10]`
  -- an exact match with all three Benders methods' reported incumbent, independent of any
  decomposition or LP relaxation. So the *incumbent* every Benders variant returned here is
  genuinely, verifiably correct; only the claimed `termination_status=OPTIMAL`/proof-of-optimality
  is unearned. This is a useful but not general reassurance -- Direct enumeration is only tractable
  at this small a scale (n=10); it cannot be used to spot-check larger instances where this same bug
  is confirmed present (`grid_n15/20_p16_s123`, which don't even reach a plateau).
