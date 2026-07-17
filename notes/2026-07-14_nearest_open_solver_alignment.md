# Cross-Solver Alignment Under NearestOpenAggregateODAssignmentPolicy
*2026-07-14, session paused pending visual inspection*

## UPDATE (later same-day session): root cause found — the enumerator is the one that's wrong

Re-opened this investigation using the `scripts/test_case_generation` test2-test6 synthetic
benchmark families (real geometry, hand-designed hypotheses, already generated under
`../Data/test{2,3,4,5_triangle,6}*/.../seed_01`) instead of only the original real-data subset,
specifically to get several independent, reasoned fixtures to check whether "ground truth"
(`DirectSolver`/`enumerate_aggregate_od_route_columns`) or the CG pricer is at fault.

Built a driver (`AggregateODRouteModel(5; assignment_policy=NearestOpenAggregateODAssignmentPolicy(:gamma_chain),
max_walking_distance=1000.0, route_regularization_weight=0.1, max_stops=3, max_wait_time=10000.0,
detour_factor=2.0)`, full un-subsetted station/request set per case) and ran all four solve paths on
five independent fixtures. **The same disagreement reproduces on every single one**, and in the three
where DirectSolver and CG/BendersXY agree on `y`, they disagree only on which routes cover that
identical `y` — i.e. it is not a station-selection bug, it's a route-covering-cost bug:

| Fixture (seed_01)                          | DirectSolver (GT) | Standalone CG | BendersY         | BendersXY | same y? |
|---------------------------------------------|-------------------|---------------|-------------------|-----------|---------|
| test2_zone_proximity/close_to_B              | 246.666 (4 routes)| 198.833 (2)   | 494.157 (diff. y) | 198.833   | yes     |
| test3_north_shift/north_shift_h              | 279.437 (4)       | 208.239 (2)   | 502.723 (diff. y) | 208.239   | yes     |
| test4_mirrored_zone/mirrored_zones           | 967.389 (4)       | 894.759 (2, diff. y) | 2011.55 (diff. y) | 894.759 | no |
| test5_triangle/corridor_base                 | 282.2 (4)         | 210.581 (2)   | 302.334 (diff. y) | 210.581   | yes     |
| test6_bidirectional/fwd100_bwd0              | 282.2 (4)         | 210.581 (2)   | 505.56 (diff. y)  | 210.581   | yes     |

(BendersY's separate premature-convergence bug, already documented above, reconfirms on all five —
not re-investigated further here.)

### Hand-verified: CG's routes are genuinely feasible, and DirectSolver is missing them

Took test2 apart by hand. Both DirectSolver and CG select the identical `y={1,4,6,7,8}` and the
identical assignment (od_idx 1-5, same pickup/dropoff pairs) — they differ *only* in which routes
cover those assignments:

- DirectSolver's pool has only 7 columns, **all singletons** (one per feasible (j,k) pair), and
  activates 4 of them: `{8,4}` τ=245.108, `{6,4}` τ=254.414, `{7,4}` τ=280.763, `{1,4}` τ=750.0 —
  total route cost 1530.285.
- Standalone CG's pool has those same 7 singletons *plus* ten 3-stop, 2-OD-pair consolidated
  routes (e.g. `{1,7,4}` τ=768.824, `{6,8,4}` τ=283.125), and activates just two of them, covering
  the same 5 assignments for total route cost 1051.949 — the exact 478.336 raw-cost gap times
  `route_regularization_weight=0.1` reproduces the 47.833 objective gap (246.666-198.833) almost
  exactly.
- Checked route `{1,7,4}` (τ=768.824=488.061+280.763, from `read_routing_costs_from_segments`)
  against every constraint: `max_stops=3` (route has exactly 3 stops) ✓; detour for passenger
  (1,4) riding the whole route = 768.824 vs direct 750.0 → ratio 1.025, well under
  `detour_factor=2.0` ✓; passenger (7,4) boards after the vehicle already visited 1, so their ride
  segment is just 7→4 = direct, zero detour ✓; `max_wait_time=10000.0` non-binding either way;
  `max_visits_per_node=2` non-binding (each station visited once). **This route is unambiguously
  feasible and CG is right to use it.** DirectSolver's enumerator is missing it — "ground truth"
  is a misnomer here.

### Root cause: dominance pruning in the shared labeling engine is unsound for exhaustive enumeration

`enumerate_aggregate_od_route_columns` (`src/opt/optimize/aggregate_od_route_covering.jl:108`)
calls the *exact same* `_enumerate_aggregate_od_route_pricing_labels` label-setting search used by
real CG pricing (`src/opt/optimize/aggregate_od_route_column_generation.jl:546`), just with
`use_reduced_cost_pruning=false` and synthetic uniform duals (`σ=1.0` for every active pair). The
docstring's claim ("unit rewards so every certifiable route prefix is retained") is false: disabling
`use_reduced_cost_pruning` only skips the *frontier-priority* cutoff (deciding whether to keep
*expanding* a popped label) — it does **not** disable the separate, always-on **dominance-bucket**
admission check in `_add_aggregate_od_route_label_to_bucket!`
(`aggregate_od_route_column_generation.jl:473`), keyed purely by `label.current`
(`_aggregate_od_route_dominance_signature(label) = label.current`, line 406) and governed by
`_dominates_aggregate_od_route_label` (line 442): label `a` dominates `b` sharing the same current
node iff `a.time<=b.time`, `a.reduced_cost<=b.reduced_cost`, `issubset(a.served_pairs,
b.served_pairs)`, and per-station `age` no worse for every station either has tracked.

That rule is the standard (and valid) one for real pricing, where the only goal is *the* most
negative reduced-cost column — if `a` is cheaper and serves a subset of what `b` serves, any
completion of `b` can be replicated more cheaply starting from `a`, so discarding `b` can't lose
the eventual optimum. It is **not valid** for one-shot exhaustive enumeration, because it can
(and here, does) discard a label that has *already reached a terminal, `max_stops`-exhausted
state representing a distinct, complete column* just because a cheaper, less-accomplished label
happens to share its current node. Concretely: the fresh 2-stop label `7→4` (`served={(7,4)}`,
reduced_cost≈27.08 under uniform duals) dominates the 3-stop label `1→7→4`
(`served={(1,4),(7,4)}`, reduced_cost≈74.88, strictly *more* value collected but nominally "worse"
reduced cost) at the shared bucket key `current=4`, because `issubset({(7,4)}, {(1,4),(7,4)})`
holds and the per-station-age check happens to pass once station 1's age is pruned from the
2-pair label after that pair is certified (`_prune_irrelevant_aggregate_od_route_station_ages`,
`aggregate_od_route_column_generation.jl` — makes both labels show `Inf`/untracked for station 1,
so the inequality trivially holds). The 3-stop label is deleted from `live_labels` at insertion
time and never reaches the `best_by_signature` recording step
(`_enumerate_aggregate_od_route_pricing_labels`, ~line 649) — so its signature is permanently lost
from the enumerated pool, for every fixture in the table above.

CG never hits this failure mode in practice because it iterates: across successive iterations
with duals that actually vary pair-to-pair (reflecting the LP's real marginal values, not a flat
1.0), a different label wins the same bucket contention on different iterations, and enough
distinct combinations accumulate in the running column pool across iterations to reach the true
optimum — even though any *single* pricing call has exactly the same soundness gap as the
"exhaustive" enumerator.

### FIXED (same-day, follow-up session): independent brute-force enumerator

Rather than patch the dominance logic in place, replaced `enumerate_aggregate_od_route_columns`
entirely with a fresh, standalone implementation in a new file,
`src/opt/optimize/aggregate_od_route_enumeration.jl`, sharing no code with the CG label-setting
pricer (no duals, no dominance buckets, no reduced-cost bookkeeping at all). It does a plain
bounded DFS over station sequences, restricted to stations that are an OD-pair endpoint (lossless,
since routing costs are direct point-to-point — no road graph to transit through), and for each
sequence prefix of length >= 2 checks, directly and existentially, whether each active pair
`(j,k)` has *some* valid boarding position `p` (`route[p]==j`, within `max_wait_time` of route
start) and later alighting position `q>p` (`route[q]==k`, ride time within `detour_factor` of
direct) — a strictly more complete check than the pricer's "most recent boarding only" dominance
shortcut. The old implementation was deleted outright from
`src/opt/optimize/aggregate_od_route_covering.jl`; `_run_direct_enumerated_aggregate_od_route`
(DirectSolver's call site) is otherwise unchanged, since the function name/signature/contract
(`max_routes`, `time_limit_sec`, throwing `ArgumentError` on overflow) is preserved.

Re-ran the same 5-fixture comparison after the fix: **DirectSolver = standalone CG = BendersXY,
exactly, on all five** (test2 246.666→198.833, test3 279.437→208.239, test4 967.389→894.759
*and* now agrees on `y` too, test5 282.2→210.581, test6 282.2→210.581). Full package test suite:
651 passed / 2 broken (both `BendersY`'s pre-existing, separate premature-convergence bug on the
*synthetic* fixture only — see below) / 0 errors / 0 failures.

One test-file update was needed beyond the enumerator swap: `BendersY`'s `@test_broken` on the
*real-data* fixture started **unexpectedly passing** once ground truth was fixed (Julia's
`Test` treats an unexpectedly-passing `@test_broken` as an error) — apparently on that fixture
BendersY's cut happens to land correctly even though its cut-derivation is still theoretically
unsound in general. `run_cross_solver_alignment_checks` in
`test/opt/test_aggregate_od_route_nearest_open_alignment.jl` now takes a
`benders_y_expected_broken::Bool` kwarg: `true` for the synthetic fixture (genuinely still
broken), `false` for the real-data fixture (now a hard `@test`). `BendersY`'s own bug is
otherwise untouched and not yet fixed — do not trust `BendersY` specifically without checking it
against `BendersXY`/CG on any new fixture.

## Goal

Build a test pipeline treating exhaustive route enumeration as ground truth for
`AggregateODRouteModel` + `NearestOpenAggregateODAssignmentPolicy`, and check that
`ColumnGenerationSolver`, `BendersSolver(BendersY)`, and `BendersSolver(BendersXY)` all agree
with it on objective value, under both `:gamma_chain` and `:big_m_nearest`. Delivered as
`test/opt/test_aggregate_od_route_nearest_open_alignment.jl` (registered in `test/runtests.jl`).
Two fixtures:
- A hand-designed synthetic 5-station fixture (disjoint pickup/dropoff candidate clusters,
  each with 2 members) — see the docstring at the top of the test file for exact construction
  and the hand-verified expected optimum (objective `0.8`, open `{1,2,4,5}`, close decoy
  station `3`).
- A real-data fixture: `../Data/test2_zone_proximity/close_to_B/seed_01` subsetted to stations
  `{3,4,5,7,8}` and requests with `origin ∈ {5,7,8}`, `destination=4`, loaded via
  `read_candidate_stations`/`read_customer_requests`/`compute_station_pairwise_costs`/
  `read_routing_costs_from_segments`/`create_station_selection_data` (the `example/run.jl`
  pipeline), `l=4`, `max_walking_distance=200.0`.

## Fix applied and confirmed (committed to working tree, not yet committed to git)

**Bug:** `run_aggregate_od_route_column_generation` on a model with
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)` crashed with `Gurobi Error 10005:
Unable to retrieve attribute 'Pi'` when extracting LP duals for pricing.

**Root cause:** `_relax_aggregate_od_route_station_and_assignment!`
(`src/opt/optimize/build_aggregate_od_route.jl`) only relaxed `m[:y]` and `m[:x]`. The
endpoint-chain binary variables (`zp`/`zd`) that `:big_m_nearest` creates via
`_endpoint_chain_variable!` (`src/opt/constraints/aggregate_od_route.jl`) were always built
`binary=true` regardless of `relax_integrality`, so the "LP relaxation" CG iterates on was
still a MIP internally — no valid simplex duals.

**Fix:** `_endpoint_chain_variable!` now reads `m[:aggregate_od_route_relax_integrality]`
(already stored on the model at build time) and builds `z` as continuous `[0,1]` when set,
mirroring how `add_aggregate_od_route_theta_variables!` already handles the same flag for
theta variables. (Relaxing in `_relax_aggregate_od_route_station_and_assignment!` itself
doesn't work — it runs *before* the endpoint-chain constraints/variables are even created in
the build order, so that would be a no-op; the fix has to be at variable-construction time.)

**Confirmed fixed:** standalone CG on the synthetic fixture under `:big_m_nearest` now proves
`cg_stop_reason == :optimality_proven` and matches ground truth exactly (0.8). Full test suite
re-run shows no regressions (`Pkg.test()`: 312 passed / 1 pre-existing unrelated failure in the
"Utilities" testset, confirmed present on a clean baseline worktree before this session).

## Open issue: unresolved, needs visual inspection before concluding

On the **synthetic** fixture, `BendersY` converges to a suboptimal-but-*correctly-costed* `y`
(closes station 1 instead of the true-optimal decoy station 3; objective 3.5 vs true 0.8).
Independently re-solving the fixed-`y`/fixed-`x` sub-problem exhaustively
(`enumerate_aggregate_od_route_columns` → inject as `initial_columns` → `DirectSolver`) for
BendersY's own chosen `y` reproduces 3.5 exactly — so on this fixture, whatever `y` BendersY
picks, its reported cost for that `y` is self-consistent. This looks like a **premature Benders
convergence** bug (the optimality cut derived in `_solve_nearest_open_y_subproblem_lp` is built
from a `y_hat`-specific *restricted* column pool — `cg_result.generated_columns` — and may not
actually be a valid global underestimator of the true value function at other `y`, letting the
master accept the first `y` it tries after a single cut). This part is marked `@test_broken` in
the test file (both fixtures, both styles) since it's at least internally consistent.

On the **real-data** fixture, something worse and *not yet root-caused* showed up, and this is
the part to visualize before trusting either side:

- Ground truth (`DirectSolver`, exhaustive enumeration): objective **82.54**, `y` closes station
  3 (array idx 1), opens `{4,5,7,8}` (array idx 2-5).
- Standalone `ColumnGenerationSolver` (`run_aggregate_od_route_column_generation` directly on
  the full model, **no Benders involved at all**): objective **56.39**.
- `BendersSolver(BendersXY)`: objective **56.39** (same wrong value).
- `BendersSolver(BendersY)`: objective **56.39**, converging to the *same* `y` as ground truth
  (`[0,1,1,1,1]`) — yet reporting a lower cost for that identical `y` than the independently
  re-solved exhaustive fixed-`y` check gives (82.54, matching ground truth).

Since all three CG-touching paths (standalone CG, BendersXY's per-iteration priming, BendersY's
per-iteration priming) land on the exact same 56.39, **the discrepancy traces to
`run_aggregate_od_route_column_generation`'s pricing engine itself, not to Benders master/cut
logic** — contradicting my initial read of this as a Benders-specific bug. `DirectSolver` (pure
enumeration, no pricing/duals at all) is the only path giving 82.54.

**The actual open question, not yet settled:** is `enumerate_aggregate_od_route_columns`
(ground truth) missing a real, cheaper, feasible route that CG's reduced-cost-driven
label-setting search finds — or is CG's pricer (`aggregate_od_route_pricing_by_label_setting`
via `_enumerate_aggregate_od_route_pricing_labels`) accepting an actually-infeasible column
(violating `max_stops=3` / `detour_factor=2.0` / `max_wait_time=10000.0` /
`max_visits_per_node=2`) due to a dominance/pruning bug? I was mid-way through pulling the exact
selected column(s)/route(s) responsible for 56.39 (via
`cg_direct.generated_columns`/`selected_column_ids`, station sequence, stop count, computed
detour ratio) to check by hand against those constraints when the session was paused. **Do not
assume ground truth is correct without this check** — the user correctly flagged that I hadn't
actually ruled out the enumerator being wrong.

## Reproduction (fresh session)

```julia
using StationSelection, DataFrames, JuMP, Gurobi
const MOI = JuMP.MOI

dir = "/home/yongzr/Documents/Research/2025-09-JacqWang-Microtransit/Data/test2_zone_proximity/close_to_B/seed_01"
stations = read_candidate_stations(joinpath(dir, "station.csv"))
requests = read_customer_requests(joinpath(dir, "order.csv"); start_time="2026-01-01 00:00:00", end_time="2026-01-02 00:00:00")
keep = [3,4,5,7,8]
stations2 = stations[in.(stations.id, Ref(Set(keep))), :]
requests2 = requests[in.(requests.origin_station_id, Ref(Set([5,7,8]))) .& (requests.destination_station_id .== 4), :]
walking_costs = compute_station_pairwise_costs(stations2)
routing_costs = read_routing_costs_from_segments(joinpath(dir, "segment.csv"), stations2)
data = create_station_selection_data(stations2, requests2, walking_costs; routing_costs=routing_costs)

model = AggregateODRouteModel(4;
    assignment_policy=NearestOpenAggregateODAssignmentPolicy(:gamma_chain),
    max_walking_distance=200.0, route_regularization_weight=0.1, repositioning_time=0.0,
    max_stops=3, max_wait_time=10000.0, detour_factor=2.0,
)

gt = run_opt(data, model, DirectSolver(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0,
    max_enumerated_routes=2000, max_enumeration_time_sec=20.0))
# gt.objective_value == 82.5415...

cg = run_aggregate_od_route_column_generation(model, data; optimizer_env=Gurobi.Env(),
    verbose=false, max_cg_iters=200, max_new_columns=20, n_candidates=20,
    ip_time_limit_sec=30.0, mip_gap=0.0, silent=true)
# cg.final_result.objective_value == 56.3888  <-- disagreement to resolve

# next step: inspect cg.generated_columns filtered to cg.selected_column_ids -- station
# sequence, tau, and hand-check stop count / detour ratio / wait time against the model's
# constraints. Also worth visualizing station positions + both solutions' routes side by side.
```

## Plan for next session

User wants to build **visualizations** (station layout + the routes/assignments each solver
actually picked) to visually confirm which side (enumeration vs. CG pricing) is wrong before
proceeding with any fix. Do this before touching `_run_aggregate_od_route_nearest_open_benders_y`
or the pricing engine again. The existing `visualize/` directory at the project root has
precedent scripts (e.g. `visualize_zhuzhou_flows.jl`, `visualize_zhuzhou_stations.jl`) worth
checking for reusable plotting patterns.

## Current repo state (uncommitted)

- `src/opt/constraints/aggregate_od_route.jl` — the relax-integrality fix (confirmed good, low
  risk, should probably be committed independent of the open issue above).
- `test/opt/test_aggregate_od_route_nearest_open_alignment.jl` (new) — cross-solver alignment
  suite described above. Real-data fixture currently has 4 hard test failures (standalone CG and
  BendersXY objective mismatches) that are **not yet marked `@test_broken`** because root cause
  is undetermined — do not mark them broken without understanding which side is actually wrong,
  since marking the wrong assertion broken could hide a ground-truth bug instead of a solver bug.
- Also in this working tree from an earlier, unrelated task this session: `HeuristicEnumerationSolver`
  (`src/opt/optimize/iterative_strategy_types.jl`,
  `src/opt/optimize/aggregate_od_route_heuristic_enumeration.jl`,
  `test/opt/test_aggregate_od_route_heuristic_enumeration.jl`) — a fixed-y-candidates warm-start
  mechanism for `AggregateODRouteModel`, unrelated to this investigation, already complete and
  tested (all passing).
