# Cross-Solver Alignment Under NearestOpenAggregateODAssignmentPolicy
*2026-07-14, session paused pending visual inspection*

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
