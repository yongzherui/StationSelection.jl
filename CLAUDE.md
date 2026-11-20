# StationSelection.jl

Julia package for VBS location models. Project-level rules, terminology and the standard protocol are in the
parent `../CLAUDE.md`; this file covers the package API. The pre-2026-11-18 version, with the full measurement
histories (Benders scaling, relaxed-cluster cut experiments, LPO completion, naming history), is archived at
`notes/2026-11-18_claude_md_before_reorg.md`.

## Problem / Formulation / Solver

```julia
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver) =
    optimize_model(build_model(problem, formulation, solver), solver)
```

- **Problem** = instance + business decision: `StationSelectionProblem(data, k; walking_limit_s, door_to_door_limit_ratio)`.
  `k` = stations built. `walking_limit_s` is a TIME in seconds despite its name (haversine m / 1.4 m/s); its
  default 300 is not the project standard (720) — always pass it. `door_to_door_limit_ratio` defaults to `Inf` (off);
  the project standard is 3.0 — always pass it. Renamed 2026-11-18 (old names `max_walking_distance`,
  `door_to_door_ratio`, and the formulations' `detour_factor` -> `ride_time_limit_ratio`); the old keyword and field
  names still work with a warning (`src/utils/core/renamed_limits.jl`) and will be removed.
- **Formulation** = the mathematical encoding (the model).
- **Solver** = the algorithm: `DirectMIPSolver`, `CGSolver` (column generation), `BendersSolver`.

```julia
problem = StationSelectionProblem(data, 20; walking_limit_s = 720.0, door_to_door_limit_ratio = 3.0)
formulation = AggregateODRouteJointRoutingAssignmentFormulation(route_regularization_weight = 10.0,
                                                                walk_cost_weight = 0.1)
result = run_opt(problem, formulation, CGSolver(config = SolverOptions(silent = false)))
```

`build_model(problem, formulation, solver)` builds without solving. `OptResult` fields: `termination_status`,
`objective_value`, `solution`, `runtime_sec`, `model`, `mapping`, `detour_combos`, `counts`,
`warm_start_solution`, `metadata`, `duals`.

## Models (formulations)

Project names in bold (see `../CLAUDE.md`, "Terminology").

| Model | Struct | Solver | Key idea | Fields | Variables |
| --- | --- | --- | --- | --- | --- |
| **Base** | `ClusteringBaseFormulation` | Direct MIP | single-scenario k-medoids | none | y, x |
| — | `ClusteringTwoStageFormulation` | Direct MIP | two-stage station assignment | `l` | y, z, x |
| **Direct-distance clustering** | `ClusteringTwoStageODFormulation` | Direct MIP | two-stage OD pickup/drop-off assignment | `l`, `in_vehicle_time_weight` | y, z, x |
| — | `ClusteringTwoStageODFlowRegularizerFormulation` | Direct MIP | the above + route-activation flow penalty | + `flow_regularization_weight` | y, z, x, f_flow |
| **LAR** | `AggregateODRouteBaseFormulation` | Direct MIP | build + OD assignment + route activation over an exhaustively enumerated route pool | route fields (below) | y, x, x_walk, θ |
| **Joint LAR** | `AggregateODRouteJointRoutingAssignmentFormulation` | CG (or Direct MIP over enumeration, or Benders) | route columns carry the OD assignment; grown by pricing | route fields + `demand_weighting`, `vehicle_capacity` | y, x_walk, θ |

- **Clustering models** keep the y (build) / z (activate per scenario) split, `l <= k`. In practice z == y.
- **LAR and Joint LAR** have no z: every built station is usable in every scenario.
- **Route fields:** `route_regularization_weight` (multiplies each route's cost), `walk_cost_weight`,
  `repositioning_time` (added to every route; project standard 20 s), `pickup_deadline_s` (the pickup deadline: a route
  picks riders up only while its clock, which starts at 0 with every request, is <= it; required, no default),
  `ride_time_limit_ratio` (min 1.0;
  ride-time limit counterpart, project standard 2.0, package default 1.5), `max_stops` (min 2, default unbounded).
- **Direct walking (`x_walk`) is OFF** for every live formulation: the simulator has no walk-only mode, so
  selection must not serve groups on foot.
- **Joint LAR `demand_weighting`** (default `:count` since 2026-11-03, `a26fbff`): Q = request rows per OD group
  (`:unit` = 1). Q scales **walking only** (column cost, direct walk, pricer reward, Benders completions); coverage
  stays >= 1 and a route's travel is charged once. Never put Q on one side alone: the CG master's reduced-cost
  assertion fires. LAR stays `:unit`; the Clustering maps keep true counts.
- **Joint LAR `vehicle_capacity`** (`nothing` = uncapacitated): requests above capacity are dropped from the
  instance and every route keeps at most that many on board; priced by `:exact_capacitated`.
- **Joint LAR `demand_groups` / `demand_weighting = :pax`** (2026-11-20): `demand_groups = :rider` makes every request
  its own demand group (riders of one OD pair may ride different routes; all still request at the scenario start);
  default `:od_pair` (all requests of an OD pair in a scenario ride one route). `demand_weighting = :pax` sets Q to
  the group's passengers (the requests' `:pax` column). Owner's capacitated setting: `demand_groups = :rider,
  demand_weighting = :pax` with the heuristic M3 `vehicle_capacity` below. Column generation only (Benders refuses
  `:rider`). `mapping.group_rows_s[s][p]` lists each group's request rows.
- **Heuristic capacity for M3 (`CGPricingConfig.vehicle_capacity`, 2026-11-20) -- a HEURISTIC, not exact capacitated
  pricing.** `CGPricingConfig(mode = :exact_no_anticipation, first_boarding = :after_last_walker, vehicle_capacity = 18)`
  (passengers per vehicle; `nothing` = off; any other mode is rejected). Not the formulation's `vehicle_capacity` above (exact, in requests); setting that with M3 is an error. Give the requests DataFrame a `:pax` column
  (passengers per request; 1 per request without it). Pricing is unchanged M3; every LP column over capacity on some
  leg is also split (first-fit interval packing of its OD groups, groups never split, each piece drives the induced
  stop subsequence and is checked against the M3 timing rules); the integer recovery keeps only columns within
  capacity (a lone group over capacity allowed). Exact: the final plan's routes are within capacity and M3-feasible.
  Heuristic: capacity-feasible routes other than these splits are never priced, and the LP bound is the
  uncapacitated one. Counters in `metadata["cg_split_stats"]` (`check_fail_*` must be 0). Code:
  `src/opt/optimize/aggregate_od_route/column_generation/capacity_split.jl` (`joint_split_column` decides the pieces);
  test `test/opt/test_joint_routing_assignment_capacity_split.jl`; study `experiments/2026-11-17_g2_time_blind`
  (`scripts/price_capsplit.jl`, `SPLIT_CAP=18`).

### Service limits on the selection side

`door_to_door_limit_ratio` (the door-to-door limit) filters station pairs in `compute_valid_jk_pairs`:
`walk(o,j) + drive(j,k) + walk(k,d) <= ratio * drive(o,d)` (seconds; a necessary condition). The OD formulations
(Direct-distance clustering family, LAR, Joint LAR) apply it. An OD left with no pair (e.g. `o == d`) raises an
`ArgumentError` (`check_door_to_door_servable`). The Joint LAR pricer also caps each rider's ride at
`min(ride_time_limit_ratio * drive(j,k), ratio * drive(o,d) - walk(o,j) - walk(k,d))`
(`joint_routing_assignment_ride_limit`, used by CG pricing, DirectMIP enumeration and Benders). LAR's pricer does
not apply it. Supplied `CGSolver(initial_columns = ...)` are validated against these limits and refused otherwise.
Write-up: `../docs/notes/2026-09-27_door_to_door_ratio_selection.md`.

## Directory layout (`src/opt/`)

```
opt/
├── abstract.jl        AbstractProblem, AbstractFormulation
├── problems/          StationSelectionProblem, RouteCoveringProblem (unwired scaffolding)
├── formulations/      clustering.jl, aggregate_od_route/* ; joint_routing_assignment/ is a family:
│                      shared.jl (the shared encoding fields), monolithic.jl, master.jl,
│                      benders_subproblem.jl, unions.jl
├── solvers/           direct_solver.jl, cg/, benders/ — each: *_config.jl, solver.jl (struct + algorithm
│                      docstring), state.jl, loop.jl, metadata.jl, hooks.jl
├── optimize/          build_model per (problem × formulation × solver); aggregate_od_route/
│                      {direct,column_generation,benders}/ + joint_shared.jl
├── label_setting/     pricing engine; joint_routing_assignment/{exact,station_simple,darp,darp_modified}/
│                      price columns; relaxed_cluster/ is a relaxed GRAPH the exact search runs on, with
│                      utils/{certification,guiding,refinement}/
├── variables/  constraints/  objectives/   shared building blocks composed by build_model
```

When adding variables in `opt/variables/`, add the matching export-variables function.

## Column generation (`CGSolver`)

`CGSolver` carries `config::SolverOptions` (`silent`, `mip_gap`, `time_limit_sec`), `pricing::CGPricingConfig`,
`max_iterations`, `recover_integer_solution`, `initial_columns`, and two pricing budgets
(`pricing_time_limit_sec`, `certifying_pricing_time_limit_sec`). **Turn `silent` off for long runs** (project rule:
long runs must log progress).

**Pricers are solver settings, not formulation ones**: two runs differing only in the pricer solve the identical
model. `CGPricingConfig` fields: `mode`, `warm_start_mode`, `compensated_dominance` (default true; CG pricer only),
`relaxed_cluster_count`, `relaxed_cluster_max_count`, `relaxed_cluster_guide_routes` (5),
`relaxed_cluster_macro_count`, `relaxed_cluster_aligned_subset_max` (15).

| `mode` | Searches | Notes |
| --- | --- | --- |
| `nothing` / `:exact` | full revisit-tolerant route universe | Joint LAR default |
| `:darp_modified`, `:darp` | same universe | exhaustive-equivalent alternatives |
| `:exact_capacitated` | capacitated universe | for `vehicle_capacity`; `../docs/notes/2026-11-16_exact_capacitated_pricer.md` |
| `:station_simple` | elementary routes only | a RESTRICTION: its optimum is scoped (below) |
| `:relaxed_cluster`, `:relaxed_cluster_two_tier` | relaxed cluster graph + exact pricing of supports | the only modes that can CERTIFY; require `relaxed_cluster_count` |

- `warm_start_mode` prices in that mode until exhausted, then hands the shared master and pool to `mode`.
  `:cluster_guide` is a warm-start-only mode. A no-op or unreachable warm start is rejected, never ignored.
- **Relaxed-cluster certification**: K-medoids cells built once per run. Each attempt exact-prices a cluster
  support; finding a column (`:column_found`, ~96% of attempts) is the normal outcome, not a failure. An
  exhausted barren support adds the no-good cut "every route visits a cluster outside T" (the stronger
  `|route ∩ T| <= |T|-1` form is INVALID). Cut management and the barren cache are always on; cap
  `RELAXED_CLUSTER_MAX_CUT_ROUNDS` = 96 (mask bits are the real bound). Inconclusive attempts escalate per
  scenario; a second inconclusive ends with `cg_stop_reason = "pricing_inconclusive"`. Certification ends with
  `"converged_by_certification"` and is full-universe regardless of the warm-start pricer.
- LAR has no selectable pricer and rejects a non-default `mode`.

## Benders (`BendersSolver`, Joint LAR only)

`build_model(problem, ::Monolith, ::BendersSolver)` derives `…JointRoutingAssignmentMasterFormulation`
(y + cut variables Θ) and `…JointRoutingAssignmentBendersSubproblemFormulation` (one scenario's x_walk, θ, y fixed)
from the ONE monolith passed to `run_opt`, copying its shared fields, so master and subproblem cannot disagree.
MultiCut(:scenario) by default. Verified exact against the monolithic mixed model.

Two properties travel with every answer:
- `metadata["benders_second_stage_relaxed"] == true`: optimal for the MIXED model (y binary, θ/x_walk
  continuous), not the all-binary optimum; the LP-IP gap is not small.
- `metadata["benders_optimality_scope"]`: `"max_stops_restricted"` when `:direct_enumeration` capped `max_stops`
  (default 4 there).

`BendersSolver` fields: `max_iterations`, `optimality_tol`, `total_time_limit_sec`, `subproblem::BendersSubproblemConfig`
(`oracle`, `max_stops`, `pricing`, `max_cg_iterations`, `cg_pricing_time_limit_sec`, `max_routes`,
`enumeration_time_limit_sec`, `time_limit_sec`, and `lpo_*` for the LPO oracle). Oracles: `:direct_enumeration`,
`:column_generation`, `:column_generation_activated`, `:column_generation_activated_lpo`,
`:column_generation_warm_start`. **A cut is derived only from an EXHAUSTED pricing round**; `:station_simple` is
rejected as a Benders pricer. The activated oracles' cuts are weak (closed-form completion credits only walking);
details and measurements in the archived file.

## Solve status

`termination_status` is the package's `SolveStatus`, not MOI's (a budget-stopped CG master still reads
`MOI.OPTIMAL`; the raw code is kept in `metadata["moi_termination_status"]`).

| Member | Meaning |
| --- | --- |
| `SOLVE_OPTIMAL` | certified: CG pricing exhausted (`cg_converged`) or certified; Benders bounds met (`benders_converged`) |
| `SOLVE_FEASIBLE` | incumbent, optimality not proven (budget-stopped, inconclusive, iteration/wall cap) |
| `SOLVE_INFEASIBLE` | solver said so, or `check_feasibility` refuted the instance (reason in `metadata["infeasibility_reason"]`; not thrown) |
| `SOLVE_NOT_SOLVED` | no incumbent |

**`OPTIMAL` is scoped to the universe priced**: check `metadata["cg_optimality_scope"]`
(`"full_route_universe"` or `"elementary_routes_only"`) before treating an objective as a true optimum.
Benders reports `benders_lower_bound` (master `objective_bound`), `benders_upper_bound` and `benders_gap`; the
reported objective is always the UB.

## Data structures

```julia
StationSelectionData
  .stations::DataFrame         # :id, :lon, :lat
  .walking_costs               # Dict{(i,j), Float64}
  .routing_costs               # Dict{(i,j), Float64} or Nothing
  .scenarios::Vector{ScenarioData}   # .label, .start_time, .end_time, .requests::DataFrame
```

Maps (`create_map`, `create_aggregate_od_route_map`) hold the index bookkeeping (station id <-> index, scenario
<-> index, `Omega_s` / `Q_s` demand groups) used by `build_model` and the analysis helpers.
