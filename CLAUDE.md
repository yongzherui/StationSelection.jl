# StationSelection.jl

Julia package implementing VBS location optimisation models. Source lives in `src/opt/`,
organised around a **Problem / Formulation / Solver** split (`opt/abstract.jl`):

```julia
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver) =
    optimize_model(build_model(problem, formulation, solver), solver)
```

- `AbstractProblem` — *what* is being solved: instance data plus the business decision on
  top of it (station count, walking limit). One concrete type today: `StationSelectionProblem`
  (`data`, `k` = stations built, `max_walking_distance`, default 300).
- `AbstractFormulation` — *how* the problem is mathematically encoded (assignment policy,
  staging, cost weights). See "Live Formulations" below for the six concrete types.
- `AbstractSolver` — *which* algorithm solves it: `DirectMIPSolver` (single `optimize!`
  call), `CGSolver` (column-generation outer loop), `BendersSolver` (Benders outer loop,
  scaffolded but no formulation implements its hooks yet).

## Directory Layout (`src/opt/`)

```
opt/
├── abstract.jl           # AbstractProblem, AbstractFormulation
├── problems/              # AbstractProblem subtypes (StationSelectionProblem, RouteCoveringProblem)
├── formulations/          # AbstractFormulation subtypes (clustering.jl, aggregate_od_route/*)
├── solvers/                # AbstractSolver subtypes + shared solver utils
├── optimize/               # build_model methods, one per (problem × formulation × solver)
├── label_setting/          # pricing/column-enumeration engine for the AggregateODRoute formulations
├── variables/               # shared variable-creation building blocks (y, z, x, θ, f, walk)
├── constraints/             # shared constraint-creation building blocks
└── objectives/               # shared objective-assembly building blocks
```

`build_model` methods compose the `variables/`/`constraints/`/`objectives/` building
blocks; they don't live inside `formulations/` or `problems/` themselves.

## Live Formulations (6)

| Formulation | Solver | Key idea | Unique fields | Unique variables |
| --- | --- | --- | --- | --- |
| `ClusteringBaseFormulation` | `DirectMIPSolver` | Single-scenario k-medoids; no build/activate split — `problem.k` stations *are* the selection | none (dispatch marker) | y, x |
| `ClusteringTwoStageFormulation` | `DirectMIPSolver` | Two-stage station-to-station assignment | `l` (activate/scenario) | y, z, x |
| `ClusteringTwoStageODFormulation` | `DirectMIPSolver` | Two-stage OD pickup/dropoff assignment | `l`, `in_vehicle_time_weight` | y, z, x |
| `ClusteringTwoStageODFlowRegularizerFormulation` | `DirectMIPSolver` | `ClusteringTwoStageODFormulation` + route-activation flow penalty | `l`, `in_vehicle_time_weight`, `flow_regularization_weight` | y, z, x, f_flow |
| `AggregateODRouteBaseFormulation` | `DirectMIPSolver` | Station build + decoupled OD assignment + route activation, against an exhaustively enumerated column pool built up front | `route_regularization_weight`, `walk_cost_weight`, `repositioning_time`, `max_wait_time`, `detour_factor`, `max_stops` | y, x, x_walk, θ |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | Same encoding-detail fields as Base, but θ columns carry OD assignment directly — no separate `x`; grown via column generation, no up-front enumeration | same field set as Base | y, x_walk, θ |

**Important departure from the old two-stage convention:** the two `AggregateODRoute*`
formulations have **no `z`/per-scenario-activation variable and no `l` distinct from
`k`** — every built station is usable in every scenario. Only the four Clustering
formulations retain the `y` (build) / `z` (activate-per-scenario) two-stage split.

`ClusteringBaseFormulation`/`ClusteringTwoStageFormulation` lost the "no walking limit"
(`nothing`) option when they moved onto `StationSelectionProblem.max_walking_distance`
(required `Float64`, default 300) — not yet verified against an "unlimited walking
distance" test case.

Both `AggregateODRoute*` formulations validate build-time feasibility
(`aggregate_od_route_validate_feasible_coverage`) and always expose direct walking
(`x_walk`, `WALK_ONLY_PAIR`) as a station-free coverage option — not configurable, no
`allow_walk_only` field.

## Kept-but-unwired scaffolding

Not dead code — deliberately preserved as a starting point for future work, but not
reachable from any `build_model`/`Solver` today:

- Five Benders formulation marker structs under `opt/formulations/aggregate_od_route/
  benders/` (`{y,xy,yz,yzh,yx}.jl` + `cut_mode.jl`'s `AbstractBendersCutMode`/`SingleCut`/
  `MultiCut`) — no formulation implements `BendersSolver`'s four hooks yet.
- `RouteCoveringProblem` (`opt/problems/route_covering.jl`) — fixed-`y`/fixed-assignment
  shape a future Benders subproblem should reuse.
- `BendersSolver` (`opt/solvers/benders_solver.jl`) — generic outer loop with four
  formulation-specific hook stubs (`extract_incumbent`, `solve_subproblem`,
  `benders_converged`, `add_benders_cut!`); none implemented yet.

See `notes/2026-08-11_problem_formulation_solver_split_progress.md` for the fuller
writeup, migration history, and remaining-work list.

## Removed entirely

The pre-split `AbstractStationSelectionModel` hierarchy and every model built on it are
gone — not migrated, just absent. If old scripts, notes, or slides reference
`TwoStageSingleDetourModel`/TSD, `ZCorridorODModel`, `XCorridorODModel`,
`XCorridorWithFlowRegularizerModel`, `TransportationModel`, `AlphaRouteModel`,
`RouteFleetLimitModel`, `RouteAlphaCapacityModel`, `RouteVehicleCapacityModel`,
`TwoStageRouteWithTimeModel`, `ExactDARPRouteModel`, or the two-arg
`run_opt(model, data; ...)`/`build_model(model, data; ...)` API, those refer to a version
of this package that no longer exists.

## Decision Variables

| Var | Domain | Meaning | Formulations |
| --- | --- | --- | --- |
| y[j] | {0,1} | Station j built | all |
| z[j,s] | {0,1} | Built station j activated in scenario s; z≤y | Clustering (two-stage only) |
| x[i,j,s] | {0,1} | Demand point i assigned to active station j in scenario s | ClusteringTwoStageFormulation |
| x[s][p][j,k] | Z₊ | OD demand group p (position within scenario s's positive-demand pairs) assigned to station pair (j,k) | ClusteringTwoStageOD* |
| x[s,p,j,k] | {0,1} | OD demand group (s,p) assigned to station pair (j,k), decoupled from routing | AggregateODRouteBaseFormulation |
| x_walk[s,p] | {0,1} | OD demand group (s,p) served by direct walk (no station) | both AggregateODRoute* |
| θ[column_id, s] | {0,1} (Base) / [0,1] LP-relaxed (Joint) | Route column activated in scenario s | both AggregateODRoute* |
| f_flow[s][(j,k)] | [0,1] | Route (j,k) activated in scenario s, for the flow-regularization penalty | ClusteringTwoStageODFlowRegularizerFormulation |

## Parameters

**On `StationSelectionProblem`:** `k` (stations built), `max_walking_distance` (feasibility
radius, shared by every formulation that restricts assignment by walk distance).

**Clustering formulations:** `l` (activate per scenario, two-stage only),
`in_vehicle_time_weight` (OD formulations only), `flow_regularization_weight`
(FlowRegularizer only).

**AggregateODRoute formulations (both):** `route_regularization_weight` (μ, multiplies
each route column's cost), `walk_cost_weight` (multiplies every walking-cost term),
`repositioning_time` (ρ, added to every route column's travel/service cost),
`max_wait_time`, `detour_factor` (min 1.0), `max_stops` (min 2, default unbounded).

**Solver-level:** `SolverOptions` (`silent`, `mip_gap`, `time_limit_sec`) shared by every
`AbstractSolver`. `CGSolver` additionally carries `max_iterations`,
`recover_integer_solution`, `initial_columns`. `BendersSolver` carries `max_iterations`,
`optimality_tol`.

## Key Constraints

| Constraint | Formula | Where |
| --- | --- | --- |
| Station limit | Σⱼ y[j] = k | all |
| Activation limit | Σⱼ z[j,s] = l ∀s | Clustering two-stage |
| Activation linking | z[j,s] ≤ y[j] ∀j,s | Clustering two-stage |
| Assignment coverage | Σⱼ x[i,j,s] = 1 (or Σ over station pairs = demand) ∀ demand group | all |
| Assignment-to-active | x ≤ z[j,s] (and z[k,s] for OD pairs) | Clustering two-stage |
| Station linking (AggregateODRoute) | x[s,p,j,k] ≤ y[j], x[s,p,j,k] ≤ y[k] | AggregateODRouteBaseFormulation |
| Route linking | Σ_columns covering (s,p,j,k) θ ≥ x[s,p,j,k] (Base) | AggregateODRouteBaseFormulation |
| Flow activation | f[j,k,s] ≥ Σ x[od,j,k,s] | ClusteringTwoStageODFlowRegularizerFormulation |

## Objective Components

- **Walking cost** (all formulations): demand-weighted walking distance/time to
  pickup/dropoff station, or (AggregateODRoute) direct-walk cost when `x_walk` is used.
- **In-vehicle routing cost** (`in_vehicle_time_weight` × routing cost): ClusteringTwoStageOD*.
- **Flow-regularization penalty** (`flow_regularization_weight` × Σ routing-time-weighted
  f_flow): ClusteringTwoStageODFlowRegularizerFormulation.
- **Route column cost** (AggregateODRoute, both): `route_regularization_weight` ×
  (column travel/service cost + `repositioning_time`), summed over active θ.

## Core Data Structures

```julia
StationSelectionData
  .stations::DataFrame        # :id, :lon, :lat
  .walking_costs               # Dict{(i,j), Float64}
  .routing_costs                # Dict{(i,j), Float64} or Nothing
  .scenarios::Vector{ScenarioData}

ScenarioData
  .label, .start_time, .end_time
  .requests::DataFrame
```

`AbstractStationSelectionMap` subtypes (`opt`-formulation-specific, built by `create_map`
or `create_aggregate_od_route_map`) hold the index bookkeeping (station id ↔ array index,
scenario label ↔ index, `Omega_s`/`Q_s` demand-group indexing for AggregateODRoute) that
`build_model` and the exported analysis helpers rely on.

## Entry Points

```julia
run_opt(problem, formulation, solver; ) -> OptResult
build_model(problem, formulation, solver)  -> BuildResult  # build only, no solve
```

```julia
problem = StationSelectionProblem(data, 10; max_walking_distance=300)
formulation = ClusteringTwoStageODFormulation(5; in_vehicle_time_weight=1.0)
solver = DirectMIPSolver(config=SolverOptions(silent=true))
result = run_opt(problem, formulation, solver)
```

`OptResult` fields: `termination_status`, `objective_value`, `solution`, `runtime_sec`,
`model`, `mapping`, `detour_combos`, `counts` (variable/constraint counts by category),
`warm_start_solution`, `metadata`, `duals` (`nothing` outside Benders dual-problem
results).

# Notes

- When adding new variables in opt/variables/ we need to make sure to add the corresponding export variables function to ensure consistency.
- The top-level project `CLAUDE.md` (`../CLAUDE.md`) still refers to seven pre-split
  model names under "Optimisation Models" — treat this file, not that one, as
  authoritative for what's actually live in `src/opt/`.
