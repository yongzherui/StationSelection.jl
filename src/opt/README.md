# Optimization Module

This folder implements every station-selection optimization model as a
**Problem / Formulation / Solver** split (`abstract.jl`):

```julia
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver) =
    optimize_model(build_model(problem, formulation, solver), solver)
```

- `problems/` — `AbstractProblem` subtypes: instance data + the business decision on top
  of it. `StationSelectionProblem(data, k; max_walking_distance=300)` is the one live
  type today.
- `formulations/` — `AbstractFormulation` subtypes: the mathematical encoding. See
  "Formulations" below.
- `solvers/` — `AbstractSolver` subtypes: `DirectMIPSolver`, `CGSolver`, `BendersSolver`
  (scaffolded, unwired).
- `optimize/` — the actual `build_model(problem, formulation, solver)` methods, one per
  combination that's actually supported.
- `label_setting/` — the pricing/column-enumeration engine the two `AggregateODRoute`
  formulations sit on (route enumeration for `DirectMIPSolver`, pricing rounds for
  `CGSolver`).
- `variables/`, `constraints/`, `objectives/` — shared building-block functions each
  `build_model` composes.

See `StationSelection.jl/CLAUDE.md` (package root) for the full model/parameter/
constraint/objective reference. This file only covers the build/run API shape.

## Formulations

| Formulation | Pairs with (solver) | Two-stage (y/z)? |
| --- | --- | --- |
| `ClusteringBaseFormulation` | `DirectMIPSolver` | No — single stage |
| `ClusteringTwoStageFormulation` | `DirectMIPSolver` | Yes |
| `ClusteringTwoStageODFormulation` | `DirectMIPSolver` | Yes |
| `ClusteringTwoStageODFlowRegularizerFormulation` | `DirectMIPSolver` | Yes |
| `AggregateODRouteBaseFormulation` | `DirectMIPSolver` | No — build only, no per-scenario activation |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | No — build only |

Construct a formulation with its own keyword arguments (e.g.
`ClusteringTwoStageODFormulation(l; in_vehicle_time_weight=1.0)`,
`AggregateODRouteBaseFormulation(; route_regularization_weight=1.0, walk_cost_weight=1.0,
repositioning_time=20.0, max_wait_time=Inf, detour_factor=1.5, max_stops=nothing)`) — see
each formulation's own docstring in `formulations/` for the authoritative field list and
defaults.

## Build/Run API

### build_model

```julia
build_result = build_model(problem, formulation, solver)
```

`BuildResult` fields:

- `model`: JuMP.Model
- `mapping`: AbstractStationSelectionMap
- `detour_combos`: DetourComboData or `nothing`
- `counts`: ModelCounts or `nothing`
- `metadata`: Dict

### run_opt

```julia
opt_result = run_opt(problem, formulation, solver)
```

`OptResult` fields:

- `termination_status`
- `objective_value`
- `solution`
- `runtime_sec`
- `model`
- `mapping`
- `detour_combos`
- `counts`
- `warm_start_solution`
- `metadata`
- `duals` (`nothing` outside Benders dual-problem results)
