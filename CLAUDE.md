# StationSelection.jl

Julia package implementing VBS location optimisation models. Source lives in `src/opt/`
with subdirectories: `models/`, `variables/`, `constraints/`, `objectives/`.

## Abstract Type Hierarchy

```
AbstractStationSelectionModel
├── AbstractSingleScenarioModel
│   └── ClusteringBaseModel          # k-medoids, no scenarios
└── AbstractMultiScenarioModel
    └── AbstractTwoStageModel        # y=build (stage 1), z=activate (stage 2)
        └── AbstractODModel
            ├── ClusteringTwoStageODModel  # Two-stage: pure cost minimisation (+ optional flow reg.)
            └── TwoStageRouteModel         # Two-stage: route-based capacity + route penalty
```

## Model Reference

| Model                       | Key idea                                                                | Unique variables              |
| --------------------------- | ----------------------------------------------------------------------- | ----------------------------- |
| `ClusteringBaseModel`       | k-medoids single scenario                                               | x[i,j]                        |
| `ClusteringTwoStageODModel` | Two-stage; minimise walk+ride; optional flow regularization penalty      | y, z, x, (f_flow)             |
| `TwoStageRouteModel`        | Two-stage; time-indexed OD; pre-generated routes; vehicle capacity link  | y, z, x[s][t][od], θ[s,r]    |

## Decision Variables

| Var               | Domain | Meaning                                                              |
| ----------------- | ------ | -------------------------------------------------------------------- |
| y[j]              | {0,1}  | Station j built (first stage)                                        |
| z[j,s]            | {0,1}  | Station j active in scenario s; z≤y                                  |
| x[s][od][j,k]     | {0,1}  | OD pair assigned to pickup j, dropoff k (ClusteringTwoStageOD)       |
| x[s][t][od][pair] | {0,1}  | OD pair (time-indexed) assigned to valid (j,k) pair (TwoStageRoute)  |
| f_flow[s][j,k]    | [0,1]  | Route (j,k) activated in scenario s (ClusteringTwoStageOD with FR)   |
| θ[s,r]            | {0,1}  | Route r activated in scenario s (TwoStageRoute)                      |

## Parameters

**Shared:** `k` (active stations/scenario), `l` (stations built), `in_vehicle_time_weight` (λ).

**TwoStageRoute only:** `route_regularization_weight` (μ), `vehicle_capacity` (C),
`time_window_sec` (discretisation step), `max_route_travel_time` (route filter),
`max_intermediate_stops` (0=direct, 1=one-stop).

**TSD only:** `vehicle_routing_weight` (γ), `time_window` (sec, discretisation step),
`routing_delay` (max detour), `detour_use_flow_bounds`.

**Corridor only:** `corridor_weight` (γ), `n_clusters`, `max_cluster_diameter`.
`clustering_mode="count"` = p-median MILP for cluster formation.
Clusters are a hard partition; overlap is open research.

**Transportation only:** `activation_cost` (fixed cost per anchor activation).

**Shared flags:**

- `use_walking_distance_limit` / `max_walking_distance` — prune x variables by walk distance
- `variable_reduction` — use sparse (Dict-based) x when walking limits are enabled
- `tight_constraints` — `x≤z[j] AND x≤z[k]` vs looser `2x≤z[j]+z[k]`

## Key Constraints

| Constraint                   | Formula                                |
| ---------------------------- | -------------------------------------- |
| Station limit                | Σⱼ y[j] = l                            |
| Activation limit             | Σⱼ z[j,s] = k ∀s                       |
| Activation linking           | z[j,s] ≤ y[j] ∀j,s                     |
| Assignment coverage          | Σⱼₖ x[s][od][j,k] = 1 ∀s,od            |
| Assignment-to-active (tight) | x[j,k,s] ≤ z[j,s], x[j,k,s] ≤ z[k,s]              |
| Route capacity               | Σ q·x_{odtjks} ≤ Σ_r C·θ^r_s  ∀(j,k,t,s) (Route)  |
| Z-cluster activation         | α[a,s] ≥ z[i,s] ∀i∈Cₐ                  |
| Z-corridor activation        | f[g,s] ≥ α[a,s]+α[b,s]-1 for g=(a,b)   |
| X-corridor activation        | f[g,s] ≥ Σ x[od,j,k,s] for j∈Cₐ, k∈C_b |

## Objective Components

- **Walking cost:** demand × walking distance to pickup/dropoff station (all OD models)
- **Route penalty:** `route_regularization_weight` × Σ τ^r θ^r_s (TwoStageRoute)
- **Routing cost:** `in_vehicle_time_weight` × routing cost between stations
- **Vehicle routing (TSD):** `vehicle_routing_weight` × arc cost × flow f
- **Pooling savings (TSD):** subtract savings when u/v detour indicators fire
- **Corridor penalty:** `corridor_weight` × Σ f_corridor[g,s]
- **Anchor activation (Transportation):** `activation_cost` × Σ u_anchor[g,s]

## Core Data Structures

```julia
StationSelectionData
  .stations::DataFrame        # :id, :lon, :lat
  .walking_costs              # Dict{(i,j), Float64}
  .routing_costs              # Dict{(i,j), Float64} or Nothing
  .scenarios::Vector{ScenarioData}

ScenarioData
  .label, .start_time, .end_time
  .requests::DataFrame
```

## Entry Points

```julia
run_opt(model, data; silent=true, warm_start=false) -> OptResult
build_model(model, data)                            -> BuildResult  # build only, no solve
```

`OptResult` fields: `termination_status`, `objective_value`, `runtime_sec`, `model`,
`mapping`, `counts` (variable/constraint counts by category).

# Notes

- When adding new variables in opt/variables/ we need to make sure to add the corresponding export variables function to ensure consistency.
