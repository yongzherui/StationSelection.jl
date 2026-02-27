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
            ├── ClusteringTwoStageODModel       # Baseline: pure cost minimisation
            ├── AbstractSingleDetourModel
            │   └── TwoStageSingleDetourModel   # TSD: same-src/dest pooling
            ├── AbstractCorridorODModel
            │   ├── ZCorridorODModel            # cluster-activation corridor penalty
            │   └── XCorridorODModel            # assignment-crossing corridor penalty
            └── AbstractTransportationModel
                └── TransportationModel         # zone-pair anchor flow
```

## Model Reference

| Model | Key idea | Unique variables |
|---|---|---|
| `ClusteringBaseModel` | k-medoids single scenario | x[i,j] |
| `ClusteringTwoStageODModel` | Baseline two-stage; minimise walk+ride | y, z, x |
| `TwoStageSingleDetourModel` | Pooling via u (same-source) and v (same-dest); time-discretised | y, z, x, f, u, v |
| `ZCorridorODModel` | Penalise corridor (a,b) when α fires for both clusters | y, z, x, α, f_corridor |
| `XCorridorODModel` | Penalise corridor (a,b) only when OD assignment crosses it | y, z, x, f_corridor |
| `TransportationModel` | Separate pickup/dropoff assignment; anchor activation cost | y, z, x_pick, x_drop, f_transport, u_anchor |

## Decision Variables

| Var | Domain | Meaning |
|---|---|---|
| y[j] | {0,1} | Station j built (first stage) |
| z[j,s] | {0,1} | Station j active in scenario s; z≤y |
| x[s][od][j,k] | {0,1} | OD pair assigned to pickup j, dropoff k |
| f[s][t][j,k] | {0,1} | Vehicle flow on arc (j,k) at time t (TSD) |
| u[s][t] | {0,1} | Same-source pooling indicator (TSD) |
| v[s][t] | {0,1} | Same-destination pooling indicator (TSD) |
| α[a,s] | [0,1] | Cluster a activation in scenario s (ZCorridor) |
| f_corridor[g,s] | {0,1} | Corridor g used in scenario s (corridor models) |
| x_pick/x_drop | {0,1} | Per-origin/dest station assignment (Transportation) |
| f_transport[j,k,g,s] | ≥0 | Continuous flow within anchor g (Transportation) |

## Parameters

**Shared:** `k` (active stations/scenario), `l` (stations built), `in_vehicle_time_weight` (λ).

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

| Constraint | Formula |
|---|---|
| Station limit | Σⱼ y[j] = l |
| Activation limit | Σⱼ z[j,s] = k  ∀s |
| Activation linking | z[j,s] ≤ y[j]  ∀j,s |
| Assignment coverage | Σⱼₖ x[s][od][j,k] = 1  ∀s,od |
| Assignment-to-active (tight) | x[j,k,s] ≤ z[j,s],  x[j,k,s] ≤ z[k,s] |
| Z-cluster activation | α[a,s] ≥ z[i,s]  ∀i∈Cₐ |
| Z-corridor activation | f[g,s] ≥ α[a,s]+α[b,s]-1  for g=(a,b) |
| X-corridor activation | f[g,s] ≥ Σ x[od,j,k,s]  for j∈Cₐ, k∈C_b |

## Objective Components

- **Walking cost:** demand × walking distance to pickup/dropoff station
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
