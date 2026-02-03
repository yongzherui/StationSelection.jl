# Optimization Module

This folder contains model definitions, mappings, and the build/run pipeline for
station selection optimization.

## Models

### TwoStageSingleDetourModel

Two-stage model with optional walking distance limits.

Constructor:

```julia
TwoStageSingleDetourModel(
    k, l, vehicle_routing_weight, time_window, routing_delay;
    in_vehicle_time_weight=vehicle_routing_weight,
    use_walking_distance_limit=false,
    max_walking_distance=nothing,
    tight_constraints=true,
    detour_use_flow_bounds=false
)
```

Behavior:

- If `use_walking_distance_limit=false`, dense assignment variables are created.
- If `use_walking_distance_limit=true`, a walking limit is enforced and sparse
  assignment variables are used based on valid (j,k) pairs.
- If `tight_constraints=false`, detour constraints use a single combined inequality
  instead of two tighter edge constraints.
- If `detour_use_flow_bounds=true`, detour bounds use flow variables `f` instead of
  summed assignment variables `x` (requires flow upper bounds).

### ClusteringTwoStageODModel

Two-stage clustering model with optional walking limits and variable reduction.

Constructor:

```julia
ClusteringTwoStageODModel(
    k, l;
    in_vehicle_time_weight=1.0,
    use_walking_distance_limit=false,
    max_walking_distance=nothing,
    variable_reduction=true,
    tight_constraints=true
)
```

Behavior:

- When `use_walking_distance_limit=true` and `variable_reduction=true`, sparse
  assignment variables are used.
- When `use_walking_distance_limit=true` and `variable_reduction=false`, dense
  assignment variables are used and walking limits are enforced via constraints.
- If `tight_constraints=false`, assignment-to-active uses a single combined inequality
  instead of two tighter station constraints.

### ClusteringBaseModel

Single-scenario clustering baseline (k-medoids).

## Build/Run API

### build_model

```julia
build_result = build_model(model, data; optimizer_env=nothing)
```

`BuildResult` fields:

- `model`: JuMP.Model
- `mapping`: AbstractStationSelectionMap
- `detour_combos`: DetourComboData or `nothing`
- `counts`: ModelCounts (always populated)
- `metadata`: Dict

### run_opt

```julia
opt_result = run_opt(model, data; silent=true, show_counts=false)
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
