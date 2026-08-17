# StationSelection.jl

A Julia package for optimizing virtual bus stop (VBS) locations for microtransit systems.

## Overview

This package implements optimization models for selecting station locations based on customer demand, walking costs, and routing constraints.

## Installation

Since this is a local package, activate it from the parent project:

```julia
using Pkg
Pkg.develop(path="StationSelection.jl")
using StationSelection
```

## Features

- **Data Loading**: Read and preprocess candidate stations and customer requests
- **Coordinate Transformation**: Convert between BD-09, GCJ-02, and WGS84 coordinate systems
- **Optimization**: a Problem/Formulation/Solver split (`opt/abstract.jl`) — six live
  formulations spanning single- and two-stage clustering (k-medoids, OD assignment,
  optional flow regularization) and aggregate-OD route models (direct MIP or column
  generation). See `StationSelection.jl/CLAUDE.md` or `src/opt/README.md` for the full
  reference.

## Usage

```julia
using StationSelection

# Load data
stations = read_candidate_stations("data/stations.csv")
requests = read_customer_requests("data/orders.csv")

# Compute costs
# `walking_costs` stores walking time in seconds (distance / walking_speed).
walking_costs = compute_station_pairwise_costs(stations)
routing_costs = read_routing_costs_from_segments("data/segment.csv", stations)

# Build problem data
scenarios = [("2025-05-12 08:00:00", "2025-05-12 12:00:00")]
data = create_station_selection_data(
    stations, requests, walking_costs;
    routing_costs=routing_costs,
    scenarios=scenarios
)

# Choose a problem + formulation + solver
problem = StationSelectionProblem(data, 10; max_walking_distance=800.0)  # k=10 stations, seconds
formulation = ClusteringTwoStageODFormulation(5; in_vehicle_time_weight=1.0)  # l=5 active/scenario
solver = DirectMIPSolver(config=SolverOptions(silent=true))

# Run optimization
result = run_opt(problem, formulation, solver)

# Access results (OptResult)
println(result.termination_status)
println(result.objective_value)

# Export results (metadata + optional station_df)
export_results(result, "output_dir"; station_df=nothing)
```

### Key return types

- `build_model(problem, formulation, solver) -> BuildResult`
- `run_opt(problem, formulation, solver) -> OptResult`

See `src/opt/README.md` for details on formulations and flags.

## Package Structure

```
src/
├── StationSelection.jl      # Main module and exports
├── data/                    # Data loading + mappings
├── opt/                     # Model definitions, build/run, variables, constraints, objectives
├── utils/                   # Utilities + result/export helpers
└── README.md                # Code-level overview (this file)
```

## Dependencies

- JuMP (optimization modeling)
- Gurobi (solver)
- DataFrames (data handling)
- CSV (file I/O)
- JSON (result export)

## Development

This package is designed to be stable and reusable. Experiments should be conducted in separate directories using this package as a dependency.

## License

MIT License
