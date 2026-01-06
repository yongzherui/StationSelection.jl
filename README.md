# StationSelection.jl

A Julia package for optimizing virtual bus stop (VBS) locations for microtransit systems.

## Overview

This package implements various optimization methods for selecting optimal station locations based on customer demand, walking costs, and routing constraints.

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
- **Multiple Optimization Methods**:
  - Base clustering (k-medoids)
  - Ideal clustering
  - Two-stage with L pre-selection
  - Two-stage with lambda routing weight
  - Routing-transportation formulation
  - Origin-destination pair formulation

## Usage

```julia
using StationSelection

# Load data
stations = read_candidate_stations("data/stations.csv")
requests = read_customer_requests("data/orders.csv")

# Compute costs
walking_costs = compute_station_pairwise_costs(stations)

# Generate scenarios
scenarios = generate_scenarios(Date("2025-05-12"), Date("2025-05-12"); segment_hours=4)

# Run optimization
result = clustering_two_stage_l_od_pair(
    stations,
    30,  # k stations per scenario
    requests,
    walking_costs,
    walking_costs,
    routing_costs,
    scenarios;
    l=50,  # total stations
    lambda=0.0
)

# Export results
export_results(result, "output_dir")
```

## Package Structure

```
src/
├── StationSelection.jl      # Main module
├── data/                    # Data loading
│   ├── stations.jl
│   └── requests.jl
├── optimization/            # Optimization methods
│   ├── base.jl
│   ├── ideal.jl
│   ├── two_stage_l.jl
│   ├── two_stage_lambda.jl
│   ├── routing_transport.jl
│   └── origin_dest_pair.jl
└── utils/                   # Utilities
    ├── coords.jl
    ├── results.jl
    ├── costs.jl
    ├── scenarios.jl
    ├── export.jl
    └── logging.jl
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
