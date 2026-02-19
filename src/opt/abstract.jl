"""
Abstract type hierarchy for station selection optimization models.

This module defines the abstract types that form the foundation of the
optimization model hierarchy. Each concrete model type should inherit
from one of these abstract types.

Type Hierarchy:
```
AbstractStationSelectionModel
├── AbstractSingleScenarioModel    # Single scenario (k-medoids style)
└── AbstractMultiScenarioModel     # Multiple scenarios
    └── AbstractTwoStageModel      # First-stage build + second-stage activate
        └── AbstractPoolingModel   # Includes passenger pooling decisions
```
"""

using JuMP
using DataFrames

export AbstractStationSelectionModel
export AbstractSingleScenarioModel
export AbstractMultiScenarioModel
export AbstractTwoStageModel
export AbstractCorridorODModel
export AbstractRoutingModel

"""
    AbstractStationSelectionModel

Base abstract type for all station selection optimization models.

All concrete model types should inherit from this or one of its subtypes.
Each concrete type must implement:
- `build_model(model, data; optimizer_env=nothing)` - construct the JuMP model
- `extract_result(model, m, data)` - extract results after optimization
"""
abstract type AbstractStationSelectionModel end

"""
    AbstractSingleScenarioModel <: AbstractStationSelectionModel

Models that optimize for a single scenario (or aggregated scenarios).

Examples: Basic k-medoids clustering, p-median problems.
"""
abstract type AbstractSingleScenarioModel <: AbstractStationSelectionModel end

"""
    AbstractMultiScenarioModel <: AbstractStationSelectionModel

Models that explicitly handle multiple scenarios.

Examples: Stochastic optimization, robust optimization.
"""
abstract type AbstractMultiScenarioModel <: AbstractStationSelectionModel end

"""
    AbstractTwoStageModel <: AbstractMultiScenarioModel

Two-stage stochastic models with:
- First stage: build/select permanent stations
- Second stage: activate subset of built stations per scenario

Examples: Two-stage with λ penalty, two-stage with L permanent stations.
"""
abstract type AbstractTwoStageModel <: AbstractMultiScenarioModel end

"""
    AbstractODModel <: AbstractTwoStageModel

Two-stage models with OD (origin-destination) pair assignment.
"""
abstract type AbstractODModel <: AbstractTwoStageModel end

"""
    AbstractCorridorODModel <: AbstractODModel

Two-stage models with corridor penalties for cross-zone vehicle movements.

Common properties:
- k: number of active stations per scenario
- l: number of stations to build
- in_vehicle_time_weight: weight for in-vehicle travel time costs
- corridor_weight: weight for corridor penalty
- max_cluster_diameter: routing distance diameter for station clustering
"""
abstract type AbstractCorridorODModel <: AbstractODModel end

"""
    AbstractSingleDetourModel <: AbstractODModel

Models with single-detour pooling mechanism.

Common properties:
- k: number of active stations per scenario
- l: number of stations to build
- vehicle_routing_weight: weight for vehicle routing costs
- in_vehicle_time_weight: weight for in-vehicle travel time in assignment costs
- time_window: time discretization window
- routing_delay: maximum detour delay
"""
abstract type AbstractSingleDetourModel <: AbstractODModel end

"""
    AbstractPoolingModel <: AbstractODModel

Models with more complex pooling mechanisms.
"""
abstract type AbstractPoolingModel <: AbstractODModel end
