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
export AbstractRoutingModel

"""
    AbstractStationSelectionModel

Base abstract type for all station selection optimization models.

All concrete model types should inherit from this or one of its subtypes.
Each concrete type must implement:
- `build_model(model, data, optimizer_env)` - construct the JuMP model
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
    AbstractPoolingModel <: AbstractTwoStageModel
We have two stage model for clustering

But in the pooling model, we start concerning ourselves with basic pooling mechanisms of the vehicles
"""
abstract type AbstractPoolingModel <: AbstractTwoStageModel end
