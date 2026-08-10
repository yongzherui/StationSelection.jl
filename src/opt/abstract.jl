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
        └── AbstractODModel        # OD pair assignment
```
"""

using JuMP
using DataFrames

export AbstractOptimizationProblem
export AbstractStationSelectionModel
export AbstractSingleScenarioModel
export AbstractMultiScenarioModel
export AbstractTwoStageModel
export AbstractBendersDualProblem

"""
    AbstractOptimizationProblem

Root type for anything `run_opt`/`build_model` can construct and solve --
not just station-selection models (`AbstractStationSelectionModel`) but also
auxiliary problems that share the `build_model(problem, data) -> BuildResult`
/ `run_opt(data, problem, solver) -> OptResult` contract without themselves
selecting stations or assignments, e.g. a Benders decomposition's
cut-derivation LPs (see [`AbstractBendersDualProblem`](@ref)).
"""
abstract type AbstractOptimizationProblem end

"""
    AbstractStationSelectionModel

Base abstract type for all station selection optimization models.

All concrete model types should inherit from this or one of its subtypes.
Each concrete type must implement:
- `build_model(model, data; optimizer_env=nothing)` - construct the JuMP model
- `extract_result(model, m, data)` - extract results after optimization
"""
abstract type AbstractStationSelectionModel <: AbstractOptimizationProblem end

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
    AbstractBendersDualProblem <: AbstractOptimizationProblem

Sibling of `AbstractStationSelectionModel`, not a subtype of it: these are
dual-feasibility LPs over a Benders decomposition's own cut algebra (e.g. a
Magnanti-Wong-style core-point finder, or a fixed-dual-block completion LP)
-- their variables are duals (`alpha`, `rho`, `sigma`, a core-point `delta`,
...), not stations or OD assignments, so there is no meaningful
`mapping::AbstractStationSelectionMap` for them to report. `build_model`
still returns a `BuildResult` (with `mapping=EmptyStationSelectionMap()`,
the same placeholder `run_opt`'s pre-solve feasibility short-circuit already
uses) and `run_opt` still returns an `OptResult`, but callers should read
the solved dual values off `OptResult.duals`, not `OptResult.solution`.
"""
abstract type AbstractBendersDualProblem <: AbstractOptimizationProblem end
