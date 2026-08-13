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
export AbstractColumnGenerationAlgorithm
export AbstractProblem
export AbstractFormulation

"""
    AbstractProblem

Root type for *what* is being solved in a station-selection problem, independent of how
it is mathematically encoded (see [`AbstractFormulation`](@ref)) or which algorithm
solves it (see `AbstractSolver`). Every concrete subtype is a composite of:

- `data`: the problem's instance data (`StationSelectionData` -- stations, requests,
  costs, scenarios)
- the business parameters scoping the decision on top of that data (station counts,
  walking/waiting limits, cost weights, capacities, ...)

so that `run_opt(problem, formulation, solver)` never needs instance data passed
separately -- it lives inside `problem`. Concrete subtypes are named `<Family>Problem`,
e.g. `AggregateODRouteProblem`.
"""
abstract type AbstractProblem end

"""
    AbstractFormulation

Root type for the mathematical/algorithmic encoding choices of a station-selection
problem -- *how* a given [`AbstractProblem`](@ref) is represented as a MILP/LP
(assignment policy, relaxation, column-generation pooling knobs, tight vs. loose
linking constraints, ...). Concrete subtypes are named `<Family>Formulation`, e.g.
`AggregateODRouteFormulation`.
"""
abstract type AbstractFormulation end

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

"""
    AbstractColumnGenerationAlgorithm

Root type for the column-generation algorithm dispatched on by
`ColumnGenerationSolver.algorithm` (mirroring `AbstractBendersDecomposition` for
`BendersSolver.decomposition`) -- e.g. `AggregateODRouteCG`, `JointRoutingAssignmentCG`. Each
concrete algorithm supplies its own methods for the shared column-generation outer loop's
dispatched hooks (see `label_setting/aggregate_od_route/generic_runner.jl`): build the restricted master, solve it, extract
duals, price and add columns, and finalize the result.
"""
abstract type AbstractColumnGenerationAlgorithm end

export AggregateODRouteCG

"""
    AggregateODRouteCG

Column-generation algorithm over the aggregate station-pair-per-request formulation (one
assignment variable per `(scenario, origin, destination)` request) -- `run_aggregate_od_route_column_generation`'s
algorithm. The knobs shared under matching semantics (`n_candidates`, `reduced_cost_tol`,
`pricing_time_limit_sec`, `max_columns_per_iteration`, `final_ip_time_limit_sec`,
`max_iterations`) live on `ColumnGenerationSolver`, exactly as before.

The remaining fields exist only because `run_aggregate_od_route_column_generation` (the public,
still-independently-callable function this algorithm's hooks were extracted from -- see
`label_setting/aggregate_od_route/generic_runner.jl`) has its own kwarg surface, used by direct callers
(`_solve_fixed_route_covering_by_cg`, tests, scripts) that predate `ColumnGenerationSolver`
entirely: per-call log file paths, and pricing knobs with dynamic (`model`/solver-dependent)
defaults that can't be resolved until a hook actually runs against a concrete `model`/`solver`
pair, hence the `Union{Nothing, _}` sentinels (resolved in `_cg_build_master`). Every field
defaults to a placeholder here since a default-constructed `AggregateODRouteCG()` is also what
`label_setting/aggregate_od_route/dispatch.jl`'s `_default_cg_algorithm` returns purely for its `isa`
mismatch check -- `run_aggregate_od_route_column_generation`'s own wrapper always constructs its
own instance with real values from its own kwargs.

(Restored here from `optimize/aggregate_od_route/benders/archive/solver_types.jl`, where an
external reorganization had archived it alongside genuinely-dead Benders scaffolding -- unlike
that scaffolding, this marker is still load-bearing for the still-intact `label_setting/aggregate_od_route/`
CG engine.)
"""
struct AggregateODRouteCG <: AbstractColumnGenerationAlgorithm
    pricing_initial_sec::Union{Nothing, Float64}
    pricing_ramp_factor::Float64
    use_station_simple::Union{Nothing, Bool}
    profile_pricing::Bool
    verbose::Bool
    cg_log_path::Union{Nothing, String}
    column_log_path::Union{Nothing, String}
    dual_log_path::Union{Nothing, String}

    function AggregateODRouteCG(;
        pricing_initial_sec::Union{Number, Nothing}=nothing,
        pricing_ramp_factor::Number=1.0,
        use_station_simple::Union{Bool, Nothing}=nothing,
        profile_pricing::Bool=false,
        verbose::Bool=true,
        cg_log_path::Union{AbstractString, Nothing}=nothing,
        column_log_path::Union{AbstractString, Nothing}=nothing,
        dual_log_path::Union{AbstractString, Nothing}=nothing,
    )
        isnothing(pricing_initial_sec) || pricing_initial_sec > 0 ||
            throw(ArgumentError("pricing_initial_sec must be positive"))
        pricing_ramp_factor > 0 || throw(ArgumentError("pricing_ramp_factor must be positive"))
        new(
            isnothing(pricing_initial_sec) ? nothing : Float64(pricing_initial_sec),
            Float64(pricing_ramp_factor),
            use_station_simple,
            profile_pricing,
            verbose,
            isnothing(cg_log_path) ? nothing : String(cg_log_path),
            isnothing(column_log_path) ? nothing : String(column_log_path),
            isnothing(dual_log_path) ? nothing : String(dual_log_path),
        )
    end
end
