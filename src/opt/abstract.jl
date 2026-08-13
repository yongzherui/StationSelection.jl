"""
Root types for the Problem/Formulation/Solver split every live optimization model in
this package is built on:

    run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver)
        = optimize_model(build_model(problem, formulation, solver), solver)

(`AbstractSolver` lives in `opt/solvers/utils/abstract.jl`, included separately since
solvers have no dependency on Problem/Formulation types.)
"""

using JuMP
using DataFrames

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
e.g. `StationSelectionProblem`.
"""
abstract type AbstractProblem end

"""
    AbstractFormulation

Root type for the mathematical/algorithmic encoding choices of a station-selection
problem -- *how* a given [`AbstractProblem`](@ref) is represented as a MILP/LP
(assignment policy, relaxation, column-generation pooling knobs, tight vs. loose
linking constraints, ...). Concrete subtypes are named `<Family>Formulation`, e.g.
`AggregateODRouteBaseFormulation`.
"""
abstract type AbstractFormulation end
