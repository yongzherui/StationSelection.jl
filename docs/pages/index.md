# StationSelection.jl

Virtual Bus Stop (VBS) location optimisation for microtransit: a stochastic MILP that
selects which stations to build, and — for the Clustering formulations — which of those to
activate in each scenario.

## One entry point

```julia
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver)
```

The package is organised around that signature, and the three arguments are three separate
questions:

| | question | page |
| --- | --- | --- |
| [`AbstractProblem`](@ref) | *what* is being solved — instance data plus the business decision on top of it (how many stations, how far people will walk) | [Problems](@ref) |
| [`AbstractFormulation`](@ref) | *how* that is encoded mathematically — assignment policy, staging, cost weights | [Formulations](@ref) |
| [`AbstractSolver`](@ref) | *which algorithm* solves it — a single MIP solve, column generation, or Benders | [Solvers](@ref) |

The split is load-bearing rather than cosmetic. A pricer and a Benders subproblem oracle
are *search algorithms*, so they live on the solver: two runs differing only in one solve
the identical model, which is what makes a mode sweep a comparison rather than a
confound.

```julia
problem     = StationSelectionProblem(data, 10; max_walking_distance=300)
formulation = ClusteringTwoStageODFormulation(5; in_vehicle_time_weight=1.0)
solver      = DirectMIPSolver(config=SolverOptions(silent=true))
result      = run_opt(problem, formulation, solver)
```

To build a model without solving it, use `build_model`, which returns a
[`BuildResult`](@ref) -- see [Model construction](@ref) for the method per combination.

## Reading a result

`run_opt` returns an [`OptResult`](@ref). Two fields deserve attention before any number
on it is used:

- `termination_status` is a package-owned [`SolveStatus`](@ref), **not** a
  `MOI.TerminationStatusCode`. See [Results & solve status](@ref).
- `metadata` carries the scope of the claim — most importantly
  `"cg_optimality_scope"`, which says which route universe an `OPTIMAL` covers.

## Where the rest of the documentation lives

This site documents the *code*. Two other places carry things it deliberately does not:

- `CLAUDE.md` at the package root is the orientation map — what is live, what pairs with
  what, and what was removed.
- `notes/` holds the dated experimental record. **Every measured number belongs there**,
  with the instance, seed and job it came from. A docstring that needs a measurement cites
  the note rather than restating it, so there is one copy to keep true.

{{autodocs opt/optimize/run_opt.jl opt/abstract.jl}}
