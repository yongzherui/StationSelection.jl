# Solvers

An [`AbstractSolver`](@ref) is the algorithm, not the model. Every solver shares
[`SolverOptions`](@ref); each adds the knobs its own algorithm needs.

| solver | how it solves | used by |
| --- | --- | --- |
| [`DirectMIPSolver`](@ref) | one `optimize!` call | all Clustering formulations, `AggregateODRouteBaseFormulation` |
| [`CGSolver`](@ref) | column-generation outer loop | `AggregateODRouteJointRoutingAssignmentFormulation` |
| [`BendersSolver`](@ref) | Benders outer loop | the same Joint formulation |

`CGSolver` and `BendersSolver` each delegate their *search* to a config object — a
[`CGPricingConfig`](@ref) and a [`BendersSubproblemConfig`](@ref). Those are solver
settings rather than formulation ones for one reason: a search algorithm does not change
the model, so two runs differing only in a pricer or an oracle are solving exactly the
same problem.

{{autodocs opt/solvers/utils}}
