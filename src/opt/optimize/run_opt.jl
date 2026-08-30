using Dates
using Logging

"""
    run_opt(instance, formulation, solver)

Construct and solve a station selection optimization model.

# Arguments
- `problem::AbstractProblem`: Problem data
- `formulation::AbstractFormulation`: Mathematical formulation
- `solver::AbstractSolver`: Solve algorithm and execution config

# Returns
- `OptResult`
"""
function run_opt(
        problem::AbstractProblem,
        formulation::AbstractFormulation,
        solver::AbstractSolver
    )

    m = build_model(problem, formulation, solver)
    check_feasibility(problem, formulation, solver)

    return optimize_model(m, solver)
end

"""
    check_feasibility(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver)

Optional fast necessary-condition gate, called by `run_opt` after `build_model` and
before `optimize_model` on every solve. Defaults to a no-op here; a
`(problem, formulation, solver)` combination that has a cheaper way to prove infeasibility
than actually solving the model `build_model` just built (e.g. `AggregateODRouteBaseFormulation`/
`AggregateODRouteJointRoutingAssignmentFormulation` via `aggregate_od_route_check_feasibility`,
`optimize/aggregate_od_route/direct/build_feasibility.jl`) adds its own method dispatching
on `formulation`'s concrete type. Must throw (not return a status) to actually gate the
solve -- `run_opt` doesn't inspect this call's return value.
"""
function check_feasibility(
        problem::AbstractProblem,
        formulation::AbstractFormulation,
        solver::AbstractSolver,
    )
    return nothing
end
