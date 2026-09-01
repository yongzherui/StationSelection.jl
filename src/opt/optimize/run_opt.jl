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

    gate_start = time()
    infeasibility_reason = check_feasibility(problem, formulation, solver)
    gate_sec = time() - gate_start
    if infeasibility_reason !== nothing
        return _infeasible_result(m, gate_sec, infeasibility_reason)
    end

    return optimize_model(m, solver)
end

"""
    check_feasibility(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver)

Optional fast necessary-condition gate, called by `run_opt` after `build_model` and
before `optimize_model` on every solve. Defaults to a no-op here; a
`(problem, formulation, solver)` combination that has a cheaper way to prove infeasibility
than actually solving the model `build_model` just built (e.g. `AggregateODRouteBaseFormulation`/
`AggregateODRouteJointRoutingAssignmentFormulation`,
`optimize/aggregate_od_route/direct/build_feasibility.jl`) adds its own method dispatching
on `formulation`'s concrete type.

# Returns
- `nothing` to let the solve proceed.
- an `AbstractString` explaining the refutation to abort it. `run_opt` then returns a
  `SOLVE_INFEASIBLE` `OptResult` carrying that string as
  `metadata["infeasibility_reason"]`, rather than raising.

A proven-infeasible instance is an *answer about the problem*, not a usage error, so it
is reported through the same `OptResult` channel as every other outcome -- a caller
sweeping a grid of `k` values can record it and move on instead of wrapping each
`run_opt` in a `try`. Genuine usage errors (a malformed formulation, an inconsistent
instance) should still throw.
"""
function check_feasibility(
        problem::AbstractProblem,
        formulation::AbstractFormulation,
        solver::AbstractSolver,
    )::Union{Nothing, String}
    return nothing
end
