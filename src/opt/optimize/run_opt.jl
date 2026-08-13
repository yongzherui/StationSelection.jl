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

    return optimize_model(m, solver)
end
