export BendersSolver

"""
    BendersSolver <: AbstractSolver

Benders decomposition outer loop: solve the master, evaluate its incumbent first-stage
decision in the second stage, add an optimality cut derived from that evaluation, repeat
until the master's lower bound meets the best second-stage upper bound.

# The loop's contract

Each iteration produces both bounds, which is what makes the run reportable even when it
stops early:

- **lower bound** = the master's own optimum (`JuMP.objective_bound`, not
  `objective_value`, so a non-zero `MIPGap` cannot inflate it). Monotone across
  iterations; every cut only tightens it.
- **upper bound** = the second stage's exact cost at the master's incumbent. This is a
  genuine feasible solution, not a heuristic, so the *best* incumbent seen -- which need
  not be the last one -- is the run's answer.

`SOLVE_OPTIMAL` is reported only when the two met inside `optimality_tol`; a run stopped
by `max_iterations` or `total_time_limit_sec` reports `SOLVE_FEASIBLE` with both bounds in
its metadata, mirroring how `CGSolver` treats a budget-stopped master. The reported
objective is the upper bound, never the master's (a lower bound is not a solution).

# Hooks

Relies on formulation-specific hooks -- implemented per `AbstractFormulation` (or per
`AbstractProblem`), not here:

    extract_incumbent(build_result, mapping, m) -> incumbent
    solve_subproblem(build_result, mapping, m, incumbent, solver::BendersSolver) -> subproblem_result
    add_benders_cut!(build_result, mapping, m, subproblem_result, solver::BendersSolver) -> n_cuts_added
    benders_upper_bound(subproblem_result) -> Float64            # optional, defaults to `.total_objective`
    benders_converged(...; lower_bound, upper_bound) -> Bool     # optional, defaults to the gap test

`mapping` (`build_result.mapping`) is passed as its own positional argument, not just
read off `build_result`, so that formulation-specific methods can dispatch on its
concrete type (e.g. `mapping::AggregateODRouteMap`) -- mirrors `CGSolver`'s identical
hook-dispatch pattern (`opt/solvers/cg/solver.jl`), see that file's docstring for why
dispatching on `build_result::BuildResult` alone can't distinguish formulations.

Only two of the five are mandatory-per-formulation, because the other three are generic:
the gap test and the upper-bound accessor have real defaults (`hooks.jl`), and
`add_benders_cut!` returning a *count* is what lets the generic loop detect a stalled cut
generator (every cut in an iteration already present) and stop instead of spinning.

# Fields

`subproblem` is a `BendersSubproblemConfig` -- the second-stage oracle, on the solver for
the same reason `CGSolver.pricing` is (a search algorithm, not part of the model).
`total_time_limit_sec` is a wall cap over the whole loop, distinct from
`config.time_limit_sec`, which reaches only the master model's own `optimize!`.

`iteration_callback` receives one `NamedTuple` per iteration the moment it ends (mirrors
`CGSolver.iteration_callback`): `(; iteration, lower_bound, upper_bound, gap,
incumbent_objective, cuts_added, cuts_total, n_stations_built, master_sec, subproblem_sec)`.
Without it the loop's bound trajectory is invisible from outside, and "why did this converge
in 3 cuts" is not answerable after the fact -- the aggregate metadata reports only the final
state. A callback that throws is not caught: a diagnostic that fails silently is worse than
one that stops the run.

Live today for `AggregateODRouteJointRoutingAssignmentFormulation`, via the master/
subproblem formulation pair in
`opt/formulations/aggregate_od_route/joint_routing_assignment/` and the builds in
`opt/optimize/aggregate_od_route/benders/`.
"""
struct BendersSolver <: AbstractSolver
    config::SolverOptions
    max_iterations::Int
    optimality_tol::Float64
    subproblem::BendersSubproblemConfig
    total_time_limit_sec::Float64
    # Solve the per-scenario subproblems concurrently. They are independent at a fixed
    # incumbent -- separate JuMP models, separate Gurobi environments, no shared state, and
    # nothing written to the master inside the loop -- and they are ~99% of the wall, so
    # this is the one place scenario-level parallelism belongs (the pricing round's own
    # docstring says so: a Benders subproblem model holds ONE scenario, so there is nothing
    # to parallelise inside a round).
    #
    # Default `false` because the label-setting pricer may thread internally, and running
    # both levels oversubscribes: 3 scenarios on 4 threads leaves the searches nothing.
    # It shortens the wall; it does NOT enlarge any budget, and it cannot turn an
    # inconclusive certification into a conclusive one.
    parallel_scenarios::Bool
    iteration_callback::Union{Nothing, Function}

    function BendersSolver(;
            config::SolverOptions=SolverOptions(),
            max_iterations::Int=1_000,
            optimality_tol::Number=1e-6,
            subproblem::BendersSubproblemConfig=BendersSubproblemConfig(),
            total_time_limit_sec::Number=Inf,
            parallel_scenarios::Bool=false,
            iteration_callback::Union{Nothing, Function}=nothing,
        )
        max_iterations > 0 || throw(ArgumentError("max_iterations must be positive"))
        optimality_tol >= 0 || throw(ArgumentError("optimality_tol must be non-negative"))
        total_time_limit_sec > 0 ||
            throw(ArgumentError("total_time_limit_sec must be positive"))
        new(config, max_iterations, Float64(optimality_tol), subproblem,
            Float64(total_time_limit_sec), parallel_scenarios, iteration_callback)
    end
end
