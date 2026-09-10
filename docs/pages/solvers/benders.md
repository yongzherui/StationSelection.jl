# BendersSolver

A Benders outer loop over the Joint formulation. `run_opt` still takes **one**
formulation — the monolith — and `build_model(problem, ::Monolith, ::BendersSolver)`
derives the master/subproblem pair, because Benders is an algorithm for the same model
rather than a different model. Those derived types are documented under
[Formulations](@ref).

Two properties of the answer travel with it in `metadata`, and both narrow what an
`OPTIMAL` means:

- **`benders_second_stage_relaxed` is always `true`.** The cut *is* the subproblem's LP
  dual, so the subproblem must be an LP, so a converged run is optimal for the **mixed**
  model (`y` binary, `θ`/`x_walk` continuous) — not for `DirectMIPSolver`'s all-binary
  optimum over the same pool.
- **`benders_optimality_scope`** reports `"max_stops_restricted"` when the enumeration cap
  narrowed the formulation's own `max_stops`, in which case optimality is claimed only
  over routes of at most that many stops.

Both bounds are reported. The lower bound is the master's `objective_bound` rather than
its `objective_value`, which a non-zero `MIPGap` would inflate into an invalid bound; the
upper bound is the best incumbent's exact second-stage cost. The reported objective is
always the upper bound — a lower bound is not a solution.

{{autodocs opt/solvers/benders/solver.jl}}

## Subproblem oracles

The oracle decides how `Q_s(ŷ)` and its duals are obtained. **A cut may only be derived
from an exhausted pricing round**: duals from a restricted pool are feasible for the
restricted dual only, and the resulting cut can exclude the true optimum. Every oracle
therefore ends in a proof of exhaustion, and a run that cannot produce one raises rather
than emitting a cut.

{{autodocs opt/solvers/benders/subproblem_config.jl}}

## Loop internals

{{autodocs opt/solvers/benders !opt/solvers/benders/solver.jl !opt/solvers/benders/subproblem_config.jl}}

## Building and cutting

{{autodocs opt/optimize/aggregate_od_route/benders}}
