# Results & solve status

{{autodocs utils/core/results.jl}}

## SolveStatus is not a MOI code

`OptResult.termination_status` is a package-owned [`SolveStatus`](@ref). The MOI code
reports the status of the *last model object optimized*, which for `CGSolver` is the master
over a restricted column pool — so a budget-stopped run that proved nothing still left that
master at `MOI.OPTIMAL`, and the raw code claimed `OPTIMAL`. MOI also has no code for
"feasible but not proven optimal" (`MOI.FEASIBLE_POINT` is a *primal* status). The raw code
is preserved as `metadata["moi_termination_status"]`.

Member names carry a `SOLVE_` prefix because `using JuMP` re-exports bare
`OPTIMAL`/`INFEASIBLE` from MOI into scope; the printed labels drop it so result CSVs stay
readable.



## `OPTIMAL` is scoped to the universe that was priced

`SOLVE_OPTIMAL` asserts "no improving column remains **in the universe pricing
searched**". For `:station_simple` that is elementary routes only, and the status alone
does not say so. Every `CGSolver` result therefore carries
`metadata["cg_optimality_scope"]`, either `"full_route_universe"` or
`"elementary_routes_only"`.

**Check `cg_optimality_scope` before pooling a certified objective with others or treating
it as a true optimum** — filtering on `termination_status` alone cannot distinguish the
two.

A relaxation certificate is full-universe whatever the pricer: when
`cg_certified_by_relaxation` is `true`, the proof came from a bound on *every* real route,
not from the active pricer exhausting its own universe. So a run that priced most of its
columns under `:station_simple` and then certified by relaxation correctly reports
`"full_route_universe"`. That combination is not a bug.

## Feasibility

`run_opt`'s `check_feasibility` hook returns `nothing` to proceed or a reason `String` to
abort, which `run_opt` turns into a `SOLVE_INFEASIBLE` result carrying it as
`metadata["infeasibility_reason"]`. It does **not** throw — a proven-infeasible instance is
an answer, not a usage error.

{{autodocs opt/optimize/aggregate_od_route/direct/build_feasibility.jl}}

