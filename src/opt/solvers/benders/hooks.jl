"""
The formulation-specific hooks `BendersSolver`'s loop calls, plus the generic defaults for
the three that do not actually vary per formulation.

The two mandatory hooks (`extract_incumbent`, `solve_subproblem`) and the one
mandatory-but-mechanical one (`add_benders_cut!`) throw here; real implementations are
methods on `mapping`'s concrete type -- `AggregateODRouteMap`'s live in
`../../optimize/aggregate_od_route/benders/dispatch.jl`. `mapping` is passed as its own
positional argument precisely so those methods can dispatch on it; see the `BendersSolver`
docstring in `solver.jl`.
"""

"""
    extract_incumbent(build_result, mapping, m) -> incumbent

The master's current first-stage decision, in whatever form this decomposition's
subproblem wants to be handed it (for the joint routing+assignment pair: a
`Vector{Float64}` of `y` values, rounded).
"""
function extract_incumbent(build_result::BuildResult, mapping, m::JuMP.Model)
    throw(MethodError(extract_incumbent, (build_result, mapping, m)))
end

"""
    solve_subproblem(build_result, mapping, m, incumbent, solver) -> subproblem_result

Evaluate the second stage at `incumbent` and return everything the loop and the cut
builder need: the exact second-stage cost (the run's upper bound, via
`benders_upper_bound`) and the dual information each cut is derived from.

Must solve the second stage to *optimality*. A truncated or otherwise non-optimal
subproblem solve yields duals that are not a valid underestimator, so the resulting cut
can exclude the true optimum -- an implementation should raise rather than return a weak
cut.
"""
function solve_subproblem(build_result::BuildResult, mapping, m::JuMP.Model, incumbent, solver::BendersSolver)
    throw(MethodError(solve_subproblem, (build_result, mapping, m, incumbent, solver)))
end

"""
    add_benders_cut!(build_result, mapping, m, subproblem_result, solver) -> Int

Add this iteration's optimality cut(s) to the master in place, and return how many were
actually **new**.

The count is not decoration: Benders subproblems are routinely dual-degenerate, and a
cut generator that re-derives a cut the master already carries makes no progress while
still reporting a fresh iteration. The loop stops on a zero count
(`stop_reason="cut_repeated"`) rather than spinning to `max_iterations`, so an
implementation must deduplicate rather than blindly `@constraint`.
"""
function add_benders_cut!(build_result::BuildResult, mapping, m::JuMP.Model, subproblem_result, solver::BendersSolver)::Int
    throw(MethodError(add_benders_cut!, (build_result, mapping, m, subproblem_result, solver)))
end

"""
    benders_upper_bound(subproblem_result) -> Float64

The second stage's total cost at the incumbent -- i.e. this iteration's upper bound on the
problem. Defaults to the `total_objective` field, which is the convention every
`solve_subproblem` result type in this package follows; override only for a result type
that cannot carry that field.

Note this is the *whole* objective for a decomposition whose first stage is costless (the
joint routing+assignment master is one). A decomposition with real first-stage cost must
add it, which is what overriding this hook is for.
"""
benders_upper_bound(subproblem_result)::Float64 = Float64(subproblem_result.total_objective)

"""
    benders_converged(build_result, mapping, m, subproblem_result, solver;
                      lower_bound, upper_bound) -> Bool

Has the run proven optimality? The default is the standard relative gap test against
`solver.optimality_tol`, which is not formulation-specific -- hence a real default rather
than a `MethodError`, unlike the hooks above. Override only for a decomposition with a
cheaper or stronger stopping certificate.

Returns `false` while `upper_bound` is infinite: no second stage has been evaluated yet,
so there is nothing to have converged to.
"""
function benders_converged(build_result::BuildResult, mapping, m::JuMP.Model,
        subproblem_result, solver::BendersSolver;
        lower_bound::Float64, upper_bound::Float64)::Bool
    isfinite(upper_bound) || return false
    isfinite(lower_bound) || return false
    return upper_bound - lower_bound <= solver.optimality_tol * max(1.0, abs(upper_bound))
end
