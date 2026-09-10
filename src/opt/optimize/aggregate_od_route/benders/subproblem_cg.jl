"""
The `:column_generation` subproblem oracle: solve one scenario's second stage by pricing
columns on demand instead of over an enumerated pool.

# Why this is sound, stated once and precisely

A Benders cut is valid iff its duals are feasible for the dual of the **full-universe**
second-stage problem (`benders/subproblem.jl` derives that). Enumeration achieves this by
containing the universe. Column generation achieves it by *proving nothing improving
remains*: at CG convergence, every column `c` in the universe -- priced or not -- satisfies

    rc_c = f_c - sum_{p in c} alpha_p + sum gamma  >=  -reduced_cost_tol

and that inequality IS `c`'s dual constraint. So a converged round's `(alpha, gamma)` is
feasible for the full dual, the restricted LP's optimum equals the full LP's optimum
(standard CG argument), and the cut derived from it is globally valid.

Everything below follows from that one fact:

- **Convergence is not optional.** A budget-stopped or iteration-capped CG leaves duals that
  are feasible only for the *restricted* dual, and a cut from those can over-estimate the
  true `Q_s` and prune the optimum. This function therefore reports `converged` as a hard
  bit, and `solve_subproblem` refuses to build a cut without it.
- **The pool is accumulated across Benders iterations, deliberately.** Validity comes from
  exhaustion at the moment of extraction, not from the pool being frozen, so keeping columns
  is a pure warm start: later Benders iterations should exhaust in one or two rounds.
- **Pricing searches the FULL station set, not just the stations built at `yhat`.** This
  looks wasteful and is not. A column through an unbuilt station `j` still carries a dual
  constraint that must hold, so restricting the search would never verify it. And the
  apparent livelock -- `j` unused means its linking row `sum(theta) <= 0` is slack, so
  `gamma_j = 0`, so a candidate through `j` prices negative, enters, and is forced to zero --
  resolves itself: once such a column is in the pool that linking row has a term, the LP can
  raise `gamma_j` above zero, and the reduced cost goes non-negative. Those columns are the
  certificate that the cut holds at other `y`, not dead weight.

# What is reused

The pricer is the ordinary joint routing+assignment label-setting engine: `_run_pricing_round`
(`label_setting/round.jl`) with its `only_scenarios` restriction, `extract_joint_routing_assignment_duals`
for the duals, and `add_joint_routing_assignment_column!` to install columns. Nothing about
pricing is Benders-specific -- the subproblem is the joint master's own shape with `y` fixed,
so the same machinery applies unchanged. `_run_pricing_round` reads only `reduced_cost_tol`
and `parallel_scenario_pricing` off its `CGSolver` argument, which is why a lightweight
settings carrier suffices rather than a real CG solve.
"""

"""
    _benders_subproblem_cg_settings(config) -> CGSolver

A `CGSolver` used purely as a settings carrier for `_run_pricing_round`, which reads exactly
two fields off it (`reduced_cost_tol`, `parallel_scenario_pricing`). Constructing one is
cheaper and less brittle than threading those two values through the pricing API, and it
keeps the pricer's signature untouched.

`parallel_scenario_pricing` is deliberately `false`: each subproblem model holds ONE
scenario, so there is nothing to parallelise inside a round. Scenario-level parallelism, if
ever wanted, belongs in the Benders loop across subproblems -- not here.
"""
_benders_subproblem_cg_settings(config::BendersSubproblemConfig) = CGSolver(
    config = SolverOptions(silent = true),
    pricing = config.pricing,
    reduced_cost_tol = config.reduced_cost_tol,
    pricing_time_limit_sec = config.cg_pricing_time_limit_sec,
    parallel_scenario_pricing = false,
)

"""
    BendersSubproblemCGResult

Outcome of one scenario's inner CG solve. `converged` is the bit cut validity hangs on: it
means pricing EXHAUSTED (no column in the universe prices below `-reduced_cost_tol`), not
merely that the loop stopped.

`stop_reason` distinguishes the ways it can fail to converge, because they call for different
responses: `pricing_inconclusive` (budget) wants more time, `iteration_limit` wants a higher
cap, and `dedup_stall` is the known stale-`tau` livelock -- the pricer reports an improving
column whose `(scenario, signature)` is already pooled at no greater `tau`, so
`add_joint_routing_assignment_column!` skips it and the LP cannot change. That last one is a
real defect elsewhere in the stack, not a budget problem, and is reported as its own reason so
it is not misread as one.
"""
struct BendersSubproblemCGResult
    scenario::Int
    converged::Bool
    stop_reason::String
    cg_iterations::Int
    columns_added::Int
    pricing_sec::Float64
    lp_sec::Float64
end

"""
    _solve_joint_routing_assignment_subproblem_by_cg!(build, config) -> BendersSubproblemCGResult

Run CG on one already-`y`-fixed subproblem model until pricing exhausts.

Assumes `y` has already been fixed by the caller (`subproblem.jl` does it before dispatching
on the oracle), so the LP solved here is `Q_s` at the master's incumbent over the current
pool.
"""
function _solve_joint_routing_assignment_subproblem_by_cg!(
        build::BuildResult,
        config::BendersSubproblemConfig,
    )::BendersSubproblemCGResult
    sm = build.model
    mapping = build.mapping
    scenario = Int(sm[:benders_subproblem_scenario])
    data = sm[:joint_routing_assignment_data]
    pricing_formulation = sm[:joint_routing_assignment_pricing_formulation]
    settings = _benders_subproblem_cg_settings(config)

    cg_iterations = 0
    columns_added = 0
    pricing_sec = 0.0
    lp_sec = 0.0

    for iteration in 1:config.max_cg_iterations
        cg_iterations = iteration

        t_lp = time()
        optimize!(sm)
        lp_sec += time() - t_lp
        status = JuMP.termination_status(sm)
        status == MOI.OPTIMAL || return BendersSubproblemCGResult(
            scenario, false, "lp_$(status)", cg_iterations, columns_added,
            pricing_sec, lp_sec,
        )

        duals = extract_joint_routing_assignment_duals(sm)

        t_price = time()
        columns = _run_pricing_round(
            pricing_formulation, mapping, sm, duals, settings;
            only_scenarios = [scenario],
            time_limit = config.cg_pricing_time_limit_sec,
        )
        pricing_sec += time() - t_price

        if isempty(columns)
            # Empty AND exhausted is the certificate: no column in the universe prices
            # below the tolerance, so these duals are full-universe dual feasible and a cut
            # may be taken. Empty but NOT exhausted only means the search ran out of budget,
            # which proves nothing.
            exhausted = _cg_pricing_exhausted(sm)
            return BendersSubproblemCGResult(
                scenario, exhausted,
                exhausted ? "converged" : "pricing_inconclusive",
                cg_iterations, columns_added, pricing_sec, lp_sec,
            )
        end

        n_added = 0
        for column in columns
            _theta, action = add_joint_routing_assignment_column!(sm, data, mapping, column)
            action === :added && (n_added += 1)
        end
        columns_added += n_added
        if n_added == 0
            # Improving columns were priced but every one was skipped as an already-pooled
            # `(scenario, signature)` at no greater `tau`. The LP is unchanged, so the next
            # iteration would price the same thing forever. See the result type's docstring:
            # this is the stale-`tau` livelock, and it is NOT convergence -- returning
            # `converged` here would hand `solve_subproblem` duals that are only
            # restricted-feasible and produce an invalid cut.
            return BendersSubproblemCGResult(
                scenario, false, "dedup_stall", cg_iterations, columns_added,
                pricing_sec, lp_sec,
            )
        end
    end

    return BendersSubproblemCGResult(
        scenario, false, "iteration_limit", cg_iterations, columns_added,
        pricing_sec, lp_sec,
    )
end
