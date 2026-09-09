"""
Solving the joint routing+assignment Benders subproblems at a master incumbent, and
turning their LP duals into cut data.

# The cut this derives

At a fixed `yhat`, scenario `s`'s subproblem is

    min  sum_p c_walk[s,p] x_walk[s,p] + sum_c f_c theta_c
    s.t. (alpha[s,p] >= 0)      x_walk[s,p] + sum_{c covers (s,p)} theta_c >= 1
         (gammaO[(s,p),j] >= 0) sum_{c: (s,p) picked up at j} theta_c <= yhat_j
         (gammaD[(s,p),k] >= 0) sum_{c: (s,p) dropped off at k} theta_c <= yhat_k
         theta, x_walk >= 0

with `alpha`/`gamma` exactly as `extract_joint_routing_assignment_duals` defines them
(`gamma` being the NEGATED duals of the `<=` rows, so both families are non-negative).
Its dual objective is

    sum_p alpha[s,p] - sum_j Gamma[s,j] yhat_j,     Gamma[s,j] = sum_p (gammaO[(s,p),j] + gammaD[(s,p),j])

and -- the whole point -- the dual's feasible region does not mention `yhat` at all. So
the same `(alpha, gamma)` is dual-feasible for **every** `y`, and

    Theta_s >= sum_p alpha[s,p] - sum_j Gamma[s,j] y_j

is a valid underestimator of scenario `s`'s second-stage cost everywhere, tight at the
`yhat` it was derived at. `Gamma[s,j]` is the same per-station dual aggregate the
station-fixing work calls `Gamma_j`.

# Two things that make this simpler than textbook Benders

**No feasibility cuts.** `x_walk` exists for every positive-demand group and carries no
`y` linking whatsoever, so "walk everybody, activate nothing" is feasible for any `y` --
the subproblem is feasible and bounded at every incumbent, and only optimality cuts are
ever needed. (Ultimately this rests on
`_aggregate_od_route_allow_walk_only` being unconditionally `true` for this family; a
variant without direct walking would need feasibility cuts, and `solve_subproblem`
raising on an infeasible subproblem is what would surface that rather than silently
producing an invalid cut.)

**No bound duals in the cut.** Neither `theta` nor `x_walk` carries an upper bound in the
relaxed build -- both are created with `lower_bound = 0.0` only, the coverage rows being
what hold them near 1 -- so the dual has no bound-multiplier term and the cut is exactly
the two-family expression above. If anyone ever adds `theta <= 1`, this derivation needs a
third term and silently under-cuts without it.

# The strong-duality check

Every solve verifies `sum(alpha) - sum(Gamma .* yhat) == objective` before the cut is
built. This is not a formality: it is one line that catches a wrong dual sign, a linking
row family missed in the aggregation, a `theta` bound sneaking in, or a subproblem whose
`mapping` disagrees with the master's -- each of which otherwise produces a run that
converges to a confidently wrong number with nothing anywhere raising.
"""

export JointRoutingAssignmentBendersScenarioResult
export JointRoutingAssignmentBendersSubproblemResult

"""
    JointRoutingAssignmentBendersScenarioResult

One scenario's second-stage evaluation: its exact cost at the incumbent, and the cut
derived from it (`cut_constant` = `sum_p alpha`, `y_coefficients[j]` = `Gamma[s,j]`,
sparse -- a station in no linking row of this scenario is simply absent).

`reduced_cost_mismatch` is a diagnostic, not a check: with `y` fixed, JuMP's
`reduced_cost(y[j])` is a second, independent route to `-Gamma[s,j]`, and this records the
largest disagreement between the two. It is reported rather than enforced because the
strong-duality identity (see this file's docstring) is the assertion that actually
establishes cut validity, and the sign convention of a fixed variable's reduced cost is a
solver-interface detail rather than a property of the decomposition.
"""
struct JointRoutingAssignmentBendersScenarioResult
    scenario::Int
    objective::Float64
    cut_constant::Float64
    y_coefficients::Dict{Int, Float64}
    reduced_cost_mismatch::Float64
end

"""
    JointRoutingAssignmentBendersSubproblemResult

Every scenario's evaluation at one incumbent. `total_objective` is the incumbent's exact
total second-stage cost, which -- the first stage being costless here -- is the whole
objective, i.e. the run's upper bound (`benders_upper_bound` reads this field).
"""
struct JointRoutingAssignmentBendersSubproblemResult
    scenarios::Vector{JointRoutingAssignmentBendersScenarioResult}
    total_objective::Float64
    solve_sec::Float64
end

"""
    _solve_joint_routing_assignment_benders_subproblems(m, incumbent, solver)
        -> JointRoutingAssignmentBendersSubproblemResult

Pin `y` to `incumbent` in every stashed per-scenario subproblem, solve each as an LP, and
read the cut data off the duals.

The models are the ones `build_master.jl` built once; only `JuMP.fix` values change
between iterations, so each solve warm-starts from the previous basis.
"""
function _solve_joint_routing_assignment_benders_subproblems(
        m::JuMP.Model,
        incumbent::Vector{Float64},
        solver::BendersSolver,
    )::JointRoutingAssignmentBendersSubproblemResult
    builds = m[:benders_subproblem_builds]::Vector{BuildResult}
    t0 = time()
    results = JointRoutingAssignmentBendersScenarioResult[]
    total = 0.0
    for build in builds
        push!(results, _solve_one_joint_routing_assignment_benders_subproblem(
            build, incumbent, solver,
        ))
        total += results[end].objective
    end
    return JointRoutingAssignmentBendersSubproblemResult(results, total, time() - t0)
end

function _solve_one_joint_routing_assignment_benders_subproblem(
        build::BuildResult,
        incumbent::Vector{Float64},
        solver::BendersSolver,
    )::JointRoutingAssignmentBendersScenarioResult
    sm = build.model
    scenario = Int(sm[:benders_subproblem_scenario])
    y = sm[:y]
    # `force=true` is required, not optional: `add_station_selection_variables!` gives the
    # relaxed `y` explicit [0,1] bounds, and `fix` refuses to override existing bounds
    # without it.
    for j in eachindex(y)
        JuMP.fix(y[j], incumbent[j]; force = true)
    end
    isnothing(solver.subproblem.time_limit_sec) ||
        set_time_limit_sec(sm, solver.subproblem.time_limit_sec)

    optimize!(sm)
    status = JuMP.termination_status(sm)
    status == MOI.OPTIMAL || error(
        "Benders subproblem for scenario $scenario returned $status, not OPTIMAL. A cut " *
        "may only be derived from an optimally solved subproblem -- a truncated or " *
        "infeasible one has no valid dual, and the resulting cut could exclude the true " *
        "optimum. (Infeasible would additionally contradict this formulation's " *
        "always-available direct-walk coverage; see subproblem.jl's docstring.)",
    )
    objective = JuMP.objective_value(sm)

    alpha_sum = 0.0
    for (_key, con) in sm[:joint_routing_assignment_coverage]
        alpha_sum += dual(con)
    end
    # Gamma[j], aggregated over both linking families and every demand group. Same sign
    # convention as extract_joint_routing_assignment_duals: negate the `<=` rows' duals so
    # gamma >= 0.
    coefficients = Dict{Int, Float64}()
    for links in (sm[:joint_routing_assignment_pickup_link], sm[:joint_routing_assignment_dropoff_link])
        for (key, con) in links
            gamma = -dual(con)
            gamma == 0.0 && continue
            j = key[2]
            coefficients[j] = get(coefficients, j, 0.0) + gamma
        end
    end

    implied = alpha_sum
    for (j, gamma) in coefficients
        implied -= gamma * incumbent[j]
    end
    isapprox(implied, objective; rtol = 1e-6, atol = 1e-6) || error(
        "Benders subproblem scenario $scenario failed the strong-duality check: the dual " *
        "objective implied by the extracted duals is $implied but the primal optimum is " *
        "$objective. The cut derived from these duals would not be a valid " *
        "underestimator -- see subproblem.jl's docstring for the identity and what " *
        "breaking it usually means.",
    )

    # Independent cross-check on the same numbers (diagnostic only, see the result type).
    # Guarded, and NaN on failure: a fixed variable's reduced cost is a solver-interface
    # detail, and this must never be the thing that fails a run whose cut validity the
    # strong-duality check above has already established.
    mismatch = try
        worst = 0.0
        for j in eachindex(y)
            worst = max(worst, abs(JuMP.reduced_cost(y[j]) + get(coefficients, j, 0.0)))
        end
        worst
    catch
        NaN
    end

    return JointRoutingAssignmentBendersScenarioResult(
        scenario, objective, alpha_sum, coefficients, mismatch,
    )
end
