"""
`BendersSolver`'s outer loop and its result packaging.

The loop itself is entirely generic -- master solve, incumbent extraction, second-stage
evaluation, bound update, convergence test, cut. Everything formulation-specific is behind
the hooks in `hooks.jl`.
"""

"""
    optimize_model(build_result::BuildResult, solver::BendersSolver) -> OptResult

Run the Benders loop over `build_result`'s master. See `BendersSolver`'s docstring for the
bound contract; `_benders_package_result` below for why the master is not the thing
reported.
"""
function optimize_model(build_result::BuildResult, solver::BendersSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)
    mapping = build_result.mapping

    st = BendersLoopState(time())
    for iteration in 1:solver.max_iterations
        if time() - st.start_time >= solver.total_time_limit_sec
            st.stop_reason = "total_budget"
            break
        end
        st.iterations = iteration

        t_master = time()
        optimize!(m)
        st.master_sec += time() - t_master
        if JuMP.termination_status(m) != MOI.OPTIMAL
            # Not a silent break: a master that did not solve to optimality invalidates
            # the lower bound this iteration would have contributed, so the run has to
            # report which status stopped it rather than looking like a clean finish.
            st.stop_reason = "master_$(JuMP.termination_status(m))"
            # An INFEASIBLE master proves the PROBLEM infeasible, and may be reported as
            # such -- unlike CGSolver, whose infeasible restricted master says only that
            # its column pool is too thin. The difference is that a Benders optimality cut
            # never removes a feasible first-stage point (it underestimates the second
            # stage everywhere, so it can only raise `Theta`), so an empty master means the
            # `y` polytope itself is empty, at iteration 1 or iteration 100 alike.
            st.master_infeasible =
                JuMP.termination_status(m) in
                    (MOI.INFEASIBLE, MOI.INFEASIBLE_OR_UNBOUNDED, MOI.LOCALLY_INFEASIBLE)
            break
        end
        # `objective_bound`, not `objective_value`: under a non-zero MIPGap the latter is
        # an incumbent's cost, i.e. an UPPER bound on the master's own optimum, and using
        # it as the problem's lower bound would let the gap test close on nothing.
        st.lower_bound = max(st.lower_bound, JuMP.objective_bound(m))

        incumbent = extract_incumbent(build_result, mapping, m)

        t_sub = time()
        subproblem_result = solve_subproblem(build_result, mapping, m, incumbent, solver)
        st.subproblem_sec += time() - t_sub

        # Every second-stage evaluation is an exact cost for a feasible first stage, so it
        # is a genuine incumbent -- kept only when it improves, since the master's `y`
        # sequence is not monotone in cost.
        upper_bound = benders_upper_bound(subproblem_result)
        if upper_bound < st.upper_bound
            st.upper_bound = upper_bound
            st.best_incumbent = incumbent
            st.best_subproblem = subproblem_result
            st.best_iteration = iteration
        end

        if benders_converged(build_result, mapping, m, subproblem_result, solver;
                             lower_bound=st.lower_bound, upper_bound=st.upper_bound)
            st.converged = true
            st.stop_reason = "converged"
            # Reported before breaking: the converging iteration is the one a reader most
            # wants in the trace, and it never reaches the cut phase below.
            _benders_report_iteration(st, solver, iteration, incumbent, upper_bound, 0,
                                      subproblem_result)
            break
        end

        n_added = add_benders_cut!(build_result, mapping, m, subproblem_result, solver)
        st.cuts_added += n_added
        _benders_report_iteration(st, solver, iteration, incumbent, upper_bound, n_added,
                                  subproblem_result)
        if n_added == 0
            # Every cut this iteration derived was already in the master, so the next
            # iteration would re-solve an unchanged master, re-derive the same incumbent
            # and repeat forever. Dual degeneracy makes that a real outcome, not a
            # theoretical one -- see `add_benders_cut!`'s docstring.
            st.stop_reason = "cut_repeated"
            break
        end
    end
    isempty(st.stop_reason) && (st.stop_reason = "iteration_limit")

    runtime_sec = time() - st.start_time
    if solver.verbose
        # The cut trajectory in one line, which is the summary this method is judged on:
        # iterations and cuts are what the decomposition is supposed to keep small, and
        # `cuts/iter` says whether MultiCut is actually contributing one per scenario or
        # collapsing to fewer through deduplication.
        @printf("  [benders done] %s after %d iterations, %d cuts (%.1f per iteration) | LB %.6f UB %.6f gap %.3e | master %.2fs sub %.1fs of %.1fs total\n",
                st.stop_reason, st.iterations, st.cuts_added,
                st.iterations == 0 ? 0.0 : st.cuts_added / st.iterations,
                st.lower_bound, st.upper_bound, _benders_gap(st),
                st.master_sec, st.subproblem_sec, runtime_sec)
        flush(stdout)
    end
    return _benders_package_result(build_result, st, solver, m, runtime_sec)
end

"""
    _benders_package_result(build_result, st, solver, m, runtime_sec) -> OptResult

Package the run's *best incumbent* as the answer -- not the master's final state, which
is what `_package_result` (`solvers/utils/common.jl`) would read.

Two reasons it cannot use the shared packager. First, the master's objective is a **lower
bound**: reporting it as the objective would report a number no feasible solution
achieves, and on an early-stopped run it can be arbitrarily far below (this master starts
at 0, since the whole objective is second-stage). Second, `MOI.OPTIMAL` on the master
means the master solved, not that the decomposition converged -- exactly the
`CGSolver`-restricted-master trap `SolveStatus` exists for -- so status has to come from
`st.converged`, not from the model.

`solution` is the best `y` plus the per-scenario second-stage costs that priced it, as a
`NamedTuple` (`OptResult.solution` accepts one; see that type's docstring for why). It
reports *station indices*, not ids: the loop is generic over decompositions and `mapping`
is only known to be an `AbstractStationSelectionMap` here, so id lookup is the caller's.
"""
function _benders_package_result(build_result::BuildResult, st::BendersLoopState,
        solver::BendersSolver, m::JuMP.Model, runtime_sec::Float64)::OptResult
    metadata = _benders_build_metadata(build_result, st, solver, m)

    has_incumbent = !isnothing(st.best_incumbent) && isfinite(st.upper_bound)
    status = if st.converged
        SOLVE_OPTIMAL
    elseif has_incumbent
        SOLVE_FEASIBLE
    elseif st.master_infeasible
        # See the loop's own comment: a Benders cut cannot remove a feasible `y`, so an
        # infeasible master is a proof about the problem, not about a restricted relaxation.
        SOLVE_INFEASIBLE
    else
        SOLVE_NOT_SOLVED
    end

    objective_value = has_incumbent ? st.upper_bound : nothing
    solution = nothing
    if has_incumbent
        y = st.best_incumbent
        solution = (
            y = y,
            selected_station_indices = findall(v -> v > 0.5, y),
            scenario_objectives = _benders_scenario_objectives(st.best_subproblem),
        )
    end

    return OptResult(
        status,
        objective_value,
        solution,
        runtime_sec,
        m,
        build_result.mapping,
        build_result.detour_combos,
        build_result.counts,
        nothing,
        metadata,
    )
end

"""
    _benders_report_iteration(st, solver, iteration, incumbent, upper_bound, n_added)

Hand one row to `solver.iteration_callback`, if there is one.

Exists so the bound TRAJECTORY is observable, not just the final state. "Why did this
converge in 3 cuts?" is unanswerable from the aggregate metadata -- it needs the per-
iteration lower/upper bounds, and specifically whether the incumbent was found early and
the remaining iterations spent raising the lower bound to meet it (the expected shape) or
whether the bounds moved together (which would suggest the master is being steered rather
than bounded).

`incumbent_objective` is THIS iteration's second-stage cost, distinct from
`upper_bound`, which is the best seen so far -- keeping both is what makes the
found-early-then-prove pattern visible instead of hidden behind a monotone envelope.
"""
function _benders_report_iteration(st::BendersLoopState, solver::BendersSolver,
        iteration::Int, incumbent, incumbent_objective::Float64, n_added::Int,
        subproblem_result=nothing)
    if solver.verbose
        # CUTS FIRST: `+a/p` is accepted-of-proposed. One cut is proposed per scenario per
        # iteration under MultiCut, and `add_benders_cut!` drops any whose signature is
        # already present -- so `a < p` means duplicate cuts, i.e. scenarios handing back
        # the same supporting hyperplane twice and the iteration buying less than it looks
        # like it did. That distinction is invisible in the totals.
        n_proposed = isnothing(subproblem_result) ? n_added :
            length(subproblem_result.scenarios)
        @printf("  [benders it=%d] LB %.6f  UB %.6f  gap %.3e | cuts +%d/%d (total %d) | built %d | master %.2fs sub %.1fs\n",
                iteration, st.lower_bound, st.upper_bound, _benders_gap(st),
                n_added, n_proposed, st.cuts_added,
                count(v -> v > 0.5, incumbent), st.master_sec, st.subproblem_sec)
        if !isnothing(subproblem_result)
            for r in subproblem_result.scenarios
                # `nnz` is how many stations the scenario's cut actually prices. A cut with
                # few nonzeros constrains few station sets, which is what "weak cut" means
                # concretely -- and it is the number that separated a real explanation of
                # the flat cut count from two wrong ones earlier.
                nnz = count(v -> abs(v) > 1e-9, values(r.y_coefficients))
                cg = r.cg
                @printf("      s=%d Q=%.4f cut const %.4f nnz %d%s\n",
                        r.scenario, r.objective, r.cut_constant, nnz,
                        isnothing(cg) ? "" :
                        @sprintf(" | cg %d it, %d cols, %s%s", cg.cg_iterations,
                                 cg.columns_added, cg.stop_reason,
                                 cg.certifications > 0 ?
                                 " ($(cg.certifications) cert attempts)" : ""))
            end
        end
        flush(stdout)
    end
    isnothing(solver.iteration_callback) && return nothing
    solver.iteration_callback((
        iteration = iteration,
        lower_bound = st.lower_bound,
        upper_bound = st.upper_bound,
        gap = _benders_gap(st),
        incumbent_objective = incumbent_objective,
        cuts_added = n_added,
        cuts_total = st.cuts_added,
        n_stations_built = count(v -> v > 0.5, incumbent),
        master_sec = st.master_sec,
        subproblem_sec = st.subproblem_sec,
        # The whole subproblem result, so a caller can report per-scenario detail (costs,
        # and under the CG oracle the inner iteration/column counts) without the loop having
        # to know anything about which oracle produced it. Additive: existing callbacks that
        # read only the scalar fields are unaffected.
        subproblem = subproblem_result,
    ))
    return nothing
end

"""
    _benders_scenario_objectives(subproblem_result) -> Union{Nothing, Vector{Float64}}

Per-scenario second-stage cost, when the result type exposes it as a `scenarios` vector of
records carrying `objective` (which the joint routing+assignment result does). `nothing`
for any other shape, so the generic packager never assumes a layout it wasn't given.
"""
function _benders_scenario_objectives(subproblem_result)
    hasproperty(subproblem_result, :scenarios) || return nothing
    return Float64[r.objective for r in subproblem_result.scenarios]
end
