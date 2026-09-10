"""
`optimize_model(::BuildResult, ::CGSolver)` -- the column-generation outer loop, and the
phases it sequences.

The loop was one 640-line function. It is broken here along the seams an iteration already
has: solve the master, certify, harvest, price. Each phase returns a **control signal**
rather than acting on the loop directly --

    :proceed   carry on to the next phase of this iteration
    :continue  this iteration is done; start the next one
    :break     the solve is over; `st.stop_reason` says why

-- so which of the eight exits fired, and in what order they are tested, stays readable in
`_cg_run_iteration!` instead of being scattered across the phases. Everything an exit needs
to report lives on `CGLoopState`/`CGIterationState` (`state.jl`).

What each phase is FOR, and why it is a phase, is documented on the `CGSolver` struct in
`solver.jl` -- that docstring is the user-facing account of the algorithm and this file is
the mechanism.
"""

"""
    _cg_remaining_budget(st, solver) -> Float64

Wall seconds left in `total_time_limit_sec`. Checked before every iteration and used to
CLAMP every pricing budget, which is what makes the cap exact: no single label search can
be granted time the loop does not have.
"""
_cg_remaining_budget(st::CGLoopState, solver::CGSolver) =
    solver.total_time_limit_sec - (time() - st.start_time)

"""
    _cg_init_loop_state!(build_result, mapping, m, solver, start_time) -> CGLoopState

Resolve the pricing phases, validate that the model can actually run them, and put phase 1
in force.

Both validations refuse rather than degrade, because both failure modes are silent: a
`warm_start_mode` on a formulation with no selectable pricer would simply never phase, and a
`:relaxed_cluster` mode on a model with no relaxed-cluster pricer would run an ordinary
solve that reports certification metadata it never earned.
"""
function _cg_init_loop_state!(
    build_result::BuildResult, mapping, m::JuMP.Model, solver::CGSolver,
    start_time::Float64,
)
    final_pricing_mode = cg_pricing_mode(build_result, mapping, m)
    warm_start_mode = solver.pricing.warm_start_mode

    if !isnothing(warm_start_mode)
        isnothing(final_pricing_mode) && throw(ArgumentError(
            "pricing.warm_start_mode=$(repr(warm_start_mode)) was requested, but this " *
            "formulation has no selectable pricer (no `cg_pricing_mode` method), so there " *
            "is nothing to warm-start from and nothing to hand off to",
        ))
        warm_start_mode === final_pricing_mode && throw(ArgumentError(
            "pricing.warm_start_mode=$(repr(warm_start_mode)) equals the pricer it would " *
            "hand off to: a warm-start phase that hands off to itself is a no-op, and " *
            "would silently halve max_iterations",
        ))
        set_cg_pricing_mode!(build_result, mapping, m, warm_start_mode)
    end

    if final_pricing_mode in (:relaxed_cluster, :relaxed_cluster_two_tier)
        cg_certification_supported(build_result, mapping, m) || throw(ArgumentError(
            "pricing.mode=$(repr(final_pricing_mode)) was requested, but this model has " *
            "no matching relaxed-cluster pricer available -- it needs a formulation that " *
            "implements one, built from a CGPricingConfig carrying " *
            "relaxed_cluster_count (and relaxed_cluster_macro_count for the two-tier mode)",
        ))
    end

    return CGLoopState(start_time, final_pricing_mode, warm_start_mode)
end

function optimize_model(build_result::BuildResult, solver::CGSolver)::OptResult
    m = build_result.model
    _apply_solver_config!(m, solver.config)
    mapping = build_result.mapping

    start_time = time()
    st = _cg_init_loop_state!(build_result, mapping, m, solver, start_time)

    for iteration in 1:solver.max_iterations
        _cg_run_iteration!(st, iteration, build_result, mapping, m, solver) === :break &&
            break
    end

    # The loop can exit having just added columns but not yet re-solved the master --
    # budget expiry, the iteration cap, or a de-dup stall all do this. Adding variables
    # invalidates JuMP's solution, so the model reports OPTIMIZE_NOT_CALLED and the run
    # would return no incumbent and no objective at all: a censored job would write an
    # empty row, which is exactly what the total budget exists to avoid. Re-solve the
    # master once (bounded by config.time_limit_sec) so a stopped-early run still yields
    # a usable LP solution and a pool for integer recovery. Only OPTIMIZE_NOT_CALLED is
    # retried -- a genuine INFEASIBLE/UNBOUNDED status is a real answer, not staleness.
    final_master_resolved = false
    if JuMP.termination_status(m) == MOI.OPTIMIZE_NOT_CALLED
        final_master_resolved = true
        optimize!(m)
    end
    lp_loop_sec = time() - start_time

    metadata = _cg_build_metadata(st, solver, m, lp_loop_sec, final_master_resolved)

    if solver.recover_integer_solution && JuMP.termination_status(m) == MOI.OPTIMAL
        metadata["cg_lp_objective_value"] = JuMP.objective_value(m)
        t0 = time()
        build_result = integer_recovery_build(build_result, mapping, m)
        m = build_result.model
        _apply_solver_config!(m, solver.config)
        optimize!(m)
        metadata["cg_integer_recovery_sec"] = time() - t0
    end
    runtime_sec = time() - start_time

    # `converged` is the ONLY thing that makes this master's optimum the problem's
    # optimum: it means pricing exhausted, i.e. no negative-reduced-cost column remains
    # outside the pool. Every other exit (total budget, iteration cap, de-dup stall,
    # inconclusive pricing) leaves the pool possibly incomplete, so the incumbent is a
    # valid upper bound and nothing more -- `_solve_status` reports SOLVE_FEASIBLE there
    # instead of the SOLVE_OPTIMAL the restricted master would otherwise claim.
    # `converged` means pricing exhausted the universe it was searching, and that is what
    # `certified` reports -- including for a restricted pricer such as `:station_simple`,
    # whose OPTIMAL is an optimum over elementary routes only. The scope of the claim is
    # carried alongside it in `cg_optimality_scope`/`cg_pricing_universe_restricted`
    # rather than folded into the status, so a caller reading the status gets the same
    # meaning it always had ("pricing exhausted") and a caller who needs to know which
    # universe was exhausted can ask.
    return _package_result(build_result, m, runtime_sec;
                           metadata=metadata, certified=st.converged)
end

"""
    _cg_run_iteration!(st, iteration, build_result, mapping, m, solver) -> Symbol

One CG iteration, as the four phases in the order they must run.

The order is load-bearing, and the comments on each phase say why. In particular
certification runs BEFORE pricing (under `:relaxed_cluster` it *is* the pricing round), and
the harvest branch sits between them because a `:negative_rc_column_found` attempt's
columns make the ordinary pricing round unnecessary for that iteration.
"""
function _cg_run_iteration!(
    st::CGLoopState, iteration::Int, build_result::BuildResult, mapping,
    m::JuMP.Model, solver::CGSolver,
)
    if _cg_remaining_budget(st, solver) <= 0
        st.budget_exhausted = true
        st.stop_reason = "total_budget"
        return :break
    end
    st.iterations_run = iteration
    it = CGIterationState(iteration)

    _cg_solve_master!(st, it, m, solver) === :break && return :break

    duals = extract_duals(build_result, mapping, m)
    # LATE-STAGE duals are the ones that matter and the ones nothing could reach: a
    # parameter diagnostic that re-runs CG only ever sees early iterations, where every
    # search exhausts in milliseconds. The regime that decides certification is a
    # near-converged master fighting a ~0 margin, and the only way to study it offline
    # is to capture the dual vectors from a real long run and replay them.
    if !isnothing(solver.dual_callback)
        _cg_guard_callback!(st, "dual_callback") do
            solver.dual_callback(iteration, duals)
        end
    end

    _cg_certification_phase!(st, it, build_result, mapping, m, duals, solver) === :break &&
        return :break

    signal = _cg_harvest_phase!(st, it, build_result, mapping, m, duals, solver)
    signal === :proceed || return signal

    return _cg_pricing_phase!(st, it, build_result, mapping, m, duals, solver)
end

"""
    _cg_solve_master!(st, it, m, solver) -> Symbol

Solve the restricted master and record its outcome on `it`.

A non-`OPTIMAL` master ends the solve: there are no duals to price against. The row is
still logged (with `master_objective` left `missing`) because the log is contracted to have
one row per iteration, including the one that breaks the loop.
"""
function _cg_solve_master!(
    st::CGLoopState, it::CGIterationState, m::JuMP.Model, solver::CGSolver,
)
    t0 = time()
    optimize!(m)
    it.master_sec = time() - t0
    status = JuMP.termination_status(m)
    it.master_status = string(status)

    if status != MOI.OPTIMAL
        _cg_log_iteration!(st, solver, _cg_log_row(st, it))
        st.stop_reason = "master_not_optimal"
        return :break
    end
    it.master_objective = JuMP.objective_value(m)
    return :proceed
end

"""
    _cg_certification_phase!(st, it, build_result, mapping, m, duals, solver) -> Symbol

The relaxed-cluster round, and the per-scenario escalation behind it.

Inert unless the active pricer is `:relaxed_cluster`/`:relaxed_cluster_two_tier`, in which
case this IS the iteration's pricing round: it certifies (and the solve ends), or it
harvests real columns for `_cg_harvest_phase!`, or it comes back inconclusive. It runs under
the ordinary `pricing_time_limit_sec` for exactly that reason -- it is doing the pricing,
not sitting in front of it.
"""
function _cg_certification_phase!(
    st::CGLoopState, it::CGIterationState, build_result::BuildResult, mapping,
    m::JuMP.Model, duals, solver::CGSolver,
)
    it.relaxed_cluster_pricing =
        st.active_pricing_mode in (:relaxed_cluster, :relaxed_cluster_two_tier)
    it.relaxed_cluster_pricing || return :proceed

    limit = min(solver.pricing_time_limit_sec, _cg_remaining_budget(st, solver))
    if limit > 0
        t0 = time()
        st.certification_rounds += 1
        certification = cg_certification_round(
            build_result, mapping, m, duals, solver;
            time_limit_sec=limit, iteration=it.iteration,
        )
        it.certification_sec = time() - t0
        st.certification_sec += it.certification_sec
        it.certified = certification.certified
        it.certification_candidates = certification.candidates
        it.rc_bound = certification.relaxed_rc_bound
        it.inconclusive_scenarios = copy(certification.inconclusive_scenarios)
        it.round_negative_rc_column = certification.improving_found
        it.certification_outcome = if certification.certified
            "certified"
        elseif certification.improving_found
            st.certification_negative_rc_column_rounds += 1
            "negative_rc_column_found"
        else
            st.certification_inconclusive_rounds += 1
            "inconclusive"
        end
    end
    # A relaxation certificate covers the FULL route universe (it bounds every real route,
    # not just the ones the active pricer would search), so unlike the warm-start phase
    # boundary there is nothing left to hand off to: the solve is genuinely done, whichever
    # pricer was in force.
    it.certified && return _cg_finish_certified!(st, it, solver)

    _cg_escalate_inconclusive_scenarios!(st, it, build_result, mapping, m, duals, solver)
    it.certified && return _cg_finish_certified!(st, it, solver)
    return :proceed
end

"""
    _cg_escalate_inconclusive_scenarios!(st, it, build_result, mapping, m, duals, solver)

Re-run the scenarios that came back `:inconclusive`, and only those, at
`certifying_pricing_time_limit_sec`.

An inconclusive scenario is one whose searches ran out of budget without reaching a verdict,
and the ONLY thing that can rescue it is a longer budget. Escalation used to be decided
round-wide, and only when the round produced no columns at all -- but the harvest branch
`continue`s straight past the escalation point, which makes escalation unreachable for any
round where some OTHER scenario was productive.

MEASURED, n=40 seed 47 (`notes/2026-09-09_n40_certification_frontier_5_of_10.md` and the
`rounds/` dumps beside it): scenario 2 priced and harvested columns in all 38 iterations,
so the round always had something to show; scenario 1 was inconclusive from iteration 13
onward and replayed a bit-identical 262 s search 24 consecutive times -- same relaxed_rc at
every tier (-2000.6/-1059.4/-1042.1/-785.7), same subset sizes, same runtimes -- while the
master objective sat frozen at 32043.0090 from iteration 19 to 38. 0 escalated attempts in
the whole run, 73% of the wall spent re-running a search that had already failed. One
productive scenario masked a permanently stuck one for the entire budget.

So escalate the inconclusive scenarios THEMSELVES: re-running a scenario that already
priced a column would re-find columns already in the pool, and re-running a certified one
would re-prove what is proved. `only_scenarios` is what makes the round restrictable;
`escalated.certified` then means "every scenario I ran certified", so the round as a whole
certifies exactly when nothing outside the escalated subset priced a column either.

This subsumes the old round-wide relaxed-cluster escalation, which is why
`_cg_pricing_phase!`'s ladder now handles only the ordinary pricer.

NOTE the bound is deliberately NOT reconstructed here. The ordinary round already reported
`NaN` (any inconclusive scenario poisons a round's bound), and this partial round bounds
only the scenarios it re-ran, so there is no honest way to combine them without the
per-scenario bounds the ordinary round did not keep. A round that ends up certified stops
the solve and needs no bound; one that does not keeps `NaN`, which is the truthful "no bound
established this iteration".
"""
function _cg_escalate_inconclusive_scenarios!(
    st::CGLoopState, it::CGIterationState, build_result::BuildResult, mapping,
    m::JuMP.Model, duals, solver::CGSolver,
)
    isempty(it.inconclusive_scenarios) && return nothing
    escalated_limit =
        min(solver.certifying_pricing_time_limit_sec, _cg_remaining_budget(st, solver))
    (escalated_limit > it.certification_sec && escalated_limit > 0) || return nothing

    t0 = time()
    st.certification_rounds += 1
    escalated = cg_certification_round(
        build_result, mapping, m, duals, solver;
        time_limit_sec=escalated_limit, iteration=it.iteration,
        only_scenarios=it.inconclusive_scenarios,
    )
    elapsed = time() - t0
    it.certification_sec += elapsed
    st.certification_sec += elapsed
    it.escalated_certification = true

    # The escalated scenarios' harvest ADDS to what the ordinary round found in the others.
    # Duplicates are harmless: `add_columns!` is signature-checked, so a candidate already
    # in the pool is counted as not accepted.
    append!(it.certification_candidates, escalated.candidates)
    it.inconclusive_scenarios = copy(escalated.inconclusive_scenarios)
    it.round_negative_rc_column = it.round_negative_rc_column || escalated.improving_found
    it.certified = !it.round_negative_rc_column && escalated.certified
    it.certification_outcome = if it.certified
        "certified_escalated"
    elseif escalated.improving_found
        st.certification_negative_rc_column_rounds += 1
        "negative_rc_column_found_escalated"
    else
        st.certification_inconclusive_rounds += 1
        "inconclusive_escalated"
    end
    # A certified round drops its harvest for the same reason the ordinary one does: CG is
    # about to stop, and churning a master just proved optimal buys nothing.
    it.certified && (it.certification_candidates = Any[])
    return nothing
end

"""
    _cg_finish_certified!(st, it, solver) -> :break

End the solve on a relaxation certificate. Both certification points -- the ordinary round
and the escalated one -- exit through here, so they cannot log different rows for the same
event.
"""
function _cg_finish_certified!(st::CGLoopState, it::CGIterationState, solver::CGSolver)
    st.converged = true
    st.certified_by_relaxation = true
    st.stop_reason = "converged_by_certification"
    _cg_log_iteration!(st, solver, _cg_log_row(st, it; certification_certified=true))
    return :break
end

"""
    _cg_harvest_phase!(st, it, build_result, mapping, m, duals, solver) -> Symbol

Take a non-certifying certification attempt's columns as this iteration's pricing result.

An attempt that did not certify is not a failed attempt. Under `:relaxed_cluster` the mode
PRICES FIRST and certifies second: it runs the real exact pricer over a station subset and
hands back the improving columns that search found, which is exactly what a pricing round
produces. Taking them here skips the regular round entirely -- the expensive full-station
search -- for the price of an attempt that had to run anyway. The `:negative_rc_column_found`
outcome this branch serves is therefore the mode's ordinary, productive path (96% of
attempts), not an error case.

Soundness: these are ordinary priced columns (same materialization, same
`_pricing_verify_column` cross-check), but the subset they came from is a RESTRICTED route
universe, so their ABSENCE would prove nothing. That is why this branch only ever *skips* a
pricing round when it has columns to show for it -- convergence is still declared only by a
full-universe certificate above, or by a full-universe pricing round exhausting below.
`cg_pricing_exhausted` is left untouched for the same reason: this round proved no
exhaustion of anything.
"""
function _cg_harvest_phase!(
    st::CGLoopState, it::CGIterationState, build_result::BuildResult, mapping,
    m::JuMP.Model, duals, solver::CGSolver,
)
    harvested = _cg_materialize_certification_columns(
        build_result, mapping, m, duals, it.certification_candidates,
    )
    isempty(harvested) && return :proceed

    t0 = time()
    accepted = add_columns!(build_result, mapping, m, harvested)
    add_sec = time() - t0
    st.cumulative_columns_added += accepted
    st.certification_harvested_columns += accepted
    _cg_log_iteration!(st, solver, _cg_log_row(
        st, it; add_columns_sec=add_sec,
        columns_added=length(harvested), columns_accepted=accepted,
    ))
    accepted == 0 && return :break   # nothing entered the master: no progress is possible
    return :continue
end

"""
    _cg_pricing_phase!(st, it, build_result, mapping, m, duals, solver) -> Symbol

The ordinary pricing round, its two-tier escalation, and the insertion of whatever it found.

Under `:relaxed_cluster` the certification phase WAS the pricing round -- it certified, or
it harvested (and the iteration ended), or it came back inconclusive, which is the one case
that reaches here. There is no second pricer to call, and the two-tier ladder below is
therefore unreachable in that mode: the escalation that mode needs is per scenario and
already ran.

The ladder itself: a round that comes back empty is only conclusive if its label searches
actually exhausted. If it merely ran out of its (short) budget, the empty result is
ambiguous, and only a re-price of the same duals at `certifying_pricing_time_limit_sec` can
turn it into `cg_pricing_exhausted`.
"""
function _cg_pricing_phase!(
    st::CGLoopState, it::CGIterationState, build_result::BuildResult, mapping,
    m::JuMP.Model, duals, solver::CGSolver,
)
    t0 = time()
    new_columns = nothing
    if !it.relaxed_cluster_pricing
        it.pricing_limit = min(solver.pricing_time_limit_sec, _cg_remaining_budget(st, solver))
        new_columns = price_columns(build_result, mapping, m, duals, solver;
                                    time_limit_sec=it.pricing_limit)
    end

    if (isnothing(new_columns) || isempty(new_columns)) &&
            !it.relaxed_cluster_pricing && !_cg_pricing_exhausted(m)
        escalated_limit =
            min(solver.certifying_pricing_time_limit_sec, _cg_remaining_budget(st, solver))
        if escalated_limit > it.pricing_limit
            it.certifying = true
            st.certifying_rounds += 1
            it.pricing_limit = escalated_limit
            new_columns = price_columns(build_result, mapping, m, duals, solver;
                                        time_limit_sec=escalated_limit)
        end
    end
    pricing_sec = time() - t0

    # An escalated certification that priced instead of certifying still hands back real
    # columns, which is progress the pricing round did not find. (A successful one cannot reach here: both
    # certification points break out of the loop.)
    if it.escalated_certification && !isempty(it.certification_candidates)
        escalated_columns = _cg_materialize_certification_columns(
            build_result, mapping, m, duals, it.certification_candidates,
        )
        isempty(escalated_columns) || (new_columns = escalated_columns)
    end

    if isnothing(new_columns) || isempty(new_columns)
        return _cg_finish_empty_pricing!(
            st, it, build_result, mapping, m, solver, pricing_sec,
        )
    end

    t0 = time()
    columns_accepted = add_columns!(build_result, mapping, m, new_columns)
    add_columns_sec = time() - t0
    st.cumulative_columns_added += columns_accepted
    _cg_log_iteration!(st, solver, _cg_log_row(
        st, it; pricing_sec=pricing_sec, add_columns_sec=add_columns_sec,
        columns_added=length(new_columns), columns_accepted=columns_accepted,
    ))

    # Pricing returned improving columns but every one was de-duplicated away, so the
    # master is unchanged and the next iteration would extract identical duals and
    # re-find the identical columns -- a livelock that otherwise burns every remaining
    # iteration (see notes/2026-08-25_study6_cg_livelock_stale_tau_columns.md).
    # `converged` deliberately stays false: pricing DID find a negative-reduced-cost
    # column, so this is a failure to make progress, never an optimality certificate.
    if columns_accepted == 0
        st.stop_reason = "no_columns_accepted"
        return :break
    end
    return :proceed
end

"""
    _cg_finish_empty_pricing!(st, it, build_result, mapping, m, solver, pricing_sec)
        -> Symbol

Decide what an empty pricing result means, which is the loop's single most consequential
branch.

An empty result certifies convergence ONLY when every underlying label search exhausted its
frontier. A time-limited pass can also return no columns; treating that as exhaustion
silently turns a pricing timeout into a false optimality certificate.

`:relaxed_cluster` never reaches here having exhausted anything -- its only certificate is
the relaxation's -- and it did not call `price_columns` at all, so the model's exhaustion
flag is stale (from a warm-start phase, or absent, in which case `_cg_pricing_exhausted`
defaults to `true`). Reading it would manufacture a certificate out of a timed-out
relaxation, hence the explicit `!relaxed_cluster_pricing`.

Phase 1 exhausting its own universe is a PHASE BOUNDARY, not the end of the solve. Hand off
to the formulation's real pricer and keep going against the same pool: `converged` is
deliberately reset to false, because exhausting the warm-start universe proves nothing about
the full one, and leaving it true here is exactly how a restricted search would masquerade
as a certificate.
"""
function _cg_finish_empty_pricing!(
    st::CGLoopState, it::CGIterationState, build_result::BuildResult, mapping,
    m::JuMP.Model, solver::CGSolver, pricing_sec::Float64,
)
    st.converged = !it.relaxed_cluster_pricing && _cg_pricing_exhausted(m)

    if st.converged && st.warm_start_active
        # Logged BEFORE the handoff, so the row names the pricer that actually ran.
        _cg_log_iteration!(st, solver, _cg_log_row(st, it; pricing_sec=pricing_sec))
        set_cg_pricing_mode!(build_result, mapping, m, st.final_pricing_mode)
        st.warm_start_active = false
        st.warm_start_iterations = it.iteration
        st.warm_start_sec = time() - st.start_time
        st.active_pricing_mode = st.final_pricing_mode
        st.converged = false
        st.stop_reason = "max_iterations"
        return :continue
    end

    if st.converged
        st.stop_reason = "converged"
    elseif _cg_remaining_budget(st, solver) <= 0
        st.budget_exhausted = true
        st.stop_reason = "total_budget"
    else
        st.stop_reason = "pricing_inconclusive"
    end
    _cg_log_iteration!(st, solver, _cg_log_row(st, it; pricing_sec=pricing_sec))
    return :break
end
