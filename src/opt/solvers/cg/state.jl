"""
The CG loop's mutable state, split by lifetime, plus the one place an iteration log row is
built.

`optimize_model` used to carry all of this as ~25 locals in one 640-line function body.
Splitting it out is what lets the loop's phases be functions at all, and the split is by
LIFETIME rather than by topic: `CGLoopState` survives the whole solve and is what the
metadata is read off at the end, `CGIterationState` is rebuilt each iteration and is what
the log row is read off. Anything that has to persist across iterations lives in the first;
anything that must NOT (a stale `certification_candidates` would be re-added to the master)
lives in the second, where a fresh construction guarantees it is cleared.
"""

"""
    CGLoopState

Everything the CG loop accumulates across iterations: the counters the metadata reports,
the pricing-phase bookkeeping, and the iteration log itself.

Constructed by `_cg_init_loop_state` (`loop.jl`), which is also where the warm-start and
certification configurations are validated -- so a state object always describes a
combination the loop can actually run.
"""
mutable struct CGLoopState
    # Wall clock. `start_time` anchors both the total budget and `warm_start_sec`.
    start_time::Float64

    iterations_run::Int
    converged::Bool
    budget_exhausted::Bool
    stop_reason::String
    cumulative_columns_added::Int

    # Two-tier ordinary-pricer ladder: how often an empty round was re-priced at
    # `certifying_pricing_time_limit_sec`.
    certifying_rounds::Int

    # Warm-start phasing. `active_pricing_mode` is the pricer in force right now; the loop
    # runs phase 1 in `pricing.warm_start_mode` until that universe is exhausted, then hands
    # off to `final_pricing_mode`, which is the only phase that can certify. Both phases
    # share one master and one column pool -- the handoff changes what pricing searches,
    # nothing else, so every column phase 1 harvested stays.
    final_pricing_mode::Union{Nothing, Symbol}
    warm_start_mode::Union{Nothing, Symbol}
    active_pricing_mode::Union{Nothing, Symbol}
    warm_start_active::Bool
    warm_start_iterations::Int
    # Wall seconds spent in phase 1, i.e. how long the restricted universe took to exhaust.
    # Recorded here rather than summed from `cg_iteration_log` afterwards because the log
    # captures only master/pricing/add-columns time, so a sum of it silently omits
    # everything else in the iteration and understates the phase.
    warm_start_sec::Float64

    # Relaxed-cluster certification. The column_found/inconclusive split is the
    # only way to read a run that never certifies, and the two are NOT two failures.
    # "column_found" means the attempt exact-priced a support and got a real
    # improving column: the mode prices first and certifies second, so this is the ordinary
    # outcome and the source of the run's columns -- a high count means CG is making
    # progress, not that anything is wrong. "inconclusive" means the search ran out of its
    # time (or cut-round) budget without settling either way, which IS a knob on this solver
    # and the only outcome an escalation can rescue. The bare attempt count cannot tell the
    # two apart, and they point at different places.
    certification_rounds::Int
    certification_column_found_rounds::Int
    certification_inconclusive_rounds::Int
    certification_sec::Float64
    # Columns recovered from certification attempts that did not certify -- overwhelmingly
    # the `:column_found` ones, where the attempt was the pricing round.
    # Reported so the feature's cost can be read net of what it gave back.
    certification_harvested_columns::Int
    certified_by_relaxation::Bool

    # One row per CG iteration, exposed as metadata["cg_iteration_log"].
    iteration_log::Vector{NamedTuple}
    # A callback that throws must NOT take the solve with it: progress logging is
    # observability, and losing a two-hour certification run to a transient filesystem error
    # would be a strictly worse outcome than losing the log line. The first failure is
    # warned about once and the rest are silent -- for both callbacks, which is why the flag
    # is shared.
    callback_failed::Bool
end

CGLoopState(start_time::Float64, final_pricing_mode, warm_start_mode) = CGLoopState(
    start_time, 0, false, false, "max_iterations", 0, 0,
    final_pricing_mode, warm_start_mode,
    isnothing(warm_start_mode) ? final_pricing_mode : warm_start_mode,
    !isnothing(warm_start_mode), 0, 0.0,
    0, 0, 0, 0.0, 0, false,
    NamedTuple[], false,
)

"""
    CGIterationState

One iteration's scratch: the master solve's outcome, and whatever the certification phase
learned before pricing runs.

Rebuilt every iteration on purpose -- see the module docstring. The defaults are also the
values every log row falls back to, so an iteration that never reaches a phase logs that
phase as not having happened rather than as having happened with stale numbers.
"""
mutable struct CGIterationState
    iteration::Int
    master_sec::Float64
    master_status::String
    # `missing` until the master comes back OPTIMAL, which is exactly what the log wants.
    master_objective::Union{Missing, Float64}

    certification_sec::Float64
    certification_candidates::Vector{Any}
    certified::Bool
    certification_outcome::String
    escalated_certification::Bool
    # The scenarios that came back `:inconclusive`, and whether any scenario priced a
    # column.
    # Escalation is decided PER SCENARIO off these two, not off whether the round as a whole
    # produced columns -- see `_cg_escalate_inconclusive_scenarios!`.
    inconclusive_scenarios::Vector{Int}
    round_found_column::Bool
    # NaN = "no valid lower bound this iteration", which covers both the pricers that never
    # attempt one and an attempt that came back inconclusive. See
    # `RelaxedClusterCertificationResult.relaxed_rc_bound`.
    rc_bound::Float64
    # True when the active pricer IS the certification loop, in which case there is no
    # separate pricing round to run.
    relaxed_cluster_pricing::Bool

    pricing_limit::Float64
    certifying::Bool
end

CGIterationState(iteration::Int) = CGIterationState(
    iteration, 0.0, "", missing,
    0.0, Any[], false, "none", false, Int[], false, NaN, false,
    0.0, false,
)

"""
    _cg_log_row(st, it; pricing_sec, add_columns_sec, columns_added, columns_accepted,
                certification_certified) -> NamedTuple

Build one `cg_iteration_log` row.

The row has sixteen fields and is emitted from seven places in the loop. Written out at each
of them -- which is how it was -- the call sites were fourteen lines each and ~110 lines of
the loop were this literal, which made the genuinely differing fields (how much time went
where, and how many columns moved) invisible among the thirteen that never differ. Every
field defaults to what `st`/`it` already say, so a call site names only what is specific to
it, and adding a field to the log is one edit rather than seven.

`certifying_pricing` and `pricing_limit_sec` deliberately read from `it` rather than being
keywords: they are set by the pricing phase itself, so any row emitted after it should
report them and any row emitted before it should report the zero/false they start at.
"""
_cg_log_row(
    st::CGLoopState, it::CGIterationState;
    pricing_sec::Float64=0.0, add_columns_sec::Float64=0.0,
    columns_added::Int=0, columns_accepted::Int=0,
    certification_certified::Bool=false,
) = (
    iteration=it.iteration,
    master_sec=it.master_sec,
    pricing_sec=pricing_sec,
    add_columns_sec=add_columns_sec,
    columns_added=columns_added,
    columns_accepted=columns_accepted,
    cumulative_columns_added=st.cumulative_columns_added,
    master_objective=it.master_objective,
    master_status=it.master_status,
    pricing_limit_sec=it.pricing_limit,
    certifying_pricing=it.certifying,
    pricing_mode=string(something(st.active_pricing_mode, :default)),
    certification_sec=it.certification_sec,
    certification_certified=certification_certified,
    certification_outcome=it.certification_outcome,
    relaxed_rc_bound=it.rc_bound,
)

"""
    _cg_log_iteration!(st, solver, row)

Record a row and hand it to `solver.iteration_callback`.

Every row goes through here so the callback sees each one AS IT HAPPENS rather than only in
the returned metadata. Without that a run that is killed -- preempted, or over its Slurm
wall -- emits nothing at all, and hours of a long solve become unobservable and
unrecoverable.
"""
function _cg_log_iteration!(st::CGLoopState, solver::CGSolver, row::NamedTuple)
    push!(st.iteration_log, row)
    isnothing(solver.iteration_callback) && return nothing
    _cg_guard_callback!(st, "iteration_callback") do
        solver.iteration_callback(row)
    end
    return nothing
end

"""
    _cg_guard_callback!(f, st, name)

Run a user callback, absorbing anything it throws.

Both callbacks are observability, and neither may take the solve with it. The first failure
is warned about and the rest are silent, so a callback that fails every iteration cannot
bury the log it was supposed to be writing.
"""
function _cg_guard_callback!(f::Function, st::CGLoopState, name::AbstractString)
    try
        f()
    catch err
        if !st.callback_failed
            st.callback_failed = true
            @warn "CGSolver $name failed; continuing without it" err
        end
    end
    return nothing
end
