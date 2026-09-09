"""
`BendersLoopState` -- the mutable bound/incumbent bookkeeping `optimize_model` carries
across iterations.

Owned by the loop, deliberately NOT stashed on the master model. Build products (the
enumerated pool, the per-scenario subproblem models) do live on `m`, because the hooks
only ever receive `build_result`/`mapping`/`m` and have to find them; loop state is
different -- nothing outside the loop reads it, and putting it on `m` would make a second
`optimize_model` call on the same `BuildResult` silently resume half-finished bounds.
Mirrors `CGLoopState` (`opt/solvers/cg/state.jl`).
"""

"""
    BendersLoopState(start_time)

`lower_bound` starts at `-Inf` and `upper_bound` at `+Inf`, so an aborted iteration 1
reports honestly rather than claiming a bound it never established.

`best_incumbent`/`best_subproblem` track the *best* iteration, not the last: every
iteration's second-stage evaluation is a genuine feasible solution, and the master's
`y` sequence is not monotone in cost, so the final iteration is frequently worse than one
seen earlier.
"""
mutable struct BendersLoopState
    start_time::Float64
    iterations::Int
    lower_bound::Float64
    upper_bound::Float64
    best_incumbent::Any
    best_subproblem::Any
    best_iteration::Int
    cuts_added::Int
    master_sec::Float64
    subproblem_sec::Float64
    converged::Bool
    stop_reason::String

    BendersLoopState(start_time::Float64) =
        new(start_time, 0, -Inf, Inf, nothing, nothing, 0, 0, 0.0, 0.0, false, "")
end

"""
    _benders_gap(st) -> Float64
    _benders_relative_gap(st) -> Float64

Absolute and relative optimality gap. `NaN` while either bound is still infinite -- a gap
against an unestablished bound is not a number, and reporting `Inf` would read as a
measured-but-huge gap.
"""
function _benders_gap(st::BendersLoopState)::Float64
    (isfinite(st.lower_bound) && isfinite(st.upper_bound)) || return NaN
    return st.upper_bound - st.lower_bound
end

function _benders_relative_gap(st::BendersLoopState)::Float64
    gap = _benders_gap(st)
    isnan(gap) && return NaN
    return gap / max(1.0, abs(st.upper_bound))
end
