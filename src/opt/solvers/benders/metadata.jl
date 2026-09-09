"""
`BendersSolver`'s result report -- what a finished (or stopped) run tells its caller about
what it actually established. Mirrors `opt/solvers/cg/metadata.jl`.
"""

"""
    _benders_build_metadata(build_result, st, solver, m) -> Dict{String, Any}

Both bounds, the gap, where the run stopped, and where the time went.

`benders_second_stage_relaxed` is the one key a reader must not skip: the subproblem is
solved as an LP (it has to be -- the cut comes from its dual), so a converged
`BendersSolver` run is optimal for the **mixed** model, `y` binary with `theta`/`x_walk`
continuous. That is a different number from `DirectMIPSolver`'s all-binary optimum over
the same column pool, by this formulation's LP-IP gap. Together with
`benders_optimality_scope` (forwarded from the build, and reporting any `max_stops`
narrowing the enumeration oracle applied), these two keys are what keep a `BendersSolver`
objective from being pooled with numbers that mean something else -- the same job
`cg_optimality_scope` does for `CGSolver`.
"""
function _benders_build_metadata(build_result::BuildResult, st::BendersLoopState,
        solver::BendersSolver, m::JuMP.Model)::Dict{String, Any}
    metadata = Dict{String, Any}(
        "benders_iterations" => st.iterations,
        "benders_cuts_added" => st.cuts_added,
        "benders_converged" => st.converged,
        "benders_stop_reason" => st.stop_reason,
        "benders_lower_bound" => st.lower_bound,
        "benders_upper_bound" => st.upper_bound,
        "benders_gap" => _benders_gap(st),
        "benders_relative_gap" => _benders_relative_gap(st),
        "benders_best_iteration" => st.best_iteration,
        "benders_master_sec" => st.master_sec,
        "benders_subproblem_sec" => st.subproblem_sec,
        "benders_second_stage_relaxed" => true,
        "benders_subproblem_oracle" => solver.subproblem.oracle,
        "moi_termination_status" => string(JuMP.termination_status(m)),
    )
    # Build-time facts the loop cannot know: pool size, and any max_stops narrowing the
    # oracle applied (with the scope label that narrowing implies).
    for (key, value) in build_result.metadata
        metadata[key] = value
    end
    return metadata
end
