"""
`metadata` assembly for a `CGSolver` result -- the ~40 keys the loop's state becomes.

Its own file because it is a *report*, not part of the algorithm: nothing here decides
anything, and reading the loop should not mean scrolling past eighty lines of key/value
pairs to reach the return. Keeping the two apart also makes the dependency one-way -- the
loop writes `CGLoopState`, this reads it -- so a new counter is added in `state.jl`, set in
`loop.jl` and reported here, with no other file involved.

Several keys carry the reasoning for their own existence rather than only their value; that
is deliberate, because most of them exist to make a *failed* run readable and their meaning
is not recoverable from the name.
"""

"""
    _cg_build_metadata(st, solver, m, lp_loop_sec, final_master_resolved) -> Dict{String,Any}

Read the finished loop's state into `OptResult.metadata`.

Called BEFORE integer recovery, which is load-bearing for the model-sourced keys: recovery
replaces `m` with a freshly built model whose stats vectors are empty, so anything read off
the model afterwards would be invisible on exactly the runs that use recovery.
`cg_integer_recovery_sec` starts at `0.0` here and is overwritten by the caller if recovery
runs.
"""
function _cg_build_metadata(
    st::CGLoopState, solver::CGSolver, m::JuMP.Model,
    lp_loop_sec::Float64, final_master_resolved::Bool,
)
    return Dict{String, Any}(
        "cg_iterations" => st.iterations_run,
        "cg_converged" => st.converged,
        "cg_pricing_exhausted" => st.converged,
        # Why the loop stopped: "converged", "converged_by_certification", "total_budget",
        # "max_iterations", "master_not_optimal", "no_columns_accepted", or
        # "pricing_inconclusive". Only the two "converged*" reasons make
        # cg_lp_objective_value a valid bound on the true optimum.
        "cg_stop_reason" => st.stop_reason,
        "cg_total_budget_exhausted" => st.budget_exhausted,
        "cg_total_time_limit_sec" => solver.total_time_limit_sec,
        "cg_certifying_rounds" => st.certifying_rounds,
        # Which pricer each phase used, and how much of the run was the warm-start phase.
        # `cg_pricing_universe_restricted` is what makes a run's OPTIMAL claim trustworthy:
        # true means pricing finished in a restricted universe, so no certificate is possible.
        "cg_warm_start_pricing_mode" => st.warm_start_mode,
        "cg_warm_start_iterations" => st.warm_start_iterations,
        # Elapsed wall at the phase-1 -> phase-2 handoff. Zero when no warm start ran, and
        # ALSO zero when a warm start was requested but its phase never exhausted (the run
        # stopped inside phase 1) -- `cg_warm_start_iterations` distinguishes those two.
        "cg_warm_start_sec" => st.warm_start_sec,
        "cg_final_pricing_mode" => st.active_pricing_mode,
        # Whether the relaxed-cluster round was configured at all, how many rounds it cost,
        # and whether it is what ended the solve. A relaxation certificate bounds EVERY
        # real route, so it is a full-route-universe certificate regardless of which pricer
        # found the columns -- which is why it overrides the two scope keys below.
        # Kept as its own key (rather than left to `cg_final_pricing_mode`) so a run that
        # stopped inside a warm-start phase still records that it was going to certify.
        "cg_certification_pricing_mode" =>
            (st.final_pricing_mode in (:relaxed_cluster, :relaxed_cluster_two_tier) ?
                st.final_pricing_mode : nothing),
        "cg_certification_rounds" => st.certification_rounds,
        "cg_certification_column_found_rounds" => st.certification_column_found_rounds,
        "cg_certification_inconclusive_rounds" => st.certification_inconclusive_rounds,
        # Columns recovered from failed certification attempts. Read against
        # `cg_certification_sec` to judge the feature's NET cost: a failed attempt that
        # hands back columns replaced a pricing round rather than adding to one.
        "cg_certification_harvested_columns" => st.certification_harvested_columns,
        # Witness-guided refinement, empty unless `relaxed_cluster_max_count` was set.
        # `K` stops being a scalar once refinement is on -- it is a starting value plus a
        # per-scenario trajectory -- so both the final cell counts and the split counts are
        # reported, or nothing downstream can attribute a result to a partition.
        "cg_relaxed_cluster_final_counts" => (
            haskey(JuMP.object_dictionary(m), :joint_routing_assignment_scenario_clusterings) ?
            [c.n_clusters for c in m[:joint_routing_assignment_scenario_clusterings]] : Int[]),
        "cg_relaxed_cluster_splits" => copy(get(JuMP.object_dictionary(m),
            :joint_routing_assignment_scenario_splits, Int[])),
        # Per scenario. These are a TWO-LEVEL hierarchy, not a flat partition -- summing
        # all five double-counts:
        #     barren_rounds  = blocked_ceiling + census_empty + census_nonempty
        #     census_nonempty = split + split_stale
        # `census_empty` vs `census_nonempty` is the "is splitting even viable" measurement,
        # and its denominator is `census_empty + census_nonempty` -- NOT `barren_rounds`,
        # because the ceiling is checked first and ceiling-blocked rounds never run a
        # census. Every nonempty census attempts its strongest split immediately.
        # `split_stale` should always be 0: witnesses are always members of the cell they
        # implicate, so a candidate is never degenerate. A nonzero value is a bug canary.
        "cg_relaxed_cluster_refine_stats" => copy(get(JuMP.object_dictionary(m),
            :joint_routing_assignment_scenario_refine_stats, Dict{Symbol, Int}[])),
        "cg_certification_sec" => st.certification_sec,
        "cg_certified_by_relaxation" => st.certified_by_relaxation,
        "cg_pricing_universe_restricted" => !st.certified_by_relaxation &&
            _cg_pricing_universe_is_restricted(st.active_pricing_mode),
        # What an OPTIMAL status on THIS result actually asserts. "elementary_routes_only"
        # means pricing never considered a revisiting column, so the optimum is optimal
        # within that restriction and may be beaten outside it.
        "cg_optimality_scope" => st.certified_by_relaxation ? "full_route_universe" :
            _cg_optimality_scope(st.active_pricing_mode),
        "cg_final_master_resolved" => final_master_resolved,
        "cg_integer_recovery" => solver.recover_integer_solution,
        "cg_iteration_log" => st.iteration_log,
        # Excludes integer recovery, unlike OptResult.runtime_sec.
        "cg_lp_loop_sec" => lp_loop_sec,
        "cg_integer_recovery_sec" => 0.0,
        "cg_pricing_stats" => copy(get(JuMP.object_dictionary(m),
            :label_setting_pricing_stats, Any[])),
        # Copied off the model HERE, before integer recovery below replaces `m` with a
        # freshly built one -- the rebuilt model carries an empty stats vector, so anything
        # left only on the model is invisible to exactly the runs that use recovery.
        "cg_relaxed_cluster_guide_stats" => copy(get(JuMP.object_dictionary(m),
            :relaxed_cluster_guide_stats, Any[])),
    )
end

