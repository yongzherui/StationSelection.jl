"""
Round wiring for `:relaxed_cluster_two_tier`: one scenario's pass, and the
`cg_certification_round` body.

Both are thin. The pass is `loop.jl`'s scenario loop plus the stat row it records, and the
round body is `../round.jl`'s driver with that pass substituted in -- concurrency, budgeting
and the round-level reduction are shared, so the two modes cannot drift apart on any of
them.
"""

"""
    _two_tier_scenario_pass(formulation, mapping, m, duals, solver, s, meso; deadline)

The `pass` the two-tier round hands `_run_relaxed_cluster_certification_round`. Same
signature and same stat record as `_relaxed_cluster_scenario_pass`, so every existing
metric (`cg_certification_*`, the guide stats, the benchmark trace summaries) reads a
two-tier run without changes. The extra per-round detail rides along in the trace rows'
`tier` / `aligned` / `macro_cuts` fields.

`meso` is the build-time partition passed down by the round; the macro layer and its parent
map are read off the model, where the build stashed them once (identical across every
iteration, for the same reason `relaxed_cluster_count` is a build-time input).

Safe to call from several threads: it only reads the model apart from the stat row, which
takes the model's lock. Refinement is not available here -- `CGPricingConfig` rejects
`relaxed_cluster_max_count` together with this mode -- so there is no per-scenario partition
to fetch and no epoch to track.
"""
function _two_tier_scenario_pass(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver,
    s::Int, meso::StationClustering; deadline::Float64, iteration::Int=0,
)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d,
        Float64(m[:joint_routing_assignment_walk_cost_weight]),
        Float64(m[:joint_routing_assignment_detour_factor]), s,
    )
    isempty(candidates) && return nothing

    result, tiers = _two_tier_certify_scenario(
        m, s, candidates, meso,
        m[:joint_routing_assignment_macro_clustering],
        m[:joint_routing_assignment_macro_parent],
        solver;
        deadline=deadline,
        aligned_subset_max=Int(m[:joint_routing_assignment_aligned_subset_max]),
    )
    _record_relaxed_cluster_stat!(m, (
        scenario=s,
        guide_routes=maximum((r.guide_routes for r in result.trace); init=0),
        subset_size=result.last_subset_size,
        n_stations=length(m[:joint_routing_assignment_nodes]),
        relaxed_exhausted=(result.outcome !== :inconclusive), fell_back=false,
        nogood_outcome=result.outcome, nogood_rounds=result.rounds,
        nogood_cuts=result.cuts_added,
        nogood_rc_trace=[r.relaxed_rc for r in result.trace],
        nogood_subset_rc_trace=[r.subset_rc for r in result.trace],
        nogood_subset_size_trace=[r.subset_size for r in result.trace],
        nogood_subset_checked_trace=[r.subset_checked for r in result.trace],
        nogood_supports=[r.support for r in result.trace if !isempty(r.support)],
        # Two-tier only: which layer each round ran on, whether the station set it priced
        # was macro-aligned (and therefore licensed a macro cut), and how the two cut pools
        # grew. This is the diagnostic for whether alignment is paying.
        two_tier_tier_trace=[r.tier for r in result.trace],
        two_tier_aligned_trace=[r.aligned for r in result.trace],
        two_tier_macro_cuts=maximum((r.macro_cuts for r in result.trace); init=0),
        two_tier_meso_cuts=maximum((r.meso_cuts for r in result.trace); init=0),
        # WHERE THE TIME WENT, per tier. A saturating attempt is otherwise a black box: the
        # n=40 stall needed a dedicated run to attribute, and this is what makes it readable
        # off a result row instead.
        two_tier_macro_rounds=tiers.macro_rounds,
        two_tier_macro_sec=tiers.macro_sec,
        two_tier_meso_rounds=tiers.meso_rounds,
        two_tier_meso_sec=tiers.meso_sec,
        two_tier_station_rounds=tiers.station_rounds,
        two_tier_station_sec=tiers.station_sec,
        two_tier_station_unexhausted=tiers.station_unexhausted,
        two_tier_align_skipped=tiers.align_skipped,
        two_tier_align_downgraded=tiers.align_downgraded,
        # How often a scheduled slice truncated a sweep that the attempt's real remaining
        # budget could then finish. High counts mean the SCHEDULE was the binding constraint,
        # which is a different fix from the instance being hard.
        two_tier_meso_escalations=tiers.meso_escalations,
        two_tier_station_escalations=tiers.station_escalations,
        # Guide-prefix length alignment actually priced, per station search; `0` means the
        # support was priced unaligned (no macro cut). Reads whether guide shrinking is what
        # is buying the macro cuts.
        two_tier_guides_used_trace=tiers.guides_used_trace,
        # Per-round cost, so the within-attempt distribution is visible rather than only the
        # per-tier totals. `slice` is what the round was GRANTED and `sec` what it used, so a
        # round that saturated its cap can be told from one that finished inside it.
        nogood_round_sec_trace=[r.sec for r in result.trace],
        nogood_round_slice_trace=[r.slice for r in result.trace],
        nogood_round_exhausted_trace=[r.exhausted for r in result.trace],
        # WHICH CG iteration this attempt belongs to. Without it attempts cannot be ordered
        # in time, and "cost at late, near-converged duals" -- the regime that decides
        # certification -- is exactly what needs to be read off them.
        iteration=iteration,
        # WHICH of the eleven exits ended this attempt.
        two_tier_exit_reason=tiers.exit_reason,
    ))
    return result
end

"""
    _run_two_tier_certification_round(formulation, mapping, m, duals, solver; time_limit)

`cg_certification_round`'s body for `:relaxed_cluster_two_tier`: the one-tier round driver,
with the two-tier per-scenario pass substituted in. Concurrency, budgeting and the
round-level reduction are shared, so the two modes cannot drift apart on any of them.
"""
_run_two_tier_certification_round(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64, iteration::Int=0,
    only_scenarios::Union{Nothing, AbstractVector{Int}}=nothing,
) = _run_relaxed_cluster_certification_round(
    formulation, mapping, m, duals, solver;
    time_limit=time_limit, pass=_two_tier_scenario_pass, iteration=iteration,
    only_scenarios=only_scenarios,
)
