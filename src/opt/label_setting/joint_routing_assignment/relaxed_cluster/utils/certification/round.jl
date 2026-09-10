"""
The round driver both certification modes run under: fan a per-scenario `pass` across the
scenarios, then reduce their verdicts into one `RelaxedClusterCertificationResult`.

Shared on purpose. `:relaxed_cluster` (`certify.jl`) and `:relaxed_cluster_two_tier`
(`two_tier/loop.jl`) differ only in what one scenario's search does; the concurrency rule,
the budget rule, the escalation-subset rule and the reduction below are identical, and a
second copy of them is a second place for the two modes to drift apart on the meaning of
`certified`.
"""

"""
    _run_relaxed_cluster_certification_round(formulation, mapping, m, duals, solver;
        time_limit) -> RelaxedClusterCertificationResult

The `cg_certification_round` body -- the only one, since `:relaxed_cluster` is the only
certification mode. `certified` only when EVERY scenario certified, and the
refuted/inconclusive split says which fix a failure calls for.

# Scenarios run CONCURRENTLY, and every scenario is always searched

Both of those changed when harvesting landed, and both were the opposite before.

The round used to walk scenarios serially and `break` at the first refutation, on the
reasoning that one refuted scenario already settles the round so the rest is wasted work.
That reasoning died with harvesting: a refuted scenario is now a *pricing round*, so the
scenarios after it are not wasted work -- skipping them forfeits their columns. Every
scenario is therefore searched, and the round's conclusion is reduced afterwards.

Serial execution died with it for a blunter reason. Harvesting moved essentially the whole
solve inside this round -- MEASURED at 97-99% of total wall across n=20/25/30 -- while the
pricing round it displaced was already threaded. A serial round therefore left
`n_scenarios - 1` cores idle for ~99% of the run. Concurrency follows the same two switches
as pricing (`solver.parallel_scenario_pricing` or the formulation's own opt-in), so a run
cannot silently thread one round shape and not the other.

`pass` is the per-scenario body, defaulting to the one-tier
`_relaxed_cluster_scenario_pass`. `:relaxed_cluster_two_tier` supplies
`_two_tier_scenario_pass` instead (`two_tier/round.jl`) and reuses everything else here --
the concurrency rule, the budget rule and the reduction below are the same for both.

`time_limit` is budgeted the way `_run_pricing_round` budgets a pricing round's, and for
the same reason: divided across scenarios when they run serially (their searches sum),
given in full to each when they run concurrently (their searches overlap). Both honour the
same round wall.
"""
function _run_relaxed_cluster_certification_round(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64, pass::Function=_relaxed_cluster_scenario_pass,
    iteration::Int=0, only_scenarios::Union{Nothing, AbstractVector{Int}}=nothing,
)::RelaxedClusterCertificationResult
    t_start = time()
    deadline = t_start + time_limit
    clustering = _joint_routing_assignment_station_clustering(m)
    all_scenarios = _pricing_scenarios(formulation, mapping, m)
    # A restricted round prices only the scenarios named, which is how an escalation buys
    # the longer budget for the scenarios that need it WITHOUT re-running the ones that
    # already reached a verdict. Re-running a refuted scenario would re-find columns already
    # in the pool; re-running a certified one would re-prove what is proved.
    scenarios = if isnothing(only_scenarios)
        collect(all_scenarios)
    else
        wanted = Set{Int}(only_scenarios)
        unknown = setdiff(wanted, Set{Int}(all_scenarios))
        isempty(unknown) || throw(ArgumentError(
            "only_scenarios names scenarios this formulation does not price: " *
            "$(sort!(collect(unknown)))",
        ))
        [s for s in all_scenarios if s in wanted]
    end
    isempty(scenarios) && throw(ArgumentError(
        "only_scenarios selected no scenarios; an empty certification round has no meaning",
    ))
    parallel = (solver.parallel_scenario_pricing || _pricing_parallel_scenarios(formulation)) &&
        length(scenarios) > 1 && Threads.nthreads() > 1

    # `nothing` marks a scenario with nothing to price -- vacuously certified, no harvest.
    results = Vector{Any}(undef, length(scenarios))
    fill!(results, nothing)
    if parallel
        # Concurrent searches overlap, so each scenario may have the WHOLE round budget and
        # the round still finishes within its wall -- exactly `_run_pricing_round`'s rule.
        Threads.@threads for i in eachindex(scenarios)
            results[i] = pass(
                formulation, mapping, m, duals, solver, scenarios[i], clustering;
                deadline=deadline, iteration=iteration,
            )
        end
    else
        # Serial: re-divide the REMAINING budget before each scenario, so one that finishes
        # early hands its slack to those after it rather than losing it.
        for (position, i) in enumerate(eachindex(scenarios))
            remaining_scenarios = length(scenarios) - position + 1
            slice_deadline = time() + max(0.0, (deadline - time()) / remaining_scenarios)
            results[i] = pass(
                formulation, mapping, m, duals, solver, scenarios[i], clustering;
                deadline=slice_deadline, iteration=iteration,
            )
        end
    end

    certified_count = 0
    any_refuted = false
    all_conclusive = true
    harvested = Any[]
    # The round's lower bound on the real minimum reduced cost. `Inf` is the identity of
    # `min` here and also the honest value for a scenario with nothing to price, so the two
    # coincide and no special case is needed for an all-vacuous round.
    rc_bound = Inf
    inconclusive_scenarios = Int[]
    inconclusive_reasons = Symbol[]
    for (i, r) in enumerate(results)
        if isnothing(r)
            certified_count += 1     # nothing to price: vacuously certified
            continue
        end
        append!(harvested, r.candidates)
        if r.outcome === :certified
            certified_count += 1
        elseif r.outcome === :refuted
            any_refuted = true
        else
            all_conclusive = false
            push!(inconclusive_scenarios, scenarios[i])
            push!(inconclusive_reasons, r.reason)
        end
        # Only an EXHAUSTED sweep bounds from below; see the struct docstring. An
        # inconclusive scenario poisons the whole round's bound rather than being skipped,
        # because the round bounds the universe only if every scenario in it does.
        rc_bound = r.outcome === :inconclusive ? NaN :
            min(rc_bound, isempty(r.trace) ? Inf : Float64(r.trace[end].relaxed_rc))
    end

    certified = !any_refuted && all_conclusive && certified_count == length(scenarios)
    # `exhausted` means: every scenario reached a conclusion AND none was skipped. Now that
    # no scenario is ever skipped, this is exactly "nothing came back inconclusive, and
    # nothing was refuted".
    conclusive_and_complete = all_conclusive && !any_refuted
    # A certified round's harvest is dropped on purpose: CG is about to stop, and adding
    # columns to a master that has just been proved optimal would only churn it.
    return RelaxedClusterCertificationResult(
        certified, any_refuted, conclusive_and_complete, certified_count, length(scenarios),
        clustering.n_clusters, time() - t_start, certified ? Any[] : harvested, rc_bound,
        inconclusive_scenarios, collect(scenarios), inconclusive_reasons,
    )
end

"""
Every other `AggregateODRouteMap` formulation: the loop relaxes the joint
routing+assignment pricing problem's per-passenger reward structure
specifically, so there is nothing to fall back to.
`cg_certification_supported` already refuses these up front -- this method is what
turns a hypothetical direct call into the same explanation rather than a
`MethodError`.
"""
_run_relaxed_cluster_certification_round(
    formulation::AbstractFormulation, mapping, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64, iteration::Int=0,
    only_scenarios::Union{Nothing, AbstractVector{Int}}=nothing,
) = throw(ArgumentError(
    "relaxed-cluster certification is only implemented for " *
    "AggregateODRouteJointRoutingAssignmentFormulation, not $(typeof(formulation))",
))
