"""
The certification round: what a relaxed-cluster search is actually *for*.

`_run_pricing_round` (`../../round.jl`) is the wrong shape for this pricer --
it exists to harvest, verify and materialize columns, and there are none here
(`types.jl`). This file is the parallel driver that runs the same
`_run_label_setting` loop per scenario and reduces every search to one bit:

    certified  <=>  every scenario's relaxed search EXHAUSTED its frontier
                    and found no route with reduced cost < -reduced_cost_tol

By `types.jl`'s equation (5) that implies no *real* route prices below the
tolerance either, in the **full revisit-tolerant** universe, so the restricted
master LP is optimal for the complete column set and CG can stop. Anything
else -- an improving relaxed route, or a search that ran out of time -- proves
nothing, and `CGSolver` falls straight through to the real pricer.

# Why this can be much cheaper than the exact certifying round

Two compounding effects:

- the search runs on `K` cluster nodes instead of `n` stations, and the exact
  pricer's cost is super-linear in node count (measured `n^3.4`-ish in label
  count alone);
- **failure is early-exit.** The moment any relaxed route prices below the
  tolerance the answer is settled, so a round that is going to fail -- every
  round early in a solve -- typically stops after a handful of pops rather than
  running to exhaustion. Only a *successful* certification pays for a full
  search, and that is the round that ends the solve.

The price of the relaxation is bound tightness: a cluster route can be cheaper
and better-rewarded than anything real, so a relaxed round can fail on
instances where pricing genuinely is exhausted. That is a wasted round, never a
wrong answer, and it is what `n_clusters` trades off -- see `clustering.jl`.
"""

export RelaxedClusterCertificationResult

"""
    RelaxedClusterCertificationResult

Outcome of one certification round, over every scenario.

- `certified` -- the whole point: no improving relaxed route exists anywhere,
  proved by exhaustion. Only this makes CG's convergence claim valid.
- `improving_found` -- a relaxed route priced below the tolerance, so the
  relaxation is too loose (or there really is an improving column). Mutually
  exclusive with `certified`.
- `exhausted` -- every scenario that was searched ran dry rather than hitting
  its time limit, and none was skipped. `certified == exhausted &&
  !improving_found`, kept separately so a failure can be attributed to
  looseness (`improving_found`) or to budget (`!exhausted`). Those point at
  different places: the budget is a solver setting this run could be given more
  of, while the partition is fixed at build time, so looseness is only ever
  something to observe *across* runs -- see `clustering.jl`, and note tightness
  is not guaranteed monotone in the cluster count either.
- `scenarios_certified` / `n_scenarios` -- how far it got before the first
  scenario refuted it. A scenario with nothing to price counts as certified
  (the real pricer skips it on the same test); one the serial early exit never
  reached does not.
"""
struct RelaxedClusterCertificationResult
    certified::Bool
    improving_found::Bool
    exhausted::Bool
    scenarios_certified::Int
    n_scenarios::Int
    n_clusters::Int
    elapsed_sec::Float64
end

"""
    _relaxed_cluster_certify_scenario(ctx, solver, time_limit) -> (improving_found, exhausted)

Run one scenario's relaxed search, stopping at the first route below the
tolerance.

The stop predicate is safe to key on the *offered* labels only: labels are
offered whenever they improve the best-so-far for their reward-layer signature,
and for a fixed signature the reduced cost is strictly increasing in `tau`, so
the minimum-reduced-cost label of every signature is always offered. Dominance
cannot hide one either -- domination in either direction requires the
dominator's reduced cost to be no larger (`_add_pricing_label_to_state!`), so
an evicted label never had the smaller reduced cost.
"""
function _relaxed_cluster_certify_scenario(
    ctx::JointRoutingAssignmentSearchContext, solver::CGSolver, time_limit::Float64,
)
    improving = Ref(false)
    function stop_if(label)
        label.reduced_cost < -solver.reduced_cost_tol || return false
        improving[] = true
        return true
    end
    _labels, exhausted, _stats = _run_label_setting(
        ctx; time_limit=time_limit, reduced_cost_tol=solver.reduced_cost_tol, stop_if=stop_if,
    )
    return improving[], exhausted
end

"""
    _relaxed_cluster_scenario_context(formulation, mapping, m, duals, s, clustering)
        -> Union{Nothing, JointRoutingAssignmentSearchContext}

Build one scenario's relaxed search context from the current RMP duals, or
`nothing` when the scenario has nothing to price at all -- which is itself a
certification for that scenario, since the exact pricer skips it on the same
test (`_pricing_build_scenario_context`).

Candidate extraction is shared with the exact pricer verbatim
(`joint_routing_assignment_pricing_candidates`), and every scalar is read off
the same model slots the exact pricer reads, so the relaxed problem is the
exact problem plus the clustering and nothing else. That is a correctness
requirement, not tidiness: the bound only holds between two pricing problems
that agree on everything the clustering does not touch.
"""
function _relaxed_cluster_scenario_context(
    ::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, s::Int, clustering::StationClustering,
)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d,
        Float64(m[:joint_routing_assignment_walk_cost_weight]),
        Float64(m[:joint_routing_assignment_detour_factor]), s,
    )
    isempty(candidates) && return nothing

    pricing_data = create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, clustering, m[:joint_routing_assignment_travel_cost], candidates;
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )
    isempty(pricing_data.inner.opportunities) && return nothing
    # The relaxation is a graph, not a pricer: the search over it is the exact one.
    return JointRoutingAssignmentSearchContext(pricing_data.inner)
end

"""
    _run_relaxed_cluster_certification_round(formulation, mapping, m, duals, solver; time_limit)
        -> RelaxedClusterCertificationResult

One whole certification round: build every scenario's relaxed context off the
current duals, search each to exhaustion or first improving route, and reduce.

`time_limit` budgets the round's wall clock, split the same way
`_run_pricing_round` splits a pricing round's: divided equally across scenarios
when they run serially (their searches sum), given in full to each when they
run concurrently (their searches overlap). Concurrency follows the same two
switches as pricing, so a run does not silently thread one round shape and not
the other.

The clustering is read off the model, where it was computed once at build time
-- never re-derived here. See `clustering.jl` for why that has to hold for the
whole solve.
"""
function _run_relaxed_cluster_certification_round(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64,
)::RelaxedClusterCertificationResult
    t_start = time()
    clustering = _joint_routing_assignment_station_clustering(m)
    scenarios = _pricing_scenarios(formulation, mapping, m)
    parallel = (solver.parallel_scenario_pricing || _pricing_parallel_scenarios(formulation)) &&
        length(scenarios) > 1 && Threads.nthreads() > 1

    # `Vector{Bool}` rather than a `BitVector`: the parallel branch writes one element per
    # thread, and BitVector elements share machine words, so concurrent writes to distinct
    # indices would race.
    improving = fill(false, length(scenarios))
    exhausted = fill(true, length(scenarios))
    # A scenario that is never searched (nothing to price, or the serial early exit below)
    # is not evidence either way, so it is tracked separately instead of being folded into
    # `exhausted` -- otherwise the reported `scenarios_certified` would count scenarios that
    # never ran.
    searched = fill(false, length(scenarios))
    settled_early = false
    if parallel
        scenario_time_limit = time_limit
        Threads.@threads for i in eachindex(scenarios)
            ctx = _relaxed_cluster_scenario_context(
                formulation, mapping, m, duals, scenarios[i], clustering,
            )
            isnothing(ctx) && continue  # nothing to price: certified for this scenario
            searched[i] = true
            improving[i], exhausted[i] =
                _relaxed_cluster_certify_scenario(ctx, solver, scenario_time_limit)
        end
    else
        round_deadline = time() + time_limit
        for (position, i) in enumerate(eachindex(scenarios))
            ctx = _relaxed_cluster_scenario_context(
                formulation, mapping, m, duals, scenarios[i], clustering,
            )
            isnothing(ctx) && continue
            remaining_scenarios = length(scenarios) - position + 1
            slice = max(0.0, (round_deadline - time()) / remaining_scenarios)
            searched[i] = true
            improving[i], exhausted[i] =
                _relaxed_cluster_certify_scenario(ctx, solver, slice)
            # A refuted scenario settles the round: the remaining ones cannot make it
            # certify, and the real pricer is about to run anyway.
            if improving[i]
                settled_early = true
                break
            end
        end
    end

    any_improving = any(improving)
    # A scenario with nothing to price is vacuously certified; one skipped by the early
    # exit above is not, so an early-exited round never claims more than it checked.
    all_exhausted = !settled_early && all(exhausted)
    scenarios_certified = count(
        i -> !improving[i] && exhausted[i] && (searched[i] || !settled_early),
        eachindex(scenarios),
    )
    return RelaxedClusterCertificationResult(
        !any_improving && all_exhausted, any_improving, all_exhausted,
        scenarios_certified, length(scenarios), clustering.n_clusters, time() - t_start,
    )
end

"""
Every other `AggregateODRouteMap` formulation: relaxed-cluster certification is
specific to the joint routing+assignment pricing problem (it relaxes *that*
problem's per-passenger reward structure), so there is nothing to fall back to.
"""
_run_relaxed_cluster_certification_round(
    formulation::AbstractFormulation, mapping, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64,
) = throw(ArgumentError(
    "relaxed-cluster certification is only implemented for " *
    "AggregateODRouteJointRoutingAssignmentFormulation, not $(typeof(formulation))",
))

"""
    _joint_routing_assignment_station_clustering(m) -> StationClustering

The partition stashed on the model at build time. Erroring here (rather than
clustering on the spot) is deliberate: a clustering derived per round could
differ between rounds, which would make the swept `n_clusters` meaningless and
two rounds' bounds incomparable.
"""
function _joint_routing_assignment_station_clustering(m::JuMP.Model)::StationClustering
    haskey(m.obj_dict, :joint_routing_assignment_station_clustering) || throw(ArgumentError(
        "this model carries no station clustering, so relaxed-cluster certification has " *
        "nothing to run on -- build the formulation with `relaxed_cluster_count = K`",
    ))
    return m[:joint_routing_assignment_station_clustering]::StationClustering
end
