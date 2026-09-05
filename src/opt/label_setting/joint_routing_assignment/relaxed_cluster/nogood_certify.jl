"""
No-good-cut certification: iteratively refine the relaxation with
no-good cuts on cluster sets until it either certifies or produces a real
improving column.

`certify.jl`'s plain loop gives up the moment the relaxation finds any improving
cluster route -- and measurement showed it always does (0/31), because a
converged master's exact minimum is exactly 0 and the relaxation's slack
overshoots it by 10^2-10^3. But an improving *relaxed* route proves nothing
about reality. This loop checks:

    1. relaxed search (respecting all cuts so far)  ->  best improving route
    2. no improving route, search exhausted         ->  CERTIFIED
    3. T := the clusters that route visits
    4. exact search over stations(T), exhaustively
         found an improving real column  ->  REFUTED (there IS one; stop)
         nothing                         ->  T is barren: cut it, go to 1

so a spurious relaxed optimum costs one cut instead of ending the round. See
`cuts.jl` for the cut's exact form and why the obvious stronger version is
unsound.

# What the loop can conclude

- **certified** -- the relaxation, restricted to routes that escape every cut,
  has no improving route. Every cut removed only cluster supports an exhaustive
  exact search had already found barren, so no real improving route's image was
  ever removed: this is a certificate over the **full** route universe.
- **refuted** -- an exhaustive exact search over some `stations(T)` produced a
  genuinely improving column. Not a failure of the relaxation; a true negative.
- **inconclusive** -- a search timed out, or the round cap or cut cap was hit.
  Proves nothing, exactly as before.

# Termination

Each cut forbids every route confined to a subset of its cluster set, so a
support once refuted can never come back and the loop cannot cycle. With `K`
clusters there are `2^K` supports, so it terminates; `max_rounds` and
`RELAXED_CLUSTER_MAX_CUTS` bound it well below that in practice, at the cost of
reporting inconclusive.

# Cuts are per attempt, and caching them across CG iterations would be UNSOUND

`cluster_sets` starts empty on every call, i.e. once per (CG iteration x
scenario), and the cuts are thrown away when the call returns. That is not a
missed optimization -- it is required.

A cut records "`stations(T)` holds no improving route", which is a statement
about the reduced costs *at the duals this attempt was given*. The next CG
iteration solves a master with new columns and therefore new duals, under which
a support that was barren can hold an improving route. Carrying the cut forward
would delete that route's image from the relaxed search while it is genuinely
improving, and the loop would then certify with an improving column still
outstanding -- the same false-certificate failure mode `cuts.jl` describes for
the over-strong cut form, reached by a different route.

So a run's cut counts only make sense *per attempt*. Summed over a solve they
measure how many times the loop ran, not how deep any one of them went; the
depth of a single loop is its round count, which is what `max_rounds` bounds.

# Harvesting: a refuted attempt is a pricing round, not waste

Step 4 runs the **real** exact pricer over `stations(T)` -- real stations, real duals, real
reward structure -- so when it refutes, the labels it just found ARE improving columns for
the master. This loop originally discarded them, and that is what made certification look
expensive: across Study 10, **753 of 788 attempts (96%) were refuted**, each one throwing
away a completed pricing search. `failed_certification_sec` was most of
`certification_sec` in every arm, and at n=25/K=10 it was 100% of it.

So step 4 now scores its labels through `_pricing_accept_closure` (`../../round.jl`),
exactly as a pricing round's phase 2 does, and the survivors ride out on the result's
`candidates`. They are deduped against the scenario's existing pool the same way,
materialized by the same `_materialize_pricing_columns`, and cross-checked against the
master by the same `_pricing_verify_column` -- a harvested column is indistinguishable
from a priced one.

**This does not weaken the certificate**, and the distinction from `guide.jl` matters.
Restricting the *pricer* to a station subset would restrict the route universe and cost
the run its full-universe claim (which is why `:relaxed_cluster_guided` reports
`cg_optimality_scope = "relaxed_cluster_station_subset_only"`). Harvesting does not do
that: these columns are never the reason CG stops. Convergence is still declared only by
a full-universe certificate (the relaxed search exhausting under the cuts) or by a
full-universe pricing round exhausting. Adding more valid columns to a master can never
make either claim weaker.

# The cost model, honestly

Every round pays one relaxed search plus one exact search over `stations(T)`. The exact
search is the expensive half, and it is exactly the work `guide.jl` already does -- so on
rounds where step 4 finds a column this loop costs what guided pricing costs and, with
harvesting, returns that column too. The remaining unrecovered spend is the barren
supports, which is the work the certificate is actually made of.
"""

export RelaxedClusterNoGoodResult

"""
Outcome of one scenario's no-good certification loop: which of the three
conclusions was reached, how many rounds and cuts it took, the size of the last
station subset it examined (the diagnostic for whether the exact half was
actually cheap), and `trace` -- one row per round recording how the bound moved.

`trace[i]` carries
`(round, relaxed_rc, support_size, subset_size, subset_rc, subset_checked)`.

- `relaxed_rc` -- the minimum reduced cost over relaxed routes escaping every cut
  so far. On the FINAL row this is the surviving minimum that is `>= -tol`, i.e.
  the value that certifies; it is a real number, not a sentinel.
- `subset_rc` -- the best REAL reduced cost the exact search found inside
  `stations(support)`. `< -tol` refutes; `>= -tol` (including `Inf`, meaning no
  reward-carrying route exists there at all) means the support is **barren** and
  a cut is added. So the barren rounds are the ones whose `subset_rc` sits at
  about zero -- not the final row.
- `subset_checked` -- `false` only on the final row, where there was no support
  left to check and `subset_rc` carries no information.

`relaxed_rc` is **monotonically non-decreasing** along the trace, and that is not
an empirical observation but a property: each cut only ever removes relaxed
routes, so the minimum over the survivors can only rise. A trace that dips is a
bug in the cut machinery -- the search would have to be finding a route a
previous round's cut should already have excluded.

`candidates` carries the improving columns the loop's step-4 searches found on the way --
see the module docstring's "Harvesting" section.
"""
struct RelaxedClusterNoGoodResult
    outcome::Symbol          # :certified, :refuted, :inconclusive
    rounds::Int
    cuts_added::Int
    last_subset_size::Int
    trace::Vector{NamedTuple}
    candidates::Vector{Any}
end

"""
    _relaxed_cluster_nogood_certify_scenario(m, s, candidates, clustering, solver;
        deadline, max_rounds) -> RelaxedClusterNoGoodResult

One scenario's loop. `deadline` is an absolute `time()` bound shared by every
round, so a scenario cannot spend more than its slice however many rounds it
takes.
"""
function _relaxed_cluster_nogood_certify_scenario(
    m::JuMP.Model, s::Int,
    candidates::AbstractVector{PassengerAssignmentCandidate},
    clustering::StationClustering, solver::CGSolver;
    deadline::Float64, max_rounds::Int,
)::RelaxedClusterNoGoodResult
    shared = (
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )
    travel_cost = m[:joint_routing_assignment_travel_cost]
    relaxed = create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, clustering, travel_cost, candidates; shared...,
    )
    # Nothing to price in the relaxation means nothing to price at all (the bound).
    isempty(relaxed.inner.opportunities) &&
        return RelaxedClusterNoGoodResult(:certified, 0, 0, 0, NamedTuple[], Any[])

    node_clusters = _relaxed_cluster_node_clusters(relaxed)
    cluster_sets = Set{Int}[]
    tol = solver.reduced_cost_tol
    last_subset_size = 0
    trace = NamedTuple[]

    # Harvesting state, shared across this scenario's rounds (see the module docstring).
    # `best_pool_tau` mirrors `_prepare_pricing_scenario`: a candidate only counts as novel
    # if it beats the best tau already in the master's pool for its signature, so a round
    # cannot re-offer a column the master already has. `harvested` is keyed by signature
    # like a pricing round's `scored`, so several rounds (each with its own subset context)
    # accumulate into one deduped set.
    existing_columns = JointRoutingAssignmentRouteColumn[
        c for c in values(m[:joint_routing_assignment_columns])
        if Int(get(c.metadata, "scenario", 0)) == s
    ]
    harvested = Dict{Any, Any}()
    _trace_row!(round, relaxed_rc, support_size, subset_size, subset_rc, subset_checked) =
        push!(trace, (
            round=round, relaxed_rc=relaxed_rc, support_size=support_size,
            subset_size=subset_size, subset_rc=subset_rc, subset_checked=subset_checked,
        ))

    for round in 1:max_rounds
        remaining = deadline - time()
        remaining > 0 || return RelaxedClusterNoGoodResult(
            :inconclusive, round - 1, length(cluster_sets),
            last_subset_size, trace, collect(values(harvested)))

        # (1) the relaxed search, respecting every cut so far.
        ctx = RelaxedClusterCutSearchContext(relaxed, cluster_sets)
        labels, exhausted, _stats = _run_label_setting(
            ctx; time_limit=remaining, reduced_cost_tol=tol,
        )
        improving = filter(l -> l.reduced_cost < -tol, labels)
        if isempty(improving)
            # (2) Nothing improving survives the cuts. Only exhaustion proves that is
            # because none exists rather than because time ran out.
            # Record the TRUE surviving minimum, not a sentinel. It is >= -tol by
            # definition of this branch, but its actual value is what makes the trace
            # readable -- the final step reads as a cliff if the landing value is hidden.
            # `Inf` here means something different and specific: not one cut-escaping
            # route carries any reward at all.
            surviving_min = isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels)
            _trace_row!(round, surviving_min, 0, 0, Inf, false)
            return RelaxedClusterNoGoodResult(
                exhausted ? :certified : :inconclusive, round, length(cluster_sets),
                last_subset_size, trace, collect(values(harvested)))
        end

        # (3) the cluster support of the best surviving relaxed route.
        best = argmin(l -> l.reduced_cost, improving)
        support = Set{Int}(node_clusters[node] for node in best.route)
        subset = relaxed_cluster_station_subset(clustering, [sort!(collect(support))])
        last_subset_size = length(subset)

        # (4) does that support hold a real improving route?
        subset_candidates = _restrict_candidates_to_subset(candidates, subset)
        # `subset_rc` is the best REAL reduced cost inside this support: below -tol means
        # the relaxation pointed somewhere genuine, `Inf` means barren.
        subset_rc, subset_exhausted = if isempty(subset_candidates)
            Inf, true         # no candidates at all: vacuously barren, and provably so
        else
            subset_pricing = create_joint_routing_assignment_pricing_data(
                s, subset, travel_cost, subset_candidates; shared...,
            )
            if isempty(subset_pricing.opportunities)
                Inf, true
            else
                remaining = deadline - time()
                remaining > 0 || return RelaxedClusterNoGoodResult(
                    :inconclusive, round, length(cluster_sets),
                    last_subset_size, trace, collect(values(harvested)))
                # Harvest while refuting. The accept closure is a pricing round's, so a
                # kept candidate is deduped against the pool and against earlier rounds
                # exactly as phase 2 would do it. It must NOT stop the search early
                # (`n_candidates = typemax`): this search's other job is to prove the
                # support barren, and a truncated search proves nothing, so a cut may only
                # be added after it runs to exhaustion.
                sub_ctx = JointRoutingAssignmentSearchContext(subset_pricing)
                sub_best_pool_tau = Dict{Any, Float64}()
                for column in existing_columns
                    sig = _pricing_pool_signature(sub_ctx, column)
                    sub_best_pool_tau[sig] = min(get(sub_best_pool_tau, sig, Inf), column.tau)
                end
                accept! = _pricing_accept_closure(
                    sub_ctx, s, sub_best_pool_tau, harvested, solver, typemax(Int) ÷ 2,
                )
                sub_labels, sub_exhausted, _ = _run_label_setting(
                    sub_ctx; time_limit=remaining, reduced_cost_tol=tol, stop_if=accept!,
                )
                (isempty(sub_labels) ? Inf : minimum(l.reduced_cost for l in sub_labels)),
                    sub_exhausted
            end
        end
        # `subset_checked = true`: a subset search really ran (or was vacuously settled by
        # there being no candidates in `S`). `subset_rc >= -tol` -- including `Inf`, meaning
        # no reward-carrying route exists in `S` at all -- is what "barren" actually means,
        # and it is the condition that adds a cut below.
        _trace_row!(round, best.reduced_cost, length(support), length(subset), subset_rc, true)

        # A real improving column exists -- the relaxation was right, and this is a true
        # negative rather than a failure of the bound.
        subset_rc < -tol && return RelaxedClusterNoGoodResult(
            :refuted, round, length(cluster_sets),
            last_subset_size, trace, collect(values(harvested)))
        # Only an EXHAUSTED subset search proves the support barren. Cutting on a
        # timed-out one would remove a support that may well hold an improving route,
        # and the loop could then certify falsely.
        subset_exhausted || return RelaxedClusterNoGoodResult(
            :inconclusive, round, length(cluster_sets),
            last_subset_size, trace, collect(values(harvested)))

        length(cluster_sets) < RELAXED_CLUSTER_MAX_CUTS || return RelaxedClusterNoGoodResult(
            :inconclusive, round, length(cluster_sets),
            last_subset_size, trace, collect(values(harvested)))
        push!(cluster_sets, support)
    end
    return RelaxedClusterNoGoodResult(
        :inconclusive, max_rounds, length(cluster_sets),
        last_subset_size, trace, collect(values(harvested)))
end

"""
    _relaxed_cluster_nogood_scenario_pass(formulation, mapping, m, duals, solver, s,
        clustering; deadline) -> Union{Nothing, RelaxedClusterNoGoodResult}

One scenario's whole contribution to a round: duals -> candidates -> the no-good loop ->
its diagnostic row. `nothing` when the scenario has nothing to price, which is itself a
certification for it (the real pricer skips it on the same test).

Factored out of the round below so the serial and concurrent branches share one body and
cannot drift. Safe to call from several threads at once: it only READS the model, and the
one write it makes -- the guide/stat row -- goes through `_record_relaxed_cluster_stat!`,
which takes the model's lock.
"""
function _relaxed_cluster_nogood_scenario_pass(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver,
    s::Int, clustering::StationClustering; deadline::Float64,
)
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]
    candidates = joint_routing_assignment_pricing_candidates(
        data, mapping, alpha, gamma_o, gamma_d,
        Float64(m[:joint_routing_assignment_walk_cost_weight]),
        Float64(m[:joint_routing_assignment_detour_factor]), s,
    )
    # Nothing to price: vacuously certified for this scenario, and it harvests nothing.
    isempty(candidates) && return nothing

    result = _relaxed_cluster_nogood_certify_scenario(
        m, s, candidates, clustering, solver;
        deadline=deadline, max_rounds=solver.certification_max_rounds,
    )
    _record_relaxed_cluster_stat!(m, (
        scenario=s, guide_routes=result.rounds, subset_size=result.last_subset_size,
        n_stations=length(m[:joint_routing_assignment_nodes]),
        relaxed_exhausted=(result.outcome !== :inconclusive), fell_back=false,
        nogood_outcome=result.outcome, nogood_rounds=result.rounds,
        nogood_cuts=result.cuts_added,
        # How the bound moved cut by cut -- non-decreasing by construction.
        nogood_rc_trace=[r.relaxed_rc for r in result.trace],
        nogood_subset_rc_trace=[r.subset_rc for r in result.trace],
        nogood_subset_size_trace=[r.subset_size for r in result.trace],
        nogood_subset_checked_trace=[r.subset_checked for r in result.trace],
    ))
    return result
end

"""
    _run_relaxed_cluster_nogood_certification_round(formulation, mapping, m, duals, solver;
        time_limit) -> RelaxedClusterCertificationResult

The `cg_certification_round` body for `:relaxed_cluster_nogood`. Same contract and same
reporting shape as `certify.jl`'s plain round, so `CGSolver` needs no special case:
`certified` only when EVERY scenario certified, and the refuted/inconclusive split still
says which fix a failure calls for.

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

`time_limit` is budgeted the way `_run_pricing_round` budgets a pricing round's, and for
the same reason: divided across scenarios when they run serially (their searches sum),
given in full to each when they run concurrently (their searches overlap). Both honour the
same round wall.
"""
function _run_relaxed_cluster_nogood_certification_round(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64,
)::RelaxedClusterCertificationResult
    t_start = time()
    deadline = t_start + time_limit
    clustering = _joint_routing_assignment_station_clustering(m)
    scenarios = _pricing_scenarios(formulation, mapping, m)
    parallel = (solver.parallel_scenario_pricing || _pricing_parallel_scenarios(formulation)) &&
        length(scenarios) > 1 && Threads.nthreads() > 1

    # `nothing` marks a scenario with nothing to price -- vacuously certified, no harvest.
    results = Vector{Any}(undef, length(scenarios))
    fill!(results, nothing)
    if parallel
        # Concurrent searches overlap, so each scenario may have the WHOLE round budget and
        # the round still finishes within its wall -- exactly `_run_pricing_round`'s rule.
        Threads.@threads for i in eachindex(scenarios)
            results[i] = _relaxed_cluster_nogood_scenario_pass(
                formulation, mapping, m, duals, solver, scenarios[i], clustering;
                deadline=deadline,
            )
        end
    else
        # Serial: re-divide the REMAINING budget before each scenario, so one that finishes
        # early hands its slack to those after it rather than losing it.
        for (position, i) in enumerate(eachindex(scenarios))
            remaining_scenarios = length(scenarios) - position + 1
            slice_deadline = time() + max(0.0, (deadline - time()) / remaining_scenarios)
            results[i] = _relaxed_cluster_nogood_scenario_pass(
                formulation, mapping, m, duals, solver, scenarios[i], clustering;
                deadline=slice_deadline,
            )
        end
    end

    certified_count = 0
    any_refuted = false
    all_conclusive = true
    harvested = Any[]
    for r in results
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
        end
    end

    certified = !any_refuted && all_conclusive && certified_count == length(scenarios)
    # `exhausted` carries `certify.jl`'s meaning: every scenario reached a conclusion AND
    # none was skipped. Now that no scenario is ever skipped, this is exactly "nothing came
    # back inconclusive, and nothing was refuted".
    conclusive_and_complete = all_conclusive && !any_refuted
    # A certified round's harvest is dropped on purpose: CG is about to stop, and adding
    # columns to a master that has just been proved optimal would only churn it.
    return RelaxedClusterCertificationResult(
        certified, any_refuted, conclusive_and_complete, certified_count, length(scenarios),
        clustering.n_clusters, time() - t_start, certified ? Any[] : harvested,
    )
end

"""
Every other `AggregateODRouteMap` formulation, mirroring `certify.jl`'s fallback:
the no-good loop relaxes the joint routing+assignment pricing problem's
per-passenger reward structure specifically, so there is nothing to fall back to.
`cg_certification_supported` already refuses these up front -- this method is what
turns a hypothetical direct call into the same explanation rather than a
`MethodError`.
"""
_run_relaxed_cluster_nogood_certification_round(
    formulation::AbstractFormulation, mapping, m::JuMP.Model, duals, solver::CGSolver;
    time_limit::Float64,
) = throw(ArgumentError(
    "relaxed-cluster no-good certification is only implemented for " *
    "AggregateODRouteJointRoutingAssignmentFormulation, not $(typeof(formulation))",
))
