"""
Relaxed-cluster certification: iteratively refine the relaxation with no-good
cuts on cluster sets until it either certifies or produces a real improving
column. This is the whole of `pricing_mode = :relaxed_cluster` --
there is no cut-free variant, because a cut-free round is exactly this loop's
round 1 and round 1 has never certified anything.

**The cuts are the mechanism, not an optimization on top of one.** A cut-free
round gives up the moment the relaxation finds any improving cluster route, and
measurement says it always does: 0/31 attempts at every `K < n`, then
`certified_at_round_1 = 0` across every attempt of a three-size grid, at every K
(`notes/2026-09-05_relaxed_cluster_certification_and_guiding.md`, `notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`).
The reason is structural -- a converged master's exact minimum reduced cost is
exactly 0, and the relaxation's slack overshoots it by 10^2-10^3. A cut-free
mode did stop there; it was removed once that measurement showed it can only
ever fail.

But an improving *relaxed* route proves nothing about reality, which is what the
loop exploits:

    1. relaxed search (respecting all cuts so far)  ->  best improving route
    2. no improving route, search exhausted         ->  CERTIFIED
    3. T := the clusters that route visits
    4. exact search over stations(T), exhaustively
         found an improving real column  ->  REFUTED (there IS one; stop)
         nothing                         ->  T is barren: cut it, go to 1

so a spurious relaxed optimum costs one cut instead of ending the round. See
`../../cuts.jl` for the cut's exact form and why the obvious stronger version is
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
clusters there are `2^K` supports, so it terminates; the caller's wall-clock deadline,
`RELAXED_CLUSTER_MAX_CUTS` and `RELAXED_CLUSTER_MAX_CUT_ROUNDS` bound it well below that in
practice, at the cost of reporting inconclusive.

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
outstanding -- the same false-certificate failure mode `../../cuts.jl` describes for
the over-strong cut form, reached by a different route.

So a run's cut counts only make sense *per attempt*. Summed over a solve they
measure how many times the loop ran, not how deep any one of them went; the
depth of a single loop is its round count, which is what
`RELAXED_CLUSTER_MAX_CUT_ROUNDS` bounds.

# Harvesting: a refuted attempt is a pricing round, not waste

Step 4 runs the **real** exact pricer over `stations(T)` -- real stations, real duals, real
reward structure -- so when it refutes, the labels it just found ARE improving columns for
the master. This loop originally discarded them, and that is what made certification look
expensive: **753 of 788 attempts (96%) were refuted** (`notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`),
each one throwing
away a completed pricing search. `failed_certification_sec` was most of
`certification_sec` in every arm, and at n=25/K=10 it was 100% of it.

So step 4 now scores its labels through `_pricing_accept_closure` (`../../../../round.jl`),
exactly as a pricing round's phase 2 does, and the survivors ride out on the result's
`candidates`. They are deduped against the scenario's existing pool the same way,
materialized by the same `_materialize_pricing_columns`, and cross-checked against the
master by the same `_pricing_verify_column` -- a harvested column is indistinguishable
from a priced one.

**This does not weaken the certificate.** Subset searches harvest columns and prove
individual supports barren, but they are never by themselves the reason CG stops.
Convergence is still declared only by
a full-universe certificate (the relaxed search exhausting under the cuts) or by a
full-universe pricing round exhausting. Adding more valid columns to a master can never
make either claim weaker.

# The cost model, honestly

A round pays one relaxed search plus, USUALLY, one exact search over `stations(T)`. The
exact search is the expensive half, and it is exactly the work `../guiding/guide.jl`
already does -- so on rounds where step 4 finds a column this loop costs what guided
pricing costs and, with harvesting, returns that column too.

What reduces that is harvesting: a refuting round's search *is* a pricing round.

A barren-support cache (infer `T'` barren from an already-proven `T` when everything
between them is reward-free) would skip step 4 on some rounds entirely, and active-cut
subsumption pruning would keep the mask narrow. Both existed and both were removed: the
measured cut load is far too small for either to pay for itself. See `../../README.md` for
the write-ups and the numbers.
"""

export RelaxedClusterCertificationResult, RelaxedClusterNoGoodResult

"""
    RelaxedClusterCertificationResult

Outcome of one certification round, over every scenario -- the shape
`CGSolver` reads.

- `certified` -- the whole point: no improving relaxed route survives the cuts
  anywhere, proved by exhaustion. Only this makes CG's convergence claim valid.
- `improving_found` -- some scenario was *refuted*: an exhaustive exact search
  over a cluster support found a genuinely improving real column. A true
  negative, and it says nothing against the relaxation. Mutually exclusive with
  `certified`.
- `exhausted` -- every scenario reached a conclusion (none came back
  `:inconclusive`) and none was refuted. `certified == exhausted`, kept
  separately so a failure can be attributed to refutation (`improving_found`)
  or to budget (`!exhausted`). Those point at different fixes: the budget is a
  solver setting this run could be given more of, while the partition is fixed
  at build time, so looseness is only ever something to observe *across*
  runs -- see `../../clustering.jl`, and note tightness is not guaranteed
  monotone in the cluster count either.
- `scenarios_certified` / `n_scenarios` -- how many scenarios certified. A
  scenario with nothing to price counts as certified (the real pricer skips it
  on the same test).
- `candidates` -- improving columns harvested from the step-4 searches while
  FAILING to certify, for `CGSolver` to add to the master. Empty on a certified
  round, which drops its harvest on purpose: CG is about to stop, and adding
  columns to a master just proved optimal would only churn it. See the module
  docstring's "Harvesting" section.
- `relaxed_rc_bound` -- a VALID lower bound on the minimum reduced cost over the
  whole real route universe, across every scenario, or `NaN` when no such bound
  was established this round. This is the number the round already computes and
  then throws away after testing it against `-tol`; recording it is what turns a
  refuted round from a pass/fail into a measurement.

  Why the final `relaxed_rc` bounds *real* routes: a real route either escapes
  every cut, in which case its image escapes too and its reduced cost is at
  least the relaxed minimum over escaping routes; or it lies inside some cut
  support `T`, and a cut is only ever added after `stations(T)` was searched
  EXHAUSTIVELY by the exact pricer and found barren, so that route's reduced
  cost is at least `-tol`. The bound is the smaller of the two, which is the
  final `relaxed_rc` in every round that failed to certify.

  `NaN` when ANY scenario came back `:inconclusive`, and that exclusion is the
  whole correctness of the field. An inconclusive sweep stopped early, so its
  running minimum is the best value *seen*, an UPPER bound on the relaxed
  minimum -- it bounds nothing from below, and plotting it as a bound would
  read as steady progress while proving nothing. A scenario with nothing to
  price contributes `Inf`, since it vacuously bounds everything.
"""
struct RelaxedClusterCertificationResult
    certified::Bool
    improving_found::Bool
    exhausted::Bool
    scenarios_certified::Int
    n_scenarios::Int
    n_clusters::Int
    elapsed_sec::Float64
    candidates::Vector{Any}
    relaxed_rc_bound::Float64
end

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
Insert a proven-barren support as a new cut, unless the `UInt64` mask is already full.

`false` means the cut cap was reached, which the caller must report as inconclusive rather
than silently dropping the cut (see `RELAXED_CLUSTER_MAX_CUTS`).
"""
function _relaxed_cluster_add_cut!(
    cluster_sets::Vector{Set{Int}}, support::Set{Int},
)::Bool
    length(cluster_sets) < RELAXED_CLUSTER_MAX_CUTS || return false
    push!(cluster_sets, copy(support))
    return true
end

"""
    _relaxed_cluster_certify_scenario(m, s, candidates, clustering, solver;
        deadline, max_rounds=RELAXED_CLUSTER_MAX_CUT_ROUNDS) -> RelaxedClusterNoGoodResult

One scenario's loop. `deadline` is an absolute `time()` bound shared by every
round, so a scenario cannot spend more than its slice however many rounds it
takes. `max_rounds` defaults to `RELAXED_CLUSTER_MAX_CUT_ROUNDS` and is a keyword only so
tests can drive a short loop deliberately -- it is not a solver-level setting (see the
constant's docstring for why the wall clock and the cut cap are the real bounds).
"""
function _relaxed_cluster_certify_scenario(
    m::JuMP.Model, s::Int,
    candidates::AbstractVector{PassengerAssignmentCandidate},
    clustering::StationClustering, solver::CGSolver;
    deadline::Float64, max_rounds::Int=RELAXED_CLUSTER_MAX_CUT_ROUNDS,
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
    # Bumped on every refinement. Cluster indices only mean anything within one epoch, so
    # every trace row carries the epoch it was recorded under -- without it a downstream
    # containment analysis (the nesting probe does exactly this on `nogood_supports`) would
    # silently compare supports drawn from two different partitions.
    partition_epoch = 1
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
    # `support` is the actual cluster set the round cut on, not just its size. Recorded so
    # cut redundancy stays measurable from a run even though nothing acts on it:
    # `Cut(T_new)` implies `Cut(T_i)` whenever `T_i` is a SUBSET of `T_new`, which makes the
    # older cut dead weight -- it holds a mask bit and doubles the `(current, satisfied)`
    # state space while excluding nothing. See
    # `benchmarks/diagnostics/nogood_cut_nesting_probe.jl` and `../../README.md`.
    _trace_row!(round, relaxed_rc, support_size, subset_size, subset_rc, subset_checked;
                support = Set{Int}(), guide_routes = 0) =
        push!(trace, (
            round=round, relaxed_rc=relaxed_rc, support_size=support_size,
            subset_size=subset_size, subset_rc=subset_rc, subset_checked=subset_checked,
            support=support, guide_routes=guide_routes, partition_epoch=partition_epoch,
        ))

    for round in 1:max_rounds
        remaining = deadline - time()
        remaining > 0 || return RelaxedClusterNoGoodResult(
            :inconclusive, round - 1, length(cluster_sets),
            last_subset_size, trace, collect(values(harvested)))

        # (1) The relaxed guide search, respecting every cut so far. Reserve half of the
        # remaining pricing budget for the real exact search below. If this search
        # exhausts early, both stages still share the same absolute deadline, so its
        # unused time remains available to exact pricing.
        ctx = RelaxedClusterCutSearchContext(relaxed, cluster_sets)
        guide_limit = 0.5 * remaining
        labels, exhausted, _stats = _run_label_setting(
            ctx; time_limit=guide_limit, reduced_cost_tol=tol,
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

        # (3) Union the supports of the best few relaxed routes. They guide one real exact
        # search and never become master columns themselves.
        sort!(improving; by=l -> (l.reduced_cost, length(l.route)))
        n_guides = Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes])
        guides = improving[1:min(n_guides, length(improving))]
        best = first(guides)
        support = Set{Int}()
        for guide in guides, node in guide.route
            push!(support, node_clusters[node])
        end
        # `relaxed.clustering`, NOT the `clustering` argument: a refinement below rebuilds
        # `relaxed` on a split partition, and the support indices name cells of whichever
        # partition produced this route. Resolving them against the original would map the
        # support to the wrong stations.
        subset = relaxed_cluster_station_subset(
            relaxed.clustering, [sort!(collect(support))],
        )
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
        _trace_row!(round, best.reduced_cost, length(support), length(subset), subset_rc, true;
                    support=copy(support), guide_routes=length(guides))

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

        # The support is proved barren, so cut it. Cuts a new one subsumes are NOT pruned:
        # `Cut(T_new)` does imply `Cut(T_old)` for `T_old ⊆ T_new`, so the older cut is
        # then dead weight in the mask, but at the measured cut load (0.5-0.75 cuts per
        # scenario attempt at n=30/40) there is nothing there to win -- see
        # `../../README.md`.
        _relaxed_cluster_add_cut!(
            cluster_sets, support,
        ) || return RelaxedClusterNoGoodResult(
            :inconclusive, round, length(cluster_sets),
            last_subset_size, trace, collect(values(harvested)))

        # The combined support is barren, so every retained guide is spurious. Inspect all
        # of their witnesses and refine against the strongest disagreement
        # (`../refinement/refine.jl`). A split rebuilds the relaxed graph, then the active
        # cuts are rewritten onto the refined partition.
        refinement = _relaxed_cluster_refine!(
            m, s, relaxed, [guide.route for guide in guides], travel_cost,
        )
        if !isnothing(refinement)
            refined, chosen_cluster = refinement
            relaxed = create_joint_routing_assignment_relaxed_cluster_pricing_data(
                s, refined, travel_cost, candidates; shared...,
            )
            isempty(relaxed.inner.opportunities) && return RelaxedClusterNoGoodResult(
                :certified, round, length(cluster_sets),
                last_subset_size, trace, collect(values(harvested)))
            node_clusters = _relaxed_cluster_node_clusters(relaxed)
            # Cut sets are compiled against cluster INDICES, and a split renumbers what
            # those indices mean -- so they MUST be reconciled or they silently mis-mask,
            # and because a wrongly applied cut EXCLUDES relaxed routes the failure mode is
            # a false certificate rather than a crash.
            #
            # Reconcile by REWRITING, not clearing. `stations(T)` is unchanged by a split,
            # so every `T` already proven barren stays proven; it just needs the new half's
            # index added wherever the split cell appears. Clearing (the first version) was
            # also sound but threw away every cut on each split, resetting all progress
            # toward the certificate exactly when the partition got better.
            rewrite_cut_sets_for_split(
                cluster_sets, chosen_cluster, refined.n_clusters,
            )
            partition_epoch += 1
        end
    end
    return RelaxedClusterNoGoodResult(
        :inconclusive, max_rounds, length(cluster_sets),
        last_subset_size, trace, collect(values(harvested)))
end

"""
    _relaxed_cluster_scenario_pass(formulation, mapping, m, duals, solver, s,
        clustering; deadline) -> Union{Nothing, RelaxedClusterNoGoodResult}

One scenario's whole contribution to a round: duals -> candidates -> the no-good loop ->
its diagnostic row. `nothing` when the scenario has nothing to price, which is itself a
certification for it (the real pricer skips it on the same test).

The `clustering` argument is the round's shared build-time partition and is deliberately
NOT used for the search: under refinement each scenario owns its own partition, so the body
reads `_relaxed_cluster_scenario_clustering(m, s)` instead. The argument is kept only so
the signature matches the round below, which reads the build-time partition for its
reporting; do not "fix" the body to use it, which would reintroduce exactly the
stale-partition class of bug refinement is careful to avoid.

Factored out of the round below so the serial and concurrent branches share one body and
cannot drift. Safe to call from several threads at once: it only READS the model, and the
one write it makes -- the guide/stat row -- goes through `_record_relaxed_cluster_stat!`,
which takes the model's lock.
"""
function _relaxed_cluster_scenario_pass(
    formulation::AggregateODRouteJointRoutingAssignmentFormulation,
    mapping::AggregateODRouteMap, m::JuMP.Model, duals, solver::CGSolver,
    s::Int, clustering::StationClustering; deadline::Float64, iteration::Int=0,
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

    result = _relaxed_cluster_certify_scenario(
        m, s, candidates, _relaxed_cluster_scenario_clustering(m, s), solver;
        deadline=deadline,
    )
    _record_relaxed_cluster_stat!(m, (
        scenario=s,
        # WHICH CG iteration this attempt belongs to, so attempts can be ordered in time and
        # cost at late near-converged duals told apart from cost at easy early ones.
        iteration=iteration,
        guide_routes=maximum((r.guide_routes for r in result.trace); init=0),
        subset_size=result.last_subset_size,
        n_stations=length(m[:joint_routing_assignment_nodes]),
        relaxed_exhausted=(result.outcome !== :inconclusive), fell_back=false,
        nogood_outcome=result.outcome, nogood_rounds=result.rounds,
        nogood_cuts=result.cuts_added,
        # How the bound moved cut by cut -- non-decreasing by construction.
        nogood_rc_trace=[r.relaxed_rc for r in result.trace],
        nogood_subset_rc_trace=[r.subset_rc for r in result.trace],
        nogood_subset_size_trace=[r.subset_size for r in result.trace],
        nogood_subset_checked_trace=[r.subset_checked for r in result.trace],
        # The actual cluster sets cut on, in order -- so cut redundancy is measurable
        # (`benchmarks/diagnostics/nogood_cut_nesting_probe.jl`). Only rounds that really
        # cut contribute; the final round records an empty support.
        nogood_supports=[r.support for r in result.trace if !isempty(r.support)],
    ))
    return result
end

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
`_two_tier_scenario_pass` instead (`two_tier.jl`) and reuses everything else here --
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
    iteration::Int=0,
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
    time_limit::Float64,
) = throw(ArgumentError(
    "relaxed-cluster certification is only implemented for " *
    "AggregateODRouteJointRoutingAssignmentFormulation, not $(typeof(formulation))",
))
