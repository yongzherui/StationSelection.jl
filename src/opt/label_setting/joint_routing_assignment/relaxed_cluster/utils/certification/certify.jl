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

"""
    _relaxed_cluster_certify_scenario(m, s, candidates, clustering, solver;
        deadline, max_rounds=RELAXED_CLUSTER_MAX_CUT_ROUNDS) -> RelaxedClusterNoGoodResult

One scenario's loop. `deadline` is an absolute `time()` bound shared by every
round, so a scenario cannot spend more than its slice however many rounds it
takes. `max_rounds` defaults to `RELAXED_CLUSTER_MAX_CUT_ROUNDS` and is a keyword only so
tests can drive a short loop deliberately -- it is not a solver-level setting (see the
constant's docstring for why the wall clock and the cut cap are the real bounds).

Each round is the four steps of the module docstring:

  1. relaxed search under every cut so far, on half the remaining budget
  2. nothing improving survives -> `:certified` if that search EXHAUSTED, else
     `:inconclusive` (a truncated search proves nothing)
  3. `T` := the clusters the best few relaxed routes visit
  4. exact search over `stations(T)` (`_certification_station_search`, shared with the
     two-tier loop) -> refuted, or barren and therefore cut; then refine and go to 1
"""
function _relaxed_cluster_certify_scenario(
    m::JuMP.Model, s::Int,
    candidates::AbstractVector{PassengerAssignmentCandidate},
    clustering::StationClustering, solver::CGSolver;
    deadline::Float64, max_rounds::Int=RELAXED_CLUSTER_MAX_CUT_ROUNDS,
)::RelaxedClusterNoGoodResult
    shared = _certification_shared_params(m)
    travel_cost = m[:joint_routing_assignment_travel_cost]
    tol = solver.reduced_cost_tol
    n_guides = Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes])

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
    last_subset_size = 0
    trace = NamedTuple[]

    # Harvesting state, shared across this scenario's rounds (see the module docstring).
    # `harvested` is keyed by signature like a pricing round's `scored`, so several rounds
    # (each with its own subset context) accumulate into one deduped set; the pool it is
    # deduped against is `existing_columns` (see `_certification_station_search`).
    existing_columns = _certification_existing_columns(m, s)
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
    # Every exit builds the same six-field result off the same loop state, and writing it
    # out at each of the eight `return`s made the exits impossible to compare at a glance --
    # which of them differed in more than the outcome symbol was a question you had to
    # answer by diffing argument lists. Only `outcome` and the round count ever vary.
    _result(outcome::Symbol, n_rounds::Int, reason::Symbol=:none) =
        RelaxedClusterNoGoodResult(
            outcome, n_rounds, length(cluster_sets), last_subset_size,
            trace, collect(values(harvested)), reason,
        )

    for round in 1:max_rounds
        remaining = deadline - time()
        remaining > 0 || return _result(:inconclusive, round - 1, :deadline)

        # ---- (1) the relaxed guide search, respecting every cut so far.
        # Reserve half of the remaining pricing budget for the real exact search below. If
        # this search exhausts early, both stages still share the same absolute deadline, so
        # its unused time remains available to exact pricing.
        ctx = RelaxedClusterCutSearchContext(relaxed, cluster_sets)
        labels, exhausted, _stats = _run_label_setting(
            ctx; time_limit=0.5 * remaining, reduced_cost_tol=tol,
        )
        improving = filter(l -> l.reduced_cost < -tol, labels)

        # ---- (2) nothing improving survives the cuts. Only exhaustion proves that is
        # because none exists rather than because time ran out.
        if isempty(improving)
            # Record the TRUE surviving minimum, not a sentinel. It is >= -tol by definition
            # of this branch, but its actual value is what makes the trace readable -- the
            # final step reads as a cliff if the landing value is hidden. `Inf` here means
            # something different and specific: not one cut-escaping route carries any
            # reward at all.
            surviving_min = isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels)
            _trace_row!(round, surviving_min, 0, 0, Inf, false)
            return _result(exhausted ? :certified : :inconclusive, round,
                           exhausted ? :none : :relaxed_not_exhausted)
        end

        # ---- (3) union the supports of the best few relaxed routes. They guide one real
        # exact search and never become master columns themselves.
        guides = _certification_top_guides(improving, n_guides)
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

        # ---- (4) does that support hold a real improving route?
        # `search.rc` is the best REAL reduced cost inside it: below -tol means the
        # relaxation pointed somewhere genuine, `Inf` means barren.
        search = _certification_station_search(
            s, subset, travel_cost, candidates, shared, existing_columns, harvested, solver;
            time_limit=deadline - time(), reduced_cost_tol=tol,
        )
        # Out of budget before the search could start: nothing was learned about this
        # support, and reading that as barren would cut a support that may well hold an
        # improving route.
        search.outcome === :no_time && return _result(:inconclusive, round, :subset_no_time)

        # `subset_checked = true`: a subset search really ran (or was vacuously settled by
        # there being no candidates in `S`). `rc >= -tol` -- including `Inf`, meaning no
        # reward-carrying route exists in `S` at all -- is what "barren" actually means,
        # and it is the condition that adds a cut below.
        _trace_row!(round, first(guides).reduced_cost, length(support), length(subset),
                    search.rc, true; support=copy(support), guide_routes=length(guides))

        # A real improving column exists -- the relaxation was right, and this is a true
        # negative rather than a failure of the bound.
        search.rc < -tol && return _result(:refuted, round)
        # Only an EXHAUSTED subset search proves the support barren. Cutting on a
        # timed-out one would remove a support that may well hold an improving route,
        # and the loop could then certify falsely.
        search.exhausted || return _result(:inconclusive, round, :subset_not_exhausted)

        # The support is proved barren, so cut it. Cuts a new one subsumes are NOT pruned:
        # `Cut(T_new)` does imply `Cut(T_old)` for `T_old ⊆ T_new`, so the older cut is
        # then dead weight in the mask, but at the measured cut load (0.5-0.75 cuts per
        # scenario attempt at n=30/40) there is nothing there to win -- see
        # `../../README.md`.
        _relaxed_cluster_add_cut!(cluster_sets, support) ||
            return _result(:inconclusive, round, :cut_mask_full)

        # ---- refinement. The combined support is barren, so every retained guide is
        # spurious. Inspect all of their witnesses and refine against the strongest
        # disagreement (`../refinement/refine.jl`).
        refined = _relaxed_cluster_refine_after_cut!(
            m, s, relaxed, guides, candidates, travel_cost, shared, cluster_sets,
        )
        isnothing(refined) && continue                    # no split available: carry on
        refined === :exhausted && return _result(:certified, round)
        relaxed, node_clusters = refined
        partition_epoch += 1
    end
    return _result(:inconclusive, max_rounds, :round_cap)
end

"""
    _relaxed_cluster_refine_after_cut!(m, s, relaxed, guides, candidates, travel_cost,
        shared, cluster_sets) -> nothing | :exhausted | (relaxed, node_clusters)

Split the cell the barren round's witnesses disagree most about, rebuild the relaxed graph
on the finer partition, and rewrite the active cuts onto it.

Three returns because a split has three outcomes and the loop reacts differently to each:
`nothing` when no split was available (the round simply continues on the same partition),
`:exhausted` when the rebuilt graph has no opportunities left at all (which certifies, by
the same bound as an empty graph at round 0), and the new `(relaxed, node_clusters)` pair
otherwise.

Cut sets are compiled against cluster INDICES, and a split renumbers what those indices
mean -- so they MUST be reconciled or they silently mis-mask, and because a wrongly applied
cut EXCLUDES relaxed routes the failure mode is a false certificate rather than a crash.

Reconcile by REWRITING, not clearing. `stations(T)` is unchanged by a split, so every `T`
already proven barren stays proven; it just needs the new half's index added wherever the
split cell appears. Clearing (the first version) was also sound but threw away every cut on
each split, resetting all progress toward the certificate exactly when the partition got
better.
"""
function _relaxed_cluster_refine_after_cut!(
    m::JuMP.Model, s::Int, relaxed, guides, candidates, travel_cost,
    shared::NamedTuple, cluster_sets::Vector{Set{Int}},
)
    refinement = _relaxed_cluster_refine!(
        m, s, relaxed, [guide.route for guide in guides], travel_cost,
    )
    isnothing(refinement) && return nothing
    refined, chosen_cluster = refinement
    rebuilt = create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, refined, travel_cost, candidates; shared...,
    )
    isempty(rebuilt.inner.opportunities) && return :exhausted
    rewrite_cut_sets_for_split(cluster_sets, chosen_cluster, refined.n_clusters)
    return rebuilt, _relaxed_cluster_node_clusters(rebuilt)
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
