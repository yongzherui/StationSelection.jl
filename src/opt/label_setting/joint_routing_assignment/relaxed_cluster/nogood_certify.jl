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

# The cost model, honestly

Every round pays one relaxed search plus one exact search over `stations(T)`.
The exact search is the expensive half, and it is exactly the work `guide.jl`
already does -- so on rounds where step 4 finds a column this loop costs what
guided pricing costs and returns the same column. The extra spend is only on
barren supports, and only near convergence, which is where certification is the
thing that matters.
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
"""
struct RelaxedClusterNoGoodResult
    outcome::Symbol          # :certified, :refuted, :inconclusive
    rounds::Int
    cuts_added::Int
    last_subset_size::Int
    trace::Vector{NamedTuple}
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
        return RelaxedClusterNoGoodResult(:certified, 0, 0, 0, NamedTuple[])

    node_clusters = _relaxed_cluster_node_clusters(relaxed)
    cluster_sets = Set{Int}[]
    tol = solver.reduced_cost_tol
    last_subset_size = 0
    trace = NamedTuple[]
    _trace_row!(round, relaxed_rc, support_size, subset_size, subset_rc, subset_checked) =
        push!(trace, (
            round=round, relaxed_rc=relaxed_rc, support_size=support_size,
            subset_size=subset_size, subset_rc=subset_rc, subset_checked=subset_checked,
        ))

    for round in 1:max_rounds
        remaining = deadline - time()
        remaining > 0 || return RelaxedClusterNoGoodResult(
            :inconclusive, round - 1, length(cluster_sets), last_subset_size, trace)

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
                exhausted ? :certified : :inconclusive,
                round, length(cluster_sets), last_subset_size, trace)
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
                    :inconclusive, round, length(cluster_sets), last_subset_size, trace)
                sub_labels, sub_exhausted, _ = _run_label_setting(
                    JointRoutingAssignmentSearchContext(subset_pricing);
                    time_limit=remaining, reduced_cost_tol=tol,
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
            :refuted, round, length(cluster_sets), last_subset_size, trace)
        # Only an EXHAUSTED subset search proves the support barren. Cutting on a
        # timed-out one would remove a support that may well hold an improving route,
        # and the loop could then certify falsely.
        subset_exhausted || return RelaxedClusterNoGoodResult(
            :inconclusive, round, length(cluster_sets), last_subset_size, trace)

        length(cluster_sets) < RELAXED_CLUSTER_MAX_CUTS || return RelaxedClusterNoGoodResult(
            :inconclusive, round, length(cluster_sets), last_subset_size, trace)
        push!(cluster_sets, support)
    end
    return RelaxedClusterNoGoodResult(
        :inconclusive, max_rounds, length(cluster_sets), last_subset_size, trace)
end

"""
    _run_relaxed_cluster_nogood_certification_round(formulation, mapping, m, duals, solver;
        time_limit) -> RelaxedClusterCertificationResult

The `cg_certification_round` body for `:relaxed_cluster_nogood`. Same contract
and same reporting shape as `certify.jl`'s plain round, so `CGSolver` needs no
special case: `certified` only when EVERY scenario certified, and the
refuted/inconclusive split still says which fix a failure calls for.

Scenarios are searched serially even when pricing is threaded. The loop is
adaptive -- how long a scenario needs depends on how many barren supports it
walks through -- so a shared deadline consumed in order lets a scenario that
certifies in one round hand its slack to a harder one, which a fixed per-scenario
split cannot. It also stops at the first refutation, since one refuted scenario
already settles the round.
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
    alpha, gamma_o, gamma_d = duals
    data = m[:joint_routing_assignment_data]

    certified_count = 0
    any_refuted = false
    all_conclusive = true
    for (position, s) in enumerate(scenarios)
        candidates = joint_routing_assignment_pricing_candidates(
            data, mapping, alpha, gamma_o, gamma_d,
            Float64(m[:joint_routing_assignment_walk_cost_weight]),
            Float64(m[:joint_routing_assignment_detour_factor]), s,
        )
        if isempty(candidates)
            certified_count += 1     # nothing to price: vacuously certified
            continue
        end
        remaining_scenarios = length(scenarios) - position + 1
        slice_deadline = min(deadline, time() + max(0.0, (deadline - time()) / remaining_scenarios))
        result = _relaxed_cluster_nogood_certify_scenario(
            m, s, candidates, clustering, solver;
            deadline=slice_deadline, max_rounds=solver.certification_max_rounds,
        )
        _record_relaxed_cluster_guide_stat!(m, (
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
        if result.outcome === :certified
            certified_count += 1
        elseif result.outcome === :refuted
            any_refuted = true
            break                     # one refuted scenario settles the round
        else
            all_conclusive = false
        end
    end

    certified = !any_refuted && all_conclusive && certified_count == length(scenarios)
    return RelaxedClusterCertificationResult(
        certified, any_refuted, all_conclusive, certified_count, length(scenarios),
        clustering.n_clusters, time() - t_start,
    )
end
