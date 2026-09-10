"""
`_two_tier_certify_scenario` -- one scenario's outer/inner loop, plus the three tier
searches it sequences and the accounting they feed.

The loop was one 375-line function. What made that unreadable was not its length but that
the three tiers, their retry policies, their five sets of counters and the eleven exits all
shared one scope, so nothing could be read without holding all of it. They are separated
here along the seam the algorithm already has: the outer macro round, the inner meso round,
and the exact station search each end in a verdict the caller reacts to, and each is
timed, counted and retried on its own schedule.

**The control flow stays in `_two_tier_certify_scenario` on purpose.** Every helper here
RETURNS a verdict; none of them returns from the loop. Which of the eleven exits fires is
the thing you most often need to trace, and burying a `return :inconclusive` inside a tier
helper would scatter that across four files.
"""

"""
    _TwoTierState

The loop's mutable accounting: the two cut pools, the trace, the harvest, and the per-tier
counters.

These are one struct rather than a dozen locals because they are read together and written
from four places. `sec`/`rounds` per tier is what makes a saturating attempt legible at all
-- without it the n=40 stall was a black box that took a dedicated diagnostic run to
attribute, and a per-round average could not be told apart from a slice cap it had
saturated.

`macro_cuts` are GLOBAL macro indices and `meso_cuts` GLOBAL meso indices; nothing here ever
holds a local index (see `tuning.jl` on the three index spaces).
"""
mutable struct _TwoTierState
    macro_cuts::Vector{Set{Int}}
    meso_cuts::Vector{Set{Int}}
    trace::Vector{NamedTuple}
    harvested::Dict{Any, Any}
    rounds::Int
    last_subset_size::Int
    macro_rounds::Int
    macro_sec::Float64
    meso_rounds::Int
    meso_sec::Float64
    station_rounds::Int
    station_sec::Float64
    station_unexhausted::Int
    align_skipped::Int
    align_downgraded::Int
    # Sweeps re-run on the attempt's real remaining budget after a scheduled slice truncated
    # them. These are the rounds that USED to come back `:inconclusive` and kill the
    # attempt, so the counters say how often the schedule was the binding constraint rather
    # than the instance.
    meso_escalations::Int
    station_escalations::Int
    # Guide-prefix length actually priced, per station search. `0` means even one guide's
    # aligned support overran `aligned_subset_max` and the support was priced unaligned.
    guides_used_trace::Vector{Int}
end

_TwoTierState() = _TwoTierState(
    Set{Int}[], Set{Int}[], NamedTuple[], Dict{Any, Any}(), 0, 0,
    0, 0.0, 0, 0.0, 0, 0.0, 0, 0, 0, 0, 0, Int[],
)

"""
    _two_tier_trace!(st, tier, rc, support_size, subset_size, subset_rc, checked; ...)

One trace row. `sec`/`slice`/`exhausted` are what makes a round's COST readable: without
them timing existed only as per-tier totals per attempt, so the within-attempt distribution
-- the thing the growing slice schedule exists to fix -- could not be seen at all.
"""
function _two_tier_trace!(
    st::_TwoTierState, tier::Symbol, rc, support_size::Int, subset_size::Int, subset_rc,
    checked::Bool; support::Set{Int}=Set{Int}(), guides::Int=0, aligned::Bool=false,
    sec::Float64=0.0, slice::Float64=0.0, exhausted::Bool=false,
)
    push!(st.trace, (
        round=st.rounds, relaxed_rc=rc, support_size=support_size,
        subset_size=subset_size, subset_rc=subset_rc, subset_checked=checked,
        support=support, guide_routes=guides, partition_epoch=1,
        tier=tier, aligned=aligned, macro_cuts=length(st.macro_cuts),
        meso_cuts=length(st.meso_cuts),
        sec=round(sec; digits=3), slice=round(slice; digits=3), exhausted=exhausted,
    ))
    return nothing
end

"""
    _two_tier_result(st, outcome, reason=:none) -> (result, tiers)

The shared result PLUS the per-tier accounting.

The counters cannot live on `RelaxedClusterNoGoodResult` without touching every
construction site in the one-tier loop, and they are two-tier-only, so they ride alongside.

`reason` names WHICH exit fired. Eleven paths return `:inconclusive` and they call for
completely different fixes -- a station search out of slice, an inner-round cap, a full cut
mask and a spent budget are not the same failure, and a bare `:inconclusive` cannot tell
them apart. Diagnosing the n=40 stall without this meant guessing from per-round averages
which one had fired.
"""
_two_tier_result(st::_TwoTierState, outcome::Symbol, reason::Symbol=:none) = (
    # `reason` goes on the RESULT as well as into `tiers`. It was only in `tiers`, so the
    # per-scenario reason the round collects (`RelaxedClusterCertificationResult
    # .inconclusive_reasons`, read off `NoGoodResult.reason`) defaulted to `:none` for every
    # two-tier attempt -- leaving the arm that certifies the most cells as the one arm with
    # no failure diagnosis, which is precisely backwards.
    result = RelaxedClusterNoGoodResult(
        outcome, st.rounds, length(st.meso_cuts) + length(st.macro_cuts),
        st.last_subset_size, st.trace, collect(values(st.harvested)), reason,
    ),
    tiers = (macro_rounds=st.macro_rounds, macro_sec=round(st.macro_sec; digits=2),
             meso_rounds=st.meso_rounds, meso_sec=round(st.meso_sec; digits=2),
             station_rounds=st.station_rounds, station_sec=round(st.station_sec; digits=2),
             station_unexhausted=st.station_unexhausted,
             align_skipped=st.align_skipped, align_downgraded=st.align_downgraded,
             meso_escalations=st.meso_escalations,
             station_escalations=st.station_escalations,
             guides_used_trace=copy(st.guides_used_trace),
             exit_reason=reason),
)

"""
    _two_tier_relaxed_sweep(ctx, tol; slice, deadline, retry_reserve)
        -> (; labels, improving, exhausted, elapsed_sec, total_sec, slice_sec, searches,
             escalated)

One relaxed sweep under the accumulated cuts, with the retry that both tiers need.

**A sweep truncated by its scheduled slice proves NOTHING either way**, and returning
`:inconclusive` on it throws away the whole attempt over an artefact of the schedule. So
when a sweep comes back with nothing improving but unexhausted, it is re-run once on what
the attempt actually has left (less `retry_reserve`, the work that must still fit after
it): if the sweep genuinely cannot exhaust, the second try reports that honestly, and if it
can, this is the round that earns a cut instead of ending the attempt.

Shared by the macro and meso tiers because the retry rule is identical and the two copies
of it had already begun to differ in what they reserved. What is NOT shared is the
attribution: the caller adds `searches` and `total_sec` to its own tier counters, and only
the meso tier counts `escalated` (a macro retry is unconditional -- see the caller).

`elapsed_sec` is the LAST run's wall, which is what the trace row wants; `total_sec` covers
both, which is what the tier total wants.
"""
function _two_tier_relaxed_sweep(
    ctx, tol::Float64; slice::Float64, deadline::Float64, retry_reserve::Float64,
)
    t0 = time()
    labels, exhausted, _ = _run_label_setting(
        ctx; time_limit=slice, reduced_cost_tol=tol,
    )
    elapsed = time() - t0
    total = elapsed
    searches = 1
    escalated = false
    improving = filter(l -> l.reduced_cost < -tol, labels)

    if isempty(improving) && !exhausted
        retry = (deadline - time()) - retry_reserve
        if retry > slice
            t0 = time()
            labels, exhausted, _ = _run_label_setting(
                ctx; time_limit=retry, reduced_cost_tol=tol,
            )
            elapsed = time() - t0
            total += elapsed
            searches += 1
            escalated = true
            slice = retry
            improving = filter(l -> l.reduced_cost < -tol, labels)
        end
    end
    return (labels=labels, improving=improving, exhausted=exhausted,
            elapsed_sec=elapsed, total_sec=total, slice_sec=slice,
            searches=searches, escalated=escalated)
end

"""
    _two_tier_station_phase!(st, s, cells, stations, macro_cut_cells, support_global, meso,
        candidates, travel_cost, shared, existing_columns, solver;
        deadline, attempt_budget, tol) -> (; rc, exhausted, cells, stations,
                                             macro_cut_cells, sec, slice)

The exact station search that consumes a meso round's support: refute and harvest, or prove
the support barren.

Runs up to THREE times on a decreasing ladder, and the order is the point:

  1. the support the alignment policy chose, capped at
     `RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC`.
  2. **escalate** -- a truncated search proves nothing and earns no cut, and an EXHAUSTED
     search is the only thing that does, so buying exhaustion is what the remaining budget
     is for; `:station_unexhausted` was otherwise ending attempts whose only problem was a
     30 s cap. Bounded, not "whatever is left" (see the constant), and BEFORE the downgrade
     because escalating keeps the ALIGNED support and therefore the macro cut, which is the
     cut that certifies.
  3. **downgrade** -- still unexhausted, so fall back to the strictly smaller UNALIGNED
     support, which is cheaper. Costs the macro cut, keeps the meso cut.

Both fallbacks are sound for the same reason: each cuts only what its own exhausted search
covered. Returns the support it ended on, since a downgrade changes which cut the caller may
add.
"""
function _two_tier_station_phase!(
    st::_TwoTierState, s::Int, cells::Vector{Int}, stations::Vector{Int},
    macro_cut_cells::Union{Nothing, Set{Int}}, support_global::Set{Int},
    meso::StationClustering, candidates, travel_cost, shared::NamedTuple,
    existing_columns, solver::CGSolver;
    deadline::Float64, attempt_budget::Float64, tol::Float64,
)
    function price(station_set::Vector{Int}, slice_cap::Float64)
        search = _certification_station_search(
            s, station_set, travel_cost, candidates, shared, existing_columns,
            st.harvested, solver;
            time_limit=min(deadline - time(), slice_cap), reduced_cost_tol=tol,
        )
        if search.outcome === :searched
            st.station_rounds += 1
            st.station_sec += search.elapsed_sec
            search.exhausted || (st.station_unexhausted += 1)
        end
        return search.rc, search.exhausted
    end

    t_all = time()
    base_slice = RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC
    rc, exhausted = price(stations, base_slice)
    slice_used = base_slice

    if !exhausted && rc >= -tol
        headroom = (deadline - time()) -
            (base_slice + RELAXED_CLUSTER_TWO_TIER_MIN_INNER_SEC)
        escalation = min(headroom,
                         RELAXED_CLUSTER_TWO_TIER_STATION_ESCALATION_SHARE * attempt_budget)
        if escalation > base_slice
            st.station_escalations += 1
            slice_used = escalation
            rc, exhausted = price(stations, escalation)
        end
    end

    if !exhausted && !isnothing(macro_cut_cells) && rc >= -tol
        plain = sort!(collect(support_global))
        if length(plain) < length(cells)
            st.align_downgraded += 1
            cells = plain
            stations = relaxed_cluster_station_subset(meso, [plain])
            macro_cut_cells = nothing
            st.last_subset_size = length(stations)
            rc, exhausted = price(stations, base_slice)
        end
    end

    return (rc=rc, exhausted=exhausted, cells=cells, stations=stations,
            macro_cut_cells=macro_cut_cells, sec=time() - t_all, slice=slice_used)
end

# ── the loop ────────────────────────────────────────────────────────────────────

"""
    _two_tier_certify_scenario(m, s, candidates, meso, macro_clustering, parent, solver;
        deadline, aligned_subset_max, max_macro_rounds, max_meso_rounds)
            -> (result::RelaxedClusterNoGoodResult, tiers::NamedTuple)

One scenario's two-tier loop, returning the same result type as the one-tier loop so the
round driver, the stats record and every downstream metric are unchanged.

Outer round: sweep the full macro graph under the macro cuts.

  * exhausted, nothing improving  ->  `:certified` (full universe, via the bound chain)
  * not exhausted                 ->  `:inconclusive`
  * otherwise, take the top guides' macro support `U` and descend

Inner round: sweep the meso graph restricted to `π⁻¹(U)` under the (globally valid) meso
cuts.

  * exhausted, nothing improving  ->  `Barren(stations(U))`: cut `U` at the macro layer and
                                      return to the outer round -- or `:certified` outright
                                      when `π⁻¹(U)` happens to be the whole meso graph,
                                      which is the graceful degradation to one-tier
  * not exhausted                 ->  `:inconclusive`
  * otherwise, align the meso support and exact-price its stations
    (`_two_tier_station_phase!`):
        improving found          ->  `:refuted`, having harvested real columns
        exhausted, barren        ->  meso cut, plus a macro cut when alignment applied
        not exhausted            ->  `:inconclusive` (a truncated search proves nothing)

Budget: the macro sweep takes an absolute slice, each inner meso sweep a slice that GROWS
with the round index, and the station search a bounded reserve out of what is left -- see
`tuning.jl`, where each of those and the measurement behind it is documented.
"""
function _two_tier_certify_scenario(
    m::JuMP.Model, s::Int,
    candidates::AbstractVector{PassengerAssignmentCandidate},
    meso::StationClustering, macro_clustering::StationClustering, parent::Vector{Int},
    solver::CGSolver;
    deadline::Float64, aligned_subset_max::Int,
    max_macro_rounds::Int=RELAXED_CLUSTER_MAX_CUT_ROUNDS,
    max_meso_rounds::Int=RELAXED_CLUSTER_MAX_CUT_ROUNDS,
)
    shared = _certification_shared_params(m)
    travel_cost = m[:joint_routing_assignment_travel_cost]
    tol = solver.reduced_cost_tol
    n_guides = Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes])

    macro_relaxed = create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, macro_clustering, travel_cost, candidates; shared...,
    )
    # Nothing to price in the coarsest relaxation means nothing to price at all: the macro
    # bound is below every real route's reduced cost, so an empty macro graph certifies.
    isempty(macro_relaxed.inner.opportunities) &&
        return _two_tier_result(_TwoTierState(), :certified)
    macro_node_clusters = _relaxed_cluster_node_clusters(macro_relaxed)

    st = _TwoTierState()
    existing_columns = _certification_existing_columns(m, s)
    # The attempt's whole budget, for slice shares that must scale with the tier the ladder
    # put us in (300 s ordinary vs 1800 s escalated) rather than being absolute.
    attempt_budget = max(deadline - time(), 0.0)

    for _macro_round in 1:max_macro_rounds
        remaining = deadline - time()
        remaining > 0 || return _two_tier_result(st, :inconclusive, :budget_outer)
        st.rounds += 1

        # ---- (1) the macro sweep, over the WHOLE macro graph: this is what certifies.
        #
        # A macro sweep that ran out of its slice with nothing improving is the ONE place a
        # truncation is worth escalating unconditionally: exhaustion with an empty improving
        # set is the certificate itself, so the difference between 15 s and the attempt's
        # real remainder is the difference between proving optimality and reporting
        # `:macro_unexhausted`. MEASURED: 10.7 s average macro sweeps on seed 45 against a
        # 15 s cap sized from a diagnostic that said 0.0-1.4 s.
        macro_ctx = RelaxedClusterCutSearchContext(macro_relaxed, st.macro_cuts)
        sweep = _two_tier_relaxed_sweep(
            macro_ctx, tol;
            slice=min(0.5 * remaining, RELAXED_CLUSTER_TWO_TIER_MACRO_SLICE_SEC),
            deadline=deadline,
            retry_reserve=RELAXED_CLUSTER_TWO_TIER_MIN_INNER_SEC,
        )
        st.macro_rounds += sweep.searches
        st.macro_sec += sweep.total_sec

        if isempty(sweep.improving)
            surviving = isempty(sweep.labels) ? Inf :
                minimum(l.reduced_cost for l in sweep.labels)
            _two_tier_trace!(st, :macro, surviving, 0, 0, Inf, false;
                             sec=sweep.elapsed_sec, slice=sweep.slice_sec,
                             exhausted=sweep.exhausted)
            return _two_tier_result(st, sweep.exhausted ? :certified : :inconclusive,
                                    sweep.exhausted ? :none : :macro_unexhausted)
        end

        macro_guides = _certification_top_guides(sweep.improving, n_guides)
        macro_support = Set{Int}()
        for guide in macro_guides, node in guide.route
            push!(macro_support, macro_node_clusters[node])
        end
        keep = [c for c in 1:meso.n_clusters if parent[c] in macro_support]
        covers_all = length(keep) == meso.n_clusters
        _two_tier_trace!(st, :macro, first(macro_guides).reduced_cost,
                         length(macro_support), length(keep), Inf, false;
                         support=copy(macro_support), guides=length(macro_guides),
                         sec=sweep.elapsed_sec, slice=sweep.slice_sec,
                         exhausted=sweep.exhausted)

        # ---- (2) the restricted meso sweep, and the station searches under it
        restricted = _two_tier_restrict(meso, keep)
        restricted_candidates = _restrict_candidates_to_subset(candidates, restricted.nodes)
        meso_relaxed = if isempty(restricted_candidates)
            # No reward-carrying candidate lives in this region at all, so it is barren
            # without a search -- exactly the `:no_passenger_served` case the one-tier loop
            # also allows.
            nothing
        else
            built = create_joint_routing_assignment_relaxed_cluster_pricing_data(
                s, restricted, travel_cost, restricted_candidates; shared...,
            )
            isempty(built.inner.opportunities) ? nothing : built
        end
        if isnothing(meso_relaxed)
            covers_all && return _two_tier_result(st, :certified)
            _relaxed_cluster_add_cut!(st.macro_cuts, macro_support) ||
                return _two_tier_result(st, :inconclusive, :mask_full_macro)
            continue
        end
        meso_node_clusters = _relaxed_cluster_node_clusters(meso_relaxed)

        region_settled = false
        for _meso_round in 1:min(max_meso_rounds, RELAXED_CLUSTER_TWO_TIER_MAX_INNER_ROUNDS)
            remaining = deadline - time()
            # A round that cannot fit a meso sweep AND the station search reading its support
            # can only end in a truncation-driven `:inconclusive`. Exit on the leftover
            # rather than spending it to prove nothing.
            remaining >= RELAXED_CLUSTER_TWO_TIER_MIN_INNER_SEC ||
                return _two_tier_result(st, :inconclusive, :budget_inner)
            st.rounds += 1

            # Global -> local translation happens HERE, at compile time, and the local
            # sets are never stored (see `tuning.jl` on index spaces).
            meso_ctx = RelaxedClusterCutSearchContext(
                meso_relaxed, _two_tier_local_cut_sets(st.meso_cuts, keep),
            )
            # GROWING slice, reserving room for the station search: round `k` is harder than
            # round `k-1` because it carries one more meso cut. See the constant's docstring
            # for why the old `0.5 * remaining` was exactly backwards.
            station_reserve = min(0.5 * remaining,
                                  RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC)
            meso_sweep = _two_tier_relaxed_sweep(
                meso_ctx, tol;
                slice=min(remaining - station_reserve,
                          RELAXED_CLUSTER_TWO_TIER_MESO_BASE_SLICE_SEC *
                              RELAXED_CLUSTER_TWO_TIER_MESO_SLICE_GROWTH^(_meso_round - 1)),
                deadline=deadline,
                retry_reserve=station_reserve,
            )
            st.meso_rounds += meso_sweep.searches
            st.meso_sec += meso_sweep.total_sec
            meso_sweep.escalated && (st.meso_escalations += 1)

            if isempty(meso_sweep.improving)
                surviving = isempty(meso_sweep.labels) ? Inf :
                    minimum(l.reduced_cost for l in meso_sweep.labels)
                _two_tier_trace!(st, :meso, surviving, length(keep), 0, Inf, false;
                                 sec=meso_sweep.elapsed_sec, slice=meso_sweep.slice_sec,
                                 exhausted=meso_sweep.exhausted)
                meso_sweep.exhausted ||
                    return _two_tier_result(st, :inconclusive, :meso_unexhausted)
                # The region's relaxation is exhausted with nothing improving, which is
                # `Barren(stations(U))` -- see `tuning.jl`'s route (2).
                covers_all && return _two_tier_result(st, :certified)
                _relaxed_cluster_add_cut!(st.macro_cuts, macro_support) ||
                    return _two_tier_result(st, :inconclusive, :mask_full_macro)
                region_settled = true
                break
            end

            meso_guides = _certification_top_guides(meso_sweep.improving, n_guides)
            # Local cell indices back to GLOBAL meso indices before anything is stored. Kept
            # PER GUIDE, in rank order, so alignment can shrink the prefix instead of giving
            # up its macro cut when the union overruns `aligned_subset_max`.
            guide_supports = Set{Int}[
                Set{Int}(keep[meso_node_clusters[node]] for node in guide.route)
                for guide in meso_guides
            ]
            support_global = Set{Int}()
            for gs in guide_supports
                union!(support_global, gs)
            end
            cells, stations, macro_cut_cells, guides_used =
                _two_tier_aligned_support(guide_supports, parent, meso, aligned_subset_max)
            push!(st.guides_used_trace, guides_used)
            isnothing(macro_cut_cells) && (st.align_skipped += 1)
            st.last_subset_size = length(stations)

            # ---- (3) the exact station search: refute-and-harvest, or prove barren
            station = _two_tier_station_phase!(
                st, s, cells, stations, macro_cut_cells, support_global, meso,
                candidates, travel_cost, shared, existing_columns, solver;
                deadline=deadline, attempt_budget=attempt_budget, tol=tol,
            )
            _two_tier_trace!(st, :station, first(meso_guides).reduced_cost,
                             length(support_global), length(station.stations), station.rc,
                             true; support=Set{Int}(station.cells), guides=guides_used,
                             aligned=!isnothing(station.macro_cut_cells),
                             sec=station.sec, slice=station.slice,
                             exhausted=station.exhausted)

            station.rc < -tol && return _two_tier_result(st, :refuted)
            # Only an EXHAUSTED station search proves barrenness. Cutting on a timed-out one
            # -- at either layer -- is the false-certificate failure.
            station.exhausted ||
                return _two_tier_result(st, :inconclusive, :station_unexhausted)

            _relaxed_cluster_add_cut!(st.meso_cuts, Set{Int}(station.cells)) ||
                return _two_tier_result(st, :inconclusive, :mask_full_meso)
            if !isnothing(station.macro_cut_cells)
                # Alignment: `stations(cells) == stations(macro_cut_cells)` exactly, so the
                # same exhausted search licenses the macro cut too.
                _relaxed_cluster_add_cut!(st.macro_cuts, station.macro_cut_cells) ||
                    return _two_tier_result(st, :inconclusive, :mask_full_macro)
            end
        end
        # Inner rounds ran out without settling the region: the macro layer learned nothing
        # this outer round, so another outer round would re-derive the same support.
        region_settled || return _two_tier_result(st, :inconclusive, :inner_cap)
    end
    return _two_tier_result(st, :inconclusive, :outer_cap)
end
