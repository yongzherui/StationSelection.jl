"""
`pricing.mode = :relaxed_cluster_two_tier` -- the same certification contract as
`:relaxed_cluster`, run over a NESTED pair of partitions instead of one.

The single-tier loop's cost is the relaxed search: MEASURED at n=40, K=24, one sweep of the
meso graph costs 10-115 s, which is the whole of a 150 s round-1 guide slice, so an attempt
gets 1-3 rounds before the geometric budget squeeze starves it (each round reserves half of
what remains). Cuts per attempt then sit at 0.11-0.57 and nothing certifies.

This mode inserts a coarse macro layer `K1 < K2` under the meso layer and searches the
cheap graph first, restricting the meso sweep to the cells the macro guides touched.
MEASURED on the same instance: 0.2-1.4 s for the pair against 22-150 s single-tier, same
column every time (`benchmarks/diagnostics/relaxed_cluster_two_tier_guide.jl`).

# The nesting, and why it is built this way

Every macro cell is exactly a union of meso cells. That is not an approximation to be
checked -- `_nested_macro_clustering` builds it by clustering the meso MEDOIDS and lifting
each meso cell to its own medoid's macro cell, so nesting holds by construction. Medoid
quality is irrelevant to the relaxation (cluster travel costs are minima over member pairs,
never medoid-to-medoid, see `../../clustering.jl`), so the medoid clustering is only a
device for choosing which meso cells group together.

Nesting buys the bound chain. For any real route `r`,

    rc_macro(image_macro r) <= rc_meso(image_meso r) <= rc_real(r)

because coarsening a partition can only lower a cluster-level cost. So an improving real
route has an improving meso image and an improving macro image, and a macro sweep that
exhausts with nothing below `-tol` is a FULL-UNIVERSE certificate exactly as the meso one
is.

# How cuts reach the macro layer

`Barren(X)` -- "the exact pricer searched station set `X` exhaustively and found nothing
improving" -- is **downward closed and not union closed**. `Barren(X1)` and `Barren(X2)`
say nothing about `Barren(X1 ∪ X2)`: a route using stations from both was examined by
neither search. So no accumulation of meso cuts can be *inferred* into a macro cut, and in
particular cutting `π(T)` because `stations(T)` is barren is UNSOUND -- `stations(π(T))`
also holds the meso cells of `π(T)` that were never in `T`, and deleting routes through
them can delete the real optimum. That is the `|route ∩ T| <= |T|-1` mistake one level up.

Two sound routes to a macro cut, both implemented here:

1. **Alignment** (`_two_tier_aligned_support`). Round the support UP to whole macro cells
   before pricing it: `T' = π⁻¹(π(T))`, so `stations(T') = stations(π(T))` exactly. One
   exhausted station search over `stations(T')` then licenses a meso cut on `T'` AND a
   macro cut on `π(T)`. Alignment never wastes work -- on a refutation the larger set can
   only harvest more -- and its cost is the station-count blow-up, which is why it is
   applied only while `|stations(T')| <= aligned_subset_max`.

   When the cap is exceeded the support is SHRUNK rather than the macro cut abandoned: the
   guide prefix is cut back (all `g` guides, then `g-1`, ...) until the aligned set fits.
   Only if even the single best guide overruns is the support priced unaligned. This matters
   because unioning `g` guide supports routinely spans more macro cells than the cap allows,
   so the cap was refusing alignment on account of `guide_routes` rather than anything about
   the instance -- MEASURED at n=40/K2=32: refused on 53-82% of station searches for the
   three seeds that fail to certify, against 0-7% for the three that succeed.

2. **Subloop exhaustion.** If the meso sweep restricted to `π⁻¹(U)` exhausts under the
   accumulated meso cuts with nothing improving, that establishes `Barren(stations(U))` and
   licenses a macro cut on `U`. Free when it happens, since the sweep already ran.

Meso cuts are ABSOLUTE statements about stations, so the pool persists across macro regions
and everything learned attacking one region carries to the next. That sharing is the
efficiency; a per-region pool would redo the work.

# Index spaces

Three of them, and a cut applied in the wrong one EXCLUDES relaxed routes, which fails as a
false certificate rather than a crash (the same hazard `../refinement/refine.jl` documents
for splits). Therefore: meso cuts are stored in GLOBAL meso indices, macro cuts in global
macro indices, and `_two_tier_local_cut_sets` translates to a restricted graph's local
numbering at the moment of compilation, never before. Nothing here stores a local index.
"""

"""Absolute wall cap on ONE macro sweep.

It must not be a fraction of the attempt's remaining budget. The macro sweep runs once per
OUTER round, so a fractional slice compounds: at `0.25 x remaining` per outer round, ten
outer rounds leave 5.6% of the attempt's budget for the work that actually finds columns,
on top of the `0.5 x remaining` each inner meso sweep already takes. That starves the loop
precisely when it is making progress.

An absolute cap is safe because the macro graph is small and its cost is bounded and
MEASURED: 0.0-1.4 s at n=40 with K1=14-16, and 0.0-0.1 s at K1<=12
(`benchmarks/diagnostics/relaxed_cluster_two_tier_guide.jl`). 15 s is an order of magnitude
of headroom over anything observed; a sweep that somehow needs more than that is reporting
that K1 is far too large for the instance, which is a configuration answer, not something
to spend an attempt's budget discovering."""
const RELAXED_CLUSTER_TWO_TIER_MACRO_SLICE_SEC = 15.0

"""Wall cap on ONE exact station search, and the reason it cannot be "whatever is left".

The search used to receive the round's entire remaining budget. A subset that does not
exhaust quickly then consumes the whole round and returns unexhausted, which proves nothing
and cuts nothing -- MEASURED at n=40, where 7 of 10 seeds burned every 300 s round while the
3 that certified ran rounds of 2-6 s. Station-search cost is super-linear in subset size
(7-8 stations exhaust in ~0.1 s, 11-13 need over a second), so the tail is steep enough that
one bad subset can eat everything.

Bounded instead, with a fallback to the smaller UNALIGNED support: spending 30 s and earning
a meso cut beats spending 300 s and earning nothing."""
const RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC = 30.0

"""Share of ONE attempt's budget a truncated station search may escalate to.

A search stopped by its slice proves nothing and earns no cut, so when the attempt has budget
left it is worth re-running -- keeping the ALIGNED support, and therefore the macro cut that
the smaller unaligned fallback would forfeit. It must still be bounded, for the reason the
base slice is bounded: one hopeless subset must not consume the budget the cheap fallback
needs.

A SHARE of the attempt's budget, not an absolute multiple of the base slice, and this
distinction was measured the hard way. The two-tier ladder runs ordinary attempts at
`pricing_time_limit_sec` (300 s) and escalated ones at `certifying_pricing_time_limit_sec`
(1800 s). An absolute cap -- this was `4 x 30 s` -- clamps the search identically in both, so
the escalated tier's extra 1500 s is unreachable by the very search that provoked the
escalation. MEASURED at n=40 seed 50, K1=14 and K1=16 alike: station slices granted were only
ever 30/48/120 s, 27 of 59 searches never exhausted, station time was 98.8% of the run's
wall, and the solve stopped on `station_unexhausted` having used 30% of its total budget.

Scaling instead: 300 s attempt -> 150 s, 1800 s attempt -> 900 s. The guard survives (half an
attempt, never all of it) and the escalated tier becomes usable. Sized at half because
station-search cost is violently super-linear in subset size -- MEASURED on the same run,
median 0.2 s at 8 stations against >180 s unexhausted at 19 -- so a search that has already
overrun its base slice is either just short of exhausting or hopeless, and half an attempt
distinguishes those without betting the attempt on it."""
const RELAXED_CLUSTER_TWO_TIER_STATION_ESCALATION_SHARE = 0.5

"""Cap on inner (meso) rounds per outer (macro) round.

One macro region must not be able to monopolise an attempt. At the default cut-round cap of
65 a single region could run 65 inner rounds, each paying a station search. Harvested
columns survive an early return (`_result` carries them), so returning after a bounded
number of inner rounds is strictly better than grinding to the deadline.

NOTE this cap has never actually bound: under the old `0.5 * remaining` inner slice only 3
inner rounds fit in a 300 s attempt and 6 in a 3600 s one (see the slice schedule below),
so the budget was the real limiter and lowering this from 65 to 8 changed nothing."""
const RELAXED_CLUSTER_TWO_TIER_MAX_INNER_ROUNDS = 8

"""Slice schedule for the inner (meso) sweeps, and why it GROWS with the round index.

The inner sweep used to take `0.5 * remaining`, which allocates the attempt exactly
backwards. Every inner round adds a meso cut, and a cut makes the label search *harder*, not
easier: escaping a cut is itself a search requirement, so the cut search must additionally
propose nodes that merely escape one (`../../cuts.jl`) and the graph it walks grows with
every round. Halving hands the hardest round the least time.

MEASURED, one 300 s attempt: slices of 142 s / 56 s / 13 s, against a meso sweep costing
59-120 s per round on the n=40 seeds that fail to certify. Round 3 is therefore truncated by
construction -- and a truncated sweep that happens to surface nothing improving used to
return `:meso_unexhausted`, killing the whole attempt over a scheduling artefact. Two such
attempts in a row end the CG solve, which is how seed 50 stopped at 663 s of a 21600 s
budget.

Growing absolute slices instead: round `k` gets `BASE * GROWTH^(k-1)`, clamped to what is
left once the station search is reserved. Rising with `k` matches the rising cost; starting
small keeps easy attempts cheap, which matters because the seeds that certify today run meso
rounds of 4-28 s and must not regress."""
const RELAXED_CLUSTER_TWO_TIER_MESO_BASE_SLICE_SEC = 25.0
const RELAXED_CLUSTER_TWO_TIER_MESO_SLICE_GROWTH = 1.7

"""Smallest inner round worth starting.

An inner round needs enough wall for a meso sweep AND the station search that consumes its
support. Below that it can only end in a truncation-driven `:inconclusive`, having spent the
tail of the attempt to prove nothing. Checked before entering the round, so the loop exits on
a budget it cannot use rather than manufacturing a false inconclusive out of it."""
const RELAXED_CLUSTER_TWO_TIER_MIN_INNER_SEC = 8.0

# ── nested partitions ───────────────────────────────────────────────────────────

"""
    _nested_macro_clustering(meso, k1, travel_cost) -> (macro_clustering, parent)

A `k1`-cell coarsening of `meso` in which every macro cell is a union of meso cells, plus
`parent[c]` giving the macro cell of meso cell `c`.

Built by clustering `meso.medoids` and lifting, so nesting is structural rather than
verified. The returned clustering spans the same station set as `meso`, which is what lets
the ordinary relaxed-graph constructor consume it unchanged.
"""
function _nested_macro_clustering(
    meso::StationClustering, k1::Int, travel_cost::Dict{Tuple{Int, Int}, Float64},
)
    1 <= k1 || throw(ArgumentError("macro count must be >= 1, got $k1"))
    k1 < meso.n_clusters || throw(ArgumentError(
        "macro count ($k1) must be below the meso count ($(meso.n_clusters)); equal " *
        "means no coarsening, which the one-tier mode already expresses",
    ))
    med = cluster_stations_by_travel_cost(meso.medoids, travel_cost, k1)
    parent = [med.cluster_of[meso.medoids[c]] for c in 1:meso.n_clusters]
    members = [Int[] for _ in 1:med.n_clusters]
    for c in 1:meso.n_clusters
        append!(members[parent[c]], meso.members[c])
    end
    foreach(sort!, members)
    all(!isempty, members) || error(
        "internal: a macro cell came out empty, which cannot happen when every macro " *
        "cell owns at least the meso cell of its own medoid",
    )
    cluster_of = Dict{Int, Int}()
    for (g, ms) in enumerate(members), st in ms
        cluster_of[st] = g
    end
    return StationClustering(
        med.n_clusters, copy(meso.nodes), cluster_of, members, copy(med.medoids),
    ), parent
end

"""
    _two_tier_restrict(meso, cells) -> StationClustering

`meso` restricted to `cells` (global meso indices), renumbered `1:length(cells)` over just
those cells' stations. `cells` is the caller's ordering and IS the local-to-global map.
"""
function _two_tier_restrict(meso::StationClustering, cells::Vector{Int})
    isempty(cells) && throw(ArgumentError("cannot restrict to an empty cell set"))
    members = [copy(meso.members[c]) for c in cells]
    nodes = sort!(unique(reduce(vcat, members; init=Int[])))
    cluster_of = Dict{Int, Int}()
    for (i, ms) in enumerate(members), st in ms
        cluster_of[st] = i
    end
    return StationClustering(
        length(cells), nodes, cluster_of, members, [meso.medoids[c] for c in cells],
    )
end

"""
    _two_tier_local_cut_sets(global_sets, cells) -> Vector{Set{Int}}

Translate cut sets from global meso indices into the local numbering of the subgraph over
`cells`. A cut says "visit at least one cell OUTSIDE this set", so the translation keeps
only the members that exist locally: local cell `i` (global `cells[i]`) is inside the cut
iff its global index is.

A cut that swallows every local cell translates to the full local set, which no local route
can escape -- correctly, because every route in this subgraph then lies inside a set already
proved barren.
"""
function _two_tier_local_cut_sets(
    global_sets::AbstractVector{Set{Int}}, cells::Vector{Int},
)::Vector{Set{Int}}
    local_of = Dict(g => i for (i, g) in enumerate(cells))
    out = Vector{Set{Int}}()
    for gs in global_sets
        push!(out, Set{Int}(local_of[g] for g in gs if haskey(local_of, g)))
    end
    return out
end

"""
    _two_tier_aligned_support(guide_supports, parent, meso, max_stations)
        -> (cells, stations, macro_cells_or_nothing, guides_used)

Round a meso support up to whole macro cells when the resulting station set fits inside
`max_stations`, SHRINKING the guide set until it does.

`guide_supports[i]` is the meso-cell support (global indices) of the `i`-th guide route, in
rank order (best reduced cost first). The union of a *prefix* is tried, longest first: all
`g` guides, then `g-1`, down to the single best one. The first prefix whose aligned station
set fits is priced aligned, which licenses BOTH a meso cut and a macro cut.

This is the fix for alignment starvation. Alignment is what carries a barrenness proof up to
the macro layer, and the macro layer is what certifies (only an exhausted macro sweep ends
the attempt with a certificate). MEASURED at n=40, K2=32/K1=16: alignment was refused on 53%
of station searches for seed 43, 54% for seed 50 and 82% for seed 45 -- none of which
certify -- against 0%, 0% and 7% for seeds 44, 48 and 49, all of which do. Refusing alignment
was previously terminal for the round's macro cut; unioning five guide supports routinely
spans more than the ~6 macro cells that 15 stations allows, so the cap was refusing
alignment because of `guide_routes`, not because of anything about the instance.

Shrinking is sound: soundness never depended on which guides produced the support, only on
cutting exclusively on a support whose station search actually EXHAUSTED. A prefix is simply
a smaller support, and `stations(pi^-1(pi(T))) == stations(pi(T))` holds for it identically.

`nothing` in the third slot means even the single best guide's aligned set overran the cap,
so the full support was priced unaligned: still a valid meso cut, no macro cut. `guides_used`
is the prefix length actually priced (`0` in the unaligned case), for instrumentation.
"""
function _two_tier_aligned_support(
    guide_supports::AbstractVector{Set{Int}}, parent::Vector{Int},
    meso::StationClustering, max_stations::Int,
)
    isempty(guide_supports) && throw(ArgumentError("need at least one guide support"))
    for n in length(guide_supports):-1:1
        support = Set{Int}()
        for i in 1:n
            union!(support, guide_supports[i])
        end
        macro_cells = Set{Int}(parent[c] for c in support)
        aligned = [c for c in 1:meso.n_clusters if parent[c] in macro_cells]
        aligned_stations = relaxed_cluster_station_subset(meso, [aligned])
        if length(aligned_stations) <= max_stations
            return aligned, aligned_stations, macro_cells, n
        end
    end
    full = Set{Int}()
    for gs in guide_supports
        union!(full, gs)
    end
    plain = sort!(collect(full))
    return plain, relaxed_cluster_station_subset(meso, [plain]), nothing, 0
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
  * otherwise, align the meso support and exact-price its stations:
        improving found          ->  `:refuted`, having harvested real columns
        exhausted, barren        ->  meso cut, plus a macro cut when alignment applied
        not exhausted            ->  `:inconclusive` (a truncated search proves nothing)

Budget: the macro sweep takes a quarter of what remains (MEASURED at 0.0-1.4 s for
`K1/K2 = 14/24` at n=40, so this is generous), each inner meso sweep half of what remains
after it, and the station search the rest -- the same shape as the one-tier round, with the
macro sweep prepended.
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
    shared = (
        route_regularization_weight=Float64(m[:joint_routing_assignment_route_regularization_weight]),
        max_wait_time=Float64(m[:joint_routing_assignment_max_wait_time]),
        repositioning_time=Float64(m[:joint_routing_assignment_repositioning_time]),
        max_stops=Int(m[:joint_routing_assignment_max_stops]),
        compensated_dominance=Bool(m[:joint_routing_assignment_compensated_dominance]),
    )
    travel_cost = m[:joint_routing_assignment_travel_cost]
    tol = solver.reduced_cost_tol
    n_guides = Int(m[:joint_routing_assignment_relaxed_cluster_guide_routes])

    macro_relaxed = create_joint_routing_assignment_relaxed_cluster_pricing_data(
        s, macro_clustering, travel_cost, candidates; shared...,
    )
    # Nothing to price in the coarsest relaxation means nothing to price at all: the macro
    # bound is below every real route's reduced cost, so an empty macro graph certifies.
    isempty(macro_relaxed.inner.opportunities) &&
        return RelaxedClusterNoGoodResult(:certified, 0, 0, 0, NamedTuple[], Any[])
    macro_node_clusters = _relaxed_cluster_node_clusters(macro_relaxed)

    # The attempt's whole budget, for slice shares that must scale with the tier the ladder
    # put us in (300 s ordinary vs 1800 s escalated) rather than being absolute.
    attempt_budget = max(deadline - time(), 0.0)
    macro_cuts = Set{Int}[]      # GLOBAL macro indices
    meso_cuts = Set{Int}[]       # GLOBAL meso indices, absolute and shared across regions
    trace = NamedTuple[]
    rounds = 0
    last_subset_size = 0
    # Per-tier accounting: which of the three searches actually consumes an attempt. Without
    # it a saturating round is a black box -- the whole reason the n=40 stall took a
    # dedicated run to explain.
    macro_rounds = 0; macro_sec = 0.0
    meso_rounds = 0;  meso_sec = 0.0
    station_rounds = 0; station_sec = 0.0
    station_unexhausted = 0; align_skipped = 0; align_downgraded = 0
    # Escalations: a sweep re-run on the attempt's real remaining budget after its scheduled
    # slice truncated it. These are the rounds that USED to come back `:inconclusive` and
    # kill the attempt, so the counters say how often the schedule was the binding
    # constraint rather than the instance.
    meso_escalations = 0; station_escalations = 0
    # Guide-prefix length actually priced, per station search. `0` means even one guide's
    # aligned support overran `aligned_subset_max` and the support was priced unaligned.
    guides_used_trace = Int[]

    existing_columns = JointRoutingAssignmentRouteColumn[
        c for c in values(m[:joint_routing_assignment_columns])
        if Int(get(c.metadata, "scenario", 0)) == s
    ]
    harvested = Dict{Any, Any}()
    # `sec`/`slice`/`exhausted` are what makes a round's COST readable. Without them timing
    # existed only as per-tier totals per attempt, so the within-attempt distribution -- the
    # thing the growing slice schedule exists to fix -- could not be seen at all, and a
    # per-round average could not be told apart from a slice cap it had saturated.
    _trace!(tier, rc, support_size, subset_size, subset_rc, checked; support=Set{Int}(),
            guides=0, aligned=false, sec=0.0, slice=0.0, exhausted=false) =
        push!(trace, (
            round=rounds, relaxed_rc=rc, support_size=support_size,
            subset_size=subset_size, subset_rc=subset_rc, subset_checked=checked,
            support=support, guide_routes=guides, partition_epoch=1,
            tier=tier, aligned=aligned, macro_cuts=length(macro_cuts),
            meso_cuts=length(meso_cuts),
            sec=round(sec; digits=3), slice=round(slice; digits=3), exhausted=exhausted,
        ))
    # Returns the shared result PLUS the per-tier accounting. The counters cannot live on
    # `RelaxedClusterNoGoodResult` without touching every construction site in
    # `certify.jl`'s one-tier loop, and they are two-tier-only, so they ride alongside.
    # `reason` names WHICH exit fired. Eleven paths return `:inconclusive` and they call for
    # completely different fixes -- a station search out of slice, an inner-round cap, a full
    # cut mask and a spent budget are not the same failure, and a bare `:inconclusive` cannot
    # tell them apart. Diagnosing the n=40 stall without this meant guessing from per-round
    # averages which one had fired.
    _result(outcome, reason::Symbol=:none) = (
        result = RelaxedClusterNoGoodResult(
            outcome, rounds, length(meso_cuts) + length(macro_cuts), last_subset_size,
            trace, collect(values(harvested)),
        ),
        tiers = (macro_rounds=macro_rounds, macro_sec=round(macro_sec; digits=2),
                 meso_rounds=meso_rounds, meso_sec=round(meso_sec; digits=2),
                 station_rounds=station_rounds, station_sec=round(station_sec; digits=2),
                 station_unexhausted=station_unexhausted,
                 align_skipped=align_skipped, align_downgraded=align_downgraded,
                 meso_escalations=meso_escalations,
                 station_escalations=station_escalations,
                 guides_used_trace=copy(guides_used_trace),
                 exit_reason=reason),
    )

    for _macro_round in 1:max_macro_rounds
        remaining = deadline - time()
        remaining > 0 || return _result(:inconclusive, :budget_outer)
        rounds += 1

        # ---- (1) the macro sweep, over the WHOLE macro graph: this is what certifies
        macro_slice = min(0.5 * remaining, RELAXED_CLUSTER_TWO_TIER_MACRO_SLICE_SEC)
        macro_ctx = RelaxedClusterCutSearchContext(macro_relaxed, macro_cuts)
        t_macro = time()
        macro_labels, macro_exhausted, _ = _run_label_setting(
            macro_ctx; time_limit=macro_slice, reduced_cost_tol=tol,
        )
        macro_elapsed = time() - t_macro
        macro_rounds += 1; macro_sec += macro_elapsed
        macro_improving = filter(l -> l.reduced_cost < -tol, macro_labels)
        # A macro sweep that ran out of its slice with nothing improving is the ONE place a
        # truncation is worth escalating unconditionally: `macro_exhausted` with an empty
        # improving set is the certificate itself, so the difference between 15 s and the
        # attempt's real remainder is the difference between proving optimality and
        # reporting `:macro_unexhausted`. MEASURED: 10.7 s average macro sweeps on seed 45
        # against a 15 s cap sized from a diagnostic that said 0.0-1.4 s.
        if isempty(macro_improving) && !macro_exhausted
            macro_retry = (deadline - time()) - RELAXED_CLUSTER_TWO_TIER_MIN_INNER_SEC
            if macro_retry > macro_slice
                t_macro = time()
                macro_labels, macro_exhausted, _ = _run_label_setting(
                    macro_ctx; time_limit=macro_retry, reduced_cost_tol=tol,
                )
                macro_elapsed = time() - t_macro
                macro_rounds += 1; macro_sec += macro_elapsed
                macro_slice = macro_retry
                macro_improving = filter(l -> l.reduced_cost < -tol, macro_labels)
            end
        end
        if isempty(macro_improving)
            surviving = isempty(macro_labels) ? Inf :
                minimum(l.reduced_cost for l in macro_labels)
            _trace!(:macro, surviving, 0, 0, Inf, false;
                    sec=macro_elapsed, slice=macro_slice, exhausted=macro_exhausted)
            return _result(macro_exhausted ? :certified : :inconclusive,
                           macro_exhausted ? :none : :macro_unexhausted)
        end

        sort!(macro_improving; by=l -> (l.reduced_cost, length(l.route)))
        macro_guides = macro_improving[1:min(n_guides, length(macro_improving))]
        macro_support = Set{Int}()
        for guide in macro_guides, node in guide.route
            push!(macro_support, macro_node_clusters[node])
        end
        keep = [c for c in 1:meso.n_clusters if parent[c] in macro_support]
        covers_all = length(keep) == meso.n_clusters
        _trace!(:macro, first(macro_guides).reduced_cost, length(macro_support),
                length(keep), Inf, false; support=copy(macro_support),
                guides=length(macro_guides), sec=macro_elapsed, slice=macro_slice,
                exhausted=macro_exhausted)

        # ---- (2) the restricted meso sweep, and the station searches under it
        restricted = _two_tier_restrict(meso, keep)
        restricted_candidates = _restrict_candidates_to_subset(candidates, restricted.nodes)
        if isempty(restricted_candidates)
            # No reward-carrying candidate lives in this region at all, so it is barren
            # without a search -- exactly the vacuous case the one-tier loop also allows.
            covers_all && return _result(:certified)
            _relaxed_cluster_add_cut!(macro_cuts, macro_support) ||
                return _result(:inconclusive, :mask_full_macro)
            continue
        end
        meso_relaxed = create_joint_routing_assignment_relaxed_cluster_pricing_data(
            s, restricted, travel_cost, restricted_candidates; shared...,
        )
        if isempty(meso_relaxed.inner.opportunities)
            covers_all && return _result(:certified)
            _relaxed_cluster_add_cut!(macro_cuts, macro_support) ||
                return _result(:inconclusive, :mask_full_macro)
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
                return _result(:inconclusive, :budget_inner)
            rounds += 1

            # Global -> local translation happens HERE, at compile time, and the local
            # sets are never stored (see the module docstring on index spaces).
            meso_ctx = RelaxedClusterCutSearchContext(
                meso_relaxed, _two_tier_local_cut_sets(meso_cuts, keep),
            )
            # GROWING slice, reserving room for the station search: round `k` is harder than
            # round `k-1` because it carries one more meso cut. See the constant's docstring
            # for why the old `0.5 * remaining` was exactly backwards.
            station_reserve = min(0.5 * remaining, RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC)
            meso_slice = min(
                remaining - station_reserve,
                RELAXED_CLUSTER_TWO_TIER_MESO_BASE_SLICE_SEC *
                    RELAXED_CLUSTER_TWO_TIER_MESO_SLICE_GROWTH^(_meso_round - 1),
            )
            t_meso = time()
            meso_labels, meso_exhausted, _ = _run_label_setting(
                meso_ctx; time_limit=meso_slice, reduced_cost_tol=tol,
            )
            meso_elapsed = time() - t_meso
            meso_rounds += 1; meso_sec += meso_elapsed
            meso_improving = filter(l -> l.reduced_cost < -tol, meso_labels)
            # A sweep truncated by its scheduled slice proves NOTHING either way, and
            # returning `:inconclusive` on it throws away the whole attempt over an artefact
            # of the schedule. Retry once on what the attempt actually has left: if the sweep
            # genuinely cannot exhaust, the second try reports that honestly, and if it can,
            # this is the round that earns a cut instead of ending the attempt.
            if isempty(meso_improving) && !meso_exhausted
                meso_retry = (deadline - time()) - station_reserve
                if meso_retry > meso_slice
                    t_meso = time()
                    meso_labels, meso_exhausted, _ = _run_label_setting(
                        meso_ctx; time_limit=meso_retry, reduced_cost_tol=tol,
                    )
                    meso_elapsed = time() - t_meso
                    meso_rounds += 1; meso_sec += meso_elapsed
                    meso_escalations += 1
                    meso_slice = meso_retry
                    meso_improving = filter(l -> l.reduced_cost < -tol, meso_labels)
                end
            end
            if isempty(meso_improving)
                surviving = isempty(meso_labels) ? Inf :
                    minimum(l.reduced_cost for l in meso_labels)
                _trace!(:meso, surviving, length(keep), 0, Inf, false;
                        sec=meso_elapsed, slice=meso_slice, exhausted=meso_exhausted)
                meso_exhausted || return _result(:inconclusive, :meso_unexhausted)
                # The region's relaxation is exhausted with nothing improving, which is
                # `Barren(stations(U))` -- see the module docstring's route (2).
                covers_all && return _result(:certified)
                _relaxed_cluster_add_cut!(macro_cuts, macro_support) ||
                    return _result(:inconclusive, :mask_full_macro)
                region_settled = true
                break
            end

            sort!(meso_improving; by=l -> (l.reduced_cost, length(l.route)))
            meso_guides = meso_improving[1:min(n_guides, length(meso_improving))]
            # Local cell indices back to GLOBAL meso indices before anything is stored. Kept
            # PER GUIDE, in rank order, so alignment can shrink the prefix instead of giving
            # up its macro cut when the union overruns `aligned_subset_max`.
            guide_supports = Set{Int}[]
            for guide in meso_guides
                gs = Set{Int}()
                for node in guide.route
                    push!(gs, keep[meso_node_clusters[node]])
                end
                push!(guide_supports, gs)
            end
            support_global = Set{Int}()
            for gs in guide_supports
                union!(support_global, gs)
            end
            cells, stations, macro_cut_cells, guides_used =
                _two_tier_aligned_support(guide_supports, parent, meso, aligned_subset_max)
            push!(guides_used_trace, guides_used)
            isnothing(macro_cut_cells) && (align_skipped += 1)
            last_subset_size = length(stations)

            # ---- (3) the exact station search: refute-and-harvest, or prove barren
            #
            # Factored into a closure so it can be called TWICE: once on the support the
            # alignment policy chose, and -- if that search did not exhaust inside its
            # slice -- again on the strictly smaller unaligned support. A search that runs
            # out of time proves nothing, so retrying smaller is the difference between
            # earning a meso cut and earning nothing. Both calls are sound: each cuts only
            # what its own exhausted search covered.
            function price_stations(
                station_set::Vector{Int};
                slice_cap::Float64=RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC,
            )
                cands = _restrict_candidates_to_subset(candidates, station_set)
                isempty(cands) && return (Inf, true)
                pd = create_joint_routing_assignment_pricing_data(
                    s, station_set, travel_cost, cands; shared...,
                )
                isempty(pd.opportunities) && return (Inf, true)
                slice = min(deadline - time(), slice_cap)
                slice > 0 || return (Inf, false)
                ctx = JointRoutingAssignmentSearchContext(pd)
                pool_tau = Dict{Any, Float64}()
                for column in existing_columns
                    sig = _pricing_pool_signature(ctx, column)
                    pool_tau[sig] = min(get(pool_tau, sig, Inf), column.tau)
                end
                # `n_candidates` unbounded on purpose: this search's other job is to prove
                # the support barren, and a truncated search proves nothing.
                accept! = _pricing_accept_closure(
                    ctx, s, pool_tau, harvested, solver, typemax(Int) ÷ 2,
                )
                t0 = time()
                labels, exhausted, _ = _run_label_setting(
                    ctx; time_limit=slice, reduced_cost_tol=tol, stop_if=accept!,
                )
                station_rounds += 1; station_sec += time() - t0
                exhausted || (station_unexhausted += 1)
                return ((isempty(labels) ? Inf : minimum(l.reduced_cost for l in labels)),
                        exhausted)
            end

            t_station_all = time()
            subset_rc, subset_exhausted = price_stations(stations)
            station_slice_used = RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC
            # A truncated station search proves nothing, so before giving up the round,
            # escalate. An EXHAUSTED search is the only thing that earns a cut, so buying
            # exhaustion is what the remaining budget is for -- `:station_unexhausted` was
            # otherwise ending attempts whose only problem was a 30 s cap.
            #
            # BOUNDED, not "whatever is left", and escalated BEFORE the downgrade below on
            # purpose. Escalating keeps the aligned support and therefore the macro cut,
            # which is the cut that certifies, so it is the more valuable of the two
            # fallbacks and goes first. But handing it the whole remainder would let one
            # hopeless subset consume the budget the cheap downgrade needs -- the very
            # failure `RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC` exists to prevent. A few
            # multiples of the base slice is enough to clear the cap-shaped failures while
            # leaving the smaller support affordable.
            if !subset_exhausted && subset_rc >= -tol
                headroom = (deadline - time()) -
                    (RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC +
                     RELAXED_CLUSTER_TWO_TIER_MIN_INNER_SEC)
                escalation = min(headroom,
                                 RELAXED_CLUSTER_TWO_TIER_STATION_ESCALATION_SHARE *
                                 attempt_budget)
                if escalation > RELAXED_CLUSTER_TWO_TIER_STATION_SLICE_SEC
                    station_escalations += 1
                    station_slice_used = escalation
                    subset_rc, subset_exhausted =
                        price_stations(stations; slice_cap=escalation)
                end
            end
            # Still unexhausted: fall back to the strictly smaller UNALIGNED support, which
            # is cheaper. Costs the macro cut, keeps the meso cut.
            if !subset_exhausted && !isnothing(macro_cut_cells) && subset_rc >= -tol
                plain = sort!(collect(support_global))
                if length(plain) < length(cells)
                    align_downgraded += 1
                    cells = plain
                    stations = relaxed_cluster_station_subset(meso, [plain])
                    macro_cut_cells = nothing
                    last_subset_size = length(stations)
                    subset_rc, subset_exhausted = price_stations(stations)
                end
            end
            _trace!(:station, first(meso_guides).reduced_cost, length(support_global),
                    length(stations), subset_rc, true; support=Set{Int}(cells),
                    guides=guides_used, aligned=!isnothing(macro_cut_cells),
                    sec=time() - t_station_all, slice=station_slice_used,
                    exhausted=subset_exhausted)

            subset_rc < -tol && return _result(:refuted)
            # Only an EXHAUSTED station search proves barrenness. Cutting on a timed-out one
            # -- at either layer -- is the false-certificate failure.
            subset_exhausted || return _result(:inconclusive, :station_unexhausted)

            _relaxed_cluster_add_cut!(meso_cuts, Set{Int}(cells)) ||
                return _result(:inconclusive, :mask_full_meso)
            if !isnothing(macro_cut_cells)
                # Alignment: `stations(cells) == stations(macro_cut_cells)` exactly, so the
                # same exhausted search licenses the macro cut too.
                _relaxed_cluster_add_cut!(macro_cuts, macro_cut_cells) ||
                    return _result(:inconclusive, :mask_full_macro)
            end
        end
        # Inner rounds ran out without settling the region: the macro layer learned nothing
        # this outer round, so another outer round would re-derive the same support.
        region_settled || return _result(:inconclusive, :inner_cap)
    end
    return _result(:inconclusive, :outer_cap)
end

# ── round wiring ────────────────────────────────────────────────────────────────

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
) = _run_relaxed_cluster_certification_round(
    formulation, mapping, m, duals, solver;
    time_limit=time_limit, pass=_two_tier_scenario_pass, iteration=iteration,
)
