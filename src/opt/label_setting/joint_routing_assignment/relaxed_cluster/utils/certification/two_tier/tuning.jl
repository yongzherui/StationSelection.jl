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
never medoid-to-medoid, see `../../../clustering.jl`), so the medoid clustering is only a
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
false certificate rather than a crash (the same hazard `../../refinement/refine.jl` documents
for splits). Therefore: meso cuts are stored in GLOBAL meso indices, macro cuts in global
macro indices, and `_two_tier_local_cut_sets` translates to a restricted graph's local
numbering at the moment of compilation, never before. Nothing here stores a local index.

# Where the pieces live

    tuning.jl      this file -- the module docstring above and the six tuning constants,
                   each carrying the measurement that set it
    partitions.jl  the nesting itself: building the macro layer, restricting the meso
                   layer to a region, translating cut sets between index spaces, and the
                   alignment policy
    loop.jl        `_two_tier_certify_scenario` -- the outer/inner loop that drives them
    round.jl       the round wiring: the per-scenario pass and the `cg_certification_round`
                   body, both of which reuse `../round.jl` unchanged

The plumbing shared with the one-tier loop (encoding parameters, the scenario column pool,
the exact station search, guide selection) lives in `../common.jl`; the result types both
modes return are in `../results.jl`.
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
propose nodes that merely escape one (`../../../cuts.jl`) and the graph it walks grows with
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
