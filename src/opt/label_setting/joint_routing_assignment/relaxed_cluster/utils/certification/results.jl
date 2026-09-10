"""
Result types shared by both certification modes (`:relaxed_cluster` and
`:relaxed_cluster_two_tier`), plus the one mutation both perform on a cut pool.

Kept apart from either loop because the round driver (`round.jl`), the solver
(`../../../../../solvers/cg/`) and the benchmark analyses all read these shapes without
caring which loop produced them -- which is exactly the property that lets the two modes
share a round driver and a stats record.
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
- `inconclusive_scenarios` -- the scenarios that came back `:inconclusive`, i.e.
  the ones whose searches ran out of budget rather than reaching a verdict.
  These are exactly the scenarios an escalation can still rescue, and naming
  them is what lets `CGSolver` escalate PER SCENARIO instead of re-running the
  whole round. Without it a round is only escalatable as a unit, so a single
  productive scenario masks a permanently stuck one -- MEASURED at n=40 seed 47,
  where scenario 2 refuted in all 38 iterations, the round therefore always had
  columns to show, and scenario 1 replayed the identical 262 s inconclusive
  search 24 times without ever reaching the escalated tier.
- `scenarios_run` -- which scenarios this round actually priced. Equal to every
  scenario for a full round; a subset for an escalation restricted via
  `only_scenarios`, whose `certified` then means "all the scenarios I ran
  certified" and must be combined with the ordinary round's verdict for the rest.
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
    inconclusive_scenarios::Vector{Int}
    scenarios_run::Vector{Int}
    # Parallel to `inconclusive_scenarios`: why each one came back inconclusive. See
    # `RelaxedClusterNoGoodResult.reason` for the values and why this is worth carrying.
    inconclusive_reasons::Vector{Symbol}
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
    # WHICH exit produced an `:inconclusive`, because the loop has several and they want
    # OPPOSITE fixes -- a finer partition versus more time -- while being externally
    # indistinguishable. An n=40 attempt reported "inconclusive after 31.8s" against a
    # nominal 1800s budget, and the elapsed time alone pointed at the wrong cause twice.
    #   :deadline            -- the round's own slice was already spent
    #   :relaxed_not_exhausted -- the relaxed guide search ran out of its half-slice
    #   :subset_no_time      -- no time left to start the real subset search
    #   :subset_not_exhausted -- the subset search was truncated, so barrenness is unproven
    #   :cut_mask_full       -- all 64 cut bits used; no further cut can be recorded
    #   :round_cap           -- RELAXED_CLUSTER_MAX_CUT_ROUNDS rounds without a conclusion
    # `:none` on a conclusive outcome.
    reason::Symbol
end

# Keeps the pre-`reason` construction sites (`two_tier/loop.jl`) working unchanged; only
# the exits that can actually BE inconclusive need to name a reason.
RelaxedClusterNoGoodResult(outcome, rounds, cuts_added, last_subset_size, trace, candidates) =
    RelaxedClusterNoGoodResult(outcome, rounds, cuts_added, last_subset_size, trace,
                               candidates, :none)


"""
    _relaxed_cluster_add_cut!(cluster_sets, support; barren_supports=nothing) -> Bool

Insert a proven-barren support as a new cut, first reclaiming the mask bits of any cut the
new one subsumes. `false` means the cap was still reached, which the caller must report as
inconclusive rather than silently dropping the cut (see `RELAXED_CLUSTER_MAX_CUTS`).

# Cut management: which cuts are worth a mask bit

A cut says "every route must visit at least one cluster OUTSIDE this support". So when
`T_old` is a SUBSET of `T_new`, every cluster outside `T_new` is also outside `T_old`, and
any route satisfying `Cut(T_new)` satisfies `Cut(T_old)`:

    T_old ⊆ T_new   ⟹   Cut(T_new) ⟹ Cut(T_old)

`T_old` therefore excludes nothing once `T_new` is active, while still holding one of the 64
mask bits and doubling the `(current, satisfied)` state space the label search carries --
which weakens dominance in the cut-aware search. Dropping it is exact: no route is
re-admitted. MEASURED at n=15: 60% of cuts (515 of 861) were dominated this way, with 0
exact duplicates.

# The proof is NOT the cut, and must outlive it

`barren_supports` is the record of every support ever PROVED barren, and pruning never
touches it. A cut is a search restriction; a barren support is a theorem. A smaller support
that a larger cut subsumes for search purposes is still the premise the barren-support cache
reasons from (`_relaxed_cluster_support_barren_by_cache`), so discarding it with its cut
would throw away a proof to save a mask bit. Keep the two side by side.

Both structures were removed from this pathway as unnecessary while the measured load was
0.5-0.75 cuts per attempt and 11 active cuts at worst against a cap of 64. They are back
because that regime ended exactly where the README predicted it would: n=40 seed 42
exhausts all 64 bits and reports `:cut_mask_full`, identically at K=24, 30 and 34.
"""
function _relaxed_cluster_add_cut!(
    cluster_sets::Vector{Set{Int}}, support::Set{Int};
    barren_supports::Union{Nothing, Vector{Set{Int}}}=nothing,
)::Bool
    # Record the theorem first and unconditionally -- even when the cut itself is refused
    # for want of a mask bit, the proof stands and the cache can still use it.
    if !isnothing(barren_supports) && !any(t -> t == support, barren_supports)
        push!(barren_supports, copy(support))
    end
    # Reclaim, then check the cap: a new cut that subsumes several old ones can free more
    # bits than it consumes, so testing the cap first would refuse a cut that actually fits.
    filter!(existing -> !issubset(existing, support), cluster_sets)
    length(cluster_sets) < RELAXED_CLUSTER_MAX_CUTS || return false
    push!(cluster_sets, copy(support))
    return true
end


"""
    _relaxed_cluster_support_barren_by_cache(barren_supports, support, reward_carrying)
        -> Bool

`true` when `support` can be declared barren from an existing proof instead of an exact
search: some proved-barren `T ⊆ support` exists whose complement `support \\ T` holds no
reward-carrying cluster at the current duals.

# Why it is sound

Take any route `R` over `stations(support)` and delete its stops in `support \\ T`, giving
`R'` over `stations(T)`. Travel does not increase (the travel matrix is metric), every
arrival is therefore no later so no pickup window or ride limit becomes harder to meet, and
no reward is lost because a reward-free cluster anchors no candidate endpoint. Hence
`rc(R) >= rc(R') >= -tol`, the last step because `T` is barren. So `support` is barren too,
and step 4's exact search over it can be skipped entirely.

The inference runs in ONE direction only. Barren-ness is downward-closed, so a barren `T`
says nothing about a superset that adds *reward-carrying* cells -- which is exactly why the
complement has to be tested and not just its size.

Note the test is on the CANDIDATE SET, not on one route: the exact search ranges over every
route in `stations(support)`, so "the witnessing route gained nothing there" would be too
weak a premise.
"""
function _relaxed_cluster_support_barren_by_cache(
    barren_supports::Vector{Set{Int}}, support::Set{Int}, reward_carrying::Set{Int},
)::Bool
    for t in barren_supports
        length(t) < length(support) || continue      # equal sets are already cut
        issubset(t, support) || continue
        # Every extra cell must be reward-free at these duals.
        all(c -> !(c in reward_carrying), setdiff(support, t)) && return true
    end
    return false
end
