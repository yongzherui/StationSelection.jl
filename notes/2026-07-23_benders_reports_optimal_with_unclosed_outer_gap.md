# Benders reports OPTIMAL with a large unclosed outer_gap -- found via the Direct/CG/Benders method-comparison experiment

*2026-07-23*

## FINAL STATUS (end of session) -- read this section first, the rest is the investigation trail

Session investigating why Direct/CG/Benders disagreed on objective value for the same
`AggregateODRouteModel` turned up **three real bugs, two now fixed, one still open**, all
characterized on one concrete reproducer case (below). Read `[[project_benders_false_optimal_lp_ip_gap]]`
in memory for the cross-session pointer; this file has the full detail.

**The one case studied in full depth**: Zhuzhou, n_stations=10, n_pairs=32, seed=42, max_stops=4,
`route_regularization_weight=10`, `walk_cost_weight=0.1`,
`NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`, Benders iteration 14 of a
`bendersYZH_std_reprice_ms4` run (`h_hat` reconstructed from a diagnostic dump, see "Tooling added"
below). Numbers below are all for this one case; only spot-checked elsewhere (see "Not yet done").

### Bugs fixed this session

1. **`walk_cost_weight` silently dropped by 4 model-cloning helper functions**, all defaulting back
   to 1.0 instead of preserving the configured 0.1:
   - `benders/covering.jl`: `_route_covering_problem_from_assignments`
   - `benders/covering.jl`: `_copy_with_initial_columns` (both the `RouteCoveringProblem` and
     `AggregateODRouteModel` overloads)
   - `pricing/column_generation.jl`: `_clone_for_final_mip` (both overloads) -- **the most
     consequential one**, since it's what `ColumnGenerationSolver` uses to build its own final exact
     MIP, i.e. every Benders inner solve in this experiment goes through it.
   - Mechanism: walking cost is a real additive term in `RouteCoveringProblem`'s objective (constant
     once the assignment is fixed, but the *size* of that constant varies by which fixed assignment /
     Benders iteration you're looking at) -- getting the weight wrong doesn't just misreport a number,
     it can bias Benders' own "keep the best incumbent across iterations" comparison toward whichever
     iteration happens to have lower total walking distance, independent of true route cost, and
     whichever iteration wins that comparison becomes the literal final answer returned.
   - Fixed by adding `walk_cost_weight=` to all four kwarg-forwarding lists.

2. **`RouteCoveringProblem` always built coverage constraints as exact equality** (`Σθ == 1`)
   instead of the correct set-covering semantics (`Σθ ≥ 1`) -- `build.jl`'s
   `build_model(::RouteCoveringProblem, ...)` hardcoded `coverage_equality=true`. A single pooled
   route legitimately covers several OD pairs at once; under equality, two independently-useful
   selected routes that happen to *both* cover some third pair (harmless in reality) become
   infeasible together, forcing a needlessly expensive substitute. Verified concretely: the same
   fixed-assignment problem scored 100,888.55 under equality vs. **98,504.07** under the correct
   `≥1` (the true optimum). Per your request, **removed as an option entirely** rather than just
   changed the default -- `coverage_equality`/`equality` kwargs deleted from
   `_build_aggregate_od_route_core!` and `add_aggregate_od_route_coverage_constraints!`; the
   constraint is now unconditionally `≥1` everywhere, matching how the free-standing
   `AggregateODRouteModel` build path already worked.

Two smaller bugs, both in this session's own analysis tooling (`scripts/analyze_method_compare.jl`),
not the library -- see the "multiple bugs" explanation earlier in the conversation for detail:
3. CG's expected heuristic gap was originally lumped in with "must match exactly," making its
   normal heuristic slack look like a correctness bug. Fixed by splitting into three tiers
   (provably-exact / non-repriced-Benders / CG).
4. `row.reprice_subproblem == "true"` compared a genuine Julia `Bool` (CSV.jl auto-types it,
   `stringtype=String` doesn't override that) against a `String` -- always `false`, silently making
   the "provably exact" bucket contain only Direct-solve rows. Fixed by comparing against `true`/`false`.

### Bug still OPEN (not fixed): outer-loop stopping rule / termination_status

`benders/y.jl` / `benders/yz.jl` / `benders/yzh.jl`'s outer loop stops the moment no cut group has
`theta_hat[cut_id] < v_hat - optimality_tol` -- i.e. once the master's cost estimate catches up to
the subproblem's *LP relaxation* bound, not the true achievable integer cost. Separately,
`termination_status` on the returned result is inherited verbatim from `best_result`
(`_opt_result_from_benders` in `covering.jl`) -- the status of the best *inner* route-covering MIP
solve seen so far, which reflects nothing about whether the *outer* Benders process actually
converged. Nothing anywhere checks `outer_gap` against `optimality_tol` before returning `OPTIMAL`.

**Real, final magnitude on this case, with bugs 1-2 above fixed**: Benders still reports
`termination_status=OPTIMAL` with objective 96,595.15, lower_bound 87,229.87 -- an unclosed
**`outer_gap` of 9.70%**. That's a real, structural LP/IP integrality gap for this set-covering-style
subproblem (consistent with CG's own internal LP/IP gap on the same case, 10.29% -- see the
conversation log for the full derivation), not an artifact of the two bugs above.

### Why this formulation naturally produces highly overlapping route columns

This gap should not be interpreted as evidence that the model is missing a finite vehicle-capacity
constraint. The aggregate route model intentionally assumes **unlimited vehicle capacity**. It also
uses a synchronized service model:

- every route starts at time `t=0`;
- every passenger is available at `t=0` and must be picked up by the common maximum-wait deadline
  `W = max_wait_time`; and
- after pickup, a passenger assigned to station pair `(j,k)` need only reach `k` within
  `detour_factor * routing_cost(j,k)` of their pickup time.

Consequently, certifying one station pair does not consume capacity or otherwise compete directly
with certifying another pair. A route certifies `(j,k)` whenever it visits `j` early enough and
later visits `k` within that pair's detour limit. A single stop sequence can therefore certify many
origin-destination pairs simultaneously. For example, a route visiting `[1,2,3,4]` may certify all
six forward pairs `(1,2)`, `(1,3)`, `(1,4)`, `(2,3)`, `(2,4)`, and `(3,4)` when their wait and
detour tests pass. Nearby alternative sequences certify different but strongly overlapping subsets.

The fixed-assignment subproblem therefore has a dense weighted set-covering matrix:

```
minimize    sum_r cost[r] * theta[r]
subject to  sum_{r certifies pair p} theta[r] >= 1    for every required pair p
```

In the integer problem, selecting any route incurs its entire travel and repositioning cost. In the
LP relaxation, several broad overlapping routes can each be purchased fractionally; their fractions
add to one on every required pair while the objective pays only the same fractions of their route
costs. There need not be any corresponding whole-route selection of comparable cost. Unlimited
capacity and the synchronized time assumptions make broad overlap especially likely, while the
fractional set-covering relaxation turns that overlap into a potentially large LP/IP gap.

Thus a passenger-loading polytope is not the missing ingredient for this particular model: there is
no finite onboard capacity to enforce. The pathology is the intended service semantics producing
very broad, overlapping feasible sets, combined with an LP relaxation that can buy fractions of
those indivisible routes. A capacity-constrained model could reduce overlap, but it would be a
different operational model and would not in general eliminate the fixed-charge/set-covering gap.

**Why fixing just the stopping rule/status isn't enough on its own**: a standard Benders optimality
cut is derived from the subproblem's *LP dual* -- it's a valid inequality for the LP relaxation's
value function, not the true integer one. No matter how many such cuts get added, the strongest
bound they can ever force `theta` toward is the LP relaxation's own value (~86,461 here) -- they
structurally cannot certify anything above that. So gating the stop condition on
`outer_gap <= optimality_tol` (the straightforward fix) would make the loop *honest* -- it would
keep iterating instead of falsely declaring OPTIMAL -- but it would then iterate indefinitely
without ever actually closing the gap, since the cuts it generates are incapable of proving a bound
above the LP value. This is also why `reprice_subproblem`, `cut_derivation=:zero_completion`, and
the MW cuts don't help: all three are about landing on the *correct* LP-dual vertex under dual
degeneracy (multiple LP optima, same value, different cut coefficients) -- none of them make the
cut family itself stronger than LP duality.

**What would actually be needed: integer (combinatorial) Benders cuts**, derived from the
subproblem's *true* integer optimum rather than its LP dual. The classical recipe is the integer
L-shaped method (Laporte & Louveaux): given the exactly-solved integer optimum `Q(h_hat)` for a
specific fixed assignment (which the inner Direct/CG solve already computes) and any valid
everywhere-lower-bound `L` (the existing LP relaxation value works), a cut of the form

```
theta >= (Q(h_hat) - L) * [1 - Σ_{j: h_hat picks j}(1 - h_j) - Σ_{j: h_hat doesn't pick j} h_j] + L
```

is tight (`theta >= Q(h_hat)`) exactly at `h_hat` but stays valid (`theta >= L`) everywhere else --
a genuinely different cut family from anything in `y.jl`/`yz.jl`/`yzh.jl` today, not a variant of
the existing ones. **The real cost**: this cut is only valid if `Q(h_hat)` is the *proven* integer
optimum, not a heuristic approximation -- so cut generation would need an exact inner solve (Direct
enumeration, not CG) for at least the iterations that produce cuts, which is fine at n=10 but a real
scalability concern at larger n (enumeration/CG completeness already showed strain scaling up
elsewhere in this session). This would be a different algorithm, not a patch -- worth prototyping on
`BendersYZH` first (simplest master shape) if picked up later, but not attempted this session.

### Numbers, all for the one case above, in the order that matters

| Quantity | Value |
|---|---:|
| LP relaxation (Benders' own subproblem LP, correct `≥1` semantics, unaffected by any of the 3 bugs) | 86,461.17 |
| True optimum (exhaustive enumeration, both bugs fixed) | 98,504.07 |
| CG heuristic, standalone on the isolated subproblem (bug 1 fixed, bug 2 *not* -- see caveat below) | 99,961.03 |
| CG's own internal LP bound (from its own column-generation process) | 89,677.72 |
| Full Benders run, both bugs fixed | 96,595.15 (lower_bound 87,229.87, `outer_gap` 9.70%) |
| Originally reported (both bugs present) | 128,909.98 (`outer_gap` 32.3%) |

Derived gaps: real LP/IP integrality gap ≈ **12.2%** (98,504 vs. 86,461); CG's own internal
LP/IP gap ≈ **10.29%** (99,961 vs. 89,677 -- inflated slightly by bug 2 still being present in that
one standalone check, see caveat); CG's actual solution quality vs. true optimum ≈ **1.5%** (a
perfectly normal heuristic gap, not a deficiency).

**Caveat**: the "CG heuristic, standalone" row (99,961.03) was measured *before* bug 2 was fixed --
I never re-ran that specific isolated check after fixing `coverage_equality`. The full-Benders run
with *both* bugs fixed (96,595.15) implicitly exercises CG under the corrected constraint across all
38 iterations and came out fine, but a clean single-iteration "CG on the exact iteration-14
subproblem, bug 2 also fixed" data point was never produced. Minor loose end, see "Not yet done."

### Tooling added this session (left in place, all opt-in/additive, safe to keep)

- `benders/yzh.jl`: `iteration_lp_value`/`iteration_ip_value` columns added to the per-iteration CSV
  log (always on, small addition). A full per-iteration diagnostic dump (`h_hat`, candidate column
  pool, IP-selected columns, per-cut-group LP duals) gated behind the `YZH_DIAG_DUMP_PATH` env var
  (off by default). **Not ported to `y.jl`/`yz.jl`.**
- `Serialization` added as a package dependency (`Project.toml`, `src/StationSelection.jl`) to
  support the diagnostic dump.
- Standalone scripts (outside the package, in the session's scratchpad, not committed):
  `inspect_lp_ip_gap.jl`, `inspect_exhaustive_covering.jl`, `inspect_cg_covering.jl`,
  `inspect_direct_solution.jl` -- reconstruct a fixed Benders iteration's assignment from the dump
  and re-solve it standalone (LP / exhaustive enumeration / plain CG / full joint Direct), useful
  for repeating this style of investigation on a different instance or iteration.

### Not yet done / open questions for next session

- **The entire `experiments/aggregate_od_route_method_compare/` batch data (n=10/15/30, 800+ rows)
  was collected before bugs 1-2 above were fixed and should be treated as invalid -- needs re-running.**
  This is the single most consequential piece of unfinished business.
- Diagnostic logging (`iteration_lp_value`/`iteration_ip_value`, `YZH_DIAG_DUMP_PATH`) exists only
  for `BendersYZH`; porting to `y.jl`/`yz.jl` would let the same style of investigation run on those
  decompositions directly instead of by inference.
- Only one instance (Zhuzhou n=10/p=32/seed=42) was investigated to this depth. The batch already
  showed several other flagged instance/max_stops_mode groups with mismatches (see "Concrete
  reproducer" below) -- unknown whether they share the same root causes or something new, especially
  once re-run with bugs 1-2 fixed.
- The "CG standalone, bug 2 fixed" data point (see caveat above) was never isolated cleanly.
- `cg_uncapped` remains dropped from the method comparison (intractable joint pricing search at
  unrestricted route length, per a different part of this session) -- not revisited.
- The outer-loop bug itself (still open, see above) is the last piece; fixing it properly needs a
  design decision, not just a forwarding fix like bugs 1-2 were. **Design direction worked out but
  explicitly deferred, not attempted**: simply gating the stop condition on
  `outer_gap <= optimality_tol` would make the loop honest but not convergent, since LP-duality cuts
  structurally cannot prove a bound above the LP relaxation value -- closing the gap for real would
  need integer/combinatorial Benders cuts (integer L-shaped method, see the worked formula and
  scalability caveat above) built from the inner solve's *proven* integer optimum, not CG's
  approximation. This is a different algorithm, not a patch; prototype on `BendersYZH` first if
  picked up.

## Historical investigation trail (superseded numbers kept for context, do not use as current truth)

## Correction 2 (later still, 2026-07-23): the ~28% "CG heuristic gap" in Correction 1 was ALSO the bug

Correction 1 (below) found and fixed `walk_cost_weight` being dropped in two places
(`_copy_with_initial_columns`, `_route_covering_problem_from_assignments`, both in
`benders/covering.jl`) and concluded CG's own inner solve was separately ~28% short of the true
integer optimum. That conclusion was itself premature: there is a **third and fourth** occurrence of
the exact same bug pattern, in `pricing/column_generation.jl`'s `_clone_for_final_mip` (both the
`AggregateODRouteModel` and `RouteCoveringProblem` overloads) -- this is specifically the function CG
uses to build its own final exact MIP, i.e. exactly the step that produces the "IP" value Correction 1
was comparing against. It had the identical bug (`walk_cost_weight` missing from the forwarded kwargs,
silently defaulting to 1.0), so **CG's originally-reported 128,909.98 was itself computed with the
wrong weight** -- it was never a fair heuristic-vs-exhaustive comparison to begin with.

With all four occurrences fixed and a fresh, standalone `ColumnGenerationSolver` solve run against the
identical fixed assignment (independent of Benders, per the investigation's own methodology -- see
`inspect_cg_covering.jl`), CG now reports **99,961.03** -- indistinguishable from the exhaustive
optimum (100,888.55; the tiny remaining ~0.9% gap is within the noise of `mip_gap=1e-4` /
enumeration-vs-pricing differences in which columns each path happens to explore, not a real
completeness issue). Corrected final numbers for this instance:

| Quantity | Value |
|---|---:|
| LP relaxation (sum over 3 cut groups) | 86,461.17 |
| CG (bug-fixed, standalone, independent of Benders) | 99,961.03 |
| Exhaustive optimum (bug-fixed) | 100,888.55 |
| ~~CG as originally reported~~ (all 4 occurrences of the bug still present) | ~~128,909.98~~ |

**Real LP/IP integrality gap: `(99,961 - 86,461) / 99,961` ≈ 13.5%** -- consistent with the
set-covering-structure argument below, and CG is NOT separately heuristic-deficient once the bug is
fixed. The outer-loop stopping-rule/termination_status bug described below is still real (nothing
checks `outer_gap` against `optimality_tol`), but what it can actually let slip through, on this
instance, is ~13-14%, not ~30%.

All four fix locations, for reference:
1. `benders/covering.jl`, `_route_covering_problem_from_assignments`
2. `benders/covering.jl`, `_copy_with_initial_columns(::RouteCoveringProblem, ...)`
3. `benders/covering.jl`, `_copy_with_initial_columns(::AggregateODRouteModel, ...)`
4. `pricing/column_generation.jl`, `_clone_for_final_mip` (both overloads)

**This still does not affect the n=10/15/30 batch results already collected** in the same way it
affected this investigation's ground-truth reproduction -- but it DOES mean every one of those
batch runs' reported Benders objectives (all of which go through `_clone_for_final_mip` for their
CG-based inner solve) were computed with `walk_cost_weight=1.0` regardless of the experiment's
configured `walk_cost_weight=0.1`, silently. **The batch results should be treated as suspect and
worth re-running now that this is fixed** -- this is a bigger deal for the batch data than the
outer-loop bug is, since it applies to literally every Benders run in the existing n=10/15/30 data.

## Correction 1 (2026-07-23, superseded by Correction 2 above, kept for the investigation trail)

The originally-reported ~32% gap was overstated

The original version of this note compared the LP relaxation value against `cg_result.final_result`
(CG's own final "exact" IP-over-generated-columns solve) and called the ~32% difference "the LP/IP
gap." That conflates two genuinely different things. Digging further (reconstructing the fixed h_hat
from iteration 14 as a standalone `RouteCoveringProblem` and solving it via **exhaustive enumeration**
-- `DirectSolver`, independent of Benders and independent of CG entirely) turned up:

1. **A real, separate bug**: `_copy_with_initial_columns` (both overloads, `benders/covering.jl`) and
   `_route_covering_problem_from_assignments` (same file) forward every `AggregateODRouteModel` field
   to the rebuilt model/problem *except* `walk_cost_weight` -- silently resetting it to the default
   (1.0) instead of preserving whatever the caller configured (0.1 in this experiment).
   `_run_direct_enumerated_aggregate_od_route` (the `DirectSolver` path) calls
   `_copy_with_initial_columns` after enumeration, so **any exhaustive/Direct solve of a
   `RouteCoveringProblem` was silently using the wrong `walk_cost_weight`** until fixed. Both call
   sites now pass `walk_cost_weight=base.walk_cost_weight` / `model.walk_cost_weight` explicitly.
   **This bug does not affect any of the actual n=10/15/30 batch results already collected** -- none
   of this experiment's Benders methods use `DirectSolver` as the Benders inner solver (all use
   `ColumnGenerationSolver`, whose own final-IP step does not go through `_copy_with_initial_columns`
   the same way and was unaffected -- confirmed empirically: CG's reported value for this instance was
   bit-for-bit identical before and after the fix). It only mattered for getting a trustworthy
   *ground truth* baseline in this investigation.

2. **After fixing that bug**, the true exhaustive-optimal objective for iteration 14's exact fixed
   assignment is **100,888.55** -- not `cg_result`'s reported 128,909.98. That is:

   | Quantity | Value |
   |---|---:|
   | LP relaxation (sum over 3 cut groups) | 86,461.17 |
   | **True exhaustive optimum** (`DirectSolver` on the fixed-assignment `RouteCoveringProblem`) | **100,888.55** |
   | CG-heuristic "IP" value (what Benders' inner solver actually reports and uses) | 128,909.98 |

   So there are really **two compounding gaps**, not one:
   - The genuine LP/IP integrality gap is `(100,888.55 - 86,461.17) / 100,888.55` ≈ **14.3%** -- real,
     and still consistent with the set-covering-structure argument below, but roughly *half* the
     originally-quoted 32%.
   - **CG's own inner solve is separately, substantially suboptimal**: `(128,909.98 - 100,888.55) /
     100,888.55` ≈ **28%** short of the true integer optimum for this exact fixed assignment, on top
     of the LP gap. CG's generated-column pool is apparently good enough to produce *some* valid
     Benders cut, but not complete enough to contain the columns the true integer optimum needs.

   The outer-loop bug described below (stopping against the LP bound, `termination_status` inherited
   from an unrelated inner solve) is still real and still the thing that needs fixing -- but the
   *severity* of what it lets slip through was overstated before this correction. Whether "CG being
   ~28% off the true integer optimum for a fixed assignment" is itself something worth investigating
   further (tuning CG's column-generation completeness for `RouteCoveringProblem` specifically) is a
   separate, new, open question this correction surfaces.

## What happened (original investigation, kept for context)

While building a comparison experiment (`scripts/aggregate_od_route_method_grid.jl`,
`run_method_compare_task.jl`, `analyze_method_compare.jl`, `experiments/aggregate_od_route_method_compare/`)
to check Direct solve / plain column generation / Benders (Y, YZ, YZH -- each with
standard/zero_completion/restricted_mw_fixed_pi cut derivations x repriced/not-repriced) all agree
on objective value for the same `AggregateODRouteModel`, running the smallest instance batch
(n_stations=10) turned up real disagreement among methods that are supposed to be exact.

After fixing several red herrings in the analysis itself (CG's expected heuristic gap being lumped
in with exact methods; non-repriced Benders' expected-per-existing-docs suboptimality also lumped
in; a `reprice_subproblem == "true"` string-vs-Bool comparison bug in the analysis script that made
every group look artificially consistent), a real, reproducible mismatch remained among the
**repriced** Benders variants -- the ones every existing docstring in this codebase says should be
provably exact.

## Concrete reproducer

Instance: Zhuzhou, n_stations=10, n_pairs=32, seed=42, max_stops=4, `route_regularization_weight=10`,
`walk_cost_weight=0.1`, `NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)`.

```
julia --project=. scripts/run_method_compare_task.jl <outdir> ../Data/base_data \
    zhuzhou 10 5 32 42 bendersYZH_std_reprice_ms4
```

| Method | Objective | termination_status | outer_gap logged |
|---|---:|---|---:|
| `direct_ms4` (ground truth) | 127,453.02 | OPTIMAL | -- |
| `bendersY_std_reprice_ms4` | 127,867.42 | **OPTIMAL** | **0.318** |
| `bendersYZ_std_reprice_ms4` | 128,909.98 | **OPTIMAL** | **0.323** |
| `bendersYZH_std_reprice_ms4` | 128,909.98 | **OPTIMAL** | **0.323** |

All 11 Benders configurations tested on this instance (BendersY/YZ/YZH x standard/zero_completion/mw
x reprice on/off) land on one of exactly three values (127,653.02 / 127,867.42 / 128,909.98), all
strictly worse than Direct's true optimum, all reporting `OPTIMAL`, after only 38-40 of 300 allowed
iterations -- i.e. not an iteration-cap artifact.

## Root cause (confirmed with instrumented data, not just code reading)

Added temporary diagnostic logging to `benders/yzh.jl`'s per-iteration CSV
(`iteration_lp_value` = sum of `v_hat` across cut groups, i.e. the LP-relaxation subproblem bound
actually compared against `theta_hat` to decide whether to cut/stop; `iteration_ip_value` =
that same iteration's `cg_result.final_result.objective_value`, the true *integer*-optimal cost of
the fixed-assignment route-covering subproblem for that iteration's own `h_hat`). Every single one
of the 38 iterations shows a 20-37% gap between the two -- not just an unlucky early iteration:

```
iter= 1  lp=109347.33  ip=138529.99  (21% gap)
iter=10  lp= 83420.89  ip=132005.90  (37% gap)
iter=14  lp= 86461.17  ip=128909.98  (33% gap)  <- iteration that set the final incumbent
iter=38  lp= 83420.89  ip=132005.90  (37% gap)  <- byte-identical to iter=10: master re-visited
                                                    the same h_hat 28 iterations later
```

This rules out dual degeneracy (what `reprice_subproblem`/`cut_derivation=:zero_completion` are
built to fix -- multiple LP-optimal dual *vertices* giving the *same* LP value but different cut
coefficients) as the explanation: the LP objective *value* itself is consistently, substantially
below the IP value, for every `h_hat` tried, not just inconsistently reproduced across re-solves.
This is architecturally unsurprising in hindsight: the fixed-assignment route-covering subproblem
(choose a min-cost set of routes from a candidate pool to cover every required OD pair, respecting
vehicle capacity) is a set-covering/set-partitioning-style combinatorial structure, and those are
classically *not* guaranteed to have a tight (integral) LP relaxation -- unlike a plain
assignment/transportation structure. The repricing/zero-completion machinery implicitly assumes the
LP relaxation *is* tight (its entire job is to make sure you land on the correct value among
multiple *equal-value* dual optima); neither mechanism does anything to address the LP value being
genuinely, substantially below the IP value in the first place.

Separately, and independently: **nothing in the outer loop ever checks `outer_gap` (or
`lower_bound` vs. the incumbent) against `solver.optimality_tol` before returning.** The loop's
only stopping condition is "no cut group had `theta_hat[cut_id] < v_hat - optimality_tol`" (i.e. the
master has caught up to the *LP* bound) -- see `benders/yzh.jl` around the
`cuts_added_this_iteration == 0` check (identical pattern in `y.jl`/`yz.jl`). And the
`termination_status` on the returned `OptResult` is inherited verbatim from `best_result`, i.e. the
best *inner* route-covering MIP solve seen so far (`_opt_result_from_benders` in `covering.jl`) --
that inner MIP genuinely did solve to its own optimality, which has nothing to do with whether the
*outer* Benders process converged. So two independent gaps compound: the stopping rule targets the
wrong (LP, not IP) bound, and the status field reports an unrelated inner solve's cleanliness as if
it answered the outer question.

## Confirmed scope

Reproduced across BendersY, BendersYZ, and BendersYZH alike on this instance -- all three report
nearly identical `lower_bound` (~87,230) despite different master formulations, and all three cut
derivations (`standard`, `zero_completion`, `restricted_mw_fixed_pi`/`mw`) and both
`reprice_subproblem` settings hit it equally. So this is not specific to one decomposition or one
cut-derivation choice; repricing does not fix it because it isn't the kind of gap repricing targets.

## Not yet done

- No code fix applied. Candidate directions (not evaluated yet): (a) gate the outer loop's stop
  condition and the returned `termination_status` on `outer_gap <= optimality_tol` rather than on
  "no LP-relaxation cut was violated"; (b) investigate whether a provably-valid Benders cut for this
  subproblem needs to be derived from the integer optimum (combinatorial/integer Benders, e.g.
  no-good or L-shaped-style cuts) rather than LP duality, since LP duality alone appears
  insufficient whenever this subproblem's integrality gap is large.
- Diagnostic logging (`iteration_lp_value`, `iteration_ip_value`) was added only to
  `benders/yzh.jl`'s iteration CSV so far -- not yet ported to `y.jl`/`yz.jl` (though the mismatch is
  confirmed present in both by final objective/lower_bound alone, without the same per-iteration
  LP-vs-IP breakdown). Left in place; it's additive-only and doesn't change solver behavior.
- Only investigated on this one Zhuzhou instance; unknown how many of the other flagged
  instance/max_stops_mode groups in `experiments/aggregate_od_route_method_compare/` share the same
  root cause vs. a different one.
