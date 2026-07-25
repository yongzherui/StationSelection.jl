# AggregateODRoute LP/IP gap: structural vs. pool-completeness, resolved and cross-validated

*2026-07-25*

## Scope and conclusion

This note closes out the open LP/IP-gap investigation tracked across
`2026-07-23_benders_reports_optimal_with_unclosed_outer_gap.md`,
`2026-07-24_benders_optimal_with_unclosed_gap_grid_reproducer.md`, and
`2026-07-24_exceptional_fixed_assignment_lp_ip_gap_cases.md` (memory:
`project_benders_false_optimal_lp_ip_gap.md`), plus the separate BendersY/BendersYZ correctness
investigation (memory: `project_bendersy_vs_bendersyz_yz_pool_gap.md`).

**Bottom line**: on a fixed station selection + nearest-open assignment (no Benders outer loop
involved), the gap between the LP relaxation and the true integer optimum of the resulting
`RouteCoveringProblem` is **usually mostly genuine and structural** -- a real weakness of the LP
relaxation for this set-covering-style problem, confirmed by full route enumeration and NOT closed
by giving column generation a more complete pool. But there is *also* a smaller, separate,
independently real defect in how `_solve_fixed_route_covering_by_cg`'s final IP solve is computed
(a "price-and-branch" pool-completeness gap), which stacks on top of the structural gap rather than
competing with it as an alternative explanation. Both are now quantified, both are confirmed via a
completely independent computational path (full enumeration, not just trusting column generation's
own convergence claims), and a related bug in `DirectSolver`'s own reporting was found and fixed
along the way.

Additionally: relaxing `max_stops` (the route-length cap) eliminates the structural gap in most,
but not all, cases tested -- much of what looked like an irreducible LP weakness turns out to be an
artifact of forcing route fragmentation under a tight stop cap.

## Three questions, three mechanisms, five test cases

Five fixed-assignment cases were used throughout (station selection + nearest-open OD assignment
already fixed, isolating the `RouteCoveringProblem` from any Benders/outer-loop concern):

| case | source | open stations |
|---|---|---|
| **sample09 n=15** | BendersYZ ramp's winning `y_hat` on the real `sample_09` fixture | `[11, 100, 101, 106, 108, 129, 158, 196]` (station IDs) |
| **grid_terminating** | `grid_n10_p16_s123`'s BendersY-terminating (non-incumbent) `y_hat` | array indices `[1, 2, 3, 5, 6]` |
| **grid_incumbent** | same instance's true global optimum (sanity check) | array indices `[6, 7, 8, 9, 10]` |
| **Z10-13** | saved dump record 13, `zhuzhou_n10_p32_s42` | station indices `[3, 4, 5, 7, 10]` |
| **Z10-14** | saved dump record 14, same instance | station indices `[1, 3, 4, 7, 10]` |

Three independent questions were asked about these cases, each answered with its own recipe:

**Q1 -- Is the LP/IP gap a genuine LP weakness, or is CG's own pool just incomplete?** Answered by
solving the SAME fixed assignment two ways: `ColumnGenerationSolver` (CG's pricing-based route
pool, `cg_stop_reason == :optimality_proven` required) vs. `DirectSolver` (full route enumeration,
audited exhaustive in `2026-07-23_directsolve_route_enumeration_audit.md`). Both go through
`_solve_fixed_route_covering_by_cg` (`benders/covering.jl`), so the comparison is apples-to-apples.

**Q2 -- Does relaxing `max_stops` close the gap?** Answered by re-running CG at `max_stops=4`
(baseline) vs. `max_stops=nothing` (uncapped) on the same 5 cases, comparing CG's own LP-vs-IP gap
at each setting.

**Q3 -- Is `DirectSolver`'s own reported LP value trustworthy?** Discovered mid-investigation that
it wasn't (see "Code fix" below) -- fixed, then re-verified against CG's LP bound as an independent
cross-check.

## Q1 result: pool-completeness gap is real but small; the dominant gap is structural

| case | LP bound | CG final IP (price-and-branch) | **True optimum (DirectSolver)** | genuine gap (LP→True) | pool-completeness gap (True→CG IP) |
|---|---:|---:|---:|---:|---:|
| sample09 n=15 | 2169.66 | 2218.98 | **2198.98** | 29.31 (1.33%) | 20.00 (0.91%) |
| grid_terminating | 377.5 | 472.5 | **472.5** | 95.0 (20.11%) | 0.0 |
| grid_incumbent | 462.6 | 462.6 | **462.6** | 0.0 | 0.0 |
| Z10-13 | 89677.72 | 98904.07 | **98504.07** | 8826.35 (8.96%) | 400.00 (0.41%) |
| Z10-14 | 104082.68 | 114010.33 | **114008.89** | 9926.21 (8.71%) | 1.44 (~0.001%) |

**Mechanism, confirmed not speculated**: `_solve_fixed_route_covering_by_cg`
(`benders/covering.jl`) is a *price-and-branch* pattern, not branch-and-price. CG's pricing proves
the restricted master's LP relaxation is exact (no column anywhere has reduced cost
`< -reduced_cost_tol`, default `1e-6`), then hands the frozen pool to ONE final MIP solve
(`column_generation.jl:453-471`) with no further pricing inside that solve's own branch-and-bound.
LP-exhaustion certifies the LP bound only -- it says nothing about whether a column with reduced
cost in `(-1e-6, 0]`, correctly never generated for the LP, is exactly what the true integer
optimum needed. This is the textbook price-and-branch heuristic gap.

**Materiality (user-set ~2-3% threshold)**: the pool-completeness component never exceeds 1% in any
case tested (0.91%, 0%, 0%, 0.41%, 0.001%) -- not worth prioritizing a fix for on its own. The
genuine/structural component exceeds the threshold in 3 of 5 cases and is the one that actually
matters (up to 20.11%).

## Q2 result: uncapped `max_stops` eliminates the structural gap in 4 of 5 cases

| case | ms4 gap (CG's own LP vs IP) | uncapped gap |
|---|---:|---:|
| sample09 n=15 | 2.17% | ~0% (machine precision) |
| grid_terminating | 20.11% | **0.0%** |
| grid_incumbent | 0.0% | 0.0% (already exact) |
| Z10-13 | 9.33% | **3.51%** (shrunk ~62%, not fully closed) |
| Z10-14 | 8.71% | **0.0%** |

**Reinterpretation**: `max_stops=4` forces a request needing to touch many stops to be served by
several overlapping shorter routes instead of one long one -- exactly the "broad overlapping
columns" LP-blending mechanism `2026-07-24_exceptional_fixed_assignment_lp_ip_gap_cases.md`
describes. Removing the cap collapses most of this. Where `LP == IP` exactly (4/5 cases), that's a
complete, self-contained proof of global optimality for that fixed assignment (a valid LP lower
bound matched by an achieved integer solution certifies optimality, full stop) -- no enumeration
needed for those cases.

**Z10-13 follow-up (`check_z10_13_route_lengths_and_enumeration.jl`, sbatch job 18842620)**: the
one case that stayed open even uncapped. Inspected what "uncapped" routes CG actually used:
`n_relevant_nodes=5` (== the 5 open stations), generated-column route lengths min=0/max=6/mean=4.48,
selected final-IP routes all length 4-5 with no repeated station. Since routing costs are direct
all-pairs costs with no through-node savings, a route can never benefit from revisiting a station or
exceeding the station count -- so `max_stops=5, max_visits_per_node=1` is a faithful,
non-restrictive stand-in for "truly uncapped" (a LITERAL `max_stops=typemax(Int)` +
`max_visits_per_node=typemax(Int)` enumeration call is not even possible --
`_resolve_aggregate_od_route_max_stops`, `pricing/data.jl:41-59`, throws `ArgumentError` if both
are unbounded). With that bound, `DirectSolver` enumeration completed in 1.3s and matched uncapped
CG's final IP **exactly** (81078.7904 = 81078.7904) -- not the LP bound (78236.18). **Z10-13's
residual 3.51% gap is confirmed genuinely structural, not fixable by any column-generation or
stop-limit lever.**

## Q3: `DirectSolver`'s own `lp_bound` was fake -- found and fixed

While cross-checking CG's LP bound against an independent source, found that `DirectSolver`'s
branch of `_solve_fixed_route_covering_by_cg` (`benders/covering.jl`) returned:

```julia
lp_bound = final_result.objective_value isa Number ? Float64(final_result.objective_value) : NaN
```

i.e. `lp_bound` was **aliased to the IP objective**, not a real LP relaxation -- a placeholder that
existed only for interface compatibility with `ColumnGenerationSolver`'s branch of the same return
type. Nothing in the existing codebase read this field expecting a genuine relaxation, so it was
latent rather than actively wrong, but it would have silently misled anyone who did.

**Fix** (`_run_direct_enumerated_aggregate_od_route`, `benders/covering.jl`): after the IP solve on
the enumerated route set, always (no opt-in flag -- an earlier version added
`DirectSolver(compute_lp_relaxation=true)`; removed per explicit feedback that a flag was
unnecessary complexity) builds a second copy of the SAME enumerated route set with
`relax_integrality=true` and solves it as a genuine continuous LP, storing the result in
`final_result.metadata["lp_relaxation_objective"]`.

**One bug surfaced by the regression suite while fixing this**: the LP solve must NOT go through
`_run_opt_impl` (used for the IP solve in the same function) because that helper unconditionally
calls `assert_endpoint_chain_near_binary` on any `MOI.OPTIMAL` solve -- correct for the IP solve (z
must resolve near-binary there) but wrong for a genuine LP relaxation, where z is legitimately
fractional. Fixed by building+solving the LP relaxation directly (`build_model` + `optimize!`,
bypassing `_run_opt_impl`) instead. `test/runtests.jl`'s "Model Integration" testset (760 tests,
everything touching `DirectSolver`/aggregate_od_route) is green after this fix.

**Result: DirectSolver's independently-computed LP relaxation matches CG's pricing-based LP bound
to floating-point precision in every case** (`~1e-11` to `~1e-13` differences, pure solver noise):

| case | CG LP (pricing-based) | Direct LP (enumeration-based) |
|---|---:|---:|
| sample09 n=15 | 2169.664 | 2169.664 |
| grid_terminating | 377.5 | 377.5 |
| grid_incumbent | 462.6 | 462.6 |
| Z10-13 | 89677.72371 | 89677.72371 |
| Z10-14 | 104082.67882 | 104082.67882 |

This retroactively upgrades every "genuine structural gap" claim above from "trusted because CG's
own pricing-exhaustion self-report says so" to "independently confirmed by a route-enumeration-based
LP relaxation that shares zero machinery with CG's pricing/duals."

## How to apply going forward

- **Cite `DirectSolver`'s LP relaxation as the authoritative LP value** for any future
  structural-gap question on this problem class (`_solve_fixed_route_covering_by_cg`'s returned
  `.lp_bound` when `inner_solver isa DirectSolver`, or `result.metadata["lp_relaxation_objective"]`
  from a raw `run_opt(..., DirectSolver(...))` call) -- it no longer depends on trusting CG's
  pricing exhaustion at all.
- **The pool-completeness bug (sub-1% everywhere tested) is low priority** -- don't chase a fix for
  it on correctness-impact grounds alone; track it as known-and-reproduced.
- **The structural gap (up to 20%) is the one that matters when it's large**, and it is NOT a bug
  to fix inside `_solve_fixed_route_covering_by_cg` -- check `max_stops` first (it closes 4/5 cases
  outright), and for whatever's left (Z10-13-like residuals), the only remaining lever is the outer
  Benders loop's stopping rule/cut strength (the original open "Bug 3" in
  `project_benders_false_optimal_lp_ip_gap.md`), not the CG/pool machinery.
- **No fix has been applied to the pool-completeness gap or to Bug 3** -- only the `DirectSolver`
  `lp_bound` bug was actually fixed this session (`benders/covering.jl`, uncommitted as of this
  writing -- `git status` will show it modified; commit only if/when asked).

## Reproducibility

All diagnostics were run via `sbatch`, never interactively (shared HPC system --
see memory `feedback_run_julia_tests_via_sbatch.md`). Scripts and their sbatch wrappers, all in
`/home/yongzr/tmp-claude-sample09-smoke/`:

- `check_fixed_assignment_gap.jl` / `sbatch_check_fixed_assignment_gap.sh` -- sample09 n=15 case.
- `check_g10_grid_lp_ip_gap.jl` / `sbatch_check_g10_grid_lp_ip_gap.sh` -- grid_terminating +
  grid_incumbent.
- `check_zhuzhou_lp_ip_gap.jl` / `sbatch_check_zhuzhou_lp_ip_gap.sh` -- Z10-13 + Z10-14.
- `check_uncapped_route_length_gap.jl` / `sbatch_check_uncapped_route_length_gap.sh` -- all 5
  cases, CG only, `max_stops=4` vs. uncapped.
- `check_z10_13_route_lengths_and_enumeration.jl` /
  `sbatch_check_z10_13_route_lengths_and_enumeration.sh` -- Z10-13's uncapped follow-up.
- `run_aggregate_od_route_tests.jl` / `sbatch_run_aggregate_od_route_tests.sh` -- regression check
  (test/runtests.jl's "Model Integration" testset) after the `DirectSolver` `lp_bound` fix.

All three fixed-assignment scripts (`check_fixed_assignment_gap.jl`, `check_g10_grid_lp_ip_gap.jl`,
`check_zhuzhou_lp_ip_gap.jl`) were updated after the `DirectSolver` fix to also print
`direct_result.lp_bound` alongside `cg_result.lp_bound`, so re-running any of them now reproduces
both the pool-completeness comparison (Q1) and the CG-vs-Direct LP cross-check (Q3) in one pass.
