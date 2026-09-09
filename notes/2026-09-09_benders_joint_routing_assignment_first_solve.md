# Benders for the Joint routing+assignment formulation: first working solve (n=10)

2026-09-09. `AggregateODRouteJointRoutingAssignmentFormulation` + `BendersSolver` now
solves, verified exact against a monolithic reference. This note records the shape it took
and the two properties of its answer that have to travel with any number it produces.

## Shape: three formulation types, one block library

The decomposition is **not** a solver that builds two models by hand. It is a formulation
family (`src/opt/formulations/aggregate_od_route/joint_routing_assignment/`):

| Type | Carries |
| --- | --- |
| `…JointRoutingAssignmentFormulation` (monolith) | the whole model; `DirectMIPSolver`, `CGSolver` |
| `…JointRoutingAssignmentMasterFormulation` | `y` + cut placeholders `Θ`; `BendersSolver` |
| `…JointRoutingAssignmentBendersSubproblemFormulation` | one scenario's `x_walk`/`θ`, `y` fixed |

The split falls out of one observation: **`y` appears in neither the coverage rows nor the
objective.** It couples the model only through `pickup_link`/`dropoff_link`. So the first
stage is `y` plus the rows written only in `y` (station budget, endpoint feasibility) and
the second stage is literally everything else — and that second stage separates exactly by
scenario, since every column belongs to one scenario and every row is keyed `(s,p)`. The
two halves therefore cover the monolith's rows exactly once each, and each half's build is
a list of existing shared blocks with nothing re-derived.

Three enabling changes made that reuse real rather than aspirational:

1. **`scenarios` kwarg** on `add_joint_routing_assignment_{coverage,station_linking}_constraints!`,
   mirroring the one `add_walk_variables!` already carried (left over from the retired
   `BendersYX` subproblem, which was also per-scenario).
2. **`y` stays a variable in the subproblem**, pinned with `JuMP.fix(y[j], ŷ[j]; force=true)`.
   This is the choice that buys the most: the linking builder is reused *verbatim*, rows
   stay `θ - y[j] ≤ 0`, and there is no numeric-RHS second code path to drift from the
   first. Per-iteration update is `n` `fix` calls instead of a `set_normalized_rhs` sweep,
   and `reduced_cost(y[j])` becomes a free second route to the cut coefficients.
3. **`_stash_joint_routing_assignment_cost_parameters!`** — the cost weights and pool
   containers that `add_joint_routing_assignment_column!` reads off `m` rather than off a
   formulation argument. Fine with one build path; a live drift hazard with three.

Derived types are constructed **only** from a parent (`MasterFormulation(parent; cut_mode)`,
`BendersSubproblemFormulation(parent; max_stops)`), which copies the family's six shared
encoding fields. A master at one `detour_factor` against a subproblem at another produces
invalid cuts and a confidently wrong `OPTIMAL` with nothing raising anywhere, so that
combination is made unrepresentable rather than merely discouraged. `run_opt` still takes
one formulation — the monolith — because Benders is an algorithm for the same model.

## The cut, and why it is the simple one

With `y = ŷ`, `α ≥ 0` on the `≥` coverage rows and `γ ≥ 0` the negated duals of the `≤`
linking rows (`extract_joint_routing_assignment_duals`' own convention, reused):

    Θ_s ≥ Σ_p α[s,p] − Σ_j Γ[s,j] y_j,     Γ[s,j] = Σ_p (γ^O[(s,p),j] + γ^D[(s,p),j])

valid for every `y` because the dual feasible region does not mention `ŷ`. Two
simplifications are structural, not conveniences:

- **No feasibility cuts, ever.** `x_walk` covers every demand group with no `y` linking, so
  the subproblem is feasible at every incumbent. `solve_subproblem` raises on a
  non-optimal subproblem instead of deriving a weak cut, which is what would surface a
  future variant that dropped walk-only coverage.
- **No bound-dual term.** Neither `θ` nor `x_walk` has an upper bound in the relaxed build
  (`lower_bound = 0.0` only; the coverage rows are what hold them near 1). If anyone adds
  `θ ≤ 1`, this derivation needs a third term and silently under-cuts without it.

Every solve asserts `Σα − Σ Γ_j ŷ_j == objective` before its cut is built. One line, and it
is the thing that catches a flipped dual sign, a dropped linking family, or a subproblem
whose `mapping` disagrees with the master's — each of which otherwise converges to a wrong
number in silence.

Deliberately **not** used: the `:zero_completion` / `:restricted_mw_fixed_pi` cut
derivations on the retired pre-split markers. Those are what produced the invalid-cut bug
on route-covering LP degeneracy (`project_yz_completion_lp_invalid_cut`). Solving the true
subproblem avoids the class entirely.

## Two properties of the answer

**It is the MIXED optimum, not the direct-MIP one.** The cut IS the subproblem's LP dual,
so the subproblem must be an LP, so a converged run is optimal for `y` binary with
`θ`/`x_walk` continuous. Recorded as `metadata["benders_second_stage_relaxed"] = true`.
Getting the all-binary optimum needs integer L-shaped / combinatorial cuts — a different
project.

**Scope narrows with the enumeration cap.** The only oracle today is
`:direct_enumeration`; its pool is exponential in `max_stops` on two axes, so
`BendersSubproblemConfig.max_stops` defaults to 4 and narrows the formulation's own value
when larger. `metadata["benders_optimality_scope"]` then reads `"max_stops_restricted"`,
in the same spirit as `cg_optimality_scope`. Exercised by the `restricted_scope` arm below,
which relies on the master carrying no `max_stops` dependence to make the narrowing
testable without enumerating the wider universe.

## Measured

### Seed sweep (Zhuzhou n=10, p=8, 1 scenario, k=5, max_stops=4)

| seed | objective | iters | cuts | cols | loop wall | enum wall |
| --- | --- | --- | --- | --- | --- | --- |
| 42 | 12249.400939 | 4 | 3 | 16320 | 2.34 s | 3.54 s |
| 43 | 8758.928834 | 3 | 2 | 7500 | 2.19 s | 2.88 s |
| 45 | 8398.668920 | 2 | 1 | 1823 | 1.30 s | 2.02 s |

### Arm sweep (seed 42, `benders_joint_n10.jl`, 29/29 checks)

| arm | objective | iters | cuts | cols | scope |
| --- | --- | --- | --- | --- | --- |
| multicut_s1 | 12249.400939 | 4 | 3 | 16320 | full_route_universe |
| singlecut_s1 | 12249.400939 | 4 | 3 | 16320 | full_route_universe |
| restricted_scope | 12249.400939 | 4 | 3 | 16320 | **max_stops_restricted** |
| multicut_s3 | 25771.187908 | 4 | 5 | 21635 | full_route_universe |
| singlecut_s3 | 25771.187908 | 4 | 3 | 21635 | full_route_universe |

Two things the arm sweep established that the seed sweep could not:

**The cut dedup is load-bearing, not defensive.** `multicut_s3` added **5** cuts over 4
iterations with 3 scenarios; an un-deduplicated `MultiCut` would have added up to 12. So
the degenerate-dual repeat that `add_benders_cut!`'s return count exists to detect is a
real occurrence at the smallest interesting size, not a theoretical concern. Without the
count the loop still terminates here (the gap closes), but a case where it does not is
clearly reachable.

**A prediction that was wrong: `SingleCut` did NOT need more iterations.** The expectation
was that aggregating three scenarios' cut data into one row would cost iterations relative
to three separate rows. At s=3 both modes took 4 iterations to the same optimum
(`SingleCut` adding 3 cuts to `MultiCut`'s 5). n=10 with 1-5 total cuts puts neither mode
under any pressure, so this cell cannot separate them; the comparison needs a harder
instance before it means anything either way.

The `restricted_scope` row is the narrowing path: formulation `max_stops=6` with the
subproblem capped at 4 returns `multicut_s1`'s objective *exactly*, because the master
carries no `max_stops` dependence at all (its rows are the station budget and endpoint
feasibility, and the map keys off `max_walking_distance`). That identity is what makes the
path testable without enumerating `max_stops=6`, which is not tractable.

### Infeasibility reporting (k=1)

A proven-infeasible instance must come back as an ANSWER, not a thrown error. Both routes
to that verified at `k=1`, which is infeasible here because most Zhuzhou OD pairs exceed
`2 * max_walking_distance` and so carry no direct-walk fallback, giving them
endpoint-feasibility rows that one station cannot satisfy:

| arm | status | how it knew |
| --- | --- | --- |
| infeasible_gate (via `run_opt`) | INFEASIBLE | `infeasibility_reason` -- "no size-1 station selection can reach every demand group that lacks a direct-walk fallback" |
| infeasible_master (bypassing the gate) | INFEASIBLE | `benders_stop_reason = "master_INFEASIBLE"`, `LB = -Inf` |

Worth being precise about the second one: **it is unreachable through `run_opt`**, because
the gate solves exactly the master's own row set and therefore always refutes first. So the
infeasible-master branch is defensive, not on the normal path -- it exists because
benchmark scripts call `build_model`/`optimize_model` directly, bypassing the gate, which
is how this arm reaches it. Keeping it is still right (an infeasible master IS a proof
about the problem, since a Benders cut can only raise `Theta` and never removes a feasible
`y`), but it should not be described as the mechanism by which infeasibility is normally
reported.

### Suite

93,086/93,086 tests pass with the Benders work in (run 22409466, 2m23s). Credit to a
parallel session for pointing out what that does and does not cover: the full suite on a
clean tree reaches the CG path, the relaxed-cluster pricers, the clustering formulations
and the include graph -- and NOT n=40/50 scale, long-running certification, or two-tier
escalation under real budgets. Those are only exercised by the Study 9/10 arrays.

LB == UB exactly on every cell, `gap = 0.000e+00`, and `benders == mixed_mono` to
`diff 0.000e+00`. Master time is negligible (0.02–0.04 s total); the loop is
subproblem-bound, and the whole run is dominated by up-front enumeration — which is the
expected and the damning fact about this oracle (see below).

**The LP-IP gap is 0% on every cell measured**, s=1 and s=3 alike, so `benders ≤ direct_mip` passes trivially and
cannot distinguish the mixed optimum from the integral one at this size. The monolithic
*mixed* comparison is the check that actually establishes exactness. A cell with a real
gap (the hub-route effect shows 21.6% at n=40) would be the stronger test and has not been
run.

Iteration counts of 2–4 against C(10,5) = 252 candidate station sets say the cuts are
strong here, but n=10 is not evidence about scaling -- note s=3 also took 4 iterations,
i.e. tripling the second stage did not move the iteration count at all, which is far more
likely to mean the instance is easy than that the method is insensitive to `s`.

A cross-implementation check worth doing when Study 11's data lands: a `y`-fixed-binary /
`theta`-and-`x_walk`-continuous snapshot over a CG pool is the same mixed model this
solves, so the two objectives must agree -- but only on seeds where CG certified
(`cg_stop_reason == "converged_by_certification"`, full-universe scope), since on a
budget-stopped seed the CG number is an upper bound and a mismatch would prove nothing.
Parameters have to match too: this oracle caps `max_stops` at 4 while the benchmarks run
at 10, so a like-for-like cell needs small `n` and `max_stops = 4`.

Repro: `sbatch benchmarks/diagnostics/run_benders_n10.sh` (env `BJ_SEED`, `BJ_N`, `BJ_P`,
`BJ_S`, `BJ_MAX_STOPS`); the script runs both monolithic references and fails loudly.

## What this is not

`:direct_enumeration` enumerating the whole route universe up front is exactly the cost
Benders exists to avoid, and it is the same objection that retired the pre-split
`AggregateODRouteBendersYXFormulation`. It is the right *first* oracle: with the pool
complete and fixed the subproblem is an honest LP with exact duals, so the loop, the cut
algebra and the hook wiring are verified before an inexact subproblem raises the question
of whether its duals still give valid cuts. `:column_generation` is the intended next
value and is currently rejected rather than ignored. `RouteCoveringProblem`
(`opt/problems/route_covering.jl`) remains the shape that oracle should reuse.
