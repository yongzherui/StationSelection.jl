# Study 5 dominance-fix pilot: spurious `INFEASIBLE` was a scenario-blind column-dedup bug

**Date:** 2026-08-28, root cause corrected and fixed 2026-08-29.
**Status:** root-caused and **FIXED**, verified end to end.
**Affects:** `AggregateODRouteJointRoutingAssignmentFormulation` under `CGSolver`, any
`pricing_mode` (`:exact`/`:darp_modified`/`:darp` all funnel through the same buggy
function) -- any multi-scenario instance where two different scenarios' demand groups
happen to land on columns with an identical `(p, j, k)` assignment set and equal-or-worse
`tau`.

## Symptom

`benchmarks/experiments/2026-08-27_study5_dominance_fix_pilot/job_0004_cg_exact.csv` (a
hand-run pilot, not the checked-in `config/stations_jobs.tsv` grid) reports
`n_stations=15, k=8, seed=42` as `termination_status=INFEASIBLE` after only
`cg_iterations=1`, `n_columns=240`. Reproduced identically against `2aeebda`/`ce0ccaf`/
`40fc65d` (HEAD at the time), so it wasn't something the surrounding dominance/seeding
commits touched. `n_stations=10` jobs in the same pilot (seeds 42-44) all converge
normally (24-30 iterations, `cg_converged=true`).

## Investigation trail (first hypothesis was wrong)

The initial read of the CSV (`cg_iterations=1`, small pool) suggested CG's livelock guard
(`cg_solver.jl:191`, `columns_accepted == 0 && break`, added in `6f8db0b`) had broken the
loop before every demand group's coverage made it into the pool, and that
`recover_integer_solution`'s restricted MIP was going infeasible over that incomplete pool
-- "pool starvation," not a genuinely infeasible instance. That motivated adding
`add_aggregate_od_route_endpoint_feasibility_constraints!`
(`constraints/endpoint_feasibility.jl`) -- a necessary-condition-on-`y`-alone check, wired
into every `AggregateODRouteBase`/`JointRoutingAssignmentFormulation` build, plus a
standalone `AggregateODRouteFeasibilityFormulation` (`y` + `station_limit` +
`endpoint_feasibility` only) for a sub-second feasibility probe ahead of a full solve
(wired via `check_feasibility(problem, formulation, solver)`, called by generic `run_opt`
between `build_model` and `optimize_model`).

Solving `job_0004`'s `y`-only necessary condition came back `OPTIMAL` in ~0.3s (12
endpoint rows, `built stations: [1, 5, 8, 11, 12, 13, 14, 15]`) -- proving *some* size-8
selection covers every station-dependent demand group. That ruled out "genuinely
infeasible instance" but didn't yet explain the real mechanism: the pool-starvation
story implied CG needed *more iterations*, but a direct dump of the CG-discovered pool's
per-demand-group coverage showed only **one** uncovered group in the entire 240-column
pool -- `(scenario=3, p=15)`, with **zero** covering columns -- and the CG iteration log's
`master_status` for iteration 1 was `"INFEASIBLE"`, not the `columns_accepted == 0`
livelock branch. That is: the very first LP solve, with just the seed pool and before any
pricing had even run, was already infeasible. "More CG iterations" was never going to fix
it -- something was wrong with the seed pool itself.

## Actual root cause

`add_joint_routing_assignment_column!` (`constraints/aggregate_od_route/
joint_routing_assignment/routing_and_assignment.jl`) dedups incoming columns by
`signature = _joint_routing_assignment_column_signature(column)` -- derived purely from
`column.assignments`, i.e. `(p, j, k)` triples -- with **no scenario in the key at all**.
`p` is only unique *within* a scenario's own `Omega_s[s]`; it is not a global demand-group
id. `job_0004`'s scenario 1 and scenario 3 both have a demand group at position 15 with an
identical candidate-pair/cost structure, so every one of scenario 3's 6 candidate seed
columns for `(3,15)` collided in tau with an already-registered scenario-1 column:

```
target column: id=228 assignments=[(15, 7, 1)] tau=661.104
  COLLIDES: id=52 scenario=1 assignments=[(15, 7, 1)] tau=661.104
target column: id=229 assignments=[(15, 7, 3)] tau=552.528
  COLLIDES: id=53 scenario=1 assignments=[(15, 7, 3)] tau=552.528
... (4 more, all 6 of scenario 3's p=15 candidates, all colliding with scenario 1's)
```

`add_joint_routing_assignment_column!`'s dedup check
(`columns[existing_id].tau <= column.tau + 1e-9 && return theta[existing_id], :skipped`)
threw away all 6, since each scenario-1 twin was registered first with an equal tau.
`coverage[(3,15)]` ended up a literal `0 >= 1` row -- infeasible from the very first
`optimize!`, exactly matching the observed `master_status="INFEASIBLE"` at iteration 1.

Confirms this is a genuine oversight, not intentional: `label_setting/
joint_routing_assignment/exact/enumeration.jl:144` already keys its own dedup on
`(Int(column.metadata["scenario"]), _joint_routing_assignment_column_signature(column))`
for exactly this reason. Only the master's own column-registration path was missing the
scenario qualifier -- and since every `pricing_mode` (`:exact`/`:darp_modified`/`:darp`)
ultimately inserts its discovered columns through this one function, the bug wasn't
specific to `:exact` pricing or to seeding; a mid-CG pricing round from any pricer could
hit the same collision.

## Fix

`add_joint_routing_assignment_column!` now keys its dedup on `(s, signature)` instead of
`signature` alone (`routing_and_assignment.jl`), matching `enumeration.jl`'s existing
pattern. Regression test:
`test/opt/test_joint_routing_assignment_column_dedup.jl` -- two scenarios, each with a
single request on the same station pair (so both land on demand-group position `p=1`
with an identical valid-pair set), asserts both scenarios' columns get `:added` and each
scenario's coverage row only ever picks up its own theta's coefficient. Verified this test
fails (5/11 assertions) against the pre-fix code and passes (11/11) with the fix.

## Verification

`job_0004` re-run post-fix (`benchmarks/study5_scaling_vs_enumeration/run_benchmark.jl`,
900s pricing budget): `status=exhausted, termination_status=OPTIMAL,
objective_value=36065.55, n_columns=5237, cg_iterations=31, cg_converged=true,
cg_pricing_exhausted=true`, `wall_sec=158.7`. Confirms the instance was feasible all
along and the CG loop, unblocked, runs a full legitimate solve to certified optimality.

Full test suite (`test/runtests.jl`): 91053/91053 passing throughout every step of this
investigation and fix.

## What's staying vs. what was a detour

`endpoint_feasibility`/`AggregateODRouteFeasibilityFormulation`/the `check_feasibility`
`run_opt` hook are kept -- they're a real, independent value-add (a sub-second
necessary-condition gate ahead of a full solve/enumeration, and useful for triaging future
`INFEASIBLE` results the way this investigation needed to), just not what actually fixed
`job_0004`. The pool-starvation hypothesis and the `cg_solver.jl:191` livelock-break framing
from the first pass at this note were incorrect for this instance -- recorded here so the
mistake doesn't get re-derived next time an `AggregateODRouteJointRoutingAssignmentFormulation`
CG run comes back unexpectedly `INFEASIBLE`.
