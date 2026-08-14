# Problem/Formulation/Solver split — status and remaining work

(Originally written 2026-08-11 during the initial migration; rewritten 2026-08-13 after
two cleanup passes removed everything the migration left unreachable. Superseded
content from the original version has been dropped rather than kept as history — see
git log for that if ever needed.)

## The pattern

```julia
# opt/optimize/run_opt.jl
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver) =
    optimize_model(build_model(problem, formulation, solver), solver)
```

- `AbstractProblem` (`opt/abstract.jl`) — *what* is being solved: instance data + the
  business decision on top of it (station count, walking limit).
- `AbstractFormulation` (`opt/abstract.jl`) — *how* it's mathematically encoded.
- `AbstractSolver` (`opt/solvers/utils/abstract.jl`) — *which* algorithm solves it.

`opt/abstract.jl` now contains only these two types — the old `AbstractStationSelectionModel`
hierarchy and everything downstream of it was removed (see "What was cut" below).

## What's live today (6 formulations, all verified via `build_model`/full test suite)

**Clustering** — `StationSelectionProblem` + one of 4 `AbstractClusteringFormulation`
subtypes + `DirectMIPSolver`: `ClusteringBaseFormulation`, `ClusteringTwoStageFormulation`,
`ClusteringTwoStageODFormulation`, `ClusteringTwoStageODFlowRegularizerFormulation`. Each
has its own independent `build_model` in `optimize/clustering/build_{single_stage,
two_stage,two_stage_od}.jl`.

**AggregateODRoute** — `StationSelectionProblem` + one of:
- `AggregateODRouteBaseFormulation` + `DirectMIPSolver` — exhaustive route enumeration
  (`enumerate_aggregate_od_route_columns`) up front, direct MIP solve.
- `AggregateODRouteJointRoutingAssignmentFormulation` + `CGSolver` — column generation,
  no up-front enumeration.

Both share the encoding-detail field set (`route_regularization_weight`,
`walk_cost_weight`, `repositioning_time`, `max_wait_time`, `detour_factor`, `max_stops`,
`allow_walk_only`); kept as separate marker types since they pair with different solvers.

The label-setting pricing engine both formulations sit on (`label_setting/engine/*.jl` +
`label_setting/aggregate_od_route/{types,data,labels,search,station_simple,
enumeration}.jl` for Base, `.../joint_routing_assignment/*.jl` for Joint) is independently
tested and untouched by any of the cleanup below.

## Kept but deliberately unwired (reminders, not dead code)

- Five Benders formulation marker structs, `opt/formulations/aggregate_od_route/benders/
  {y,xy,yz,yzh,yx}.jl` + `cut_mode.jl` (`AbstractBendersCutMode`/`SingleCut`/`MultiCut`) —
  now actually `include`d (previously y/xy/yz/yzh weren't even wired into
  `StationSelection.jl`).
- `RouteCoveringProblem` (`opt/problems/route_covering.jl`) — retyped onto
  `StationSelectionProblem` (was `AggregateODRouteProblem`, since removed), included.
- `BendersSolver` (`opt/solvers/benders_solver.jl`) — generic outer loop + 4 hook stubs
  (`extract_incumbent`/`solve_subproblem`/`benders_converged`/`add_benders_cut!`,
  dispatched via `mapping`, mirroring `CGSolver`'s pattern). No formulation implements
  the hooks currently.

## What was cut (three passes, all confirmed zero live callers, full suite re-verified
clean after each — no regressions across any of them)

1. The old nearest-open-assignment-policy Benders implementation: archived
   Benders/branch-and-Benders/heuristic-enumeration code, nearest-open endpoint-chain
   constraints, Benders variable/objective builders, and every test that only exercised
   that machinery.
2. A second, unrelated dead layer: the old pre-split solver architecture
   (`iterative_strategy_types.jl`'s `AbstractStationSelectionSolver`/`SolverConfig`/
   `DirectSolver`/`ColumnGenerationSolver`, `iterative_runner.jl`, `feasibility_check.jl`),
   the old `AggregateODRouteCG` outer-loop/dual-extraction/logging files (not needed by
   `enumerate_aggregate_od_route_columns`, which builds its own duals directly), the old
   `AbstractStationSelectionModel` type hierarchy in `opt/abstract.jl`, and 10 test
   sub-testsets that only exercised any of this.
3. Stragglers found by re-auditing every remaining file against the 6 live formulations:
   `objectives/aggregate_od_route/core.jl` (`set_aggregate_od_route_objective!`, the old
   combined-model objective, superseded by Base's/Joint's own), `add_benders_cut_placeholder_variables!`
   (dead function inside an otherwise-live shared file), `opt/solvers/heuristic_solver.jl`
   (`HeuristicDispatchSolver` — same shape as `BendersSolver` but zero callers/tests,
   unlike `BendersSolver` which was kept as the deliberate Benders reminder),
   `assert_no_walk_only_pairs` (dead, referenced an already-removed function in its own
   docstring), plus a few stale docstrings pointing at files/types that no longer exist.

ExactDARP (`ExactDARPRouteModel` etc., mentioned as a pending migration in the original
version of this note) is gone entirely now — not migrated, just absent. Nothing
references it.

## Remaining work

**Benders**, next attempt (see `opt/formulations/aggregate_od_route/benders/yx.jl`'s own
docstring for the fuller writeup): a first working `AggregateODRouteBendersYXFormulation`
(master = `y`, subproblem = free `x`/`θ` reusing `AggregateODRouteBaseFormulation`'s own
constraint code, standard LP-duality cuts, `DirectMIPSolver` subproblem) was built,
verified exact against `DirectMIPSolver`, then deliberately removed — it needed the
subproblem's route pool exhaustively enumerated up front to be correct, defeating
Benders' actual point. Current thinking for the real attempt:
- Subproblem should go through column generation, likely meaning `x` gets fixed too (not
  left free) — i.e. a `RouteCoveringProblem`-shaped (fix `y` and `x`, `θ`-only)
  subproblem, which may mean this decomposition's real shape converges with `xy.jl`'s
  rather than staying a distinct `YX` — worth resolving before implementing.
- Default `cut_derivation` should be `:zero_completion`, not `:standard` (matching
  `y.jl`/`yz.jl`'s existing default) — worth reasoning through what "zero completion"
  concretely means for this CG-based (not nearest-open-based) redesign before coding it,
  since the archived meaning was tied to nearest-open endpoint-chain duals this design no
  longer has.
- `CGSolver`'s hooks (`extract_duals`/`price_columns`/`add_columns!`) currently dispatch
  on `mapping::AggregateODRouteMap` alone — fine while only
  `AggregateODRouteJointRoutingAssignmentFormulation` uses that map type via `CGSolver`,
  but a `RouteCoveringProblem` subproblem solved via `CGSolver` would share that same map
  type and collide on method redefinition. Needs a real disambiguator (e.g. dispatch on
  `formulation`/`problem` instead of `mapping`, which likely means adding a field to
  `BuildResult` — it doesn't currently carry either) before this can be implemented
  safely.

**Test debt** (not dead code — tests exercising currently-relevant functionality through
a stale, pre-split API; needs rewriting, not deleting): `test/opt/test_integration.jl`
(Clustering formulation builds call `run_opt(data, model, DirectSolver(...))`, the removed
two-arg-model API), `test/utils/test_export_variables.jl`, `test/utils/test_solution_analysis.jl`,
`test/utils/test_objective_decomposition.jl`, and the 5 `test/utils/test_case_generators/
test_test{2,3,4,5,6}*.jl` hypothesis tests (same root cause). 7 fail + 18 error, unchanged
across all three cleanup passes — none of it is a regression, all pre-existing.

**`ClusteringBaseFormulation`/`ClusteringTwoStageFormulation`** lost the
`Union{Float64,Nothing}` "no walking limit" option when they moved onto
`StationSelectionProblem.max_walking_distance` (required `Float64`, defaults to 300) — a
real behavior narrowing, still not verified against any test that exercises "unlimited
walking distance." `ClusteringTwoStageODFormulation` already required a non-`nothing`
value before the migration, so this only affects the first two.

**Top-level `CLAUDE.md`** (`StationSelection.jl/CLAUDE.md`) still documents the old
`AbstractStationSelectionModel` hierarchy exclusively and doesn't mention
`AbstractProblem`/`AbstractFormulation`/`AbstractSolver` at all.
