# Problem/Formulation/Solver split — progress and remaining cleanup

## Goal

Replace the old combined "Model" types (one struct mixing business decision + encoding
choices, sometimes solver config too) with three orthogonal axes, dispatched through one
generic entry point:

```julia
# opt/optimize/run_opt.jl
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver) =
    optimize_model(build_model(problem, formulation, solver), solver)
```

- `AbstractProblem` (`opt/abstract.jl`) — *what* is being solved: instance data + the
  business decision on top of it (how many stations, walking limit).
- `AbstractFormulation` (`opt/abstract.jl`) — *how* it's mathematically encoded (variable
  structure, staging, objective weights).
- `AbstractSolver` (`opt/solvers/utils/abstract.jl`) — *which* algorithm solves it.

This is real Julia multiple dispatch on the `(problem, formulation, solver)` triplet,
replacing the older pattern of scattered `run_opt` overloads plus manual `isa`-branching
choke points (old `pricing/dispatch.jl` / Benders `dispatch.jl`).

## Fully migrated to the new pattern (working, verified)

**AggregateODRoute family** — `StationSelectionProblem` + one of:
- `AggregateODRouteBaseFormulation` + `DirectMIPSolver` — exhaustive route enumeration,
  direct MIP solve
- `AggregateODRouteJointRoutingAssignmentFormulation` + `CGSolver` — column generation

Both formulations carry the same encoding-detail field set (`route_regularization_weight`,
`walk_cost_weight`, `repositioning_time`, `max_wait_time`, `detour_factor`, `max_stops`,
`allow_walk_only`) but are kept as separate marker types since they pair with different
solvers and may diverge structurally later. No `assignment_policy` field on either — both
only ever supported free assignment in practice, so that field (and the
`AbstractAggregateODAssignmentPolicy` hierarchy behind it) was removed entirely.

**Clustering family** — `StationSelectionProblem` + one of 4 concrete
`AbstractClusteringFormulation` subtypes + `DirectMIPSolver`:
- `ClusteringBaseFormulation` — single-scenario k-medoids. **No fields** — no
  build/activate split, so `problem.l` alone is the station count.
- `ClusteringTwoStageFormulation` — two-stage station-to-station. Carries `k`.
- `ClusteringTwoStageODFormulation` — two-stage OD. Carries `k`, `in_vehicle_time_weight`.
- `ClusteringTwoStageODFlowRegularizerFormulation` — split out of the old optional
  `flow_regularization_weight` field (it changed which variables/constraints existed, a
  structural difference, not a parameter — same precedent as `XCorridorODModel` vs.
  `XCorridorWithFlowRegularizerModel` at the top level). `flow_regularization_weight` is
  a *required* keyword here (no "off" value — that's what the sibling type already is).

Each of the 4 has its own independent `build_model(problem, formulation, solver)` method
(in `optimize/clustering/build_single_stage.jl` / `build_two_stage.jl` /
`build_two_stage_od.jl`, two methods in the last file) — no shared dispatch-faking
`_build_clustering!` layer. `create_map` still has its own per-formulation methods in
`data/maps/create_map.jl` (real dispatch, not fake — kept).

## Key structural decisions made along the way

1. `StationSelectionProblem` (`opt/problems/station_selection.jl`) carries only `data`,
   `l`, `max_walking_distance` — the two business-decision fields genuinely shared across
   every formulation family. Everything else lives on the formulation.
2. Cross-field validation that used to happen at construction time (e.g. `k <= l`) moved
   into `build_model` (first thing it does), since `problem` and `formulation` aren't both
   in scope until then — a formulation can't validate against a `problem.l` it hasn't
   seen yet, and shouldn't hold a reference to one specific `problem` instance (risk of
   staleness if reused against a different `problem` later).
3. `opt/problems/` is reserved for genuine Problem types only. Currently: `station_selection.jl`
   (live) and `route_covering.jl` (present on disk, unwired — see below). `AggregateODRouteProblem`
   (the old combined type) was removed entirely, not just relocated.
4. `opt/formulations/` holds everything encoding-shaped, *including* formulations not yet
   fully migrated to the triplet pattern (see ExactDARP below) — retyped `<: AbstractFormulation`
   even where `build_model` hasn't caught up to the 3-arg signature yet, since the type
   itself is correctly categorized regardless of how much of the surrounding machinery
   has moved.
5. Plain data/value types (not a Problem, not a Formulation) live next to their dominant
   consumer rather than in `opt/problems/`. `AggregateODRouteColumn` now lives in
   `data/maps/aggregate_od_route_map.jl`, right before the `AggregateODRouteMap` struct
   whose `.columns` field is its main reason to exist — not a `problems/` file.

## Deliberately NOT migrated / unwired (remaining cleanup)

**ExactDARP** (`ExactDARPRouteModel`, `ExactDARPRouteWarmStartModel`,
`opt/formulations/exact_darp_route*.jl`) — retyped `<: AbstractFormulation` and moved out
of `opt/problems/`, but otherwise untouched: own `l`/`k` fields, two-arg
`build_model(formulation, data)`, and `run_opt(instance::StationSelectionData,
formulation::ExactDARPRouteModel, solver::AbstractStationSelectionSolver)` in
`opt/optimize/exact_darp/runner.jl` — a fully separate, older solver hierarchy
(`DirectSolver`/`ColumnGenerationSolver`/`HeuristicSolver` from
`opt/optimize/iterative_strategy_types.jl` + `opt/optimize/exact_darp/solver_types.jl`,
*not* the new `AbstractSolver`). `run_opt(::StationSelectionProblem,
::ExactDARPRouteModel, ::DirectMIPSolver)` does not exist yet — would need the same
treatment Clustering just got (drop the redundant `l`/`k` split, real `build_model`
dispatch, `k <= problem.l` check moved into it).

**RouteCoveringProblem / the `AggregateODRouteCG` column-generation engine** — unwired,
not deleted. Real, previously-working machinery (unlike ExactDARP/old-Clustering, which
were already unreachable through the new `run_opt`) — migrating it properly means
rewiring `label_setting/aggregate_od_route/{generic_runner,column_generation,dispatch}.jl`
and `constraints/aggregate_od_route/core.jl`'s `add_fixed_open_station_constraints!` onto
`StationSelectionProblem`/formulation, which wasn't done this session. Currently commented
out (not deleted) at:
- `StationSelection.jl`: the `opt/problems/route_covering.jl` include
- `opt/optimize.jl`: `label_setting/aggregate_od_route/{generic_runner,column_generation,dispatch}.jl`
- `opt/constraints.jl`: `constraints/aggregate_od_route/benders/completion.jl`
- `opt/objective.jl`: `expressions/aggregate_od_route/benders.jl` +
  `aggregate_od_route/benders/{subproblem,master,dual_lp}.jl`

`PassengerFreeAssignmentCG` (the working `JointRoutingAssignmentFormulation`+`CGSolver`
triplet) is unaffected — it never used any of this engine; its hooks are
`label_setting/aggregate_od_route/joint_routing_assignment/{duals,pricing_round,routing_and_assignment}.jl`
via `opt/solvers/cg_solver.jl`'s generic outer loop.

**Benders** (all four decompositions: Y/XY/YZ/YZH) — was already unwired before this
session by an external reorganization (most of it archived to
`optimize/aggregate_od_route/benders/archive/`). The formulation marker types
(`opt/formulations/aggregate_od_route/benders/{y,xy,yz,yzh}.jl`) still exist on disk but
aren't included — they need `AbstractBendersCutMode`/`SingleCut`/`MultiCut`, "not yet
(re)built anywhere" per the pre-existing comment. Not touched this session.

## Known loose ends to check when resuming

- `opt/optimize/run_opt.jl.bak` and `opt/constraints/aggregate_od_route/core.jl.bak` —
  old-version backups sitting in the tree, not included anywhere; probably safe to
  delete once confirmed unneeded.
- `opt/optimize/aggregate_od_route/benders/archive/` and
  `opt/optimize/exact_darp/column_generation/archive/` — archived dead code from the
  external reorganization, predates this session.
- `ClusteringBaseFormulation`/`ClusteringTwoStageFormulation` lost the
  `Union{Float64,Nothing}` "no walking limit" option when they moved onto
  `StationSelectionProblem.max_walking_distance` (required `Float64`, defaults to 300) —
  a real behavior narrowing, not verified against any test that exercised "unlimited
  walking distance." `ClusteringTwoStageODFormulation` already required a non-`nothing`
  value before this session, so this only newly affects the first two.
- The repo showed signs of a **second, external process editing the same files
  concurrently** mid-session: large unrelated renames/deletions under `src/opt/pricing/`
  → `label_setting/`, `benders/*` deletions, and at one point a file this session was
  actively editing (`opt/problems/aggregate_od_route.jl`) disappeared from disk entirely
  between edits, taking `AggregateODRouteColumn`'s only definition with it (recovered by
  rewriting it from context, now relocated per the point above). Worth diffing against
  git/remote before resuming in case more has moved since this note was written.
- `StationSelection.jl`'s "Re-export optimization functions/types" block duplicates
  exports already declared inline in the files that define those types (older
  convention, still present for the non-Clustering/non-AggregateODRoute stuff) — not
  cleaned up broadly, only the entries that had gone stale from this session's removals.
- Top-level `CLAUDE.md` (`StationSelection.jl/CLAUDE.md`) still documents the old
  `AbstractStationSelectionModel` hierarchy exclusively and doesn't mention
  `AbstractProblem`/`AbstractFormulation`/`AbstractSolver` at all — worth a refresh once
  the remaining migrations (ExactDARP, RouteCoveringProblem) land, rather than updating
  it piecemeal now while it's still mid-flight.
- Test suite: `test/opt/test_integration.jl`, `test/utils/test_export_variables.jl`,
  `test/utils/test_solution_analysis.jl`, `test/utils/test_objective_decomposition.jl`,
  and the 5 `test/utils/test_case_generators/test_test{2,3,4,5,6}*.jl` files were updated
  to use the new Clustering formulation names, but their `run_opt(data, model,
  DirectSolver(...))` calls still fail with a `MethodError` — pre-existing gap (old
  `DirectSolver`, no matching `run_opt` method for *any* Clustering type, old names or
  new), not something this session fixed or broke. Separately, several other test files
  reference a now-nonexistent `AggregateODRouteModel` name, stale from before the
  external reorganization, unrelated to this session's work.

## Verification performed

All smoke-tested via ad hoc scripts (not added to the test suite) on a 5-station
synthetic fixture (`mw_cut_fixture`-style, from
`test/opt/test_aggregate_od_route_restricted_mw_cut.jl`):

- `AggregateODRouteBaseFormulation`+`DirectMIPSolver` and
  `JointRoutingAssignmentFormulation`+`CGSolver`: both `OPTIMAL`, obj=`0.8` (agree, as
  expected for two encodings of the same problem).
- All 4 Clustering formulations via `run_opt(problem, formulation, DirectMIPSolver())`:
  all `OPTIMAL`.
- `k > l` now throws `ArgumentError` from `build_model`, as intended.
- `methods(build_model, (StationSelectionProblem, AbstractClusteringFormulation, DirectMIPSolver))`
  confirms 4 independent methods, no shared dispatch layer.
