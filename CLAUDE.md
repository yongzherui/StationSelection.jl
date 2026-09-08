# StationSelection.jl

Julia package implementing VBS location optimisation models. Source lives in `src/opt/`,
organised around a **Problem / Formulation / Solver** split (`opt/abstract.jl`):

```julia
run_opt(problem::AbstractProblem, formulation::AbstractFormulation, solver::AbstractSolver) =
    optimize_model(build_model(problem, formulation, solver), solver)
```

- `AbstractProblem` — *what* is being solved: instance data plus the business decision on
  top of it (station count, walking limit). One concrete type today: `StationSelectionProblem`
  (`data`, `k` = stations built, `max_walking_distance`, default 300).
- `AbstractFormulation` — *how* the problem is mathematically encoded (assignment policy,
  staging, cost weights). See "Live Formulations" below for the six concrete types.
- `AbstractSolver` — *which* algorithm solves it: `DirectMIPSolver` (single `optimize!`
  call), `CGSolver` (column-generation outer loop), `BendersSolver` (Benders outer loop,
  scaffolded but no formulation implements its hooks yet).

## Directory Layout (`src/opt/`)

```
opt/
├── abstract.jl           # AbstractProblem, AbstractFormulation
├── problems/              # AbstractProblem subtypes (StationSelectionProblem, RouteCoveringProblem)
├── formulations/          # AbstractFormulation subtypes (clustering.jl, aggregate_od_route/*)
├── solvers/                # AbstractSolver subtypes + shared solver utils
├── optimize/               # build_model methods, one per (problem × formulation × solver)
├── label_setting/          # pricing/column-enumeration engine for the AggregateODRoute formulations
│                          #   joint_routing_assignment/{exact,station_simple,darp,darp_modified}/ price columns;
│                          #   joint_routing_assignment/relaxed_cluster/ is a relaxed GRAPH, not
│                          #   a pricer: the exact search runs on it. Label-setting core at its
│                          #   top level; the drivers that use it live in its utils/, one folder
│                          #   per optimization — certification/ (one-shot + no-good cut loop),
│                          #   guiding/ (station subset for the exact pricer), refinement/
├── variables/               # shared variable-creation building blocks (y, z, x, θ, f, walk)
├── constraints/             # shared constraint-creation building blocks
└── objectives/               # shared objective-assembly building blocks
```

`build_model` methods compose the `variables/`/`constraints/`/`objectives/` building
blocks; they don't live inside `formulations/` or `problems/` themselves.

## Live Formulations (6)

| Formulation | Solver | Key idea | Unique fields | Unique variables |
| --- | --- | --- | --- | --- |
| `ClusteringBaseFormulation` | `DirectMIPSolver` | Single-scenario k-medoids; no build/activate split — `problem.k` stations *are* the selection | none (dispatch marker) | y, x |
| `ClusteringTwoStageFormulation` | `DirectMIPSolver` | Two-stage station-to-station assignment | `l` (activate/scenario) | y, z, x |
| `ClusteringTwoStageODFormulation` | `DirectMIPSolver` | Two-stage OD pickup/dropoff assignment | `l`, `in_vehicle_time_weight` | y, z, x |
| `ClusteringTwoStageODFlowRegularizerFormulation` | `DirectMIPSolver` | `ClusteringTwoStageODFormulation` + route-activation flow penalty | `l`, `in_vehicle_time_weight`, `flow_regularization_weight` | y, z, x, f_flow |
| `AggregateODRouteBaseFormulation` | `DirectMIPSolver` | Station build + decoupled OD assignment + route activation, against an exhaustively enumerated column pool built up front | `route_regularization_weight`, `walk_cost_weight`, `repositioning_time`, `max_wait_time`, `detour_factor`, `max_stops`, `compensated_dominance` | y, x, x_walk, θ |
| `AggregateODRouteJointRoutingAssignmentFormulation` | `CGSolver` | Same encoding-detail fields as Base, but θ columns carry OD assignment directly — no separate `x`; grown via column generation, no up-front enumeration | Base's field set exactly (the pricer lives on `CGSolver.pricing`) | y, x_walk, θ |

**Important departure from the old two-stage convention:** the two `AggregateODRoute*`
formulations have **no `z`/per-scenario-activation variable and no `l` distinct from
`k`** — every built station is usable in every scenario. Only the four Clustering
formulations retain the `y` (build) / `z` (activate-per-scenario) two-stage split.

`ClusteringBaseFormulation`/`ClusteringTwoStageFormulation` lost the "no walking limit"
(`nothing`) option when they moved onto `StationSelectionProblem.max_walking_distance`
(required `Float64`, default 300) — not yet verified against an "unlimited walking
distance" test case.

Both `AggregateODRoute*` formulations validate build-time feasibility
(`aggregate_od_route_validate_feasible_coverage`) and always expose direct walking
(`x_walk`, `WALK_ONLY_PAIR`) as a station-free coverage option — not configurable, no
`allow_walk_only` field.

## Kept-but-unwired scaffolding

Not dead code — deliberately preserved as a starting point for future work, but not
reachable from any `build_model`/`Solver` today:

- Five Benders formulation marker structs under `opt/formulations/aggregate_od_route/
  benders/` (`{y,xy,yz,yzh,yx}.jl` + `cut_mode.jl`'s `AbstractBendersCutMode`/`SingleCut`/
  `MultiCut`) — no formulation implements `BendersSolver`'s four hooks yet.
- `RouteCoveringProblem` (`opt/problems/route_covering.jl`) — fixed-`y`/fixed-assignment
  shape a future Benders subproblem should reuse.
- `BendersSolver` (`opt/solvers/benders_solver.jl`) — generic outer loop with four
  formulation-specific hook stubs (`extract_incumbent`, `solve_subproblem`,
  `benders_converged`, `add_benders_cut!`); none implemented yet.

See `notes/2026-08-11_problem_formulation_solver_split_progress.md` for the fuller
writeup, migration history, and remaining-work list.

## Removed entirely

The pre-split `AbstractStationSelectionModel` hierarchy and every model built on it are
gone — not migrated, just absent. If old scripts, notes, or slides reference
`TwoStageSingleDetourModel`/TSD, `ZCorridorODModel`, `XCorridorODModel`,
`XCorridorWithFlowRegularizerModel`, `TransportationModel`, `AlphaRouteModel`,
`RouteFleetLimitModel`, `RouteAlphaCapacityModel`, `RouteVehicleCapacityModel`,
`TwoStageRouteWithTimeModel`, `ExactDARPRouteModel`, or the two-arg
`run_opt(model, data; ...)`/`build_model(model, data; ...)` API, those refer to a version
of this package that no longer exists.

## Decision Variables

| Var | Domain | Meaning | Formulations |
| --- | --- | --- | --- |
| y[j] | {0,1} | Station j built | all |
| z[j,s] | {0,1} | Built station j activated in scenario s; z≤y | Clustering (two-stage only) |
| x[i,j,s] | {0,1} | Demand point i assigned to active station j in scenario s | ClusteringTwoStageFormulation |
| x[s][p][j,k] | Z₊ | OD demand group p (position within scenario s's positive-demand pairs) assigned to station pair (j,k) | ClusteringTwoStageOD* |
| x[s,p,j,k] | {0,1} | OD demand group (s,p) assigned to station pair (j,k), decoupled from routing | AggregateODRouteBaseFormulation |
| x_walk[s,p] | {0,1} | OD demand group (s,p) served by direct walk (no station) | both AggregateODRoute* |
| θ[column_id, s] | {0,1} (Base) / [0,1] LP-relaxed (Joint) | Route column activated in scenario s | both AggregateODRoute* |
| f_flow[s][(j,k)] | [0,1] | Route (j,k) activated in scenario s, for the flow-regularization penalty | ClusteringTwoStageODFlowRegularizerFormulation |

## Parameters

**On `StationSelectionProblem`:** `k` (stations built), `max_walking_distance` (feasibility
radius, shared by every formulation that restricts assignment by walk distance).

**Clustering formulations:** `l` (activate per scenario, two-stage only),
`in_vehicle_time_weight` (OD formulations only), `flow_regularization_weight`
(FlowRegularizer only).

**AggregateODRoute formulations (both):** `route_regularization_weight` (μ, multiplies
each route column's cost), `walk_cost_weight` (multiplies every walking-cost term),
`repositioning_time` (ρ, added to every route column's travel/service cost),
`max_wait_time`, `detour_factor` (min 1.0), and `max_stops` (min 2, default unbounded).
`CGPricingConfig.compensated_dominance` (default `true`) affects only `CGSolver`'s
label-setting pricer; `DirectMIPSolver`'s enumeration never runs dominance.

**Pricers are solver settings, not formulation ones.** `pricing_mode` and every
`relaxed_cluster_*` field moved off `AggregateODRouteJointRoutingAssignmentFormulation`
onto `CGSolver.pricing`, a `CGPricingConfig` (`opt/solvers/cg_pricing_config.jl`) — a
pricer is a search algorithm, so two runs differing only in it solve the *identical*
model. A mode sweep therefore varies one solver and reuses one formulation instead of
constructing a "different" formulation per arm:

```julia
run_opt(problem,
        AggregateODRouteJointRoutingAssignmentFormulation(max_stops=4),
        CGSolver(pricing=CGPricingConfig(mode=:relaxed_cluster, relaxed_cluster_count=9)))
```

`CGPricingConfig` carries `mode` (default `nothing` = the formulation's own default pricer,
which for Joint resolves to `:exact`; plus `:station_simple`, `:darp_modified`, `:darp`,
`:relaxed_cluster`), `warm_start_mode`, `relaxed_cluster_count`,
`relaxed_cluster_max_count`, `relaxed_cluster_guide_routes`,
`compensated_dominance`.
`AggregateODRouteBaseFormulation` has no selectable pricer, so it rejects any non-default
`mode`/`warm_start_mode` at build time rather than ignoring it. Its single CG pricer still
reads `compensated_dominance` from this config.

`warm_start_mode=:cluster_guide` is a warm-start-only alternative to
`:station_simple`: relaxed cluster routes select a real-station subset, the ordinary exact
engine produces real columns within it, and exhaustion hands the shared master and column
pool to the final mode (normally `:exact`). It requires `relaxed_cluster_count`. The guide
may consume at most half of its `pricing_time_limit_sec` slice; the subset search receives
the rest, including unused guide time. Priced columns record their origin in
`column.metadata["pricing_mode"]` for column-quality comparisons.

`:exact`/`:darp_modified`/`:darp` all search the full revisit-tolerant route
universe and are exhaustive-equivalent; `:station_simple` searches elementary routes only
and is therefore a *restriction* of the universe, not just a different search of it — its
optimum is scoped, see the `cg_optimality_scope` note under "Solve status".

`relaxed_cluster_count = K` builds a k-medoids station partition **once at build time**
(stashed as `m[:joint_routing_assignment_station_clustering]`) for relaxed-cluster
pricing (`label_setting/joint_routing_assignment/relaxed_cluster/`). It is read at
build time rather than per iteration precisely because the cells must be identical across
every CG iteration of a run, which is what makes `K` a meaningful swept parameter. The
relaxed-cluster mode **requires** it; setting it without the mode is legal and inert
apart from building the partition, which is what the guide-recovery diagnostics want. Note
the relaxed routes themselves are never columns: they are cluster routes, not real routes.

`pricing.mode = :relaxed_cluster` (`relaxed_cluster/cuts.jl` +
`relaxed_cluster/utils/certification/certify.jl`) is the no-good-cut loop, and the only
mode that can certify. Each cut round reserves half of its remaining pricing budget for
the cluster search, unions the supports of up to `relaxed_cluster_guide_routes` promising
cluster routes, and exact-prices that real-station subset. It verifies: take support `T`, search
`stations(T)` **exhaustively** with the exact pricer, and if that finds nothing improving,
`T` is barren -- add the cut *"every route must visit at least one cluster outside T"* and
search again. MEASURED: certifies at K=9 and K=12 with 5/4/1 and 10/6/1 cuts, same LP
objective as baseline.

The one experimental optimization still exposed is `relaxed_cluster_max_count`, which
enables refinement. Core no-good cut generation remains mandatory because it is the
certification mechanism itself. Two other switches -- a barren-support cache and active-cut
subsumption pruning -- were removed as unnecessary at the measured cut load (0.5-0.75 cuts
per scenario attempt at n=30/40, 11 active cuts at worst against a cap of 64); both are
written up as possible future work in
`src/opt/label_setting/joint_routing_assignment/relaxed_cluster/README.md`.

**The cuts are the mechanism, not an optimization on top of a working relaxation.** A
cut-free round is exactly this loop's round 1, and round 1 certified 0 times across ~1130
measured attempts at every size and every K (0/31 at every `K < n`, reproduced at three
further sizes -- `notes/2026-09-06_relaxed_cluster_harvesting_refinement_and_cuts.md`) -- because a converged master's exact minimum is exactly 0 while the
relaxation's slack is 10^2--10^3. A separate cut-free mode existed for that comparison and
was removed once the answer was in; `:relaxed_cluster_nogood`, the loop's old name from
when both existed (and back when a `CGSolver.certification_pricing_mode` flag, since
removed, selected it), is now rejected with a message pointing at `:relaxed_cluster`.

**The cut direction matters and the obvious stronger form is invalid.** `|route ∩ T| ≤ |T|-1`
is unsound: a real improving route touching `A,B,C,D` was never examined by the exact search
over `stations({A,B,C})`, yet that cut deletes its image -- a false certificate. Only an
*exhausted* subset search may be cut on. And because the exact pricer's candidate generation
is reward-driven, the cut search must additionally propose nodes that merely *escape* a cut
(see `cuts.jl`) -- without that it under-reports and certifies falsely.

**Solver-level:** `SolverOptions` (`silent`, `mip_gap`, `time_limit_sec`) shared by every
`AbstractSolver`. `CGSolver` additionally carries `pricing` (a `CGPricingConfig`, above),
`max_iterations`, `recover_integer_solution`, `initial_columns`, and the two pricing
budgets. `pricing.warm_start_mode` (default `nothing`) -- when set, CG prices in that mode
until its universe exhausts, then hands off to `pricing.mode`, which is the phase that
certifies. Both phases share one master and one column pool. Requires a formulation with a
selectable pricer (only `AggregateODRouteJointRoutingAssignmentFormulation` today, via the
`cg_pricing_mode`/`set_cg_pricing_mode!` hooks); a warm start that would be a no-op (same
resolved mode both phases) or that has nothing to hand off to is rejected, never silently
ignored.

**`CGSolver` has no certification knobs of its own.** It used to carry
`certification_pricing_mode`/`certification_time_limit_sec`/`certification_max_rounds`;
all three are gone. `:relaxed_cluster` is now selected as `pricing.mode`, it runs under
the pricing budgets already there
(`pricing_time_limit_sec` for the ordinary attempt, `certifying_pricing_time_limit_sec` for
the escalated one -- the same two-tier ladder every other mode uses), and the cut-round cap
is the constant `RELAXED_CLUSTER_MAX_CUT_ROUNDS` (65, in `relaxed_cluster/cuts.jl`) rather
than a swept parameter, because the wall clock and the 64-bit cut mask are the real bounds.

Each iteration the mode runs a **relaxation** of the pricing problem whose minimum reduced
cost lower-bounds the real one, so exhausting it without finding anything below
`-reduced_cost_tol` proves no real improving column exists -- ending the solve with
`cg_stop_reason="converged_by_certification"`. A refuted attempt proves nothing about
optimality, but it harvests the real columns its exhaustive subset searches found, so it
*is* that iteration's pricing round rather than being wasted (96% of attempts were
refuted). An inconclusive attempt (budget or cut cap) is the only one that escalates, and
a second inconclusive result ends the loop with `cg_stop_reason="pricing_inconclusive"`.
The certificate covers the **full** route universe (it bounds every real route, not just
the ones the active pricer searches), so such a run reports
`cg_optimality_scope="full_route_universe"` even when a
`pricing.warm_start_mode=:station_simple` phase found most of the columns, with
`cg_certified_by_relaxation=true` recording where the certificate came from. Requires a
formulation implementing `cg_certification_supported`/`cg_certification_round` -- which for
both relaxed-cluster modes means `relaxed_cluster_count` was set at build time; a
`:relaxed_cluster` mode nothing supports is rejected up front, never silently ignored.
`pricing.warm_start_mode=:relaxed_cluster` is rejected too: the mode never reports its own
exhaustion, so phase 2 would be unreachable.

`BendersSolver` carries `max_iterations`, `optimality_tol`.

## Key Constraints

| Constraint | Formula | Where |
| --- | --- | --- |
| Station limit | Σⱼ y[j] = k | all |
| Activation limit | Σⱼ z[j,s] = l ∀s | Clustering two-stage |
| Activation linking | z[j,s] ≤ y[j] ∀j,s | Clustering two-stage |
| Assignment coverage | Σⱼ x[i,j,s] = 1 (or Σ over station pairs = demand) ∀ demand group | all |
| Assignment-to-active | x ≤ z[j,s] (and z[k,s] for OD pairs) | Clustering two-stage |
| Station linking (AggregateODRoute) | x[s,p,j,k] ≤ y[j], x[s,p,j,k] ≤ y[k] | AggregateODRouteBaseFormulation |
| Route linking | Σ_columns covering (s,p,j,k) θ ≥ x[s,p,j,k] (Base) | AggregateODRouteBaseFormulation |
| Flow activation | f[j,k,s] ≥ Σ x[od,j,k,s] | ClusteringTwoStageODFlowRegularizerFormulation |

## Objective Components

- **Walking cost** (all formulations): demand-weighted walking distance/time to
  pickup/dropoff station, or (AggregateODRoute) direct-walk cost when `x_walk` is used.
- **In-vehicle routing cost** (`in_vehicle_time_weight` × routing cost): ClusteringTwoStageOD*.
- **Flow-regularization penalty** (`flow_regularization_weight` × Σ routing-time-weighted
  f_flow): ClusteringTwoStageODFlowRegularizerFormulation.
- **Route column cost** (AggregateODRoute, both): `route_regularization_weight` ×
  (column travel/service cost + `repositioning_time`), summed over active θ.

## Core Data Structures

```julia
StationSelectionData
  .stations::DataFrame        # :id, :lon, :lat
  .walking_costs               # Dict{(i,j), Float64}
  .routing_costs                # Dict{(i,j), Float64} or Nothing
  .scenarios::Vector{ScenarioData}

ScenarioData
  .label, .start_time, .end_time
  .requests::DataFrame
```

`AbstractStationSelectionMap` subtypes (`opt`-formulation-specific, built by `create_map`
or `create_aggregate_od_route_map`) hold the index bookkeeping (station id ↔ array index,
scenario label ↔ index, `Omega_s`/`Q_s` demand-group indexing for AggregateODRoute) that
`build_model` and the exported analysis helpers rely on.

## Entry Points

```julia
run_opt(problem, formulation, solver; ) -> OptResult
build_model(problem, formulation, solver)  -> BuildResult  # build only, no solve
```

```julia
problem = StationSelectionProblem(data, 10; max_walking_distance=300)
formulation = ClusteringTwoStageODFormulation(5; in_vehicle_time_weight=1.0)
solver = DirectMIPSolver(config=SolverOptions(silent=true))
result = run_opt(problem, formulation, solver)
```

`OptResult` fields: `termination_status`, `objective_value`, `solution`, `runtime_sec`,
`model`, `mapping`, `detour_combos`, `counts` (variable/constraint counts by category),
`warm_start_solution`, `metadata`, `duals` (`nothing` outside Benders dual-problem
results).

## Solve status

`OptResult.termination_status` is a package-owned `SolveStatus` enum, **not**
`MOI.TerminationStatusCode`. The MOI code reports the status of the last model object that
was optimized, which for `CGSolver` is the master over a restricted column pool -- a
budget-stopped run still leaves that master at `MOI.OPTIMAL`, so the raw code claimed
`OPTIMAL` for runs that proved nothing. MOI also has no code for "feasible but not proven
optimal" (`MOI.FEASIBLE_POINT` is a *primal* status).

| Member | Prints as | Meaning |
| --- | --- | --- |
| `SOLVE_OPTIMAL` | `OPTIMAL` | Certified optimum. For `CGSolver` this additionally requires pricing to have exhausted (`metadata["cg_converged"]`), i.e. a pool complete **for the universe pricing searched** -- see the scope note below |
| `SOLVE_FEASIBLE` | `FEASIBLE` | Valid incumbent / upper bound, optimality NOT proven: budget-stopped or pricing-inconclusive CG, or a MIP that hit a limit with an incumbent |
| `SOLVE_INFEASIBLE` | `INFEASIBLE` | No feasible solution: solver said so, or `check_feasibility`'s gate refuted the instance before any solve |
| `SOLVE_NOT_SOLVED` | `NOT_SOLVED` | No incumbent to report |

Member names carry the `SOLVE_` prefix because `using JuMP` re-exports bare
`OPTIMAL`/`INFEASIBLE` from MOI into scope; the printed labels drop it so result CSVs stay
readable. The raw MOI code is preserved as `metadata["moi_termination_status"]`.

**`OPTIMAL` is scoped to the route universe that was priced.** `SOLVE_OPTIMAL` asserts "no
improving column remains in the universe pricing searched", which for
`pricing.mode=:station_simple` is elementary routes only -- a revisiting column can beat
that optimum, and the status alone does not say so. Every `CGSolver` result therefore
carries `metadata["cg_optimality_scope"]`, either `"full_route_universe"` or
`"elementary_routes_only"`, alongside `cg_final_pricing_mode` and
`cg_pricing_universe_restricted`. **Check `cg_optimality_scope` before pooling a certified
objective with others or treating it as a true optimum**; filtering on
`termination_status` alone cannot distinguish the two. A `pricing.warm_start_mode` run
ends in the full-universe pricer, so it reports `"full_route_universe"` despite having
priced part of the run in the restricted one.

**A relaxation certificate is full-universe regardless of the pricer.** When
`pricing.mode = :relaxed_cluster` is what ended the solve
(`cg_certified_by_relaxation == true`, `cg_stop_reason == "converged_by_certification"`),
the proof came from a relaxation that lower-bounds *every* real route's reduced cost, not
from the active pricer exhausting its own universe — so `cg_optimality_scope` reads
`"full_route_universe"` and `cg_pricing_universe_restricted` is `false` even when a
`pricing.warm_start_mode = :station_simple` phase found most of the columns. That
combination is correct, not a bug: the restricted pricer found the columns, the relaxation
certified there are no more.

`run_opt`'s `check_feasibility` hook returns `nothing` to proceed or a reason `String` to
abort; `run_opt` converts the string into a `SOLVE_INFEASIBLE` result carrying it as
`metadata["infeasibility_reason"]`. It does **not** throw -- a proven-infeasible instance
is an answer, not a usage error.

# Notes

- When adding new variables in opt/variables/ we need to make sure to add the corresponding export variables function to ensure consistency.
- The top-level project `CLAUDE.md` (`../CLAUDE.md`) still refers to seven pre-split
  model names under "Optimisation Models" — treat this file, not that one, as
  authoritative for what's actually live in `src/opt/`.
