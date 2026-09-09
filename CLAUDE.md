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
  live for `AggregateODRouteJointRoutingAssignmentFormulation` — see "Benders
  decomposition" below).

## Directory Layout (`src/opt/`)

```
opt/
├── abstract.jl           # AbstractProblem, AbstractFormulation
├── problems/              # AbstractProblem subtypes (StationSelectionProblem, RouteCoveringProblem)
├── formulations/          # AbstractFormulation subtypes (clustering.jl, aggregate_od_route/*).
│                          #   aggregate_od_route/joint_routing_assignment/ is a FAMILY:
│                          #   shared.jl (the six shared encoding fields + the derivation
│                          #   helper), monolithic.jl, master.jl, benders_subproblem.jl,
│                          #   unions.jl
├── solvers/                # AbstractSolver subtypes + shared solver utils.
│                          #   direct_solver.jl, plus cg/ and benders/ — each split by the
│                          #   same roles: {pricing,subproblem}_config.jl (the search-
│                          #   algorithm config), solver.jl (the struct + the docstring
│                          #   that documents the algorithm), state.jl (loop state),
│                          #   loop.jl (the loop), metadata.jl (result report), hooks.jl
│                          #   (per-formulation hook fallbacks)
├── optimize/               # build_model methods, one per (problem × formulation × solver);
│                          #   aggregate_od_route/{direct,column_generation,benders}/ by
│                          #   solver, plus joint_shared.jl (the cost/pool stash all three
│                          #   joint builds share)
├── label_setting/          # pricing/column-enumeration engine for the AggregateODRoute formulations
│                          #   joint_routing_assignment/{exact,station_simple,darp,darp_modified}/ price columns;
│                          #   joint_routing_assignment/relaxed_cluster/ is a relaxed GRAPH, not
│                          #   a pricer: the exact search runs on it. Label-setting core at its
│                          #   top level; the drivers that use it live in its utils/, one folder
│                          #   per optimization — certification/, guiding/ (station subset
│                          #   for the exact pricer), refinement/. certification/ holds both
│                          #   modes: results.jl + common.jl + round.jl are shared, certify.jl
│                          #   is :relaxed_cluster, two_tier/ is :relaxed_cluster_two_tier
├── variables/               # shared variable-creation building blocks (y, z, x, θ, f, walk)
├── constraints/             # shared constraint-creation building blocks
└── objectives/               # shared objective-assembly building blocks
```

`build_model` methods compose the `variables/`/`constraints/`/`objectives/` building
blocks; they don't live inside `formulations/` or `problems/` themselves.

## Live Formulations (6 + 2 derived)

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

## Benders decomposition (Joint formulation)

`AggregateODRouteJointRoutingAssignmentFormulation` is also solvable by `BendersSolver`.
The decomposition is expressed as **two derived formulation types**, not as a solver that
builds two models by hand — so master and subproblem compose the same
`variables/`/`constraints/`/`objectives/` blocks the monolith does:

| Formulation | Carries | Blocks |
| --- | --- | --- |
| `…JointRoutingAssignmentMasterFormulation` | `y` + cut placeholders `Θ` | `add_station_selection_variables!`, `add_benders_cut_variables!`, `add_station_limit_constraint!`, `add_aggregate_od_route_endpoint_feasibility_constraints!`, `set_benders_master_objective!` |
| `…JointRoutingAssignmentBendersSubproblemFormulation` | one scenario's `x_walk`, `θ`, with `y` fixed | `add_walk_variables!`, `add_joint_routing_assignment_{coverage,station_linking}_constraints!`, `set_joint_routing_assignment_objective!`, `add_joint_routing_assignment_column!` — all with the new `scenarios` kwarg |

The two partition the monolith's rows exactly: `y` appears in neither the coverage rows nor
the objective, so the first stage is `y` plus the rows written only in `y`, and everything
else is the second stage — which separates exactly by scenario (every column belongs to one
scenario, every row is keyed `(s,p)`), hence `MultiCut(:scenario)` and one subproblem model
per scenario, built once.

**Both derived types are derived, never built from loose keywords.**
`MasterFormulation(parent; cut_mode)` / `BendersSubproblemFormulation(parent; max_stops)`
copy the family's six shared encoding fields off `parent`
(`joint_routing_assignment/shared.jl`). A master at one `detour_factor` against a
subproblem at another yields invalid cuts and a confidently wrong `OPTIMAL` with nothing
raising, so the inconsistent combination is made unrepresentable. `run_opt` still takes
ONE formulation — the monolith — and `build_model(problem, ::Monolith, ::BendersSolver)`
derives both halves, because Benders is an algorithm for the same model, not a different
model.

`y` is kept as a (relaxed) variable in the subproblem and pinned with
`JuMP.fix(y[j], ŷ[j]; force=true)`, which is what lets the linking-constraint builder be
reused **verbatim** — the rows stay `θ - y[j] ≤ 0` and there is no numeric-RHS code path to
drift from the first.

### Two properties of the answer that must travel with it

- **`metadata["benders_second_stage_relaxed"] == true`, always.** The cut IS the
  subproblem's LP dual, so the subproblem must be an LP, so a converged run is optimal for
  the **mixed** model (`y` binary, `θ`/`x_walk` continuous) — NOT `DirectMIPSolver`'s
  all-binary optimum over the same pool. Those differ by this formulation's LP–IP gap,
  which is not small here.
- **`metadata["benders_optimality_scope"]`.** The only oracle today is
  `:direct_enumeration`, whose pool is exponential in `max_stops`, so
  `BendersSubproblemConfig.max_stops` defaults to **4** and narrows the formulation's own
  value when that is larger. When it does, the scope reads `"max_stops_restricted"` and
  the claim is optimality over routes of at most that many stops — the same discipline
  `cg_optimality_scope` enforces for `CGSolver`. `nothing` disables the cap.

MEASURED (Zhuzhou n=10 p=8 sc=1 seed=42, k=5, `max_stops=4`, 16,320 enumerated columns):
converges in **4 iterations / 3 cuts**, LB == UB exactly at 12249.400939, loop wall 2.3 s
(master 0.04 s, subproblems 1.50 s) on top of 3.5 s of enumeration. Verified exact against
the same mixed model solved monolithically over the same pool (`diff 0.000e+00`) --
`benchmarks/diagnostics/benders_joint_n10.jl`, which runs that check plus
`benders <= direct_mip` on every invocation. Note the LP-IP gap on that cell is 0%, so
`benders <= direct_mip` passes trivially there; the monolithic *mixed* comparison is the
check that actually establishes exactness.

That script's five arms (25/25 checks) additionally cover `SingleCut` and 3 scenarios, and
the `max_stops`-narrowing path -- a formulation at `max_stops=6` with the subproblem capped
at 4 returns the `max_stops=4` objective *exactly*, since the master carries no `max_stops`
dependence, which is what makes that path testable without enumerating the wider universe.
`MultiCut` at s=3 added 5 cuts over 4 iterations against a possible 12, so the cut
deduplication behind `add_benders_cut!`'s return count is exercised, not merely defensive.
Full suite: 93,086/93,086.

Both bounds are reported (`benders_lower_bound`/`benders_upper_bound`/`benders_gap`): the
LB is the master's `objective_bound` (not `objective_value`, which a non-zero `MIPGap`
would inflate into an invalid bound), the UB is the best incumbent's exact second-stage
cost. `SOLVE_OPTIMAL` requires the two to have met inside `optimality_tol`; a run stopped
by `max_iterations`/`total_time_limit_sec` reports `SOLVE_FEASIBLE`, and the reported
objective is always the UB — a lower bound is not a solution.

No feasibility cuts are needed — but **not** because `x_walk` covers everything. `x_walk`
exists only for groups within `2 × max_walking_distance`; beyond that a group's coverage row
has no walk term and needs a route column with both stations built. The actual guarantee is
the master's `add_aggregate_od_route_endpoint_feasibility_constraints!` rows (the same ones
the CG master carries): a `y` failing them is never an incumbent. That condition is
*necessary, not sufficient*, so this is measured rather than proven — 0 of 86
endpoint-feasible station sets have an infeasible subproblem at n=10 seed 42, s=1 and s=3
(`benders_brute_force_certificate.jl`), while 166 of the 252 sets *outside* the master's
feasible set do. `solve_subproblem` accordingly keeps its raise as a live guard rather than
a can't-happen branch.

Every solve asserts the strong-duality identity `Σα − Σ Γⱼ ŷⱼ == objective` before its cut
is built — one line that catches a wrong dual sign, a dropped linking family, or a
subproblem whose `mapping` disagrees with the master's.

## Kept-but-unwired scaffolding

Not dead code — deliberately preserved as a starting point for future work, but not
reachable from any `build_model`/`Solver` today:

- Five Benders formulation marker structs under `opt/formulations/aggregate_od_route/
  benders/` (`{y,xy,yz,yzh,yx}.jl`) — pre-split scaffolding, superseded by the live
  master/subproblem pair above and still wired to nothing. `cut_mode.jl`
  (`AbstractBendersCutMode`/`SingleCut`/`MultiCut`) in that same directory is NOT
  scaffolding — it is live, read by the master formulation.
- `RouteCoveringProblem` (`opt/problems/route_covering.jl`) — fixed-`y`/fixed-assignment
  shape; the live Benders subproblem fixes `y` but leaves assignment to `θ`, so this
  remains the shape a `:column_generation` subproblem oracle would reuse rather than one
  anything builds today.

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
onto `CGSolver.pricing`, a `CGPricingConfig` (`opt/solvers/cg/pricing_config.jl`) — a
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

**Escalation is PER SCENARIO, not per round.** A round names the scenarios that came back
inconclusive (`RelaxedClusterCertificationResult.inconclusive_scenarios`) and
`cg_certification_round`'s `only_scenarios` re-runs exactly those at
`certifying_pricing_time_limit_sec` -- never the refuted ones (their columns are already in
the pool) or the certified ones. It had to become per-scenario: escalation used to be
decided round-wide *and only when the round produced no columns at all*, so the
harvest-and-continue path skipped straight past it and one productive scenario masked a
permanently stuck one. MEASURED at n=40 seed 47: scenario 2 refuted in all 38 iterations, so
the round always had columns; scenario 1 replayed a bit-identical 262 s inconclusive search
24 consecutive times, 0 escalated attempts in the whole run, master objective frozen at
32043.0090 from iteration 19 to 38, 73% of the wall proving nothing. A partial round's
`certified` means "every scenario I ran certified", so the round certifies only when nothing
outside the escalated subset refuted either; its `relaxed_rc_bound` stays `NaN` because a
partial round bounds only what it re-ran.
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

`BendersSolver` carries `max_iterations`, `optimality_tol`, `total_time_limit_sec` (a wall
cap over the whole loop, distinct from `config.time_limit_sec`, which reaches only the
master's own `optimize!`), and `subproblem` — a `BendersSubproblemConfig`
(`opt/solvers/benders/subproblem_config.jl`). **The subproblem oracle is a solver setting,
not a formulation one**, for exactly the reason pricers are: it is a search algorithm, so
two runs differing only in it solve the identical model. It carries `oracle`
(`:direct_enumeration` only today; `:column_generation` is the intended next value and is
rejected rather than ignored), `max_stops` (the enumeration cap — **default 4**, see the
scope note under "Benders decomposition"), `max_routes` and
`enumeration_time_limit_sec` (the enumerator's own guard rails, which throw rather than
truncate), and `time_limit_sec` (per subproblem LP; a subproblem that hits it is a hard
error, since a truncated LP's duals are not a valid underestimator).

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
| `SOLVE_OPTIMAL` | `OPTIMAL` | Certified optimum. For `CGSolver` this additionally requires pricing to have exhausted (`metadata["cg_converged"]`), i.e. a pool complete **for the universe pricing searched** -- see the scope note below. For `BendersSolver` it requires the two bounds to have MET (`metadata["benders_converged"]`) -- a master solving to `MOI.OPTIMAL` only means the master solved, and its objective is a lower bound |
| `SOLVE_FEASIBLE` | `FEASIBLE` | Valid incumbent / upper bound, optimality NOT proven: budget-stopped or pricing-inconclusive CG, a Benders run stopped by its iteration cap or wall budget, or a MIP that hit a limit with an incumbent |
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
