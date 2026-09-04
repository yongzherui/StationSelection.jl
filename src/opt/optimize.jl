# `optimize/aggregate_od_route/solver_types.jl` and `optimize/formulations/aggregate_od_route/*`
# no longer exist -- superseded by `opt/solvers/*.jl` and `opt/formulations/aggregate_od_route/*.jl`,
# included directly from `StationSelection.jl`. None of the five Benders formulation
# marker structs (`benders/{y,xy,yz,yzh,yx}.jl`) are wired into a build_model/Solver here --
# kept purely as a reminder of the decompositions to (re)build, see those files' own
# docstrings and `opt/problems/route_covering.jl` (`RouteCoveringProblem`, likewise kept
# unwired). A first working attempt (`AggregateODRouteBendersYXFormulation` against
# `AggregateODRouteBaseFormulation`'s free-assignment machinery) was built, verified
# exact against `DirectMIPSolver`, and then deliberately removed in favor of restarting
# against `RouteCoveringProblem` + column generation -- see git history
# ("AggregateODRouteBendersYXFormulation") if useful.
#
# `optimize/iterative_strategy_types.jl` (old `AbstractStationSelectionSolver`/
# `SolverConfig`/`DirectSolver`/`ColumnGenerationSolver`/iterative-solve-strategy types),
# `optimize/iterative_runner.jl` (`run_iterative_solve`, zero callers), and
# `optimize/feasibility_check.jl` (`check_model_feasibility`/`EmptyStationSelectionMap`,
# never called by the current 3-arg `run_opt`) were all leftover scaffolding from before
# the Problem/Formulation/Solver split -- none of the 6 live formulations
# (ClusteringBaseFormulation/ClusteringTwoStageFormulation/ClusteringTwoStageODFormulation/
# ClusteringTwoStageODFlowRegularizerFormulation/AggregateODRouteBaseFormulation/
# AggregateODRouteJointRoutingAssignmentFormulation) touch any of it -- only already-broken
# tests referencing the removed `AggregateODRouteModel` did. Removed entirely.
include("optimize/clustering/build_two_stage_od.jl")
include("optimize/clustering/build_two_stage.jl")
include("optimize/clustering/build_single_stage.jl")
# build.jl (a single build_model(problem, formulation::AbstractClusteringFormulation,
# solver::DirectMIPSolver) dispatching internally to per-type _build_clustering!) was
# removed: each concrete formulation now has its own build_model method directly (real
# multiple dispatch, matching AggregateODRouteBase/JointRoutingAssignmentFormulation's
# pattern) instead of a redundant second dispatch layer.
# `optimize/aggregate_od_route/build.jl` (old monolithic build) no longer exists --
# superseded by `optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`
# and `optimize/aggregate_od_route/direct/build_base.jl` (both included below).
include("optimize/run_opt.jl")
include("label_setting/utils.jl")
include("label_setting/types.jl")
include("label_setting/engine.jl")
include("label_setting/round.jl")
include("label_setting/route_covering/types.jl")
include("label_setting/route_covering/data.jl")
include("label_setting/route_covering/exact/types.jl")
# seed.jl/extend.jl/dominate.jl have no dependency on the context struct;
# context.jl is self-contained (only needs types.jl); prune.jl needs
# RouteCoveringSearchContext (context.jl) since this pricer's precomputed
# indices live on the context struct itself rather than a separate index
# type (see prune.jl's own module docstring); hooks.jl loads last, needing
# the context struct in every method signature.
include("label_setting/route_covering/exact/seed.jl")
include("label_setting/route_covering/exact/extend.jl")
include("label_setting/route_covering/exact/dominate.jl")
include("label_setting/route_covering/exact/context.jl")
include("label_setting/route_covering/exact/prune.jl")
include("label_setting/route_covering/exact/hooks.jl")
include("label_setting/route_covering/station_simple/types.jl")
# seed.jl/extend.jl/dominate.jl/prune.jl have no dependency on the context
# struct here (unlike ../exact/, this pricer's bound needs nothing off the
# context beyond pricing_data/duals); context.jl is self-contained; hooks.jl
# loads last, needing the context struct in every method signature.
include("label_setting/route_covering/station_simple/seed.jl")
include("label_setting/route_covering/station_simple/extend.jl")
include("label_setting/route_covering/station_simple/prune.jl")
include("label_setting/route_covering/station_simple/dominate.jl")
include("label_setting/route_covering/station_simple/context.jl")
include("label_setting/route_covering/station_simple/hooks.jl")
# generic_runner.jl, column_generation.jl, duals.jl, logging.jl, and dispatch.jl (the old
# AggregateODRouteCG engine's outer loop, master-facing dual extraction, CG logging, and
# CG-algorithm dispatch choke point) were all removed -- none of them were reachable from
# `enumerate_aggregate_od_route_columns` (below, `AggregateODRouteBaseFormulation`'s own
# route pool builder, which builds its own uniform-reward duals directly and needs none of
# this) or from PassengerFreeAssignmentCG (AggregateODRouteJointRoutingAssignmentFormulation
# + CGSolver), which goes through opt/solvers/cg_solver.jl's generic outer loop with its own
# joint_routing_assignment/{duals,pricing_round,routing_and_assignment}.jl hooks instead.
include("label_setting/route_covering/exact/enumeration.jl")
# Shared by both AggregateODRouteBaseFormulation build_model methods below (DirectMIPSolver
# and CGSolver) -- see its own module docstring for why theta creation itself stays outside
# it.
include("optimize/aggregate_od_route/base_shared.jl")
include("optimize/aggregate_od_route/direct/build_base.jl")
include("optimize/aggregate_od_route/direct/build_feasibility.jl")
include("label_setting/joint_routing_assignment/types.jl")
include("label_setting/joint_routing_assignment/data.jl")
include("label_setting/joint_routing_assignment/exact/types.jl")
# logging.jl declares the rejection-census counters dominate.jl's
# _pricing_dominates_at_state increments, so it loads first, standalone (no
# dependency on anything else in this directory). seed.jl/extend.jl/
# dominate.jl themselves have no dependency on search_data.jl's types, but
# prune.jl/context.jl (struct fields) and hooks.jl (method signatures
# dispatching on JointRoutingAssignmentSearchContext) do -- see each file's
# own module docstring. Order below reflects that: everything that can load
# before search_data.jl does, then search_data.jl, then prune.jl/context.jl
# (need its types), then accept.jl (pure logic, no context dependency), then
# hooks.jl last -- it's pure wiring forwarding to every file above it,
# including context.jl's struct and accept.jl's route replay.
include("label_setting/joint_routing_assignment/exact/logging.jl")
include("label_setting/joint_routing_assignment/exact/seed.jl")
include("label_setting/joint_routing_assignment/exact/extend.jl")
include("label_setting/joint_routing_assignment/exact/dominate.jl")
include("label_setting/joint_routing_assignment/search_data.jl")
include("label_setting/joint_routing_assignment/exact/prune.jl")
include("label_setting/joint_routing_assignment/exact/context.jl")
include("label_setting/joint_routing_assignment/exact/accept.jl")
include("label_setting/joint_routing_assignment/exact/hooks.jl")
# Exhaustive enumeration for AggregateODRouteJointRoutingAssignmentFormulation's own
# DirectMIPSolver build (below, after column_generation/build_joint_routing_assignment.jl,
# which it also depends on) -- reuses route_covering/exact/enumeration.jl's raw physical-
# route DFS (already included above), so it must come after that too. See its own module
# docstring for why reusing that DFS is exact, not an approximation.
include("label_setting/joint_routing_assignment/exact/enumeration.jl")
include("label_setting/joint_routing_assignment/station_simple/types.jl")
# No prune.jl/accept.jl here: this pricer reuses ../exact/prune.jl's bound and
# ../exact/accept.jl's route replay directly (both already loaded by this
# point), wired straight into hooks.jl. seed.jl/extend.jl/dominate.jl have no
# dependency on the context struct; context.jl is self-contained; hooks.jl
# loads last, needing the context struct in every method signature.
include("label_setting/joint_routing_assignment/station_simple/seed.jl")
include("label_setting/joint_routing_assignment/station_simple/extend.jl")
include("label_setting/joint_routing_assignment/station_simple/dominate.jl")
include("label_setting/joint_routing_assignment/station_simple/context.jl")
include("label_setting/joint_routing_assignment/station_simple/hooks.jl")
# darp_modified/ and darp/ are two controlled comparison points against
# exact/'s running-max passenger crediting, both selectable per solve via
# `AggregateODRouteJointRoutingAssignmentFormulation`'s `pricing_mode` field
# (`:exact`/`:darp_modified`/`:darp`), branched on in
# `joint_routing_assignment/pricing_round.jl`'s `_pricing_build_scenario_context`
# (below). Both need `joint_routing_assignment/duals.jl` for
# `_verify_joint_routing_assignment_master_reduced_cost`, so their own context
# files are included after it.
#
# darp_modified/: value-equivalent to exact/ (branches commit-or-skip per
# passenger instead of running-max), served keyed by passenger with
# compensated dominance -- see darp_modified/types.jl's module docstring.
# seed.jl/extend.jl/dominate.jl have no dependency on the context struct;
# prune.jl's bound takes an untyped `ctx` so it has no load-order dependency
# on context.jl either (see prune.jl's own module docstring); context.jl is
# self-contained; hooks.jl loads after it, needing the context struct in
# every method signature; driver.jl loads last, needing hooks.jl's hooks.
include("label_setting/joint_routing_assignment/darp_modified/types.jl")
include("label_setting/joint_routing_assignment/darp_modified/data.jl")
include("label_setting/joint_routing_assignment/darp_modified/seed.jl")
include("label_setting/joint_routing_assignment/darp_modified/extend.jl")
include("label_setting/joint_routing_assignment/darp_modified/dominate.jl")
include("label_setting/joint_routing_assignment/duals.jl")
include("label_setting/joint_routing_assignment/darp_modified/prune.jl")
include("label_setting/joint_routing_assignment/darp_modified/context.jl")
include("label_setting/joint_routing_assignment/darp_modified/hooks.jl")
include("label_setting/joint_routing_assignment/darp_modified/driver.jl")
# darp/: literal onboard-bitset DARP-style pricer -- boarding commits to a
# specific (j,k) pair, served keyed by the full triple with plain (not
# compensated) dominance, ride-limit violations are hard infeasibility (the
# whole label is discarded, not just the one commitment) -- see
# darp/types.jl's module docstring. Load order mirrors darp_modified/'s:
# seed.jl/extend.jl/dominate.jl have no dependency on the context struct;
# prune.jl takes pricing_data directly (no dependency on context.jl at all,
# unlike either sibling pricer's bound); context.jl is self-contained;
# hooks.jl loads after it; driver.jl loads last, needing hooks.jl's hooks.
include("label_setting/joint_routing_assignment/darp/types.jl")
include("label_setting/joint_routing_assignment/darp/data.jl")
include("label_setting/joint_routing_assignment/darp/seed.jl")
include("label_setting/joint_routing_assignment/darp/extend.jl")
include("label_setting/joint_routing_assignment/darp/dominate.jl")
include("label_setting/joint_routing_assignment/darp/prune.jl")
include("label_setting/joint_routing_assignment/darp/context.jl")
include("label_setting/joint_routing_assignment/darp/hooks.jl")
include("label_setting/joint_routing_assignment/darp/driver.jl")
include("label_setting/joint_routing_assignment/seeding.jl")
include("label_setting/joint_routing_assignment/pricing_round.jl")
# relaxed_cluster/: NOT a fourth column-producing pricer -- a relaxation of the pricing
# problem, whose exhaustion certifies that no improving column exists in the FULL route
# universe without ever finding one. Selected via `CGSolver.certification_pricing_mode`
# (never `pricing_mode`), and only for a formulation built with `relaxed_cluster_count`.
# See relaxed_cluster/types.jl for the bound it rests on.
#
# Only four files: the relaxation is a *graph*, not a pricer, so there is no relaxed
# label/seed/extend/dominate/replay -- `../exact/`'s search runs on it unchanged (see
# relaxed_cluster/types.jl). clustering.jl is standalone; types.jl needs it (struct field);
# data.jl needs both; certify.jl and guide.jl reach out to the formulation/mapping/CGSolver
# layer and use pricing_round.jl's candidate extraction, so they load after it.
include("label_setting/joint_routing_assignment/relaxed_cluster/clustering.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/types.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/data.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/certify.jl")
# guide.jl loads last: it builds an EXACT context (exact/context.jl) out of a relaxed
# search, so it needs both pricers plus certify.jl's clustering accessor.
include("label_setting/joint_routing_assignment/relaxed_cluster/guide.jl")
# cuts.jl adds the no-good-cut resource (its own label + context); nogood_certify.jl
# is the loop around it and needs guide.jl's subset extraction, so it loads last.
include("label_setting/joint_routing_assignment/relaxed_cluster/cuts.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/nogood_certify.jl")
include("optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl")
# AggregateODRouteJointRoutingAssignmentFormulation + DirectMIPSolver: same y/x_walk/theta
# master CGSolver's build (above) solves, seeded with the exhaustive pool
# (enumerate_joint_routing_assignment_columns) instead of the two-stop seed, via the same
# shared _build_joint_routing_assignment_model body -- Joint's counterpart to Base's own
# DirectMIPSolver build below.
include("optimize/aggregate_od_route/direct/build_joint_routing_assignment.jl")
# AggregateODRouteBaseFormulation + CGSolver: same y/x/theta master DirectMIPSolver's build
# (above) solves, grown from an empty column pool via add_aggregate_od_route_base_column!
# (constraints/aggregate_od_route/base/route_activation.jl, part of opt/constraints.jl)
# instead of DirectMIPSolver's own up-front exhaustive enumeration. dispatch.jl
# disambiguates its 4 CGSolver hooks from AggregateODRouteJointRoutingAssignmentFormulation's
# own (both share mapping::AggregateODRouteMap) by formulation type.
include("label_setting/route_covering/duals.jl")
include("label_setting/route_covering/exact/pricing_round.jl")
include("optimize/aggregate_od_route/column_generation/build_base.jl")
include("optimize/aggregate_od_route/column_generation/dispatch.jl")
# `optimize/aggregate_od_route/heuristic_enumeration.jl` and the old nearest-open-assignment-
# policy Benders/branch-and-Benders machinery under `optimize/aggregate_od_route/benders/`
# were removed entirely -- see this file's top comment for what's kept as a reminder
# instead. Plain exhaustive enumeration (`enumerate_aggregate_od_route_columns`,
# `AggregateODRouteBaseFormulation`'s `θ` pool) was recovered and adapted -- see
# `label_setting/route_covering/exact/enumeration.jl`, included above (it's a degenerate
# label-setting run: uniform rewards, no dominance pruning, so it lives alongside the
# search machinery it reuses rather than under `optimize/`).
