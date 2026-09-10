# `optimize/aggregate_od_route/solver_types.jl` and `optimize/formulations/aggregate_od_route/*`
# no longer exist -- superseded by `opt/solvers/*.jl` and `opt/formulations/aggregate_od_route/*.jl`,
# included directly from `StationSelection.jl`. The live Benders path is the
# `AggregateODRouteJointRoutingAssignment{Master,BendersSubproblem}Formulation` pair, derived
# by `build_model(problem, ::Monolith, ::BendersSolver)`. The five pre-split Benders marker
# structs (`benders/{y,xy,yz,yzh,yx}.jl`) that used to sit here unwired are deleted --
# they were counterparts to `BendersY`/`XY`/`YZ`/`YZH` solver markers that no longer exist,
# and the decomposition they were a placeholder for now works. Recoverable from git history
# ("AggregateODRouteBendersYXFormulation"), which also holds an earlier working YX attempt
# against `AggregateODRouteBaseFormulation`'s free-assignment machinery -- verified exact
# against `DirectMIPSolver`, then removed in favor of restarting against
# `RouteCoveringProblem` + column generation. `RouteCoveringProblem`
# (`opt/problems/route_covering.jl`) is still kept unwired.
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
# + CGSolver), which goes through opt/solvers/cg/loop.jl's generic outer loop with its own
# joint_routing_assignment/{duals,pricing_round,routing_and_assignment}.jl hooks instead.
include("label_setting/route_covering/exact/enumeration.jl")
# Shared by both AggregateODRouteBaseFormulation build_model methods below (DirectMIPSolver
# and CGSolver) -- see its own module docstring for why theta creation itself stays outside
# it.
include("optimize/aggregate_od_route/base_shared.jl")
include("optimize/aggregate_od_route/direct/build_base.jl")
# The cost-weight/pool stash every AnyJointRoutingAssignmentFormulation build shares --
# needed by the CG master build, the Benders subproblem build, and the integer-recovery
# rebuild, so it loads before all of them.
include("optimize/aggregate_od_route/joint_shared.jl")
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
# `CGSolver.pricing.mode` (a `CGPricingConfig`)
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
# universe without ever finding one -- and, via harvesting, a pricer too. Selected as
# `CGSolver.pricing.mode = :relaxed_cluster`, which requires a `relaxed_cluster_count`.
# See relaxed_cluster/relaxation.jl for the bound it rests on.
#
# The directory splits into label-setting CORE at its top level and the drivers that use
# it under utils/, one subdirectory per optimization (certification/, guiding/,
# refinement/). Folder grouping is NOT load order: guiding/ needs only the relaxed graph,
# while certification/ needs the cut-aware pricer and refinement/, so it loads last.
#
# CORE, half one -- the RELAXED GRAPH (clustering.jl/relaxation.jl/data.jl) is a graph, not
# a pricer. clustering.jl is standalone (and carries the model-side partition accessor both
# uses share); relaxation.jl needs it (struct field); data.jl needs both.
include("label_setting/joint_routing_assignment/relaxed_cluster/clustering.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/relaxation.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/data.jl")
# The CUT-FREE search over that graph is `../exact/`'s context handed the relaxed pricing
# data -- no files of its own. utils/guiding/guide.jl is its only driver: it prices the
# cluster graph to pick a station subset, then builds an EXACT context (exact/context.jl)
# over it, so it needs both pricers plus pricing_round.jl's candidate extraction and loads
# after them. It cannot certify, and nothing here tries to -- see below.
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/guiding/guide.jl")
# CORE, half two -- the CUT-AWARE SEARCH is this directory's own pricer, and the only
# search anywhere that can certify: a cut-free round is the loop's round 1, which certified
# 0 times in ~1130 measured attempts. It therefore takes the unprefixed file roles every
# other pricer directory uses (see ../../README.md), minus dominate/prune/accept, which are
# `../exact/`'s verbatim: a cut changes which routes may be REPORTED, not which label is
# better at a state. The satisfied-cuts mask has to be on the label, in the state and in
# the best-so-far signature (cuts.jl explains why none of that can be a post-hoc filter),
# which is what `../exact/`'s label cannot carry. Load order is the standard one: the cut
# resource first (seed/extend take it as an argument), then types, the logic files,
# context, and hooks last.
include("label_setting/joint_routing_assignment/relaxed_cluster/cuts.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/types.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/seed.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/extend.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/context.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/hooks.jl")
# utils/refinement/refine.jl: witness-guided cluster refinement. Needs data.jl's reward
# witness and ../exact/accept.jl's replay, and is consumed by the certification loop, so it
# loads between them. Pure functions -- no model, no solver, no search state.
# utils/certification/ is the loop around the cut-aware search -- the whole of
# `pricing.mode = :relaxed_cluster` and `:relaxed_cluster_two_tier`. It needs
# guiding/guide.jl's subset extraction, the cut context and refine.jl, so it loads last of
# all. Within it the order is bottom-up: shared result types, then the plumbing both modes
# use, then the one-tier loop, then the round driver that fans a per-scenario pass across
# scenarios, then the two-tier mode -- which is the same certification contract over a
# NESTED macro/meso partition pair and reuses the driver, the result type and the stat
# record, so it has to come after them.
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/refinement/refine.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/results.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/common.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/certify.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/round.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/two_tier/tuning.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/two_tier/partitions.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/two_tier/loop.jl")
include("label_setting/joint_routing_assignment/relaxed_cluster/utils/certification/two_tier/round.jl")
include("optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl")
# AggregateODRouteJointRoutingAssignmentFormulation + DirectMIPSolver: same y/x_walk/theta
# master CGSolver's build (above) solves, seeded with the exhaustive pool
# (enumerate_joint_routing_assignment_columns) instead of the two-stop seed, via the same
# shared _build_joint_routing_assignment_model body -- Joint's counterpart to Base's own
# DirectMIPSolver build below.
include("optimize/aggregate_od_route/direct/build_joint_routing_assignment.jl")
# optimize/aggregate_od_route/benders/: AggregateODRouteJointRoutingAssignmentFormulation +
# BendersSolver -- the master/subproblem pair the formulation family's master.jl and
# benders_subproblem.jl declare, built from the same variables/constraints/objectives
# blocks the monolith uses (see each build's own "Blocks used" list). Load order is
# bottom-up: the subproblem build first (build_master.jl calls it directly to construct
# one model per scenario), then the solve/dual extraction, then the cut builder, then the
# master build, and dispatch.jl last -- its hooks forward to everything above.
include("optimize/aggregate_od_route/benders/build_subproblem.jl")
include("optimize/aggregate_od_route/benders/completion_lpo.jl")
include("optimize/aggregate_od_route/benders/subproblem_cg.jl")
include("optimize/aggregate_od_route/benders/subproblem.jl")
include("optimize/aggregate_od_route/benders/cuts.jl")
include("optimize/aggregate_od_route/benders/build_master.jl")
include("optimize/aggregate_od_route/benders/dispatch.jl")
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
