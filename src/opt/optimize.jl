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
include("label_setting/aggregate_od_route/types.jl")
include("label_setting/aggregate_od_route/data.jl")
include("label_setting/aggregate_od_route/labels.jl")
include("label_setting/aggregate_od_route/exact.jl")
include("label_setting/aggregate_od_route/station_simple.jl")
# generic_runner.jl, column_generation.jl, duals.jl, logging.jl, and dispatch.jl (the old
# AggregateODRouteCG engine's outer loop, master-facing dual extraction, CG logging, and
# CG-algorithm dispatch choke point) were all removed -- none of them were reachable from
# `enumerate_aggregate_od_route_columns` (below, `AggregateODRouteBaseFormulation`'s own
# route pool builder, which builds its own uniform-reward duals directly and needs none of
# this) or from PassengerFreeAssignmentCG (AggregateODRouteJointRoutingAssignmentFormulation
# + CGSolver), which goes through opt/solvers/cg_solver.jl's generic outer loop with its own
# joint_routing_assignment/{duals,pricing_round,routing_and_assignment}.jl hooks instead.
include("label_setting/aggregate_od_route/enumeration.jl")
# Shared by both AggregateODRouteBaseFormulation build_model methods below (DirectMIPSolver
# and CGSolver) -- see its own module docstring for why theta creation itself stays outside
# it.
include("optimize/aggregate_od_route/base_shared.jl")
include("optimize/aggregate_od_route/direct/build_base.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/types.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/data.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/labels.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/search_data.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/exact.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/station_simple.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/duals.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/seeding.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/pricing_round.jl")
include("optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl")
# AggregateODRouteBaseFormulation + CGSolver: same y/x/theta master DirectMIPSolver's build
# (above) solves, grown from an empty column pool via add_aggregate_od_route_base_column!
# (constraints/aggregate_od_route/base/route_activation.jl, part of opt/constraints.jl)
# instead of DirectMIPSolver's own up-front exhaustive enumeration. dispatch.jl
# disambiguates its 4 CGSolver hooks from AggregateODRouteJointRoutingAssignmentFormulation's
# own (both share mapping::AggregateODRouteMap) by formulation type.
include("label_setting/aggregate_od_route/base/duals.jl")
include("label_setting/aggregate_od_route/base/pricing_round.jl")
include("optimize/aggregate_od_route/column_generation/build_base.jl")
include("optimize/aggregate_od_route/column_generation/dispatch.jl")
# `optimize/aggregate_od_route/heuristic_enumeration.jl` and the old nearest-open-assignment-
# policy Benders/branch-and-Benders machinery under `optimize/aggregate_od_route/benders/`
# were removed entirely -- see this file's top comment for what's kept as a reminder
# instead. Plain exhaustive enumeration (`enumerate_aggregate_od_route_columns`,
# `AggregateODRouteBaseFormulation`'s `θ` pool) was recovered and adapted -- see
# `label_setting/aggregate_od_route/enumeration.jl`, included above (it's a degenerate
# label-setting run: uniform rewards, no dominance pruning, so it lives alongside the
# search machinery it reuses rather than under `optimize/`).
