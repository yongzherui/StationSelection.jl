include("optimize/iterative_strategy_types.jl")
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
include("optimize/feasibility_check.jl")
include("optimize/run_opt.jl")
include("optimize/iterative_runner.jl")
include("label_setting/engine/types.jl")
include("label_setting/engine/mechanics.jl")
include("label_setting/aggregate_od_route/types.jl")
include("label_setting/aggregate_od_route/duals.jl")
include("label_setting/aggregate_od_route/data.jl")
include("label_setting/aggregate_od_route/labels.jl")
include("label_setting/aggregate_od_route/search.jl")
include("label_setting/aggregate_od_route/station_simple.jl")
include("label_setting/aggregate_od_route/logging.jl")
# generic_runner.jl and column_generation.jl (the AggregateODRouteCG engine's own outer
# loop/main loop, run_aggregate_od_route_column_generation) were removed along with the
# rest of the old nearest-open-assignment-policy Benders machinery -- PassengerFreeAssignmentCG
# (AggregateODRouteJointRoutingAssignmentFormulation + CGSolver) never used these, it goes
# through opt/solvers/cg_solver.jl's generic outer loop with its own
# joint_routing_assignment/{duals,pricing_round,routing_and_assignment}.jl hooks instead.
include("label_setting/aggregate_od_route/enumeration.jl")
include("optimize/aggregate_od_route/direct/build_base.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/types.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/data.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/labels.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/search_data.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/search.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/station_simple.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/duals.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/seeding.jl")
include("label_setting/aggregate_od_route/joint_routing_assignment/pricing_round.jl")
include("optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl")
# dispatch.jl is the CG-algorithm choke point for the *old* ColumnGenerationSolver
# (solver.algorithm-based dispatch) -- AggregateODRouteJointRoutingAssignmentFormulation
# pairs with the new CGSolver directly (no further algorithm dispatch needed) and never
# goes through here. Unwired along with the rest of the old AggregateODRouteCG engine.
# include("label_setting/aggregate_od_route/dispatch.jl")
# `optimize/aggregate_od_route/heuristic_enumeration.jl` and the old nearest-open-assignment-
# policy Benders/branch-and-Benders machinery under `optimize/aggregate_od_route/benders/`
# were removed entirely -- see this file's top comment for what's kept as a reminder
# instead. Plain exhaustive enumeration (`enumerate_aggregate_od_route_columns`,
# `AggregateODRouteBaseFormulation`'s `θ` pool) was recovered and adapted -- see
# `label_setting/aggregate_od_route/enumeration.jl`, included above (it's a degenerate
# label-setting run: uniform rewards, no dominance pruning, so it lives alongside the
# search machinery it reuses rather than under `optimize/`).
