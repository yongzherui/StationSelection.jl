include("optimize/iterative_strategy_types.jl")
# `optimize/aggregate_od_route/solver_types.jl` and `optimize/formulations/aggregate_od_route/*`
# no longer exist -- superseded by `opt/solvers/*.jl` and `opt/formulations/aggregate_od_route/*.jl`,
# included directly from `StationSelection.jl`. `AggregateODRouteBendersYXFormulation`
# (`benders/yx.jl`, wired below at `optimize/aggregate_od_route/benders/yx/`) is the first
# Benders formulation wired into this layout; the historical `benders/{y,xy,yz,yzh}.jl`
# (built around a now-removed nearest-open assignment-policy concept) remain unwired --
# out of scope for now.
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
# generic_runner.jl and column_generation.jl are the AggregateODRouteCG engine's own
# outer loop/main loop (run_aggregate_od_route_column_generation) -- AnyAggregateODRouteProblem
# (AggregateODRouteProblem/RouteCoveringProblem)-typed throughout, removed along with
# AggregateODRouteProblem. Unwired; PassengerFreeAssignmentCG (AggregateODRouteJointRoutingAssignmentFormulation
# + CGSolver) doesn't use these -- it goes through opt/solvers/cg_solver.jl's generic
# outer loop with its own joint_routing_assignment/{duals,pricing_round,routing_and_assignment}.jl
# hooks instead. See StationSelection.jl's include comments.
# include("label_setting/aggregate_od_route/generic_runner.jl")
# include("label_setting/aggregate_od_route/column_generation.jl")
include("label_setting/aggregate_od_route/enumeration.jl")
include("optimize/aggregate_od_route/direct/build_base.jl")
include("optimize/aggregate_od_route/benders/yx/subproblem.jl")
include("optimize/aggregate_od_route/benders/yx/build_master.jl")
include("optimize/aggregate_od_route/benders/yx/hooks.jl")
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
# goes through here. AnyAggregateODRouteProblem-typed throughout; unwired along with the
# rest of the AggregateODRouteCG engine.
# include("label_setting/aggregate_od_route/dispatch.jl")
# `optimize/aggregate_od_route/heuristic_enumeration.jl` and all of
# `optimize/aggregate_od_route/benders/*.jl` (except the empty `y/build_{master,subproblem}.jl`
# stubs, not yet wired) were archived by the same external reorganization noted above --
# only `archive/` copies remain. This is the pre-refactor Benders/heuristic-enumeration
# machinery for `AggregateODRouteProblem`; rebuilding it under the new Problem/Formulation/
# Solver layout is out of scope for this session's work (see `AggregateODRouteBendersYFormulation`
# etc.'s own "not wired yet" comment near this file's top). Plain exhaustive enumeration
# (`enumerate_aggregate_od_route_columns`, `AggregateODRouteBaseFormulation`'s `θ` pool) was
# recovered and adapted -- see `label_setting/aggregate_od_route/enumeration.jl`, included
# above (it's a degenerate label-setting run: uniform rewards, no dominance pruning, so
# it lives alongside the search machinery it reuses rather than under `optimize/`).
