module StationSelection

# Core dependencies
using CSV
using Combinatorics
using DataFrames
using DataStructures
using Dates
using Distances
using Gurobi
using JSON
using JuMP
using Logging
using Printf
using Random
using Serialization
using Statistics

# Data loading - core data structures
include("data/core/struct.jl")

# Core utilities
include("utils/core/coords.jl")
include("utils/core/results.jl")
include("utils/core/costs.jl")
include("utils/core/export.jl")
include("utils/core/logging.jl")

# Data preparation utilities
include("utils/data/scenarios.jl")
include("utils/data/candidate_stations.jl")

include("data/io/stations.jl")
include("data/io/requests.jl")

# Synthetic and file-backed experiment generators
include("generators/grid.jl")
include("generators/zhuzhou.jl")

# Synthetic test-case generators (middle-zone benchmark family)
include("generators/test_cases/common.jl")
include("generators/test_cases/base_middle_zone.jl")
include("generators/test_cases/test1_vehicle.jl")
include("generators/test_cases/test2_zone_proximity.jl")
include("generators/test_cases/test3_north_shift.jl")
include("generators/test_cases/test4_mirrored_zone.jl")
include("generators/test_cases/test5_triangle.jl")
include("generators/test_cases/test6_bidirectional.jl")

# Optimization framework - abstract types first
include("opt/abstract.jl")

# Solvers (AbstractSolver + concrete solvers) -- generic, no Problem/Formulation deps
include("opt/solvers/utils/abstract.jl")
include("opt/solvers/utils/common.jl")
include("opt/solvers/direct_solver.jl")
include("opt/solvers/cg_solver.jl")
include("opt/solvers/benders_solver.jl")
# heuristic_solver.jl (HeuristicDispatchSolver) removed -- generic run_heuristic! hook
# shell with zero implementations and zero callers/tests, unlike BendersSolver (kept
# despite having no formulation wired to it yet, since it's the Benders-specific
# reminder scaffold this cleanup pass deliberately preserved).

include("opt/problems/station_selection.jl")
# aggregate_od_route.jl (AggregateODRouteProblem) was removed entirely -- AggregateODRouteColumn,
# its only piece still needed, now lives in data/maps/aggregate_od_route_map.jl (its
# dominant consumer, AggregateODRouteMap.columns) since it's a plain data type, not a
# Problem. AggregateODRouteBaseFormulation/JointRoutingAssignmentFormulation pair with
# StationSelectionProblem directly.
# route_covering.jl (RouteCoveringProblem) is kept (not wired to any build_model/Solver)
# as a reminder of the fixed-y/fixed-assignment shape a future Benders subproblem should
# reuse -- see its own docstring.
include("opt/problems/route_covering.jl")

# Formulations (AbstractFormulation). The four Clustering formulations are retyped to
# <: AbstractFormulation but not yet split into a StationSelectionProblem pairing --
# they still carry their own l/k and use the old two-arg build_model(formulation, data);
# a deliberate halfway state, not a bug. The five Benders formulation marker structs
# (benders/{y,xy,yz,yzh,yx}.jl) are, likewise, kept but not wired to any build_model/
# Solver -- see opt/optimize.jl's top comment for why.
include("opt/formulations/clustering.jl")
include("opt/formulations/aggregate_od_route/base.jl")
include("opt/formulations/aggregate_od_route/benders/cut_mode.jl")
include("opt/formulations/aggregate_od_route/benders/y.jl")
include("opt/formulations/aggregate_od_route/benders/xy.jl")
include("opt/formulations/aggregate_od_route/benders/yz.jl")
include("opt/formulations/aggregate_od_route/benders/yzh.jl")
include("opt/formulations/aggregate_od_route/benders/yx.jl")
include("opt/formulations/aggregate_od_route/joint_routing_assignment.jl")

# Clustering OD map (depends on AbstractClusteringTwoStageODFormulation)
include("data/maps/clustering_od_map.jl")

# Clustering station map (depends on ClusteringTwoStageFormulation)
include("data/maps/clustering_two_stage_station_map.jl")

# Clustering base map (depends on ClusteringBaseFormulation)
include("data/maps/clustering_base_map.jl")

# Aggregate OD route OD map for AggregateODRouteProblem
include("data/maps/aggregate_od_route_map.jl")

# Model-to-map dispatch
include("data/maps/create_map.jl")

# Optimization components
include("opt/variables.jl")
include("opt/constraints.jl")
include("opt/objective.jl")
include("opt/optimize.jl")

# Variable export (depends on OptResult and all mapping types)
include("utils/analysis/export_variables.jl")
include("utils/analysis/solution_analysis.jl")
include("utils/analysis/objective_decomposition.jl")

# Re-export key types and functions

export ModelCounts, DetourComboData, BuildResult, OptResult
export bd09_to_wgs84
export read_candidate_stations, read_customer_requests
export GridStation, GridInstance, generate_grid_instance
export grid_station_id, grid_manhattan_dist, grid_travel_cost_dict
export create_grid_problem_data, create_grid_station_selection_data, print_grid_summary
export ZhuzhouStation, ZhuzhouInstance, generate_zhuzhou_instance
export create_zhuzhou_problem_data, create_zhuzhou_station_selection_data, print_zhuzhou_summary

# Synthetic test-case generators (middle-zone benchmark family)
export MiddleZoneBenchmarkInstance, generate_middle_zone_benchmark_instance, build_middle_zone_benchmark_cases, MZB_PROFILES
export create_middle_zone_problem_data, create_middle_zone_station_selection_data, print_middle_zone_summary
export T1Instance, generate_test1_instance, build_test1_cases, T1_FLEET_CONFIGS
export create_test1_problem_data, create_test1_station_selection_data, print_test1_summary
export T2Instance, generate_test2_instance, build_test2_cases, T2_VARIANTS
export create_test2_problem_data, create_test2_station_selection_data, print_test2_summary
export T3Instance, generate_test3_instance, build_test3_cases, T3_VARIANTS
export create_test3_problem_data, create_test3_station_selection_data, print_test3_summary
export T4Instance, generate_test4_instance, build_test4_cases, T4_VARIANTS
export create_test4_problem_data, create_test4_station_selection_data, print_test4_summary
export T5Instance, generate_test5_instance, build_test5_cases, T5_CASES, T5_DEMAND_CONFIGS
export create_test5_problem_data, create_test5_station_selection_data, print_test5_summary
export T6Instance, generate_test6_instance, build_test6_cases, T6_DEMAND_CONFIGS
export create_test6_problem_data, create_test6_station_selection_data, print_test6_summary

# Re-export data structures
export StationSelectionData, ScenarioData
export AbstractStationSelectionMap, AbstractClusteringMap
export ClusteringTwoStageODMap, ClusteringBaseModelMap
export ClusteringTwoStageStationMap
export AggregateODRouteMap
export create_station_selection_data, create_scenario_data
export create_clustering_two_stage_od_map
export create_clustering_two_stage_station_map
export create_clustering_base_model_map
export create_aggregate_od_route_map
export aggregate_od_route_validate_feasible_coverage
export create_map
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, get_walking_cost_by_id, get_routing_cost_by_id, has_routing_costs

# Re-export helper functions for testing
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_count_mapping
export has_walking_distance_limit, get_valid_jk_pairs
export get_valid_j_assignments

# Re-export optimization framework types
export AbstractClusteringFormulation, AbstractClusteringTwoStageODFormulation
export ClusteringBaseFormulation, ClusteringTwoStageFormulation
export ClusteringTwoStageODFormulation, ClusteringTwoStageODFlowRegularizerFormulation
# AggregateODRouteProblem, AbstractAggregateODAssignmentPolicy/Free/NearestOpen,
# RouteCoveringProblem, AnyAggregateODRouteProblem, and AggregateODRouteEndpointChainKey
# (nearest_open/endpoint_chain.jl, already unwired) are gone -- AggregateODRouteColumn is
# exported from its new home, data/maps/aggregate_od_route_map.jl.

# Re-export optimization functions
export run_opt, build_model
export RouteCoveringPricingData, RouteCoveringPricingDuals, RouteCoveringPricingLabel
export initial_route_covering_pricing_labels
export extend_route_covering_pricing_label
export RouteCoveringStationSimpleLabel
export RewardLayerBitset
export PassengerAssignmentCandidate, PassengerAssignmentOpportunity
export JointRoutingAssignmentPricingData, JointRoutingAssignmentPricingLabel
export JointRoutingAssignmentRouteColumn
export create_joint_routing_assignment_pricing_data
export initial_joint_routing_assignment_pricing_labels
export extend_joint_routing_assignment_pricing_label
export JointRoutingAssignmentStationSimpleLabel
export add_joint_routing_assignment_column!, joint_routing_assignment_column_cost
export extract_joint_routing_assignment_duals, joint_routing_assignment_pricing_candidates
export joint_routing_assignment_two_stop_seed_columns
export add_joint_routing_assignment_coverage_constraints!, add_joint_routing_assignment_station_linking_constraints!
export set_joint_routing_assignment_objective!
export add_station_selection_variables!, add_scenario_activation_variables!
export add_assignment_variables!
export add_flow_variables!
export add_route_variables!
export add_aggregate_od_route_base_column!
export extract_aggregate_od_route_base_duals
export add_assignment_constraints!, add_station_limit_constraint!
export add_scenario_activation_limit_constraints!, add_activation_linking_constraints!
export add_assignment_to_active_constraints!, add_assignment_to_selected_constraints!
export add_flow_activation_constraints!
export set_clustering_od_objective!, set_clustering_base_objective!
export set_clustering_od_flow_regularizer_objective!
export set_clustering_two_stage_station_objective!

export compute_station_pairwise_costs, read_routing_costs_from_segments
export select_top_used_candidate_stations
export generate_scenarios
export generate_scenarios_from_ranges
export generate_scenarios_by_datetimes
export generate_scenarios_by_profile
export export_results
export export_variables

end # module
