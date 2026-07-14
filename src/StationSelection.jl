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

# Route utilities
include("utils/routes/route_data.jl")
include("utils/routes/route_evaluation.jl")
include("utils/routes/route_pool_types.jl")             # includes IterativeRouteGenerationConfig
include("utils/routes/iterative_route_strategies.jl")   # candidate generation helpers
include("utils/routes/generate_routes_from_orders.jl")
include("utils/routes/generate_iterative_routes.jl")
include("utils/routes/route_generation_dispatch.jl")
include("utils/routes/route_io.jl")
include("utils/routes/generate_exact_darp_routes.jl")
include("utils/routes/route_pool_initialization.jl")
include("utils/routes/route_pool_iteration.jl")
include("utils/routes/route_pool_enrichment.jl")
include("utils/routes/route_pool_export.jl")
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
include("opt/models/clustering.jl")
include("opt/models/exact_darp_route_model.jl")
include("opt/models/aggregate_od_route_model.jl")

# Clustering OD map (depends on TwoStageODPolicy)
include("data/maps/clustering_od_map.jl")

# Clustering station map (depends on TwoStagePolicy)
include("data/maps/clustering_two_stage_station_map.jl")

# Clustering base map (depends on SingleStagePolicy)
include("data/maps/clustering_base_map.jl")

# Exact DARP route OD map for ExactDARPRouteModel (depends on RouteData, route_io)
include("data/maps/exact_darp_route_od_map.jl")

# Aggregate OD route OD map for AggregateODRouteModel
include("data/maps/aggregate_od_route_map.jl")

# Model-to-map dispatch
include("data/maps/create_map.jl")

# Optimization components
include("opt/variables.jl")
include("opt/constraints.jl")
include("opt/objective.jl")
include("opt/optimize.jl")

# Warm start model for ExactDARPRouteModel (depends on opt/optimize.jl)
include("opt/models/exact_darp_route_warm_start.jl")

# Variable export (depends on OptResult and all mapping types)
include("utils/analysis/export_variables.jl")

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
export ExactDARPRouteODMap
export AggregateODRouteMap
export create_station_selection_data, create_scenario_data
export create_clustering_two_stage_od_map
export create_clustering_two_stage_station_map
export create_clustering_base_model_map
export create_exact_darp_route_od_map
export create_aggregate_od_route_map
export create_map
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, get_walking_cost_by_id, get_routing_cost_by_id, has_routing_costs

# Re-export helper functions for testing
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_count_mapping
export has_walking_distance_limit, get_valid_jk_pairs
export get_valid_j_assignments

# Re-export route utilities
export RouteData, generate_simple_routes
export IterativeRouteGenerationConfig, generate_iterative_routes
export RouteIOData, load_routes_and_alpha
export default_iterative_route_generation_config, generate_routes_for_bucket
export RoutePoolInitSpec, RoutePoolState, ExactDARPRouteBucketPoolsState, ExactDARPRouteEnrichmentConfig, ExactDARPRouteRunnerConfig, ExactDARPRouteColumnGenerationConfig
export ExactDARPRouteIterationSummary, ExactDARPRouteRunnerResult, ExactDARPRouteColumnGenerationRunnerResult
export initialize_route_pool, export_route_pool_state, export_exact_darp_route_bucket_pools_state
export run_exact_darp_route_iterative, run_exact_darp_route_column_generation
export AbstractStationSelectionSolver
export SolverConfig, DirectSolver, ColumnGenerationSolver, BendersSolver, HeuristicSolver
export AbstractBendersDecomposition, BendersY, BendersXY
export AbstractBendersCutMode, SingleCut, MultiCut
export AbstractSolveStrategy, AbstractIterativeSolveStrategy
export IterativeSolveIterationSummary, IterativeSolveResult
export ExactDARPRouteIterativeStrategy, ExactDARPRouteColumnGenerationStrategy
export ExactDARPRouteCGDuals, ExactDARPRoutePricedColumn, ExactDARPRoutePricingResult

# Re-export optimization framework types
export AbstractStationSelectionModel
export AbstractSingleScenarioModel, AbstractMultiScenarioModel
export AbstractTwoStageModel, AbstractODModel
export AbstractClusteringPolicy
export SingleStagePolicy, TwoStagePolicy, TwoStageODPolicy
export ClusteringModel
export ExactDARPRouteWarmStartModel
export ExactDARPRouteModel
export AggregateODRouteModel
export AbstractAggregateODAssignmentPolicy, FreeAggregateODAssignmentPolicy, NearestOpenAggregateODAssignmentPolicy
export RouteCoveringProblem, AnyAggregateODRouteModel, AggregateODRouteColumn

# Re-export optimization functions
export run_opt, build_model
export build_exact_darp_route_restricted_master, extract_exact_darp_route_cg_duals, solve_exact_darp_route_pricing
export add_aggregate_od_route_column!, add_or_update_aggregate_od_route_column!
export extract_aggregate_od_route_coverage_duals
export aggregate_od_route_coverage_sigma, generate_aggregate_od_route_columns
export run_aggregate_od_route_column_generation, AggregateODRouteCoverageDuals
export AggregateODRouteColumnGenerationResult
export AggregateODRouteCGLogger, AggregateODRouteCGIterationLog, AggregateODRouteCGTerminationLog
export AggregateODRoutePricingData, AggregateODRoutePricingDuals, AggregateODRoutePricingLabel
export create_aggregate_od_route_pricing_data, initial_aggregate_od_route_pricing_labels
export extend_aggregate_od_route_pricing_label, aggregate_od_route_pricing_by_label_setting
export get_exact_darp_route_warm_start_solution
export add_station_selection_variables!, add_scenario_activation_variables!
export add_assignment_variables!
export add_flow_variables!
export add_theta_r_ts_variables!
export add_aggregate_od_route_theta_variables!
export compute_beta_r_jkl
export add_assignment_constraints!, add_station_limit_constraint!
export add_scenario_activation_limit_constraints!, add_activation_linking_constraints!
export add_assignment_to_active_constraints!, add_assignment_to_selected_constraints!
export add_flow_activation_constraints!
export add_route_capacity_constraints!
export add_route_capacity_lazy_constraints!
export add_aggregate_od_route_coverage_constraints!
export set_clustering_od_objective!, set_clustering_base_objective!
export set_clustering_od_flow_regularizer_objective!
export set_clustering_two_stage_station_objective!
export set_route_od_objective!
export set_aggregate_od_route_objective!

export compute_station_pairwise_costs, read_routing_costs_from_segments
export select_top_used_candidate_stations
export generate_scenarios
export generate_scenarios_from_ranges
export generate_scenarios_by_datetimes
export generate_scenarios_by_profile
export export_results
export export_variables

end # module
