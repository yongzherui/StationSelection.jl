module StationSelection

# Core dependencies
using CSV
using Combinatorics
using DataFrames
using Dates
using Distances
using Gurobi
using JSON
using JuMP
using Logging
using Statistics

# Data loading - core data structures
include("data/struct.jl")

# Utils
include("utils/coords.jl")
include("utils/results.jl")
include("utils/costs.jl")
include("utils/scenarios.jl")
include("utils/export.jl")
include("utils/logging.jl")
include("utils/candidate_stations.jl")
include("utils/transform_orders.jl")
include("utils/transform_stations.jl")
include("data/stations.jl")
include("data/requests.jl")

# Optimization framework - abstract types first
include("opt/abstract.jl")
include("opt/models/clustering_two_stage_od.jl")
include("opt/models/clustering_base.jl")

# Clustering OD map (depends on ClusteringTwoStageODModel)
include("data/clustering_od_map.jl")

# Clustering base map (depends on ClusteringBaseModel)
include("data/clustering_base_map.jl")

# Model-to-map dispatch
include("data/create_map.jl")

# Optimization components
include("opt/variables.jl")
include("opt/constraints.jl")
include("opt/objective.jl")
include("opt/optimize.jl")

# Variable export (depends on OptResult and all mapping types)
include("utils/export_variables.jl")

# Analysis helpers for exported variables (does not depend on OptResult)
include("utils/solution_analysis_from_exported_variables.jl")

# Objective decomposition (post-hoc attribution from exported CSVs)
include("utils/objective_decomposition.jl")

# Solution analysis (depends on OptResult, mapping types, and StationSelectionData)
include("utils/solution_analysis.jl")

# Re-export key types and functions

export ModelCounts, DetourComboData, BuildResult, OptResult
export bd09_to_wgs84
export read_candidate_stations, read_customer_requests

# Re-export data structures
export StationSelectionData, ScenarioData
export AbstractStationSelectionMap, AbstractClusteringMap
export ClusteringTwoStageODMap, ClusteringBaseModelMap
export create_station_selection_data, create_scenario_data
export create_clustering_two_stage_od_map
export create_clustering_base_model_map
export create_map
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, has_routing_costs

# Re-export helper functions for testing
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_count_mapping
export has_walking_distance_limit, get_valid_jk_pairs

# Re-export optimization framework types
export AbstractStationSelectionModel
export AbstractSingleScenarioModel, AbstractMultiScenarioModel
export AbstractTwoStageModel, AbstractODModel
export ClusteringTwoStageODModel
export ClusteringBaseModel

# Re-export optimization functions
export run_opt, build_model
export get_warm_start_solution
export add_station_selection_variables!, add_scenario_activation_variables!
export add_assignment_variables!
export add_flow_variables!
export add_assignment_constraints!, add_station_limit_constraint!
export add_scenario_activation_limit_constraints!, add_activation_linking_constraints!
export add_assignment_to_active_constraints!, add_assignment_to_selected_constraints!
export add_flow_activation_constraints!
export set_clustering_od_objective!, set_clustering_base_objective!
export set_clustering_od_flow_regularizer_objective!

export compute_station_pairwise_costs, read_routing_costs_from_segments
export select_top_used_candidate_stations
export generate_scenarios
export generate_scenarios_from_ranges
export generate_scenarios_by_datetimes
export generate_scenarios_by_profile
export export_results
export export_variables
export load_exported_assignment_variables
export build_station_selection_data_from_config
export build_od_counts_from_data
export calculate_exported_walking_distance
export calculate_exported_vehicle_routing_distance
export ObjectiveDecomposition, decompose_objective

# Re-export solution analysis functions
export annotate_orders_with_solution
export calculate_model_walking_distance
export calculate_model_in_vehicle_time
export calculate_model_vehicle_routing_distance

# Re-export transform_orders functions
export transform_orders
export transform_orders_from_assignments
export parse_station_list
export precompute_distances
export find_closest_selected_station
export get_timeframe_column

# Re-export transform_stations functions
export prepare_station_data, prepare_vehicle_data

end # module
