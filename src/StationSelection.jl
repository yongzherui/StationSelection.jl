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
include("utils/corridor_clustering.jl")
include("utils/export.jl")
include("utils/logging.jl")
include("utils/candidate_stations.jl")
include("utils/transform_orders.jl")
include("utils/transform_stations.jl")
include("data/stations.jl")
include("data/requests.jl")

# Optimization framework - abstract types first
include("opt/abstract.jl")
include("opt/models/two_stage_single_detour.jl")
include("opt/models/clustering_two_stage_od.jl")
include("opt/models/two_stage_corridor_od.jl")
include("opt/models/x_corridor_od.jl")
include("opt/models/transportation.jl")
include("opt/models/clustering_base.jl")

# Utility functions that depend on model types
include("utils/detour_combinations.jl")

# Pooling map (depends on TwoStageSingleDetourModel and find_detour_combinations)
include("data/pooling_map.jl")

# Clustering OD map (depends on ClusteringTwoStageODModel)
include("data/clustering_od_map.jl")

# Corridor OD map (depends on AbstractCorridorODModel and corridor_clustering)
include("data/corridor_od_map.jl")

# Transportation map (depends on TransportationModel and corridor_clustering)
include("data/transportation_map.jl")

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

# Solution analysis (depends on OptResult, mapping types, and StationSelectionData)
include("utils/solution_analysis.jl")

# Re-export key types and functions

export ModelCounts, DetourComboData, BuildResult, OptResult
export bd09_to_wgs84
export read_candidate_stations, read_customer_requests

# Re-export data structures
export StationSelectionData, ScenarioData
export AbstractStationSelectionMap, AbstractClusteringMap, AbstractPoolingMap
export TwoStageSingleDetourMap
export ClusteringTwoStageODMap, ClusteringBaseModelMap
export CorridorTwoStageODMap
export TransportationMap
export create_station_selection_data, create_scenario_data
export create_two_stage_single_detour_map
export create_clustering_two_stage_od_map
export create_corridor_two_stage_od_map
export create_transportation_map
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
export AbstractTwoStageModel, AbstractODModel, AbstractPoolingModel
export AbstractSingleDetourModel
export TwoStageSingleDetourModel
export ClusteringTwoStageODModel
export AbstractCorridorODModel
export AbstractTransportationModel
export ZCorridorODModel
export XCorridorODModel
export TransportationModel
export ClusteringBaseModel

# Re-export detour combinations
export find_detour_combinations
export find_same_source_detour_combinations
export find_same_dest_detour_combinations

# Re-export feasible detour helper functions
export get_feasible_same_source_indices, get_feasible_same_dest_indices

# Re-export optimization functions
export run_opt, build_model
export warm_start, get_warm_start_solution
export add_station_selection_variables!, add_scenario_activation_variables!
export add_assignment_variables!
export add_flow_variables!, add_detour_variables!
export add_cluster_activation_variables!, add_corridor_variables!
export add_assignment_constraints!, add_station_limit_constraint!
export add_scenario_activation_limit_constraints!, add_activation_linking_constraints!
export add_assignment_to_active_constraints!, add_assignment_to_selected_constraints!
export add_assignment_to_flow_constraints!
export add_assignment_to_same_source_detour_constraints!, add_assignment_to_same_dest_detour_constraints!
export add_cluster_activation_constraints!, add_corridor_activation_constraints!
export add_corridor_x_activation_constraints!
export add_transportation_assignment_variables!, add_transportation_aggregation_variables!
export add_transportation_flow_variables!, add_transportation_activation_variables!
export add_transportation_assignment_constraints!, add_transportation_aggregation_constraints!
export add_transportation_flow_conservation_constraints!, add_transportation_flow_activation_constraints!
export add_transportation_viability_constraints!
export set_transportation_objective!
export set_two_stage_single_detour_objective!
export set_clustering_od_objective!, set_corridor_od_objective!, set_clustering_base_objective!

# Re-export objective expression functions (for debugging/customization)
export assignment_cost_expr, flow_cost_expr
export same_source_pooling_savings_expr, same_dest_pooling_savings_expr

export compute_station_pairwise_costs, read_routing_costs_from_segments
export cluster_stations_by_diameter, cluster_stations_by_count, compute_cluster_diameter, compute_corridor_data
export select_top_used_candidate_stations
export generate_scenarios
export generate_scenarios_from_ranges
export generate_scenarios_by_datetimes
export generate_scenarios_by_profile
export export_results
export export_variables
export load_exported_assignment_variables
export load_exported_flow_variables
export load_exported_same_source_pooling
export load_exported_same_dest_pooling
export build_station_selection_data_from_config
export build_od_counts_from_data
export calculate_exported_walking_distance
export calculate_exported_vehicle_routing_distance

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
