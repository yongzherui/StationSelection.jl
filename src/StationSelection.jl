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

# Utils
include("utils/coords.jl")
include("utils/results.jl")
include("utils/costs.jl")
include("utils/scenarios.jl")
include("utils/export.jl")
include("utils/logging.jl")
include("utils/transform_orders.jl")
include("utils/transform_stations.jl")

# Data loading - core data structures
include("data/struct.jl")
include("data/stations.jl")
include("data/requests.jl")

# Optimization framework - abstract types first
include("opt/abstract.jl")
include("opt/models/two_stage_single_detour.jl")
include("opt/models/clustering_two_stage_od.jl")
include("opt/models/clustering_base.jl")

# Utility functions that depend on model types
include("utils/detour_combinations.jl")

# Pooling map (depends on TwoStageSingleDetourModel and find_detour_combinations)
include("data/pooling_map.jl")

# Clustering OD map (depends on ClusteringTwoStageODModel)
include("data/clustering_od_map.jl")

# Clustering base map (depends on ClusteringBaseModel)
include("data/clustering_base_map.jl")

# Optimization components
include("opt/variables.jl")
include("opt/constraints.jl")
include("opt/objective.jl")
include("opt/optimize.jl")

# Re-export key types and functions
using .CoordTransform
using .Results

export Result
export bd09_to_wgs84
export read_candidate_stations, read_customer_requests

# Re-export data structures
export StationSelectionData, ScenarioData, PoolingScenarioOriginDestTimeMap
export ClusteringScenarioODMap, ClusteringBaseMap
export create_station_selection_data, create_scenario_data
export create_pooling_scenario_origin_dest_time_map
export create_clustering_scenario_od_map
export create_clustering_base_map
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, has_routing_costs

# Re-export helper functions for testing
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_count_mapping

# Re-export optimization framework types
export AbstractStationSelectionModel
export AbstractSingleScenarioModel, AbstractMultiScenarioModel
export AbstractTwoStageModel, AbstractODModel, AbstractPoolingModel
export TwoStageSingleDetourModel
export ClusteringTwoStageODModel
export ClusteringBaseModel

# Re-export detour combinations
export find_detour_combinations
export find_same_source_detour_combinations
export find_same_dest_detour_combinations

# Re-export optimization functions
export run_opt, build_model, build_model_with_counts
export add_station_selection_variables!, add_scenario_activation_variables!
export add_assignment_variables!, add_flow_variables!, add_detour_variables!
export add_assignment_constraints!, add_station_limit_constraint!
export add_scenario_activation_limit_constraints!, add_activation_linking_constraints!
export add_assignment_to_active_constraints!, add_assignment_to_selected_constraints!
export add_assignment_to_flow_constraints!
export add_assignment_to_same_source_detour_constraints!, add_assignment_to_same_dest_detour_constraints!
export set_two_stage_single_detour_objective!, set_clustering_od_objective!, set_clustering_base_objective!

# Re-export utility functions
using .StationCosts
using .GenerateScenarios
using .ExportResults

export compute_station_pairwise_costs, read_routing_costs_from_segments
export generate_scenarios
export export_results

# Re-export transform_orders functions
export transform_orders
export parse_station_list
export precompute_distances
export find_closest_selected_station
export get_timeframe_column

# Re-export transform_stations functions
export prepare_station_data, prepare_vehicle_data

end # module
