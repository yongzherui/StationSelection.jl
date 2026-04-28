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
include("data/core/struct.jl")

# Core utilities
include("utils/core/coords.jl")
include("utils/core/results.jl")
include("utils/core/costs.jl")
include("utils/core/export.jl")
include("utils/core/logging.jl")

# Data preparation utilities
include("utils/data/scenarios.jl")
include("utils/data/demand_bounds.jl")
include("utils/data/candidate_stations.jl")
include("utils/data/transform_orders.jl")
include("utils/data/transform_stations.jl")

# Route utilities
include("utils/routes/route_data.jl")
include("utils/routes/generate_routes_from_orders.jl")
include("utils/routes/route_io.jl")
include("utils/routes/generate_alpha_routes.jl")
include("data/io/stations.jl")
include("data/io/requests.jl")

# Optimization framework - abstract types first
include("opt/abstract.jl")
include("opt/models/clustering_two_stage_od.jl")
include("opt/models/robust_total_demand_cap.jl")
include("opt/models/clustering_base.jl")
include("opt/models/route_vehicle_capacity_model.jl")
include("opt/models/alpha_route_model.jl")
include("opt/models/route_fleet_limit_model.jl")

# Clustering OD map (depends on ClusteringTwoStageODModel)
include("data/maps/clustering_od_map.jl")

# Robust OD map (depends on RobustTotalDemandCapModel and clustering_od_map for compute_valid_jk_pairs)
include("data/maps/robust_od_map.jl")

# Clustering base map (depends on ClusteringBaseModel)
include("data/maps/clustering_base_map.jl")

# Vehicle capacity OD map for RouteVehicleCapacityModel (depends on RouteData)
include("data/maps/vehicle_capacity_od_map.jl")

# Alpha route OD map for AlphaRouteModel (depends on RouteData, route_io)
include("data/maps/alpha_route_od_map.jl")

# Fleet limit OD map for RouteFleetLimitModel (depends on VehicleCapacityODMap)
include("data/maps/fleet_limit_od_map.jl")

# Model-to-map dispatch
include("data/maps/create_map.jl")

# Optimization components
include("opt/variables.jl")
include("opt/constraints.jl")
include("opt/objective.jl")
include("opt/optimize.jl")

# Warm start model for RouteVehicleCapacityModel (depends on opt/optimize.jl)
include("opt/models/route_vehicle_capacity_warm_start.jl")

# Warm start model for AlphaRouteModel (depends on opt/optimize.jl)
include("opt/models/alpha_route_warm_start.jl")

# Variable export (depends on OptResult and all mapping types)
include("utils/analysis/export_variables.jl")

# Analysis helpers for exported variables (does not depend on OptResult)
include("utils/analysis/solution_analysis_from_exported_variables.jl")

# Objective decomposition (post-hoc attribution from exported CSVs)
include("utils/analysis/objective_decomposition.jl")

# Solution analysis (depends on OptResult, mapping types, and StationSelectionData)
include("utils/analysis/solution_analysis.jl")

# Re-export key types and functions

export ModelCounts, DetourComboData, BuildResult, OptResult
export bd09_to_wgs84
export read_candidate_stations, read_customer_requests

# Re-export data structures
export StationSelectionData, ScenarioData
export AbstractStationSelectionMap, AbstractClusteringMap
export ClusteringTwoStageODMap, ClusteringBaseModelMap
export RobustTotalDemandCapMap
export VehicleCapacityODMap
export AlphaRouteODMap
export FleetLimitODMap
export create_station_selection_data, create_scenario_data
export create_clustering_two_stage_od_map
export create_clustering_base_model_map
export create_vehicle_capacity_od_map
export create_alpha_route_od_map
export create_fleet_limit_od_map
export create_map
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, get_walking_cost_by_id, get_routing_cost_by_id, has_routing_costs

# Re-export helper functions for testing
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_count_mapping
export has_walking_distance_limit, get_valid_jk_pairs

# Re-export route utilities
export RouteData, generate_simple_routes
export RouteIOData, load_routes_and_alpha

# Re-export optimization framework types
export AbstractStationSelectionModel
export AbstractSingleScenarioModel, AbstractMultiScenarioModel
export AbstractTwoStageModel, AbstractODModel
export ClusteringTwoStageODModel
export RobustTotalDemandCapModel
export ClusteringBaseModel
export RouteVehicleCapacityModel
export RouteVehicleCapacityWarmStartModel
export AlphaRouteWarmStartModel
export AlphaRouteModel
export RouteFleetLimitModel

# Re-export optimization functions
export run_opt, run_opt_fleet_search, build_model
export get_warm_start_solution
export add_station_selection_variables!, add_scenario_activation_variables!
export add_assignment_variables!
export add_robust_assignment_variables!, add_robust_dual_variables!
export add_flow_variables!
export add_alpha_r_jkts_variables!, add_theta_r_ts_variables!
export add_v_jkts_variables!
export compute_beta_r_jkl
export add_assignment_constraints!, add_station_limit_constraint!
export add_robust_assignment_constraints!, add_robust_assignment_to_active_constraints!
export add_robust_recourse_cost_constraints!, add_robust_dual_constraints!
export add_scenario_activation_limit_constraints!, add_activation_linking_constraints!
export add_assignment_to_active_constraints!, add_assignment_to_selected_constraints!
export add_flow_activation_constraints!
export add_route_capacity_constraints!
export add_route_capacity_lazy_constraints!
export add_fleet_limit_constraints!
export set_clustering_od_objective!, set_clustering_base_objective!
export set_robust_total_demand_cap_objective!
export set_clustering_od_flow_regularizer_objective!
export set_route_od_objective!
export set_fleet_limit_objective!

export compute_station_pairwise_costs, read_routing_costs_from_segments
export select_top_used_candidate_stations
export generate_scenarios
export generate_scenarios_from_ranges
export generate_scenarios_by_datetimes
export generate_scenarios_by_profile
export compute_demand_bounds
export group_scenarios_by_period
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
export remap_order_times_stacked
export parse_station_list
export precompute_distances
export find_closest_selected_station
export get_timeframe_column

# Re-export transform_stations functions
export prepare_station_data, prepare_vehicle_data

end # module
