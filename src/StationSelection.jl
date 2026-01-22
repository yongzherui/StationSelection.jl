module StationSelection

# Core dependencies
using CSV
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

# Data loading
include("data/stations.jl")
include("data/requests.jl")

# =============================================================================
# NEW REFACTORED OPTIMIZATION FRAMEWORK
# =============================================================================

# Core data structures
include("data/structs.jl")

# Abstract model type hierarchy
include("opt/abstract.jl")

# Concrete model definitions
include("opt/models/base.jl")
include("opt/models/two_stage_lambda.jl")
include("opt/models/two_stage_l.jl")
include("opt/models/routing_transport.jl")

# Shared optimization components
include("opt/variables.jl")
include("opt/constraints.jl")
include("opt/objectives.jl")

# Main optimization runner
include("opt/optimize.jl")

# =============================================================================
# LEGACY OPTIMIZATION METHODS (for backward compatibility)
# =============================================================================

# Optimization methods
include("optimization/base.jl")
include("optimization/ideal.jl")
include("optimization/two_stage_l.jl")
include("optimization/two_stage_lambda.jl")
include("optimization/routing_transport.jl")
include("optimization/origin_dest_pair.jl")

# Re-export key types and functions
using .CoordTransform
using .Results
using .Stations
using .ReadCustomerRequests

export Result
export read_candidate_stations, read_customer_requests
export bd09_to_wgs84

# Re-export optimization functions
using .ClusteringBase
using .ClusteringIdeal
using .ClusteringTwoStageL
using .ClusteringTwoStageLambda
using .ClusteringTwoStageLRoutingTransportation
using .ClusteringTwoStageLOriginDestPair

export clustering_base
export clustering_ideal
export clustering_two_stage_l
export clustering_two_stage_lambda
export clustering_two_stage_l_routing_transportation
export clustering_two_stage_l_od_pair
export validate_request_flow_mapping

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

# =============================================================================
# NEW FRAMEWORK EXPORTS
# =============================================================================

# Re-export data structures
using .DataStructs
export StationSelectionData, ScenarioData
export create_station_selection_data, create_scenario_data
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, has_routing_costs

# Re-export abstract model types
using .AbstractModels
export AbstractStationSelectionModel
export AbstractSingleScenarioModel
export AbstractMultiScenarioModel
export AbstractTwoStageModel
export AbstractRoutingModel

# Re-export concrete model types
using .BaseModelDef
using .TwoStageLambdaModelDef
using .TwoStageLModelDef
using .RoutingTransportModelDef
export BaseModel
export TwoStageLambdaModel
export TwoStageLModel
export RoutingTransportModel

# Re-export optimization components (for advanced users)
using .Variables
using .Constraints
using .Objectives
export add_station_selection_variables!, add_assignment_variables!
export add_scenario_activation_variables!, add_flow_variables!
export add_pickup_assignment_variables!, add_dropoff_assignment_variables!
export add_station_limit_constraint!, add_scenario_activation_limit_constraints!
export add_activation_linking_constraints!
export create_walking_cost_expression!, create_routing_cost_expression!
export set_minimize_objective!

# Re-export main optimization function
using .Optimize
export optimize_model, OptimizationResult

end # module
