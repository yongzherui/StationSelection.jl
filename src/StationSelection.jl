module StationSelection

# Core dependencies
using CSV
using DataFrames
using Dates
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

# Data loading
include("data/stations.jl")
include("data/requests.jl")

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

end # module
