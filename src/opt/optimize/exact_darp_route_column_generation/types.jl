export ExactDARPRouteCGDuals
export ExactDARPRoutePricedColumn
export ExactDARPRoutePricingResult
export ExactDARPRouteColumnGenerationState
export ExactDARPRouteBucketDemandCaps
export ExactDARPRouteBucketPricingData
export ExactDARPRoutePricingLabel

struct ExactDARPRouteCGDuals
    route_capacity::Dict{NTuple{4, Int}, Float64}  # (s, t_id, j_idx, k_idx) => nonnegative covering price
    raw_route_capacity::Dict{NTuple{4, Int}, Float64}

    function ExactDARPRouteCGDuals(
        route_capacity::Dict{NTuple{4, Int}, Float64},
        raw_route_capacity::Dict{NTuple{4, Int}, Float64}=copy(route_capacity),
    )
        new(route_capacity, raw_route_capacity)
    end
end

struct ExactDARPRouteBucketDemandCaps
    scenario_idx::Int
    time_id::Int
    caps::Dict{Tuple{Int, Int}, Int}  # (pickup_j, dropoff_k) => qbar_jk for this bucket
end

struct ExactDARPRouteBucketPricingData
    scenario_idx::Int
    time_id::Int
    candidate_stations::Vector{Int}
    demand_caps::ExactDARPRouteBucketDemandCaps
    duals::Dict{Tuple{Int, Int}, Float64}  # (pickup_j, dropoff_k) => π_jkts
    vehicle_capacity::Int
    max_route_length::Int
    stop_dwell_time::Float64
    route_regularization_weight::Float64
    repositioning_time::Float64
end

mutable struct ExactDARPRoutePricingLabel
    current_station::Int
    visited::BitSet
    route_sequence::Vector{Int}
    resource_tau::Float64
    onboard::Dict{Tuple{Int, Int}, Int}
    alpha::Dict{Tuple{Int, Int}, Int}
    reduced_cost::Float64
end

struct ExactDARPRoutePricedColumn
    scenario_idx::Int
    time_id::Int
    route::RouteData
    alpha_profile::Dict{NTuple{3, Int}, Float64}
    reduced_cost::Float64
end

struct ExactDARPRoutePricingResult
    columns::Vector{ExactDARPRoutePricedColumn}
    status::Symbol
    message::String
    metadata::Dict{String, Any}
end

mutable struct ExactDARPRouteColumnGenerationState
    route_pool::ExactDARPRouteBucketPoolsState
    last_duals::Union{Nothing, ExactDARPRouteCGDuals}
    last_pricing_result::Union{Nothing, ExactDARPRoutePricingResult}
end
