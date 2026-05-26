export AlphaRouteCGDuals
export AlphaRoutePricedColumn
export AlphaRoutePricingResult
export AlphaRouteColumnGenerationState
export AlphaRouteBucketDemandCaps
export AlphaRouteBucketPricingData
export AlphaRoutePricingLabel

struct AlphaRouteCGDuals
    route_capacity::Dict{NTuple{4, Int}, Float64}  # (s, t_id, j_idx, k_idx) => nonnegative covering price
    raw_route_capacity::Dict{NTuple{4, Int}, Float64}

    function AlphaRouteCGDuals(
        route_capacity::Dict{NTuple{4, Int}, Float64},
        raw_route_capacity::Dict{NTuple{4, Int}, Float64}=copy(route_capacity),
    )
        new(route_capacity, raw_route_capacity)
    end
end

struct AlphaRouteBucketDemandCaps
    scenario_idx::Int
    time_id::Int
    caps::Dict{Tuple{Int, Int}, Int}  # (pickup_j, dropoff_k) => qbar_jk for this bucket
end

struct AlphaRouteBucketPricingData
    scenario_idx::Int
    time_id::Int
    candidate_stations::Vector{Int}
    demand_caps::AlphaRouteBucketDemandCaps
    duals::Dict{Tuple{Int, Int}, Float64}  # (pickup_j, dropoff_k) => π_jkts
    vehicle_capacity::Int
    max_route_length::Int
    stop_dwell_time::Float64
    route_regularization_weight::Float64
    repositioning_time::Float64
end

mutable struct AlphaRoutePricingLabel
    current_station::Int
    visited::BitSet
    route_sequence::Vector{Int}
    resource_tau::Float64
    onboard::Dict{Tuple{Int, Int}, Int}
    alpha::Dict{Tuple{Int, Int}, Int}
    reduced_cost::Float64
end

struct AlphaRoutePricedColumn
    scenario_idx::Int
    time_id::Int
    route::RouteData
    alpha_profile::Dict{NTuple{3, Int}, Float64}
    reduced_cost::Float64
end

struct AlphaRoutePricingResult
    columns::Vector{AlphaRoutePricedColumn}
    status::Symbol
    message::String
    metadata::Dict{String, Any}
end

mutable struct AlphaRouteColumnGenerationState
    route_pool::AlphaRouteBucketPoolsState
    last_duals::Union{Nothing, AlphaRouteCGDuals}
    last_pricing_result::Union{Nothing, AlphaRoutePricingResult}
end
