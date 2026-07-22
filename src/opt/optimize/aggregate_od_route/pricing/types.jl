"""
Plain data containers for the AggregateODRouteModel pricing label search.
"""

export AggregateODRoutePricingData
export AggregateODRoutePricingDuals
export AggregateODRoutePricingLabel

struct AggregateODRoutePricingData
    scenario::Int
    nodes::Vector{Int}
    travel_cost::Dict{Tuple{Int, Int}, Float64}
    active_pairs::Vector{Tuple{Int, Int}}
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    max_visits_per_node::Int
    bounded_max_stops::Bool
end

struct AggregateODRoutePricingDuals
    sigma::Dict{Tuple{Int, Int}, Float64}
end

struct AggregateODRoutePricingLabel
    current::Int
    route::Vector{Int}
    time::Float64
    station_age::Dict{Int, Float64}
    served_pairs::Set{Tuple{Int, Int}}
    tau::Float64
    reduced_cost::Float64
    route_length::Int
end

const AggregateODRouteLabelId = Int
const AggregateODRouteLabelOrderKey = Tuple{Float64, Float64, Int, Int}

struct AggregateODRouteLabelBitsets
    served_bits::BitSet
    station_age::Vector{Float64}
end
