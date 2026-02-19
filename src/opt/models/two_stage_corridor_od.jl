"""
ZCorridorODModel - Two-stage station selection with z-based corridor penalties.

Extends ClusteringTwoStageODModel with corridor variables that penalize
cross-zone vehicle movements. Zones are activated based on station activation
variables (z). Zones are defined by clustering stations on routing distance
with a diameter constraint.
"""

export ZCorridorODModel

"""
    ZCorridorODModel <: AbstractCorridorODModel

Two-stage stochastic station selection model with OD pair assignments and
z-based corridor penalties for cross-zone vehicle movements.

Corridor g=(a,b) is activated when both zones a and b have active stations,
regardless of whether any trip actually crosses from zone a to zone b.

# Fields
- `k::Int`: Number of stations to activate per scenario (second stage)
- `l::Int`: Number of stations to build (first stage)
- `in_vehicle_time_weight::Float64`: Weight λ for in-vehicle travel time costs
- `corridor_weight::Float64`: Weight γ for corridor penalty
- `max_cluster_diameter::Float64`: Maximum routing distance diameter for clustering
- `use_walking_distance_limit::Bool`: Whether to enforce a walking distance limit
- `max_walking_distance::Union{Float64, Nothing}`: Maximum walking distance
- `variable_reduction::Bool`: Whether to reduce assignment variables when walking limit is enabled
- `tight_constraints::Bool`: Whether to use tighter assignment-to-active constraints

# Additional variables
- α[a,s] ∈ [0,1]: cluster activation indicator (continuous)
- f[g,s] ∈ {0,1}: corridor usage indicator

# Additional constraints
- |C_a|·α_{as} ≥ Σ_{i∈C_a} z_{is}       (cluster activation)
- f_{gs} ≥ α_{as} + α_{bs} - 1  for a≠b  (corridor activation)
- f_{gs} ≥ α_{as}               for a==b  (self-corridor activation)
"""
struct ZCorridorODModel <: AbstractCorridorODModel
    k::Int
    l::Int
    in_vehicle_time_weight::Float64
    corridor_weight::Float64
    max_cluster_diameter::Float64
    use_walking_distance_limit::Bool
    max_walking_distance::Union{Float64, Nothing}
    variable_reduction::Bool
    tight_constraints::Bool

    function ZCorridorODModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            corridor_weight::Number=1.0,
            max_cluster_diameter::Number=1000.0,
            use_walking_distance_limit::Bool=false,
            max_walking_distance::Union{Number, Nothing}=nothing,
            variable_reduction::Bool=true,
            tight_constraints::Bool=true
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        corridor_weight >= 0 || throw(ArgumentError("corridor_weight must be non-negative"))
        max_cluster_diameter > 0 || throw(ArgumentError("max_cluster_diameter must be positive"))

        if use_walking_distance_limit
            isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided when walking distance limit is enabled"))
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))

            new(k, l, Float64(in_vehicle_time_weight), Float64(corridor_weight),
                Float64(max_cluster_diameter), true,
                Float64(max_walking_distance), variable_reduction, tight_constraints)
        else
            new(k, l, Float64(in_vehicle_time_weight), Float64(corridor_weight),
                Float64(max_cluster_diameter), false,
                nothing, variable_reduction, tight_constraints)
        end
    end
end
