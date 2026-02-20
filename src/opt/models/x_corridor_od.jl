"""
XCorridorODModel - Two-stage station selection with x-based corridor penalties.

Like ZCorridorODModel but corridor activation is based on assignment variables (x)
rather than station activation variables (z). A corridor is only activated when
an actual OD assignment crosses it.
"""

export XCorridorODModel

"""
    XCorridorODModel <: AbstractCorridorODModel

Two-stage stochastic station selection model with OD pair assignments and
x-based corridor penalties for cross-zone vehicle movements.

Corridor g=(a,b) is activated only when some OD pair is assigned to a pickup
station in cluster a and a dropoff station in cluster b.

# Fields
- `k::Int`: Number of stations to activate per scenario (second stage)
- `l::Int`: Number of stations to build (first stage)
- `in_vehicle_time_weight::Float64`: Weight λ for in-vehicle travel time costs
- `corridor_weight::Float64`: Weight γ for corridor penalty
- `max_cluster_diameter::Union{Float64, Nothing}`: Maximum routing distance diameter for clustering (diameter mode)
- `n_clusters::Union{Int, Nothing}`: Fixed number of clusters (count mode)
- `use_walking_distance_limit::Bool`: Whether to enforce a walking distance limit
- `max_walking_distance::Union{Float64, Nothing}`: Maximum walking distance
- `variable_reduction::Bool`: Whether to reduce assignment variables when walking limit is enabled
- `tight_constraints::Bool`: Whether to use tighter assignment-to-active constraints

# Additional variables
- f[g,s] ∈ {0,1}: corridor usage indicator (no α variables needed)

# Additional constraints
- f_{gs} ≥ x_{odjks}  ∀(o,d), j∈C_a, k∈C_b, s  for g=(a,b)
"""
struct XCorridorODModel <: AbstractCorridorODModel
    k::Int
    l::Int
    in_vehicle_time_weight::Float64
    corridor_weight::Float64
    max_cluster_diameter::Union{Float64, Nothing}
    n_clusters::Union{Int, Nothing}
    use_walking_distance_limit::Bool
    max_walking_distance::Union{Float64, Nothing}
    variable_reduction::Bool
    tight_constraints::Bool

    function XCorridorODModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            corridor_weight::Number=1.0,
            max_cluster_diameter::Union{Number, Nothing}=nothing,
            n_clusters::Union{Int, Nothing}=nothing,
            use_walking_distance_limit::Bool=false,
            max_walking_distance::Union{Number, Nothing}=nothing,
            variable_reduction::Bool=true,
            tight_constraints::Bool=true
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        corridor_weight >= 0 || throw(ArgumentError("corridor_weight must be non-negative"))

        # Clustering mode: exactly one of max_cluster_diameter or n_clusters
        if !isnothing(max_cluster_diameter) && !isnothing(n_clusters)
            throw(ArgumentError("Cannot specify both max_cluster_diameter and n_clusters"))
        end
        if isnothing(max_cluster_diameter) && isnothing(n_clusters)
            max_cluster_diameter = 1000.0  # backward-compatible default
        end
        if !isnothing(max_cluster_diameter)
            max_cluster_diameter > 0 || throw(ArgumentError("max_cluster_diameter must be positive"))
        end
        if !isnothing(n_clusters)
            n_clusters > 0 || throw(ArgumentError("n_clusters must be positive"))
        end

        mcd = isnothing(max_cluster_diameter) ? nothing : Float64(max_cluster_diameter)

        if use_walking_distance_limit
            isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided when walking distance limit is enabled"))
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))

            new(k, l, Float64(in_vehicle_time_weight), Float64(corridor_weight),
                mcd, n_clusters, true,
                Float64(max_walking_distance), variable_reduction, tight_constraints)
        else
            new(k, l, Float64(in_vehicle_time_weight), Float64(corridor_weight),
                mcd, n_clusters, false,
                nothing, variable_reduction, tight_constraints)
        end
    end
end
