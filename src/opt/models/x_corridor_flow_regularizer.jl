"""
XCorridorWithFlowRegularizerModel - XCorridorODModel with route-activation regularization.

Extends XCorridorODModel by adding a penalty on the number of distinct (j,k)
route segments used per scenario. This encourages multiple OD pairs to share the
same pickup→dropoff station pair, promoting vehicle pooling at the route level.
"""

export XCorridorWithFlowRegularizerModel

"""
    XCorridorWithFlowRegularizerModel <: AbstractCorridorODModel

Two-stage stochastic station selection model with OD pair assignments,
x-based corridor penalties, and route-activation regularization.

Extends XCorridorODModel with an additional penalty term:
    + μ Σ_s Σ_{(j,k)} w_route[s][(j,k)]

where w_route[s][(j,k)] ∈ [0,1] indicates that route (j→k) is used by at least
one OD assignment in scenario s.

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
- `max_active_per_zone::Union{Int, Nothing}`: Maximum active stations per cluster per scenario (nothing = disabled)
- `flow_regularization_weight::Float64`: Weight μ for route-activation regularization penalty

# Additional variables
- f[g,s] ∈ {0,1}: corridor usage indicator
- w_route[s][(j,k)] ∈ [0,1]: sparse route activation indicator

# Additional constraints
- f_{gs} ≥ x_{odjks}  ∀(o,d), j∈C_a, k∈C_b, s  for g=(a,b)
- w_route[s][(j,k)] ≥ x[s][od_idx][...] for all (o,d) in Ω_s and valid (j,k)
"""
struct XCorridorWithFlowRegularizerModel <: AbstractCorridorODModel
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
    max_active_per_zone::Union{Int, Nothing}
    flow_regularization_weight::Float64

    function XCorridorWithFlowRegularizerModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            corridor_weight::Number=1.0,
            max_cluster_diameter::Union{Number, Nothing}=nothing,
            n_clusters::Union{Int, Nothing}=nothing,
            use_walking_distance_limit::Bool=false,
            max_walking_distance::Union{Number, Nothing}=nothing,
            variable_reduction::Bool=true,
            tight_constraints::Bool=true,
            max_active_per_zone::Union{Int, Nothing}=nothing,
            flow_regularization_weight::Number=1.0
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        corridor_weight >= 0 || throw(ArgumentError("corridor_weight must be non-negative"))
        flow_regularization_weight >= 0 || throw(ArgumentError("flow_regularization_weight must be non-negative"))

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

        if !isnothing(max_active_per_zone)
            max_active_per_zone >= 1 ||
                throw(ArgumentError("max_active_per_zone must be >= 1"))
            # Early feasibility check when n_clusters is known at construction time
            if !isnothing(n_clusters)
                max_active_per_zone * n_clusters >= k ||
                    throw(ArgumentError(
                        "max_active_per_zone ($max_active_per_zone) * n_clusters ($n_clusters)" *
                        " = $(max_active_per_zone * n_clusters) < k ($k): infeasible"
                    ))
            end
        end

        if use_walking_distance_limit
            isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided when walking distance limit is enabled"))
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))

            new(k, l, Float64(in_vehicle_time_weight), Float64(corridor_weight),
                mcd, n_clusters, true,
                Float64(max_walking_distance), variable_reduction, tight_constraints,
                max_active_per_zone, Float64(flow_regularization_weight))
        else
            new(k, l, Float64(in_vehicle_time_weight), Float64(corridor_weight),
                mcd, n_clusters, false,
                nothing, variable_reduction, tight_constraints,
                max_active_per_zone, Float64(flow_regularization_weight))
        end
    end
end
