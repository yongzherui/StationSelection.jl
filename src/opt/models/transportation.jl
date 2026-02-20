"""
TransportationModel - Two-stage station selection with zone-pair anchor transportation flow.

Separates pickup and dropoff assignments (x_pick, x_drop), uses ordered zone-pair
anchors (g = (zone_a, zone_b)) to capture movement directionality, and models
per-passenger transportation flow (f) on allowed station pairs within each anchor.

Unlike corridor models that use joint (j,k) assignment, this model independently
assigns origins to pickup stations and destinations to dropoff stations, then links
them through flow conservation constraints within each anchor.
"""

export TransportationModel

"""
    TransportationModel <: AbstractTransportationModel

Two-stage stochastic station selection model with zone-pair anchor transportation flow.

# Fields
- `k::Int`: Number of stations to activate per scenario (second stage)
- `l::Int`: Number of stations to build (first stage)
- `in_vehicle_time_weight::Float64`: Weight on routing flow cost
- `activation_cost::Float64`: Fixed cost per active anchor per scenario
- `max_cluster_diameter::Union{Float64, Nothing}`: Maximum routing distance diameter for zone clustering (diameter mode)
- `n_clusters::Union{Int, Nothing}`: Fixed number of clusters (count mode)
- `use_walking_distance_limit::Bool`: Whether to enforce a walking distance limit
- `max_walking_distance::Union{Float64, Nothing}`: Maximum walking distance

# Variables
- `y[j]` in {0,1}: station j is built
- `z[j,s]` in {0,1}: station j is active in scenario s
- `x_pick[i,j,g,s]` in {0,1}: origin i picks up at station j in anchor g, scenario s
- `x_drop[i,k,g,s]` in {0,1}: destination i drops off at station k in anchor g, scenario s
- `p[j,g,s]` >= 0: total pickups at station j for anchor g, scenario s
- `d[k,g,s]` >= 0: total dropoffs at station k for anchor g, scenario s
- `f[j,k,g,s]` >= 0: flow from station j to station k for anchor g, scenario s
- `u[g,s]` in {0,1}: anchor g is used in scenario s

# Constraints
1. Assignment: each origin/dest assigned to exactly one station per anchor/scenario
2. Aggregation: p = sum(x_pick), d = sum(x_drop)
3. Flow conservation: sum_k f[j,k,g,s] = p[j,g,s], sum_j f[j,k,g,s] = d[k,g,s]
4. Flow activation: f[j,k,g,s] <= M * u[g,s] for (j,k) in P(g)
5. Viability: x_pick <= z, x_drop <= z
6. Station limit: sum(y) = l
7. Activation limit: sum(z[:,s]) = k for all s
8. Linking: z[j,s] <= y[j]
"""
struct TransportationModel <: AbstractTransportationModel
    k::Int
    l::Int
    in_vehicle_time_weight::Float64
    activation_cost::Float64
    max_cluster_diameter::Union{Float64, Nothing}
    n_clusters::Union{Int, Nothing}
    use_walking_distance_limit::Bool
    max_walking_distance::Union{Float64, Nothing}

    function TransportationModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            activation_cost::Number=0.0,
            max_cluster_diameter::Union{Number, Nothing}=nothing,
            n_clusters::Union{Int, Nothing}=nothing,
            use_walking_distance_limit::Bool=false,
            max_walking_distance::Union{Number, Nothing}=nothing
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        activation_cost >= 0 || throw(ArgumentError("activation_cost must be non-negative"))

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

            new(k, l, Float64(in_vehicle_time_weight), Float64(activation_cost),
                mcd, n_clusters, true, Float64(max_walking_distance))
        else
            new(k, l, Float64(in_vehicle_time_weight), Float64(activation_cost),
                mcd, n_clusters, false, nothing)
        end
    end
end
