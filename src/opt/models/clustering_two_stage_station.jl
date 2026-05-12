"""
ClusteringTwoStageStationModel - Two-stage stochastic clustering with i→j assignments.

This model selects l stations to build (first stage), activates k built stations
per scenario (second stage), and assigns each demand station i to one active
station j in that scenario. Unlike ClusteringTwoStageODModel, it ignores OD
direction and does not use pickup/dropoff station pairs.
"""

export ClusteringTwoStageStationModel

"""
    ClusteringTwoStageStationModel <: AbstractTwoStageModel

Two-stage stochastic station selection model with station-to-station clustering
assignments.

# Fields
- `k::Int`: Number of stations to activate per scenario
- `l::Int`: Number of stations to build
- `max_walking_distance::Union{Float64, Nothing}`: Optional assignment-radius
  limit. When provided, station i may only be assigned to station j if walking
  cost d(i,j) is within this threshold.

# Mathematical Formulation
For each scenario s, let q_{is} be the number of request endpoints located at
candidate station i (counting both origins and destinations, aggregated across
time). Decision variables:

- y_j ∈ {0,1}: station j is built in the first stage
- z_{js} ∈ {0,1}: built station j is activated in scenario s
- x_{ijs} ∈ {0,1}: demand point i is assigned to active station j in scenario s

Objective:
    min  Σ_s Σ_i Σ_{j ∈ A_i} q_{is} d_{ij} x_{ijs}

Subject to:
    Σ_j y_j = l
    Σ_j z_{js} = k                     ∀s
    z_{js} ≤ y_j                       ∀j,s
    Σ_{j ∈ A_i} x_{ijs} = 1            ∀i,s with q_{is} > 0
    x_{ijs} ≤ z_{js}                   ∀i,j,s

Here A_i is the set of admissible cluster centers for demand point i; with a
walking limit it is {j : d(i,j) ≤ mwd}, otherwise all candidate stations.

This is a scenario-wise p-median / k-medoids style clustering model with a
first-stage build decision shared across scenarios.
"""
struct ClusteringTwoStageStationModel <: AbstractTwoStageModel
    k::Int
    l::Int
    max_walking_distance::Union{Float64, Nothing}

    function ClusteringTwoStageStationModel(
            k::Int,
            l::Int;
            max_walking_distance::Union{Number, Nothing}=nothing
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        if !isnothing(max_walking_distance)
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))
        end
        new(k, l, isnothing(max_walking_distance) ? nothing : Float64(max_walking_distance))
    end
end
