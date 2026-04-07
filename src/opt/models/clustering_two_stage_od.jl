"""
ClusteringTwoStageODModel - Two-stage stochastic station selection with OD pair assignments.

This model selects l stations to build (first stage) and activates k stations
per scenario (second stage), assigning OD pairs to station pairs.
"""

export ClusteringTwoStageODModel

"""
    ClusteringTwoStageODModel <: AbstractODModel

Two-stage stochastic station selection model with OD pair assignments.

# Fields
- `k::Int`: Number of stations to activate per scenario (second stage)
- `l::Int`: Number of stations to build (first stage)
- `in_vehicle_time_weight::Float64`: Weight for in-vehicle travel time costs (c_{jk})
- `max_walking_distance::Float64`: Maximum walking distance used for sparse valid-pair construction
- `flow_regularization_weight::Union{Float64, Nothing}`: Weight μ for route-activation penalty (optional).
  When set, adds f_flow[s][(j,k)] variables and penalises distinct (j,k) segments weighted by routing time.
  Uses sparse x. Must be ≥ 0.

# Mathematical Formulation
First stage: Select l stations to build (y[j] ∈ {0,1})
Second stage: For each scenario s, activate k stations (z[j,s] ∈ {0,1})
              and assign OD demand counts to valid station pairs (x[s][od][idx] ∈ Z₊)

Objective (base):
    min Σ_s Σ_{(o,d)∈Ω_s} Σ_{(j,k)∈A_od} (d^origin_{oj} + d^dest_{dk} + w_ivt·c_{jk}) x_{od,jk,s}

Objective (with flow regularization):
    + μ Σ_s Σ_{(j,k)} c_{jk} × f_flow[s][(j,k)]

Constraints:
- Σ_j y[j] = l                              (build exactly l stations)
- Σ_j z[j,s] = k  ∀s                        (activate k stations per scenario)
- z[j,s] ≤ y[j]  ∀j,s                       (can only activate built stations)
- Σ_{(j,k)∈A_od} x[s][od][j,k] = Q_s[s][(o,d)]  ∀s,od
- x[s][od][j,k] ≤ Q_s[s][(o,d)] * z[j,s], Q_s[s][(o,d)] * z[k,s]  ∀s,od,(j,k)∈A_od
"""
struct ClusteringTwoStageODModel <: AbstractODModel
    k::Int              # Number of active stations per scenario
    l::Int              # Number of stations to build
    in_vehicle_time_weight::Float64  # Weight for in-vehicle travel time costs (w_ivt)
    max_walking_distance::Float64
    flow_regularization_weight::Union{Float64, Nothing}

    function ClusteringTwoStageODModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            max_walking_distance::Union{Number, Nothing}=300,
            flow_regularization_weight::Union{Number, Nothing}=nothing
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))

        if !isnothing(flow_regularization_weight)
            flow_regularization_weight >= 0 || throw(ArgumentError("flow_regularization_weight must be non-negative"))
        end

        isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided"))
        max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))

        new(k, l, Float64(in_vehicle_time_weight),
            Float64(max_walking_distance),
            isnothing(flow_regularization_weight) ? nothing : Float64(flow_regularization_weight))
    end
end
