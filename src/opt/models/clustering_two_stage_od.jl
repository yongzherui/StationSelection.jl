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
- `routing_weight::Float64`: Weight λ for routing costs in objective

# Mathematical Formulation
First stage: Select l stations to build (y[j] ∈ {0,1})
Second stage: For each scenario s, activate k stations (z[j,s] ∈ {0,1})
              and assign OD pairs to station pairs (x[s][od][j,k] ∈ {0,1})

Objective:
    min Σ_s Σ_{(o,d)∈Ω_s} Σ_{j,k} q_{od,s} (d^origin_{oj} + d^dest_{dk} + λ·c_{jk}) x_{od,jk,s}

Constraints:
- Σ_j y[j] = l                              (build exactly l stations)
- Σ_j z[j,s] = k  ∀s                        (activate k stations per scenario)
- z[j,s] ≤ y[j]  ∀j,s                       (can only activate built stations)
- Σ_{j,k} x[s][od][j,k] = 1  ∀s,od          (each OD pair assigned to one station pair)
- 2·x[s][od][j,k] ≤ z[j,s] + z[k,s]  ∀s,od,j,k  (can only use active stations)
"""
struct ClusteringTwoStageODModel <: AbstractODModel
    k::Int              # Number of active stations per scenario
    l::Int              # Number of stations to build
    routing_weight::Float64  # Weight for routing costs (λ)

    function ClusteringTwoStageODModel(k::Int, l::Int, routing_weight::Float64=1.0)
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        routing_weight > 0 || throw(ArgumentError("routing_weight must be positive"))
        new(k, l, routing_weight)
    end
end
