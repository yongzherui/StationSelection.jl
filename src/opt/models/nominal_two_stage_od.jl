"""
NominalTwoStageODModel - Two-stage nominal station selection with demand averaging.

Uses period-aggregated scenarios (4 periods, demand averaged over all days in the
optimization window). Assignment variables x are binary; mean daily demand q_{ods}
enters as an objective coefficient rather than a variable upper bound.
"""

export NominalTwoStageODModel
export SmoothedNominalTwoStageODModel

"""
    NominalTwoStageODModel <: AbstractODModel

Two-stage nominal station selection model with OD pair assignments and averaged demand.

# Fields
- `k::Int`: Number of stations to activate per scenario (second stage)
- `l::Int`: Number of stations to build (first stage)
- `in_vehicle_time_weight::Float64`: Weight λ for in-vehicle routing cost c_{jk}
- `max_walking_distance::Float64`: Walking-distance filter for valid (j,k) pairs

# Mathematical Formulation
First stage: Select l stations to build (y[j] ∈ {0,1})
Second stage: For each scenario s, activate k stations (z[j,s] ∈ {0,1})
              and select one station pair per OD pair (x[s][od][idx] ∈ {0,1})

Objective:
    min Σ_s Σ_{(o,d)∈Ω_s} q_{ods} · Σ_{(j,k)∈A_od} (walk_{oj} + walk_{kd} + λ·route_{jk}) · x_{od,jk,s}

Constraints:
- Σ_j y[j] = l                               (build exactly l stations)
- Σ_j z[j,s] = k  ∀s                         (activate k stations per scenario)
- z[j,s] ≤ y[j]  ∀j,s                        (can only activate built stations)
- Σ_{(j,k)∈A_od} x[s][od][j,k] = 1  ∀s,od   (assign each OD to exactly one pair)
- x[s][od][j,k] ≤ z[j,s]  ∀s,od,(j,k)
- x[s][od][j,k] ≤ z[k,s]  ∀s,od,(j,k)
"""
struct NominalTwoStageODModel <: AbstractODModel
    k::Int
    l::Int
    in_vehicle_time_weight::Float64
    max_walking_distance::Float64

    function NominalTwoStageODModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            max_walking_distance::Union{Number, Nothing}=300,
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided"))
        max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))

        new(k, l, Float64(in_vehicle_time_weight), Float64(max_walking_distance))
    end
end


"""
    SmoothedNominalTwoStageODModel <: AbstractODModel

Two-stage nominal station selection model with smoothed OD demand.

This variant assigns positive mean demand to every walk-feasible OD pair in each
scenario by shrinking sparse empirical means toward a gravity-style prior:

    q_{ods} = n/(n+τ) * q̄_{ods} + τ/(n+τ) * q̃_{ods}

where q̄ is the empirical mean daily demand, n is the number of historical days
in which the OD pair was active in that scenario, and q̃ is a strictly positive
gravity prior with a small uniform mixture.
"""
struct SmoothedNominalTwoStageODModel <: AbstractODModel
    k::Int
    l::Int
    in_vehicle_time_weight::Float64
    max_walking_distance::Float64
    smoothing_tau::Float64
    pseudo_demand_fraction::Float64
    gravity_uniform_mix::Float64

    function SmoothedNominalTwoStageODModel(
            k::Int,
            l::Int;
            in_vehicle_time_weight::Number=1.0,
            max_walking_distance::Union{Number, Nothing}=300,
            smoothing_tau::Number=5.0,
            pseudo_demand_fraction::Number=0.02,
            gravity_uniform_mix::Number=0.05,
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        in_vehicle_time_weight >= 0 || throw(ArgumentError("in_vehicle_time_weight must be non-negative"))
        isnothing(max_walking_distance) && throw(ArgumentError("max_walking_distance must be provided"))
        max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))
        smoothing_tau > 0 || throw(ArgumentError("smoothing_tau must be positive"))
        0 <= pseudo_demand_fraction || throw(ArgumentError("pseudo_demand_fraction must be non-negative"))
        0 <= gravity_uniform_mix <= 1 || throw(ArgumentError("gravity_uniform_mix must lie in [0,1]"))

        new(
            k,
            l,
            Float64(in_vehicle_time_weight),
            Float64(max_walking_distance),
            Float64(smoothing_tau),
            Float64(pseudo_demand_fraction),
            Float64(gravity_uniform_mix),
        )
    end
end
