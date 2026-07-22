"""
ClusteringModel - Station selection with clustering-style assignment.

Three sub-policies capture the previously-separate clustering models:
- `SingleStagePolicy`: single-scenario k-medoids, station-to-station assignment
- `TwoStagePolicy`: two-stage (build/activate) station-to-station assignment
- `TwoStageODPolicy`: two-stage (build/activate) OD pickup/dropoff assignment
"""

export AbstractClusteringPolicy
export SingleStagePolicy, TwoStagePolicy, TwoStageODPolicy
export ClusteringModel

abstract type AbstractClusteringPolicy end

"""
    SingleStagePolicy <: AbstractClusteringPolicy

Basic k-medoids clustering policy for station selection.

Selects k stations to minimize total walking distance from request
origins and destinations to their nearest selected station.

# Fields
- `k::Int`: Number of stations to select
- `max_walking_distance::Union{Float64, Nothing}`: Optional assignment-radius
  limit. When provided, station i may only be assigned to station j if walking
  cost d(i,j) is within this threshold.

# Decision Variables
- `y[j]`: Binary, 1 if station j is selected as a medoid
- `x[i,j]`: Binary, 1 if request point i is assigned to station j

# Objective
Minimize total weighted walking cost:
    min Σᵢ Σ_{j ∈ Aᵢ} qᵢ · d(i,j) · x[i,j]

where qᵢ is the request count at station location i. Here Aᵢ is the set of
admissible cluster centers for demand point i; with a walking limit it is
{j : d(i,j) ≤ mwd}, otherwise all candidate stations.

# Constraints
- Each station location assigned to exactly one medoid
- Can only assign to selected stations
- Select exactly k stations
"""
struct SingleStagePolicy <: AbstractClusteringPolicy
    k::Int
    max_walking_distance::Union{Float64, Nothing}

    function SingleStagePolicy(
            k::Int;
            max_walking_distance::Union{Number, Nothing}=nothing
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        if !isnothing(max_walking_distance)
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))
        end
        new(k, isnothing(max_walking_distance) ? nothing : Float64(max_walking_distance))
    end
end

"""
    TwoStagePolicy <: AbstractClusteringPolicy

Two-stage stochastic station selection policy with station-to-station clustering
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

This is a scenario-wise p-median / k-medoids style clustering policy with a
first-stage build decision shared across scenarios.
"""
struct TwoStagePolicy <: AbstractClusteringPolicy
    k::Int
    l::Int
    max_walking_distance::Union{Float64, Nothing}

    function TwoStagePolicy(
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

"""
    TwoStageODPolicy <: AbstractClusteringPolicy

Two-stage stochastic station selection policy with OD pair assignments.

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
struct TwoStageODPolicy <: AbstractClusteringPolicy
    k::Int              # Number of active stations per scenario
    l::Int               # Number of stations to build
    in_vehicle_time_weight::Float64  # Weight for in-vehicle travel time costs (w_ivt)
    max_walking_distance::Float64
    flow_regularization_weight::Union{Float64, Nothing}

    function TwoStageODPolicy(
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

"""
    ClusteringModel <: AbstractStationSelectionModel

Station-selection clustering model. Behavior is selected entirely by `policy`:
- `SingleStagePolicy`: single-scenario k-medoids, station-to-station assignment
- `TwoStagePolicy`: two-stage (build/activate) station-to-station assignment
- `TwoStageODPolicy`: two-stage (build/activate) OD pickup/dropoff assignment

Field access forwards to `policy` (e.g. `model.k`, `model.max_walking_distance`)
so callers don't need to unwrap `model.policy` themselves. Accessing a field that
doesn't exist for the active policy (e.g. `.l` under `SingleStagePolicy`) raises
Julia's normal "no field" error.
"""
struct ClusteringModel <: AbstractStationSelectionModel
    policy::AbstractClusteringPolicy
end

Base.getproperty(m::ClusteringModel, name::Symbol) =
    name === :policy ? getfield(m, :policy) : getproperty(getfield(m, :policy), name)
