"""
ClusteringBaseModel - Basic k-medoids style clustering for station selection.

This model treats all scenarios as one aggregated dataset and performs
simple station-to-station assignment (no pickup/dropoff pairs).

The objective is to minimize total walking distance from all request
origins and destinations to their assigned stations.
"""

export ClusteringBaseModel

"""
    ClusteringBaseModel <: AbstractSingleScenarioModel

Basic k-medoids clustering model for station selection.

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
struct ClusteringBaseModel <: AbstractSingleScenarioModel
    k::Int
    max_walking_distance::Union{Float64, Nothing}

    function ClusteringBaseModel(
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
