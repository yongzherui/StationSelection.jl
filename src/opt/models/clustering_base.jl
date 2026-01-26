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

# Decision Variables
- `y[j]`: Binary, 1 if station j is selected as a medoid
- `x[i,j]`: Binary, 1 if request point i is assigned to station j

# Objective
Minimize total weighted walking cost:
    min Σᵢⱼ qᵢ · d(i,j) · x[i,j]

where qᵢ is the request count at station location i.

# Constraints
- Each station location assigned to exactly one medoid
- Can only assign to selected stations
- Select exactly k stations
"""
struct ClusteringBaseModel <: AbstractSingleScenarioModel
    k::Int  # Number of stations to select

    function ClusteringBaseModel(k::Int)
        k > 0 || throw(ArgumentError("k must be positive"))
        new(k)
    end
end
