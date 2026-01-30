"""
Objective function for ClusteringBaseModel.

Contains objective for:
- ClusteringBaseModel: simple walking cost minimization (k-medoids style)
"""

using JuMP

export set_clustering_base_objective!


"""
    set_clustering_base_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseMap
    )

Set the minimization objective for ClusteringBaseModel.

Objective:
    min Σᵢⱼ qᵢ · d(i,j) · x[i,j]

Where:
- qᵢ = request_counts[i] = total pickup + dropoff requests at station location i
- d(i,j) = walking cost from station i to station j
- x[i,j] = 1 if station location i is assigned to medoid station j

This is the classic k-medoids/p-median objective - minimizing total weighted
distance from demand points to their assigned facilities.

# Arguments
- `m::Model`: JuMP model with variables x, y already added
- `data::StationSelectionData`: Problem data with walking_costs
- `mapping::ClusteringBaseMap`: Mapping with request counts
"""
function set_clustering_base_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseMap
    )

    n = mapping.n_stations
    x = m[:x]

    @objective(m, Min,
        sum(
            mapping.request_counts[mapping.array_idx_to_station_id[i]] *
            get_walking_cost(data, mapping.array_idx_to_station_id[i], mapping.array_idx_to_station_id[j]) *
            x[i, j]
            for i in 1:n
            for j in 1:n
        )
    )

    return nothing
end
