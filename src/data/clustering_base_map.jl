"""
Clustering base mapping for ClusteringBaseModel.

This module provides data structures for the basic k-medoids clustering
model that aggregates all scenarios and counts request origins/destinations.
"""

using DataFrames

export ClusteringBaseModelMap
export create_clustering_base_model_map

"""
    ClusteringBaseModelMap

Maps station locations to aggregated request counts for basic clustering.

Request counts include both pickup (origin) and dropoff (destination) requests
at each station location, aggregated across all scenarios.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `request_counts::Dict{Int, Int}`: Station ID → total request count (pickups + dropoffs)
- `n_stations::Int`: Number of candidate stations
"""
struct ClusteringBaseModelMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    # request_counts[station_id] = count of requests (both pickup and dropoff)
    request_counts::Dict{Int, Int}

    n_stations::Int
end


"""
    compute_request_counts(data::StationSelectionData) -> Dict{Int, Int}

Compute aggregated request counts per station location.

Counts both pickup requests (start_station_id) and dropoff requests (end_station_id)
across all scenarios.

# Arguments
- `data::StationSelectionData`: Problem data with scenarios containing requests

# Returns
- Dict mapping station_id → total count of pickups + dropoffs
"""
function compute_request_counts(data::StationSelectionData)::Dict{Int, Int}
    request_counts = Dict{Int, Int}()

    # Initialize all candidate stations with zero count
    for station_id in data.stations.id
        request_counts[station_id] = 0
    end

    # Aggregate across all scenarios
    for scenario in data.scenarios
        for row in eachrow(scenario.requests)
            # Count pickup at origin
            o = row.start_station_id
            request_counts[o] = get(request_counts, o, 0) + 1

            # Count dropoff at destination
            d = row.end_station_id
            request_counts[d] = get(request_counts, d, 0) + 1
        end
    end

    return request_counts
end


"""
    create_clustering_base_model_map(
        model::ClusteringBaseModel,
        data::StationSelectionData
    ) -> ClusteringBaseModelMap

Create a clustering base map with aggregated request counts.

# Arguments
- `model::ClusteringBaseModel`: The clustering model
- `data::StationSelectionData`: Problem data with stations and scenarios

# Returns
- `ClusteringBaseModelMap` with station mappings and request counts
"""
function create_clustering_base_model_map(
    model::ClusteringBaseModel,
    data::StationSelectionData
)::ClusteringBaseModelMap

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Compute request counts (pickups + dropoffs aggregated)
    request_counts = compute_request_counts(data)

    return ClusteringBaseModelMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        request_counts,
        data.n_stations
    )
end
