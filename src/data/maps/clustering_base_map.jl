"""
Clustering base mapping for SingleStagePolicy.

This module provides data structures for the basic k-medoids clustering
model that aggregates all scenarios and counts request origins/destinations.
"""

using DataFrames

export ClusteringBaseModelMap
export create_clustering_base_model_map
export get_valid_j_assignments

"""
    ClusteringBaseModelMap

Maps station locations to aggregated request counts for basic clustering.

Request counts include both pickup (origin) and dropoff (destination) requests
at each station location, aggregated across all scenarios.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Original scenarios retained for shared exports
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index mapping
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label mapping
- `request_counts::Dict{Int, Int}`: Station index → total request count (pickups + dropoffs)
- `max_walking_distance::Union{Float64, Nothing}`: Optional assignment-radius limit
- `valid_j_assignments::Dict{Int, Vector{Int}}`: Admissible cluster centers for each i
- `n_stations::Int`: Number of candidate stations
"""
struct ClusteringBaseModelMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}
    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}
    request_counts::Dict{Int, Int}
    max_walking_distance::Union{Float64, Nothing}
    valid_j_assignments::Dict{Int, Vector{Int}}

    n_stations::Int
end


"""
    compute_request_counts(data::StationSelectionData) -> Dict{Int, Int}

Compute aggregated request counts per station location.

Counts both pickup requests (origin_idx) and dropoff requests (dest_idx)
across all scenarios.

# Arguments
- `data::StationSelectionData`: Problem data with scenarios containing requests

# Returns
- Dict mapping station_idx → total count of pickups + dropoffs
"""
function compute_request_counts(data::StationSelectionData)::Dict{Int, Int}
    request_counts = Dict{Int, Int}()

    # Initialize all candidate stations with zero count
    for station_idx in 1:data.n_stations
        request_counts[station_idx] = 0
    end

    # Aggregate across all scenarios
    for scenario in data.scenarios
        _require_indexed_request_columns(scenario.requests)
        for row in eachrow(scenario.requests)
            # Count pickup at origin
            o = row.origin_idx
            request_counts[o] = get(request_counts, o, 0) + 1

            # Count dropoff at destination
            d = row.dest_idx
            request_counts[d] = get(request_counts, d, 0) + 1
        end
    end

    return request_counts
end

function compute_base_valid_j_assignments(
    data::StationSelectionData,
    max_walking_distance::Union{Float64, Nothing}
)::Dict{Int, Vector{Int}}
    valid = Dict{Int, Vector{Int}}()
    n = data.n_stations

    for i in 1:n
        js = Int[]
        for j in 1:n
            if isnothing(max_walking_distance) || get_walking_cost(data, i, j) <= max_walking_distance
                push!(js, j)
            end
        end
        valid[i] = js
    end

    return valid
end


"""
    create_clustering_base_model_map(
        model::SingleStagePolicy,
        data::StationSelectionData
    ) -> ClusteringBaseModelMap

Create a clustering base map with aggregated request counts.

# Arguments
- `model::SingleStagePolicy`: The clustering policy
- `data::StationSelectionData`: Problem data with stations and scenarios

# Returns
- `ClusteringBaseModelMap` with station mappings and request counts
"""
function create_clustering_base_model_map(
    model::SingleStagePolicy,
    data::StationSelectionData
)::ClusteringBaseModelMap

    request_counts = compute_request_counts(data)
    valid_j_assignments = compute_base_valid_j_assignments(data, model.max_walking_distance)
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    return ClusteringBaseModelMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        request_counts,
        model.max_walking_distance,
        valid_j_assignments,
        data.n_stations
    )
end

has_walking_distance_limit(mapping::ClusteringBaseModelMap) = !isnothing(mapping.max_walking_distance)

function get_valid_j_assignments(mapping::ClusteringBaseModelMap, i::Int)
    return get(mapping.valid_j_assignments, i, Int[])
end
