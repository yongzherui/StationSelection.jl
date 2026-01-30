"""
Clustering OD mapping for ClusteringTwoStageODModel.

This module provides data structures for mapping scenarios to origin-destination pairs
for the clustering two-stage optimization (without time dimension).
"""

using DataFrames

export ClusteringScenarioODMap
export create_clustering_scenario_od_map

"""
    ClusteringScenarioODMap

Maps scenarios to origin-destination pairs for clustering optimization.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Reference to scenario data
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label
- `Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}`: Maps scenario → OD pairs with positive demand
- `Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}`: Demand count q_{od,s} per OD pair per scenario
- `max_walking_distance::Union{Float64, Nothing}`: Maximum walking distance constraint (optional)
- `valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}`: Maps OD pair (o,d) → valid (j,k) station pairs
"""
struct ClusteringScenarioODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    # Omega[scenario_id] = [(o1, d1), (o2, d2), ...]
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}

    # Q[scenario_id][(o, d)] = count of requests for OD pair (o,d)
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}

    # Walking distance constraint
    max_walking_distance::Union{Float64, Nothing}
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
end


"""
    compute_scenario_od_count(scenario_data::ScenarioData) -> Dict{Tuple{Int, Int}, Int}

Compute OD pair demand counts for a single scenario (aggregated across all times).

# Arguments
- `scenario_data::ScenarioData`: Scenario containing requests

# Returns
- Dict mapping (origin, destination) → count
"""
function compute_scenario_od_count(scenario_data::ScenarioData)::Dict{Tuple{Int, Int}, Int}
    od_count = Dict{Tuple{Int, Int}, Int}()

    for row in eachrow(scenario_data.requests)
        o = row.start_station_id
        d = row.end_station_id
        od_pair = (o, d)
        od_count[od_pair] = get(od_count, od_pair, 0) + 1
    end

    return od_count
end


"""
    create_clustering_scenario_od_map(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData
    ) -> ClusteringScenarioODMap

Create a clustering scenario OD map with OD pairs organized by scenario.
"""
function create_clustering_scenario_od_map(
    model::ClusteringTwoStageODModel,
    data::StationSelectionData
)::ClusteringScenarioODMap

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    # Compute Omega_s and Q_s for all scenarios
    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        # Get OD pair counts (aggregated across all times)
        od_count = compute_scenario_od_count(scenario_data)

        # Store unique OD pairs and counts
        Omega_s[scenario_id] = collect(keys(od_count))
        Q_s[scenario_id] = od_count
        union!(all_od_pairs, Omega_s[scenario_id])
    end

    max_walking_distance = model.use_walking_distance_limit ? model.max_walking_distance : nothing
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    if model.use_walking_distance_limit
        valid_jk_pairs = compute_valid_jk_pairs(
            all_od_pairs,
            data,
            station_id_to_array_idx,
            array_idx_to_station_id,
            model.max_walking_distance
        )
    end

    return ClusteringScenarioODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        max_walking_distance,
        valid_jk_pairs
    )
end

"""
    has_walking_distance_limit(mapping::ClusteringScenarioODMap) -> Bool

Check if the mapping has valid (j, k) pairs computed based on walking distance limits.
"""
has_walking_distance_limit(mapping::ClusteringScenarioODMap) = !isnothing(mapping.max_walking_distance)

"""
    get_valid_jk_pairs(mapping::ClusteringScenarioODMap, o::Int, d::Int) -> Vector{Tuple{Int, Int}}

Get the valid (j, k) station pairs for an OD pair. Returns all pairs if no walking limit is set.
"""
function get_valid_jk_pairs(mapping::ClusteringScenarioODMap, o::Int, d::Int)
    if has_walking_distance_limit(mapping)
        return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
    else
        n = length(mapping.array_idx_to_station_id)
        return [(j, k) for j in 1:n for k in 1:n]
    end
end
