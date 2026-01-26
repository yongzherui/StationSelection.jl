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
"""
struct ClusteringScenarioODMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    # Omega[scenario_id] = [(o1, d1), (o2, d2), ...]
    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}

    # Q[scenario_id][(o, d)] = count of requests for OD pair (o,d)
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}
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

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        # Get OD pair counts (aggregated across all times)
        od_count = compute_scenario_od_count(scenario_data)

        # Store unique OD pairs and counts
        Omega_s[scenario_id] = collect(keys(od_count))
        Q_s[scenario_id] = od_count
    end

    return ClusteringScenarioODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s
    )
end
