"""
Clustering OD mapping for ClusteringTwoStageODModel.

This module provides data structures for mapping scenarios to origin-destination pairs
for the clustering two-stage optimization (without time dimension).
"""

using DataFrames

export ClusteringTwoStageODMap
export create_clustering_two_stage_od_map

"""
    ClusteringTwoStageODMap

Maps scenarios to origin-destination pairs for clustering optimization.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Reference to scenario data
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label
- `Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}`: Maps scenario → OD index pairs with positive demand
- `Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}`: Demand count q_{od,s} per OD index pair per scenario
- `max_walking_distance::Float64`: Maximum walking distance constraint
- `valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}`: Maps OD pair (o,d) → valid (j,k) station pairs
"""
struct ClusteringTwoStageODMap <: AbstractClusteringMap
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
    max_walking_distance::Float64
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
end


"""
    compute_valid_jk_pairs(
        all_od_pairs, data, max_walking_distance
    ) -> Dict{Tuple{Int,Int}, Vector{Tuple{Int,Int}}}

For each OD index pair (origin_idx,dest_idx), compute which station index pairs
(pickup_idx, dropoff_idx) satisfy both walking distance limits.
"""
function compute_valid_jk_pairs(
    all_od_pairs::Set{Tuple{Int, Int}},
    data::StationSelectionData,
    max_walking_distance::Float64
)::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}
    n = data.n_stations
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()

    for (o, d) in all_od_pairs
        pairs = Tuple{Int, Int}[]
        for j in 1:n
            get_walking_cost(data, o, j) <= max_walking_distance || continue
            for k in 1:n
                get_walking_cost(data, k, d) <= max_walking_distance || continue
                push!(pairs, (j, k))
            end
        end
        valid_jk_pairs[(o, d)] = pairs
    end

    return valid_jk_pairs
end


"""
    compute_scenario_od_count(scenario_data::ScenarioData) -> Dict{Tuple{Int, Int}, Int}

Compute OD pair demand counts for a single scenario (aggregated across all times).

# Arguments
- `scenario_data::ScenarioData`: Scenario containing requests

# Returns
- Dict mapping (origin_idx, dest_idx) → count
"""
function compute_scenario_od_count(scenario_data::ScenarioData)::Dict{Tuple{Int, Int}, Int}
    od_count = Dict{Tuple{Int, Int}, Int}()
    _require_indexed_request_columns(scenario_data.requests)

    for row in eachrow(scenario_data.requests)
        o = row.origin_idx
        d = row.dest_idx
        od_pair = (o, d)
        od_count[od_pair] = get(od_count, od_pair, 0) + 1
    end

    return od_count
end


"""
    create_clustering_two_stage_od_map(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData
    ) -> ClusteringTwoStageODMap

Create a clustering scenario OD map with OD pairs organized by scenario.
"""
function create_clustering_two_stage_od_map(
    model::ClusteringTwoStageODModel,
    data::StationSelectionData
)::ClusteringTwoStageODMap

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

    valid_jk_pairs = compute_valid_jk_pairs(
        all_od_pairs,
        data,
        model.max_walking_distance
    )

    return ClusteringTwoStageODMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        model.max_walking_distance,
        valid_jk_pairs
    )
end

"""
    has_walking_distance_limit(mapping::ClusteringTwoStageODMap) -> Bool

Check if the mapping has valid (j, k) pairs computed based on walking distance limits.
"""
has_walking_distance_limit(mapping::ClusteringTwoStageODMap) = true

"""
    get_valid_jk_pairs(mapping::ClusteringTwoStageODMap, o::Int, d::Int) -> Vector{Tuple{Int, Int}}

Get the valid (j, k) station pairs for an OD pair.
"""
function get_valid_jk_pairs(mapping::ClusteringTwoStageODMap, o::Int, d::Int)
    return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
end
