"""
Clustering map for TwoStagePolicy.

This module aggregates scenario demand to endpoint counts q_{is} and enumerates
admissible i→j assignments for each demand station i.
"""

using DataFrames

export ClusteringTwoStageStationMap
export create_clustering_two_stage_station_map
export get_valid_j_assignments

struct ClusteringTwoStageStationMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    I_s::Dict{Int, Vector{Int}}
    q_s::Dict{Int, Dict{Int, Int}}

    max_walking_distance::Union{Float64, Nothing}
    valid_j_assignments::Dict{Int, Vector{Int}}

    n_stations::Int
end

function compute_scenario_endpoint_count(
    scenario_data::ScenarioData,
    n_stations::Int
)::Dict{Int, Int}
    counts = Dict(i => 0 for i in 1:n_stations)
    _require_indexed_request_columns(scenario_data.requests)

    for row in eachrow(scenario_data.requests)
        counts[row.origin_idx] += 1
        counts[row.dest_idx] += 1
    end

    return counts
end

function compute_valid_j_assignments(
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

function create_clustering_two_stage_station_map(
    model::TwoStagePolicy,
    data::StationSelectionData
)::ClusteringTwoStageStationMap
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    I_s = Dict{Int, Vector{Int}}()
    q_s = Dict{Int, Dict{Int, Int}}()
    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        counts = compute_scenario_endpoint_count(scenario_data, data.n_stations)
        q_s[scenario_id] = counts
        I_s[scenario_id] = sort([i for (i, q) in counts if q > 0])
    end

    valid_j_assignments = compute_valid_j_assignments(data, model.max_walking_distance)

    return ClusteringTwoStageStationMap(
        data.station_id_to_array_idx,
        data.array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        I_s,
        q_s,
        model.max_walking_distance,
        valid_j_assignments,
        data.n_stations
    )
end

has_walking_distance_limit(mapping::ClusteringTwoStageStationMap) = !isnothing(mapping.max_walking_distance)

function get_valid_j_assignments(mapping::ClusteringTwoStageStationMap, i::Int)
    return get(mapping.valid_j_assignments, i, Int[])
end
