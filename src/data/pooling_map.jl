"""
Pooling scenario origin-destination time mapping for TwoStageSingleDetourModel.

This module provides data structures for mapping scenarios and time windows
to origin-destination pairs for pooling optimization.

Note: Detour combinations (Xi) are computed separately via find_detour_combinations,
find_same_source_detour_combinations, and find_same_dest_detour_combinations
in detour_combinations.jl.
"""

using DataFrames
using Dates

export PoolingScenarioOriginDestTimeMap
export create_pooling_scenario_origin_dest_time_map
export create_station_id_mappings, create_scenario_label_mappings
export compute_time_to_od_mapping


"""
    PoolingScenarioOriginDestTimeMap

Maps scenarios, time windows, and origin-destination pairs for pooling optimization.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Reference to scenario data
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label
- `time_window::Int`: Time discretization window in seconds
- `Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}`: Maps (scenario, time) → OD pairs
"""
struct PoolingScenarioOriginDestTimeMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    time_window::Int

    # Omega[scenario_id][time_id] = [(o1, d1), (o2, d2), ...]
    Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}
end


"""
    create_station_id_mappings(station_ids::Vector{Int}) -> (Dict{Int, Int}, Vector{Int})

Create bidirectional mappings between station IDs and array indices.

# Returns
- Tuple of (id_to_idx::Dict, idx_to_id::Vector)
"""
function create_station_id_mappings(station_ids::Vector{Int})
    station_id_to_array_idx = Dict{Int, Int}()
    array_idx_to_station_id = Vector{Int}()

    for (idx, station_id) in enumerate(station_ids)
        station_id_to_array_idx[station_id] = idx
        push!(array_idx_to_station_id, station_id)
    end

    return station_id_to_array_idx, array_idx_to_station_id
end


"""
    create_scenario_label_mappings(scenarios::Vector{ScenarioData}) -> (Dict{String, Int}, Vector{String})

Create bidirectional mappings between scenario labels and array indices.

# Returns
- Tuple of (label_to_idx::Dict, idx_to_label::Vector)
"""
function create_scenario_label_mappings(scenarios::Vector{ScenarioData})
    scenario_label_to_array_idx = Dict{String, Int}()
    array_idx_to_scenario_label = Vector{String}()

    for (idx, scenario) in enumerate(scenarios)
        scenario_label_to_array_idx[scenario.label] = idx
        push!(array_idx_to_scenario_label, scenario.label)
    end

    return scenario_label_to_array_idx, array_idx_to_scenario_label
end


"""
    compute_time_to_od_mapping(
        scenario_data::ScenarioData,
        time_window::Int;
        time_column::Symbol=:request_time
    ) -> Dict{Int, Set{Tuple{Int, Int}}}

Compute mapping from time IDs to origin-destination pairs for a single scenario.

The time_id is computed as floor((request_time - scenario_start_time) / time_window).

# Arguments
- `scenario_data::ScenarioData`: Scenario containing requests
- `time_window::Int`: Time discretization window in seconds
- `time_column::Symbol`: Column name for request time (default :request_time)

# Returns
- Dict mapping time_id → Set of (origin, destination) tuples
"""
function compute_time_to_od_mapping(
    scenario_data::ScenarioData,
    time_window::Int;
    time_column::Symbol=:request_time
)::Dict{Int, Set{Tuple{Int, Int}}}

    time_to_od_set = Dict{Int, Set{Tuple{Int, Int}}}()

    scenario_start_time = scenario_data.start_time
    if isnothing(scenario_start_time)
        error("scenario_start_time cannot be nothing for time-based OD mapping")
    end

    for row in eachrow(scenario_data.requests)
        o = row.start_station_id
        d = row.end_station_id

        # Get the request time
        request_time = row[time_column]

        # Handle both DateTime and String types
        if request_time isa String
            request_time = DateTime(request_time, "yyyy-mm-dd HH:MM:SS")
        end

        # Compute time relative to scenario start in seconds
        time_diff_seconds = (request_time - scenario_start_time) / Dates.Second(1)
        time_id = floor(Int, time_diff_seconds / time_window)

        if !haskey(time_to_od_set, time_id)
            time_to_od_set[time_id] = Set{Tuple{Int, Int}}()
        end

        push!(time_to_od_set[time_id], (o, d))
    end

    return time_to_od_set
end


"""
    create_pooling_scenario_origin_dest_time_map(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData
    ) -> PoolingScenarioOriginDestTimeMap

Create a pooling scenario map with OD pairs organized by scenario and time.

Note: Detour combinations (Xi) should be computed separately using:
- find_same_source_detour_combinations(model, data) -> Vector{Tuple{Int,Int,Int}}
- find_same_dest_detour_combinations(model, data) -> Vector{Tuple{Int,Int,Int,Int}}
"""
function create_pooling_scenario_origin_dest_time_map(
    model::TwoStageSingleDetourModel,
    data::StationSelectionData
)::PoolingScenarioOriginDestTimeMap

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    time_window = floor(Int, model.time_window)

    # Compute Omega_s_t for all scenarios
    Omega_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        time_to_od_set = compute_time_to_od_mapping(scenario_data, time_window)

        # Convert Sets to Vectors for consistent interface
        Omega_s_t[scenario_id] = Dict{Int, Vector{Tuple{Int, Int}}}()
        for (time_id, od_set) in time_to_od_set
            Omega_s_t[scenario_id][time_id] = collect(od_set)
        end
    end

    return PoolingScenarioOriginDestTimeMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        time_window,
        Omega_s_t
    )
end
