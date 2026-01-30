"""
Pooling scenario origin-destination time mapping for TwoStageSingleDetourNoWalkingLimitModel.

This is the original mapping without walking distance constraints - all (j,k) station
pairs are considered valid for assignment.
"""

using DataFrames
using Dates

export PoolingScenarioOriginDestTimeMapNoWalkingLimit
export create_pooling_scenario_origin_dest_time_map_no_walking_limit


"""
    PoolingScenarioOriginDestTimeMapNoWalkingLimit

Maps scenarios, time windows, and origin-destination pairs for pooling optimization.
This version does not have walking distance constraints.

# Fields
- `station_id_to_array_idx::Dict{Int, Int}`: Station ID → array index mapping
- `array_idx_to_station_id::Vector{Int}`: Array index → station ID mapping
- `scenarios::Vector{ScenarioData}`: Reference to scenario data
- `scenario_label_to_array_idx::Dict{String, Int}`: Scenario label → array index
- `array_idx_to_scenario_label::Vector{String}`: Array index → scenario label
- `time_window::Int`: Time discretization window in seconds
- `Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}`: Maps (scenario, time) → OD pairs
- `Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}`: Demand count q_{od,s,t}
"""
struct PoolingScenarioOriginDestTimeMapNoWalkingLimit
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    time_window::Int

    # Omega[scenario_id][time_id] = [(o1, d1), (o2, d2), ...]
    Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}

    # Q[scenario_id][time_id][(o, d)] = count of requests for OD pair (o,d)
    Q_s_t::Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}
end


"""
    create_pooling_scenario_origin_dest_time_map_no_walking_limit(
        model::TwoStageSingleDetourNoWalkingLimitModel,
        data::StationSelectionData
    ) -> PoolingScenarioOriginDestTimeMapNoWalkingLimit

Create a pooling scenario map with OD pairs organized by scenario and time.
No walking distance constraints are applied.
"""
function create_pooling_scenario_origin_dest_time_map_no_walking_limit(
    model::TwoStageSingleDetourNoWalkingLimitModel,
    data::StationSelectionData
)::PoolingScenarioOriginDestTimeMapNoWalkingLimit

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    time_window = floor(Int, model.time_window)

    # Compute Omega_s_t and Q_s_t for all scenarios
    Omega_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    Q_s_t = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, Int}}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        # Get OD pair counts (single pass through data)
        time_to_od_count = compute_time_to_od_count_mapping(scenario_data, time_window)

        # Derive unique OD pairs from count dictionary keys
        Omega_s_t[scenario_id] = Dict{Int, Vector{Tuple{Int, Int}}}()
        for (time_id, od_count_dict) in time_to_od_count
            Omega_s_t[scenario_id][time_id] = collect(keys(od_count_dict))
        end

        # Store the counts
        Q_s_t[scenario_id] = time_to_od_count
    end

    return PoolingScenarioOriginDestTimeMapNoWalkingLimit(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        time_window,
        Omega_s_t,
        Q_s_t
    )
end
