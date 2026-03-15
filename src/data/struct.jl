"""
Core data structures for StationSelection optimization models.

This module provides reusable data structures that encapsulate problem data,
making it easy to pass consistent data to different optimization models.
"""

using DataFrames
using Dates

export ScenarioData, StationSelectionData
export create_station_selection_data, create_scenario_data
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, has_routing_costs
export AbstractStationSelectionMap
export AbstractClusteringMap

"""
    AbstractStationSelectionMap

Base abstract type for all station selection mapping structs.
"""
abstract type AbstractStationSelectionMap end

"""
    AbstractClusteringMap <: AbstractStationSelectionMap

Base abstract type for clustering mapping structs.
"""
abstract type AbstractClusteringMap <: AbstractStationSelectionMap end


"""
    ScenarioData

Encapsulates request data for a single scenario (time period).

# Fields
- `label::String`: Human-readable label for the scenario
- `start_time::Union{DateTime, Nothing}`: Start of the scenario time window
- `end_time::Union{DateTime, Nothing}`: End of the scenario time window
- `requests::DataFrame`: Customer requests in this scenario
"""
struct ScenarioData
    label::String
    start_time::Union{DateTime, Nothing}
    end_time::Union{DateTime, Nothing}
    requests::DataFrame
end

"""
    create_scenario_data(requests::DataFrame, label::String;
                         start_time=nothing, end_time=nothing) -> ScenarioData

Create a ScenarioData struct from a DataFrame of requests.
"""
function create_scenario_data(
    requests::DataFrame,
    label::String;
    start_time::Union{DateTime, Nothing}=nothing,
    end_time::Union{DateTime, Nothing}=nothing
)::ScenarioData
    return ScenarioData(label, start_time, end_time, requests)
end


"""
    StationSelectionData

Central data structure containing all problem data for station selection optimization.

This struct encapsulates stations, costs, and scenario data in a format that
can be reused across different optimization models.

# Fields
- `stations::DataFrame`: Station data with columns :id, :lon, :lat
- `n_stations::Int`: Number of candidate stations
- `walking_costs::Dict{Tuple{Int,Int}, Float64}`: Walking costs between locations
- `routing_costs::Union{Dict{Tuple{Int,Int}, Float64}, Nothing}`: Vehicle routing costs (optional)
- `scenarios::Vector{ScenarioData}`: Scenario data for optimization
"""
struct StationSelectionData
    # Station information
    stations::DataFrame
    n_stations::Int

    # Cost matrices
    walking_costs::Dict{Tuple{Int, Int}, Float64}
    routing_costs::Union{Dict{Tuple{Int, Int}, Float64}, Nothing}

    # Scenario data
    scenarios::Vector{ScenarioData}
end

"""
    create_station_selection_data(
        stations::DataFrame,
        requests::DataFrame,
        walking_costs::Dict{Tuple{Int,Int}, Float64};
        routing_costs=nothing,
        scenarios=nothing
    ) -> StationSelectionData

Create a StationSelectionData struct from raw inputs.

# Arguments
- `stations::DataFrame`: Must have columns :id, :lon, :lat
- `requests::DataFrame`: Must have columns :start_station_id, :end_station_id, :request_time
- `walking_costs::Dict{Tuple{Int,Int}, Float64}`: Pairwise walking costs
- `routing_costs::Union{Dict, Nothing}`: Optional vehicle routing costs
- `scenarios::Union{Vector{Tuple{String,String}}, Nothing}`: Optional time windows for scenarios

If `scenarios` is nothing, all requests are treated as a single scenario.
"""
function create_station_selection_data(
    stations::DataFrame,
    requests::DataFrame,
    walking_costs::Dict{Tuple{Int, Int}, Float64};
    routing_costs::Union{Dict{Tuple{Int, Int}, Float64}, Nothing}=nothing,
    scenarios::Union{Vector{Tuple{String, String}}, Nothing}=nothing
)::StationSelectionData

    # Validate required columns
    @assert :id in propertynames(stations) "stations must have :id column"
    @assert :lon in propertynames(stations) "stations must have :lon column"
    @assert :lat in propertynames(stations) "stations must have :lat column"
    @assert :start_station_id in propertynames(requests) "requests must have :start_station_id column"
    @assert :end_station_id in propertynames(requests) "requests must have :end_station_id column"
    @assert :request_time in propertynames(requests) "requests must have :request_time column"

    n_stations = nrow(stations)

    # Create scenario data
    scenario_data = Vector{ScenarioData}()

    if isnothing(scenarios) || isempty(scenarios)
        # Single scenario with all requests
        scenario = create_scenario_data(requests, "all_requests")
        push!(scenario_data, scenario)
    else
        # Split requests into scenarios based on time windows
        for (i, (start_str, end_str)) in enumerate(scenarios)
            start_dt = DateTime(start_str, "yyyy-mm-dd HH:MM:SS")
            end_dt = DateTime(end_str, "yyyy-mm-dd HH:MM:SS")

            # Filter requests for this time window
            mask = (requests.request_time .>= start_dt) .& (requests.request_time .<= end_dt)
            scenario_requests = requests[mask, :]

            # Skip empty scenarios
            if nrow(scenario_requests) > 0
                label = "$(start_str)_$(end_str)"
                scenario = create_scenario_data(
                    scenario_requests,
                    label;
                    start_time=start_dt,
                    end_time=end_dt
                )
                push!(scenario_data, scenario)
            end
        end
    end

    return StationSelectionData(
        stations,
        n_stations,
        walking_costs,
        routing_costs,
        scenario_data
    )
end

# Convenience accessor functions
"""
    n_scenarios(data::StationSelectionData) -> Int

Return the number of scenarios in the problem data.
"""
n_scenarios(data::StationSelectionData) = length(data.scenarios)

"""
    get_walking_cost(data::StationSelectionData, from_id::Int, to_id::Int) -> Float64

Get walking cost between two stations by ID.
"""
get_walking_cost(data::StationSelectionData, from_id::Int, to_id::Int) =
    data.walking_costs[(from_id, to_id)]

"""
    get_routing_cost(data::StationSelectionData, from_id::Int, to_id::Int) -> Float64

Get routing cost between two stations by ID. Throws error if routing_costs is nothing.
"""
function get_routing_cost(data::StationSelectionData, from_id::Int, to_id::Int)
    isnothing(data.routing_costs) && error("Routing costs not available")
    return data.routing_costs[(from_id, to_id)]
end

"""
    has_routing_costs(data::StationSelectionData) -> Bool

Check if routing costs are available.
"""
has_routing_costs(data::StationSelectionData) = !isnothing(data.routing_costs)


# =============================================================================
# Station and Scenario Index Mapping Helpers
# =============================================================================

export create_station_id_mappings, create_scenario_label_mappings
export get_station_id, get_station_idx
export compute_time_to_od_count_mapping

"""
    create_station_id_mappings(station_ids::Vector{Int}) -> (Dict{Int,Int}, Vector{Int})

Build bidirectional station ID ↔ array-index mappings.
Returns `(id_to_idx, idx_to_id)`.
"""
function create_station_id_mappings(station_ids::Vector{Int})
    id_to_idx = Dict{Int, Int}()
    for (idx, id) in enumerate(station_ids)
        id_to_idx[id] = idx
    end
    return id_to_idx, copy(station_ids)
end

"""
    create_scenario_label_mappings(scenarios::Vector{ScenarioData}) -> (Dict{String,Int}, Vector{String})

Build bidirectional scenario label ↔ array-index mappings.
Returns `(label_to_idx, idx_to_label)`.
"""
function create_scenario_label_mappings(scenarios::Vector{ScenarioData})
    label_to_idx = Dict{String, Int}()
    idx_to_label = String[]
    for (idx, s) in enumerate(scenarios)
        label_to_idx[s.label] = idx
        push!(idx_to_label, s.label)
    end
    return label_to_idx, idx_to_label
end

"""
    get_station_id(mapping, idx::Int) -> Int

Get the station ID at array index `idx`.
Works for any mapping with an `array_idx_to_station_id` field.
"""
get_station_id(mapping, idx::Int) = mapping.array_idx_to_station_id[idx]

"""
    get_station_idx(mapping, id::Int) -> Int

Get the array index for station with the given ID.
Works for any mapping with a `station_id_to_array_idx` field.
"""
get_station_idx(mapping, id::Int) = mapping.station_id_to_array_idx[id]

"""
    compute_time_to_od_count_mapping(scenario::ScenarioData, time_window_sec::Int)
    -> Dict{Int, Dict{Tuple{Int,Int}, Int}}

Group requests by time window and count OD pair demand.

For each request, the time window index is:
    t = floor((request_time - scenario.start_time) / time_window_sec)

Returns: `time_id → (origin_id, dest_id) → count`.

Requires `scenario.start_time` to be set.
"""
function compute_time_to_od_count_mapping(
    scenario::ScenarioData,
    time_window_sec::Int
)::Dict{Int, Dict{Tuple{Int, Int}, Int}}
    isnothing(scenario.start_time) && error(
        "Scenario '$(scenario.label)' must have a start_time to compute time window mappings"
    )
    time_to_od = Dict{Int, Dict{Tuple{Int, Int}, Int}}()

    for row in eachrow(scenario.requests)
        o = row.start_station_id
        d = row.end_station_id

        req_time = row.request_time isa AbstractString ?
            DateTime(row.request_time, "yyyy-mm-dd HH:MM:SS") :
            row.request_time

        t_diff_sec = (req_time - scenario.start_time) / Dates.Second(1)
        t_id = floor(Int, t_diff_sec / time_window_sec)

        if !haskey(time_to_od, t_id)
            time_to_od[t_id] = Dict{Tuple{Int, Int}, Int}()
        end
        od = (o, d)
        time_to_od[t_id][od] = get(time_to_od[t_id], od, 0) + 1
    end

    return time_to_od
end
