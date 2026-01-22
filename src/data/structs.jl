"""
Core data structures for StationSelection optimization models.

This module provides reusable data structures that encapsulate problem data,
making it easy to pass consistent data to different optimization models.
"""
module DataStructs

using DataFrames
using Dates

export ScenarioData, StationSelectionData
export create_station_selection_data, create_scenario_data

"""
    ScenarioData

Encapsulates request data for a single scenario (time period).

# Fields
- `label::String`: Human-readable label for the scenario
- `start_time::Union{DateTime, Nothing}`: Start of the scenario time window
- `end_time::Union{DateTime, Nothing}`: End of the scenario time window
- `requests::DataFrame`: Customer requests in this scenario
- `pickup_counts::Dict{Int, Int}`: Station ID → pickup count
- `dropoff_counts::Dict{Int, Int}`: Station ID → dropoff count
- `total_counts::Dict{Int, Int}`: Station ID → total (pickup + dropoff) count
"""
struct ScenarioData
    label::String
    start_time::Union{DateTime, Nothing}
    end_time::Union{DateTime, Nothing}
    requests::DataFrame
    pickup_counts::Dict{Int, Int}
    dropoff_counts::Dict{Int, Int}
    total_counts::Dict{Int, Int}
end

"""
    create_scenario_data(requests::DataFrame, label::String, station_ids::Vector{Int};
                         start_time=nothing, end_time=nothing) -> ScenarioData

Create a ScenarioData struct from a DataFrame of requests.

Automatically computes pickup, dropoff, and total counts for each station.
Initializes counts to 0 for stations with no requests.
"""
function create_scenario_data(
    requests::DataFrame,
    label::String,
    station_ids::Vector{Int};
    start_time::Union{DateTime, Nothing}=nothing,
    end_time::Union{DateTime, Nothing}=nothing
)::ScenarioData

    # Count pickups (start_station_id)
    pickup_counts = Dict{Int, Int}()
    for station_id in requests.start_station_id
        pickup_counts[station_id] = get(pickup_counts, station_id, 0) + 1
    end

    # Count dropoffs (end_station_id)
    dropoff_counts = Dict{Int, Int}()
    for station_id in requests.end_station_id
        dropoff_counts[station_id] = get(dropoff_counts, station_id, 0) + 1
    end

    # Initialize all stations to 0 if not present, compute totals
    total_counts = Dict{Int, Int}()
    for station_id in station_ids
        pickup_counts[station_id] = get(pickup_counts, station_id, 0)
        dropoff_counts[station_id] = get(dropoff_counts, station_id, 0)
        total_counts[station_id] = pickup_counts[station_id] + dropoff_counts[station_id]
    end

    return ScenarioData(
        label,
        start_time,
        end_time,
        requests,
        pickup_counts,
        dropoff_counts,
        total_counts
    )
end

"""
    StationSelectionData

Central data structure containing all problem data for station selection optimization.

This struct encapsulates stations, costs, and scenario data in a format that
can be reused across different optimization models.

# Fields
- `stations::DataFrame`: Station data with columns :id, :lon, :lat
- `n_stations::Int`: Number of candidate stations
- `id_to_idx::Dict{Int, Int}`: Station ID → array index mapping
- `idx_to_id::Dict{Int, Int}`: Array index → station ID mapping
- `walking_costs::Dict{Tuple{Int,Int}, Float64}`: Walking costs between locations
- `routing_costs::Union{Dict{Tuple{Int,Int}, Float64}, Nothing}`: Vehicle routing costs (optional)
- `scenarios::Vector{ScenarioData}`: Scenario data for optimization
"""
struct StationSelectionData
    # Station information
    stations::DataFrame
    n_stations::Int
    id_to_idx::Dict{Int, Int}
    idx_to_id::Dict{Int, Int}

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
    station_ids = Vector{Int}(stations.id)

    # Create ID mappings
    id_to_idx = Dict{Int, Int}()
    idx_to_id = Dict{Int, Int}()
    for (idx, station_id) in enumerate(station_ids)
        id_to_idx[station_id] = idx
        idx_to_id[idx] = station_id
    end

    # Create scenario data
    scenario_data = Vector{ScenarioData}()

    if isnothing(scenarios) || isempty(scenarios)
        # Single scenario with all requests
        scenario = create_scenario_data(requests, "all_requests", station_ids)
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
                    label,
                    station_ids;
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
        id_to_idx,
        idx_to_id,
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
    get_station_id(data::StationSelectionData, idx::Int) -> Int

Convert array index to station ID.
"""
get_station_id(data::StationSelectionData, idx::Int) = data.idx_to_id[idx]

"""
    get_station_idx(data::StationSelectionData, id::Int) -> Int

Convert station ID to array index.
"""
get_station_idx(data::StationSelectionData, id::Int) = data.id_to_idx[id]

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

export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, has_routing_costs

end # module
