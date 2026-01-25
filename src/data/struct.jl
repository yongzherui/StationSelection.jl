"""
Core data structures for StationSelection optimization models.

This module provides reusable data structures that encapsulate problem data,
making it easy to pass consistent data to different optimization models.
"""

using DataFrames
using Dates
using Combinatorics

export ScenarioData, StationSelectionData
export create_station_selection_data, create_scenario_data
export n_scenarios, get_station_id, get_station_idx
export get_walking_cost, get_routing_cost, has_routing_costs


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
    start_time::Union{DateTime, Nothing}=nothing,
    end_time::Union{DateTime, Nothing}=nothing
)::ScenarioData


    return ScenarioData(
        label,
        start_time,
        end_time,
        requests
    )
end

struct PoolingScenarioOriginDestTimeMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{Int}


    # we need this to know what the time window is
    time_window::Int

    # scenario_id: id of the scenario
    # time_id: Request floor(time/time_window)
    #
    # this works like Omega[scenario_id][time_id]= [(o1, d1), (o2, d2)]
    # thus it contains all the time_ids and the OD pairs for each time_id
    Omega_s_t::Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}
end

function create_pooling_scenario_origin_dest_time_map(model::TwoStageSingleDetourModel, data::StationSelectionData)::PoolingScenarioOriginDestTimeMap
    station_id_to_array_idx = Dict{Int, Int}()
    array_idx_to_station_id = Vector{Int}()

    station_ids = Vector{Int}(data.stations.id)
    for (idx, station_id) in enumerate(station_ids)
        station_id_to_array_idx[station_id] = idx
        array_idx_to_station_id[idx] = station_id
    end

    # now we need the (o,d,t) mapping for the scenario
    time_window = model.time_window

    Omega_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int}}}}()
    Xi_same_source_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int, Int}}}}()
    Xi_same_dest_s_t = Dict{Int, Dict{Int, Vector{Tuple{Int, Int, Int}}}}()

    scenario_label_to_array_idx = Dict{String, Int}()
    array_idx_to_scenario_label = Vector{String}()

    # we find the detour combinations
    detour_combinations = find_detour_combinations(model, data)

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        # we want to create an entry
        Omega_s_t[scenario_id] = Dict{Int, Set{Tuple{Int, Int}}}()
        Xi_same_source_s_t[scenario_id] = Dict{Int, Set{Tuple{Int, Int, Int}}}()
        Xi_same_dest_s_t[scenario_id] = Dict{Int, Set{Tuple{Int, Int, Int}}}()

        scenario_start_time = scenario_data.start_time

        if isnothing(scenario_start_time)
            @error "scenario_start_time cannot be nothing for two stage single detour model"
        end

        time_to_od_set = Dict{Int, Set{Tuple{Int, Int}}}()

        for row in eachrow(scenario_data.requests)
            o = row.start_station_id
            d = row.end_station_id

            # now we work on getting the time_id
            request_time_str = row.order_time

            # we take the time relative to the start time of the scenario
            request_time_dt = DateTime(request_time_str, "yyyy-mm-dd HH:MM:SS")

            time_btwn_in_seconds = (request_time_dt - scenario_start_time) / Second(1)

            time_id = time_btwn_in_seconds / time_window

            if !haskey(time_to_od_set, time_id)
                time_to_od_set[time_id] = Set{Tuple{Int, Int}}()
            end

            # we only have unique (o,d)s
            push!(od_set, (o, d))
        end

        # convert to vector for consistent mapping
        for (time_id, od_set) in time_to_od_set
            Omega_s_t[scenario_id][time_id] = collect(od_set)
        end

        for (time_id) in keys(Omega_s_t[scenario_id])
            Xi_same_source[scenario_id][time_id] = Vector{Tuple{Int, Int, Int}}()
            Xi_same_dest[scenario_id][time_id] = Vector{Tuple{Int, Int, Int}}()

            # it is easier to check from here
            # for same origin, we check within the same time period
            # all possible permutations and see if they are in the set Xi
            for (od1, od2) in permutations(Omega_s_t[scenario_id][time_id], 2)
                # we are looking for same source
                if (od1[1] != od2[1])
                    # otherwise continue
                    continue
                end

                # we construct the set we want to check for
                jkl = (od1[1], od1[2], od2[1])

                # we add it if its in the combinations
                if jkl in detour_combinations
                    push!(Xi_same_source, jkl)
                end
            end

            # for the same destination We will have to do some math for the time_id
            # given the detour_combinations
            for od in Omega_s_t[scenario_id][time_id]
                # we find the set of detour_combinations where the source and dest is in detour combinations
                possible_intermediate_stations = filter(detour -> detour[1] == od[1] && detour[3] == od[2], detour_combinations)

                for (j, k, l) in possible_intermediate_stations
                    # then we find the time_id of the future
                    future_time_id = time_id + Int(get_routing_cost(data, j, k) / model.time_window)

                    # if we do not have this future time id we skip it
                    if !(future_time_id in keys(Omega_s_t[scenario_id]))
                        continue
                    end

                    if (k, l) in Omega_s_t[scenario_id][future_time_id]
                        # if we can find the desired od
                        push!(Xi_same_dest, (j, k, l))
                    end
                end
            end

        end
    end

    return PoolingScenarioOriginDestTimeMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_arary_idx,
        array_idx_to_scenario_label,
        time_window,
        Omega_s_t,
        Xi_same_source,
        Xi_same_dest,
       )

end

