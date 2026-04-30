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
export get_walking_cost, get_routing_cost, get_walking_cost_by_id, get_routing_cost_by_id, has_routing_costs
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
- `requests::DataFrame`: Customer requests in this scenario. Model internals expect
  indexed `:origin_idx` and `:dest_idx` columns; `create_station_selection_data`
  adds them from raw station IDs.
"""
struct ScenarioData
    label::String
    start_time::Union{DateTime, Nothing}
    end_time::Union{DateTime, Nothing}
    requests::DataFrame
    n_days::Int   # number of calendar days spanned; used to average demand (default 1)
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
    end_time::Union{DateTime, Nothing}=nothing,
    n_days::Int=1
)::ScenarioData
    return ScenarioData(label, start_time, end_time, requests, n_days)
end


"""
    StationSelectionData

Central data structure containing all problem data for station selection optimization.

This struct encapsulates stations, costs, and scenario data in a format that
can be reused across different optimization models.

# Fields
- `stations::DataFrame`: Station data with columns :id, :lon, :lat
- `n_stations::Int`: Number of candidate stations
- `station_id_to_array_idx::Dict{Int,Int}`: Station ID → compact internal index
- `array_idx_to_station_id::Vector{Int}`: Compact internal index → station ID
- `walking_costs::Matrix{Float64}`: Walking costs indexed by compact station index
- `routing_costs::Union{Matrix{Float64}, Nothing}`: Vehicle routing costs indexed by compact station index
- `scenarios::Vector{ScenarioData}`: Scenario data for optimization
"""
struct StationSelectionData
    # Station information
    stations::DataFrame
    n_stations::Int
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    # Cost matrices indexed by station_idx
    walking_costs::Matrix{Float64}
    routing_costs::Union{Matrix{Float64}, Nothing}

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
- `requests::DataFrame`: Must have origin/destination station columns and :request_time.
  Preferred columns are :origin_station_id and :destination_station_id. Legacy
  :start_station_id/:end_station_id and single-item available station lists are
  accepted as compatibility fallbacks.
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
    @assert :request_time in propertynames(requests) "requests must have :request_time column"

    n_stations = nrow(stations)
    station_ids = Vector{Int}(stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)
    walking_costs_idx = _cost_dict_to_idx_matrix(walking_costs, array_idx_to_station_id)
    routing_costs_idx = isnothing(routing_costs) ? nothing :
        _cost_dict_to_idx_matrix(routing_costs, array_idx_to_station_id)
    indexed_requests = _add_request_station_indices(requests, station_id_to_array_idx)

    # Create scenario data
    scenario_data = Vector{ScenarioData}()

    if isnothing(scenarios) || isempty(scenarios)
        # Single scenario with all requests
        scenario = create_scenario_data(indexed_requests, "all_requests")
        push!(scenario_data, scenario)
    else
        # Split requests into scenarios based on time windows
        for (i, (start_str, end_str)) in enumerate(scenarios)
            start_dt = DateTime(start_str, "yyyy-mm-dd HH:MM:SS")
            end_dt = DateTime(end_str, "yyyy-mm-dd HH:MM:SS")

            # Filter requests for this time window
            mask = (indexed_requests.request_time .>= start_dt) .& (indexed_requests.request_time .<= end_dt)
            scenario_requests = indexed_requests[mask, :]

            # Skip empty scenarios
            if nrow(scenario_requests) > 0
                label = "$(start_str)_$(end_str)"
                # n_days: calendar days spanned. Single-day windows → 1 (no averaging).
                # Month-spanning ranges from create_period_aggregated_data → averaging denominator.
                n_days = Dates.value(Date(end_dt) - Date(start_dt)) + 1
                scenario = create_scenario_data(
                    scenario_requests,
                    label;
                    start_time=start_dt,
                    end_time=end_dt,
                    n_days=n_days
                )
                push!(scenario_data, scenario)
            end
        end
    end

    return StationSelectionData(
        stations,
        n_stations,
        station_id_to_array_idx,
        array_idx_to_station_id,
        walking_costs_idx,
        routing_costs_idx,
        scenario_data
    )
end

function _cost_dict_to_idx_matrix(
    costs_by_id::Dict{Tuple{Int, Int}, Float64},
    array_idx_to_station_id::Vector{Int}
)::Matrix{Float64}
    n = length(array_idx_to_station_id)
    costs = fill(Inf, n, n)
    for from_idx in 1:n, to_idx in 1:n
        from_id = array_idx_to_station_id[from_idx]
        to_id = array_idx_to_station_id[to_idx]
        costs[from_idx, to_idx] = from_idx == to_idx ? 0.0 : get(costs_by_id, (from_id, to_id), Inf)
    end
    return costs
end

function _add_request_station_indices(
    requests::DataFrame,
    station_id_to_array_idx::Dict{Int, Int}
)::DataFrame
    indexed_requests = copy(requests)
    origin_ids = _request_station_ids(indexed_requests, :origin)
    destination_ids = _request_station_ids(indexed_requests, :destination)
    indexed_requests.origin_station_id = origin_ids
    indexed_requests.destination_station_id = destination_ids
    indexed_requests.start_station_id = origin_ids
    indexed_requests.end_station_id = destination_ids
    indexed_requests.origin_idx = [station_id_to_array_idx[Int(id)] for id in origin_ids]
    indexed_requests.dest_idx = [station_id_to_array_idx[Int(id)] for id in destination_ids]
    return indexed_requests
end

function _request_station_ids(requests::DataFrame, side::Symbol)::Vector{Int}
    names_set = Set(propertynames(requests))
    candidates = side == :origin ?
        (:origin_station_id, :start_station_id, :origin_id) :
        (:destination_station_id, :end_station_id, :target_id, :dest_station_id)

    for col in candidates
        if col in names_set
            station_ids = Int.(requests[!, col])
            _warn_if_legacy_station_column_disagrees(requests, side, station_ids, col)
            return station_ids
        end
    end

    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    if legacy_col in names_set
        return [_first_legacy_station_id(value, side) for value in requests[!, legacy_col]]
    end

    label = side == :origin ? "origin" : "destination"
    error("requests must include a $label station column")
end

function _first_legacy_station_id(value, side::Symbol)::Int
    values = parse_station_list(string(value))
    isempty(values) && error("request $(side) station column is empty")
    return first(values)
end

function _warn_if_legacy_station_column_disagrees(
    requests::DataFrame,
    side::Symbol,
    station_ids::Vector{Int},
    scalar_col::Symbol
)
    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in propertynames(requests) || return

    for (row_idx, value) in enumerate(requests[!, legacy_col])
        ismissing(value) && continue
        values = parse_station_list(string(value))
        isempty(values) && continue
        legacy_id = first(values)
        station_id = station_ids[row_idx]
        if legacy_id != station_id
            @warn "Scalar station column disagrees with legacy station list; using scalar column" side row_idx scalar_col station_id legacy_col legacy_id
        end
    end
end

# Convenience accessor functions
"""
    n_scenarios(data::StationSelectionData) -> Int

Return the number of scenarios in the problem data.
"""
n_scenarios(data::StationSelectionData) = length(data.scenarios)

"""
    get_walking_cost(data::StationSelectionData, from_idx::Int, to_idx::Int) -> Float64

Get walking cost between two stations by compact internal station index.
"""
get_walking_cost(data::StationSelectionData, from_idx::Int, to_idx::Int) =
    data.walking_costs[from_idx, to_idx]

"""
    get_routing_cost(data::StationSelectionData, from_idx::Int, to_idx::Int) -> Float64

Get routing cost between two stations by compact internal station index.
Throws error if routing_costs is nothing.
"""
function get_routing_cost(data::StationSelectionData, from_idx::Int, to_idx::Int)
    isnothing(data.routing_costs) && error("Routing costs not available")
    return data.routing_costs[from_idx, to_idx]
end

"""
    get_walking_cost_by_id(data::StationSelectionData, from_id::Int, to_id::Int) -> Float64

Boundary helper for walking costs keyed by raw station ID.
"""
get_walking_cost_by_id(data::StationSelectionData, from_id::Int, to_id::Int) =
    get_walking_cost(data, data.station_id_to_array_idx[from_id], data.station_id_to_array_idx[to_id])

"""
    get_routing_cost_by_id(data::StationSelectionData, from_id::Int, to_id::Int) -> Float64

Boundary helper for routing costs keyed by raw station ID.
"""
get_routing_cost_by_id(data::StationSelectionData, from_id::Int, to_id::Int) =
    get_routing_cost(data, data.station_id_to_array_idx[from_id], data.station_id_to_array_idx[to_id])

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

Returns: `time_id → (origin_idx, dest_idx) → count`.

Requires `scenario.start_time` to be set.
"""
function compute_time_to_od_count_mapping(
    scenario::ScenarioData,
    time_window_sec::Int
)::Dict{Int, Dict{Tuple{Int, Int}, Int}}
    isnothing(scenario.start_time) && error(
        "Scenario '$(scenario.label)' must have a start_time to compute time window mappings"
    )
    _require_indexed_request_columns(scenario.requests)
    time_to_od = Dict{Int, Dict{Tuple{Int, Int}, Int}}()

    for row in eachrow(scenario.requests)
        o = row.origin_idx
        d = row.dest_idx

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

function _require_indexed_request_columns(requests::DataFrame)
    (:origin_idx in propertynames(requests) && :dest_idx in propertynames(requests)) ||
        error("Scenario requests must include :origin_idx and :dest_idx. Use create_station_selection_data to build indexed scenarios.")
    return nothing
end
