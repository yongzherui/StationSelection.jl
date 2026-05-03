"""
Order transformation utilities for converting selected station results to simulation-ready orders.

This module provides functionality to:
1. Read station selection results (with 'selected' column or timeframe-specific columns)
2. Transform original orders by assigning pickup/dropoff to closest selected stations
3. Output simulation-ready orders with assigned_pickup_id and assigned_dropoff_id

The transformation uses Haversine distance to find the closest selected station
for each order's origin and destination.
"""

using CSV
using DataFrames
using Distances
using Dates
using JSON

export transform_orders,
       transform_orders_from_assignments,
       transform_orders_for_month_backtest,
       transform_orders_quick_extend,
       split_transformed_orders_by_day,
       remap_order_times_stacked,
       parse_station_list,
       precompute_distances,
       find_closest_selected_station,
       get_timeframe_column

"""
    parse_station_list(list_str::String) -> Vector{Int}

Parse a string like "[1,2,3]" or "[1 2 3]" into a vector of integers.

# Arguments
- `list_str`: String representation of station list (e.g., "[1,2,3]")

# Returns
- Vector{Int} of station IDs, or empty vector if invalid
"""
function parse_station_list(list_str)
    cleaned = strip(replace(string(list_str), "[" => "", "]" => "", "," => " "))
    if isempty(cleaned) || cleaned == "missing"
        return Int[]
    end
    return parse.(Int, split(cleaned))
end

function _row_station_id(row, side::Symbol)::Int
    columns = propertynames(row)
    candidates = side == :origin ?
        (:origin_station_id, :start_station_id, :origin_id) :
        (:destination_station_id, :end_station_id, :target_id, :dest_station_id)

    for col in candidates
        if col in columns && !ismissing(row[col])
            station_id = Int(row[col])
            _warn_if_legacy_station_disagrees(row, side, station_id, col)
            return station_id
        end
    end

    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in columns || return 0
    stations = parse_station_list(string(row[legacy_col]))
    return isempty(stations) ? 0 : first(stations)
end

function _warn_if_legacy_station_disagrees(row, side::Symbol, station_id::Int, scalar_col::Symbol)
    legacy_col = side == :origin ? :available_pickup_station_list : :available_dropoff_station_list
    legacy_col in propertynames(row) || return
    ismissing(row[legacy_col]) && return

    stations = parse_station_list(string(row[legacy_col]))
    isempty(stations) && return
    legacy_id = first(stations)
    if legacy_id != station_id
        @warn "Scalar station column disagrees with legacy station list; using scalar column" side scalar_col station_id legacy_col legacy_id
    end
end

_row_origin_station_id(row)::Int = _row_station_id(row, :origin)
_row_destination_station_id(row)::Int = _row_station_id(row, :destination)

function _row_value_or_missing(row, col::Symbol)
    return col in propertynames(row) ? row[col] : missing
end

"""
    get_timeframe_column(order_time::DateTime, columns::Vector{String}) -> Union{String, Nothing}

Find the timeframe column that matches the given order time.

Timeframe columns are expected to have format: "YYYY-MM-DD HH:MM:SS_YYYY-MM-DD HH:MM:SS"

# Arguments
- `order_time`: DateTime of the order
- `columns`: List of column names from the stations dataframe

# Returns
- Column name matching the timeframe, or nothing if not found
"""
function get_timeframe_column(order_time::DateTime, columns::Vector{String})
    for col in columns
        # Parse column name format: "2025-05-01 04:00:00_2025-05-01 07:59:59"
        m = match(r"(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})_(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})", col)
        if m !== nothing
            start_time = DateTime(m.captures[1], "yyyy-mm-dd HH:MM:SS")
            end_time = DateTime(m.captures[2], "yyyy-mm-dd HH:MM:SS")

            if order_time >= start_time && order_time <= end_time
                return col
            end
        end
    end
    return nothing
end

"""
    get_selected_station_ids(order_time, stations_df, all_columns, timeframe_columns, use_timeframes)
        -> Vector{Int}

Return the selected station IDs used for closest-station fallback.
"""
function get_selected_station_ids(
    order_time::DateTime,
    stations_df::DataFrame,
    all_columns::Vector{String},
    timeframe_columns::Vector{String},
    use_timeframes::Bool
)::Vector{Int}
    if use_timeframes && !isempty(timeframe_columns)
        timeframe_col = get_timeframe_column(order_time, timeframe_columns)
        if timeframe_col !== nothing
            return [r.id for r in eachrow(stations_df) if r[timeframe_col] == 1.0 || r[timeframe_col] == 1]
        end
        return Int[]
    end

    if "selected" in all_columns
        return [r.id for r in eachrow(stations_df) if r.selected == 1]
    end

    return Int[]
end

"""
    find_matching_scenario(order_time, scenario_ranges) -> Union{Int, Nothing}

Find the exported scenario index whose time range contains the order time.
"""
function find_matching_scenario(
    order_time::DateTime,
    scenario_ranges::Dict{Int, Tuple{DateTime, DateTime}}
)::Union{Int, Nothing}
    for (s_idx, (s_start, s_end)) in scenario_ranges
        if order_time >= s_start && order_time <= s_end
            return s_idx
        end
    end
    return nothing
end

"""
    compute_order_time_id(order_time, scenario_start, time_window_sec) -> Int

Compute the order's time bucket relative to the scenario start.
"""
function compute_order_time_id(
    order_time::DateTime,
    scenario_start::DateTime,
    time_window_sec::Int
)::Int
    time_window_sec > 0 || error("time_window_sec must be positive, got $time_window_sec")
    t_diff_sec = (order_time - scenario_start) / Dates.Second(1)
    return floor(Int, t_diff_sec / time_window_sec)
end

"""
    parse_exported_datetime(value, field_name, scenario_idx) -> DateTime

Parse a scenario timestamp exported to CSV, accepting either strings or DateTime values.
"""
function parse_exported_datetime(value, field_name::String, scenario_idx::Int)::DateTime
    if value isa DateTime
        return value
    end

    value_str = string(value)
    isempty(value_str) && error(
        "scenario_info.csv has empty $field_name for scenario $scenario_idx"
    )
    return DateTime(value_str)
end

function _period_of_datetime(order_time::DateTime, profile::Symbol)::Union{Int, Nothing}
    h = hour(order_time)
    if profile == :four_period
        if 6 <= h < 10
            return 1
        elseif 10 <= h < 15
            return 2
        elseif 15 <= h < 20
            return 3
        elseif 20 <= h < 24
            return 4
        end
    elseif profile == :full_day
        return 1
    elseif profile == :commute
        if 7 <= h < 10
            return 1
        elseif 16 <= h < 19
            return 2
        end
    elseif profile == :morning
        return 6 <= h < 10 ? 1 : nothing
    elseif profile == :midday
        return 10 <= h < 14 ? 1 : nothing
    elseif profile == :evening
        return 16 <= h < 20 ? 1 : nothing
    elseif profile == :night
        return 20 <= h < 24 ? 1 : nothing
    end
    return nothing
end

_json_int_dict(values::AbstractDict{Int, <:Any}) = Dict(string(k) => v for (k, v) in sort(collect(values); by=first))

function _ensure_assignment_columns!(df::DataFrame,
                                     origin_ids::Vector{Int},
                                     dest_ids::Vector{Int},
                                     pickup_ids::Vector{Int},
                                     dropoff_ids::Vector{Int})
    df.origin_station_id = origin_ids
    df.destination_station_id = dest_ids
    df.assigned_pickup_id = pickup_ids
    df.assigned_dropoff_id = dropoff_ids
    return df
end

"""
    precompute_distances(stations::DataFrame) -> Dict{Tuple{Int,Int}, Float64}

Precompute all pairwise distances between stations using Haversine distance.

# Arguments
- `stations`: DataFrame with columns: id, lon, lat

# Returns
- Dictionary mapping (from_id, to_id) => distance in meters
"""
function precompute_distances(stations::DataFrame)
    distances = Dict{Tuple{Int,Int}, Float64}()
    dist_func = Haversine()  # Returns distance in meters

    for i in 1:nrow(stations)
        for j in 1:nrow(stations)
            if i == j
                distances[(stations[i, :id], stations[j, :id])] = 0.0
            else
                # Haversine expects [lat, lon] format
                p1 = [stations[i, :lat], stations[i, :lon]]
                p2 = [stations[j, :lat], stations[j, :lon]]
                distance = evaluate(dist_func, p1, p2)
                distances[(stations[i, :id], stations[j, :id])] = distance
            end
        end
    end

    return distances
end

function precompute_walking_costs(stations::DataFrame; walking_speed::Float64=1.4)
    distances_m = precompute_distances(stations)
    return Dict(key => value / walking_speed for (key, value) in distances_m)
end

"""
    find_closest_selected_station(candidate_station_id::Int,
                                  selected_station_ids::Vector{Int},
                                  distance_matrix::Dict{Tuple{Int,Int}, Float64}) -> Int

Find the closest selected station to a given candidate station using precomputed distances.

# Arguments
- `candidate_station_id`: The candidate station ID (origin or destination)
- `selected_station_ids`: Vector of selected station IDs
- `distance_matrix`: Precomputed distance matrix from precompute_distances

# Returns
- ID of the closest selected station, or 0 if candidate is 0 or no selected stations
"""
function find_closest_selected_station(candidate_station_id::Int,
                                       selected_station_ids::Vector{Int},
                                       distance_matrix::Dict{Tuple{Int,Int}, Float64})
    if candidate_station_id == 0 || isempty(selected_station_ids)
        return 0
    end

    # Find the closest selected station to the candidate
    min_distance = Inf
    closest_selected_id = 0

    for selected_id in selected_station_ids
        # Look up precomputed distance
        distance = get(distance_matrix, (candidate_station_id, selected_id), Inf)

        if distance < min_distance
            min_distance = distance
            closest_selected_id = selected_id
        end
    end

    return closest_selected_id
end

function find_best_feasible_station_pair(origin_id::Int,
                                         dest_id::Int,
                                         candidate_station_ids::Vector{Int},
                                         walking_costs::Dict{Tuple{Int,Int}, Float64},
                                         routing_costs::Dict{Tuple{Int,Int}, Float64},
                                         max_walking_distance::Float64,
                                         in_vehicle_time_weight::Float64)::Tuple{Int, Int}
    if origin_id == 0 || dest_id == 0 || isempty(candidate_station_ids)
        return 0, 0
    end

    best_cost = Inf
    best_pair = (0, 0)

    for pickup_id in candidate_station_ids
        walk_pickup = get(walking_costs, (origin_id, pickup_id), Inf)
        walk_pickup <= max_walking_distance || continue
        for dropoff_id in candidate_station_ids
            walk_dropoff = get(walking_costs, (dropoff_id, dest_id), Inf)
            walk_dropoff <= max_walking_distance || continue
            route_cost = pickup_id == dropoff_id ? 0.0 : get(routing_costs, (pickup_id, dropoff_id), Inf)
            isfinite(route_cost) || continue

            total_cost = walk_pickup + walk_dropoff + in_vehicle_time_weight * route_cost
            if total_cost < best_cost
                best_cost = total_cost
                best_pair = (pickup_id, dropoff_id)
            end
        end
    end

    return best_pair
end

"""
    transform_orders(order_file::String,
                    cluster_file::String;
                    start_date::Union{DateTime, Nothing}=nothing,
                    end_date::Union{DateTime, Nothing}=nothing,
                    use_timeframes::Bool=false) -> DataFrame

Transform orders by assigning pickup and dropoff to selected stations.

This function:
1. Reads original orders with origin/destination station IDs
2. Reads station selection results (with 'selected' column or timeframe columns)
3. Optionally filters orders by date range
4. For each order, finds the closest selected station for pickup and dropoff
5. Returns transformed orders with assigned_pickup_id and assigned_dropoff_id

# Arguments
- `order_file`: Path to order CSV file with columns:
  - order_id, pax_num, order_time
  - origin_station_id
  - destination_station_id
- `cluster_file`: Path to station selection results CSV with columns:
  - id, lon, lat
  - 'selected' column (1/0) for default mode
  - OR timeframe columns "YYYY-MM-DD HH:MM:SS_YYYY-MM-DD HH:MM:SS" for timeframe mode
- `start_date`: Optional start date filter (DateTime)
- `end_date`: Optional end date filter (DateTime)
- `use_timeframes`: If true, use timeframe-specific columns; if false, use 'selected' column

# Returns
- DataFrame with transformed orders

# Example
```julia
# Default mode (uses 'selected' column)
orders_df = transform_orders(
    "Data/order.csv",
    "results/cluster_results.csv"
)

# Timeframe mode with date filtering
orders_df = transform_orders(
    "Data/order.csv",
    "results/cluster_results.csv";
    start_date=DateTime("2025-05-01 00:00:00", "yyyy-mm-dd HH:MM:SS"),
    end_date=DateTime("2025-05-31 23:59:59", "yyyy-mm-dd HH:MM:SS"),
    use_timeframes=true
)

# Write to file (handled by caller)
CSV.write("results/order_transformed.csv", orders_df)
```
"""
function transform_orders(order_file::String,
                         cluster_file::String;
                         start_date::Union{DateTime, Nothing}=nothing,
                         end_date::Union{DateTime, Nothing}=nothing,
                         use_timeframes::Bool=false)
    println("Reading input files...")

    # Read order data
    orders_df = CSV.read(order_file, DataFrame)
    println("Loaded $(nrow(orders_df)) orders from $order_file")

    # Filter by date range if specified
    if !isnothing(start_date) || !isnothing(end_date)
        println("\nFiltering orders by date range...")
        initial_count = nrow(orders_df)

        # Parse order_time to DateTime
        orders_df.order_time_parsed = DateTime.(orders_df.order_time, "yyyy-mm-dd HH:MM:SS")

        if !isnothing(start_date)
            orders_df = filter(row -> row.order_time_parsed >= start_date, orders_df)
            println("  After start_date filter ($start_date): $(nrow(orders_df)) orders")
        end

        if !isnothing(end_date)
            orders_df = filter(row -> row.order_time_parsed <= end_date, orders_df)
            println("  After end_date filter ($end_date): $(nrow(orders_df)) orders")
        end

        filtered_count = initial_count - nrow(orders_df)
        println("✓ Filtered out $filtered_count orders, $(nrow(orders_df)) orders remaining")

        # Remove the temporary parsed column
        select!(orders_df, Not(:order_time_parsed))
    end

    # Read cluster/station selection results
    stations_df = CSV.read(cluster_file, DataFrame)
    println("Loaded $(nrow(stations_df)) stations from $cluster_file")

    # Identify timeframe columns (columns with date_time format)
    all_columns = names(stations_df)
    timeframe_columns = filter(col -> occursin(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}_\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}", col), all_columns)

    if use_timeframes
        println("\nUsing time-frame specific station selection")
        println("Found $(length(timeframe_columns)) timeframe columns")

        if isempty(timeframe_columns)
            error("use_timeframes=true but no timeframe columns found in cluster file!")
        end
    else
        println("\nUsing default 'selected' column")

        # Verify selected column exists
        if !("selected" in all_columns)
            error("'selected' column not found in cluster file! Set use_timeframes=true or ensure the cluster file has a 'selected' column.")
        end
    end

    # Precompute all pairwise distances
    println("\nPrecomputing distances between all stations...")
    distance_matrix = precompute_distances(stations_df)
    println("✓ Computed $(length(distance_matrix)) pairwise distances")

    # Process each order
    println("\nProcessing orders...")
    transformed_orders = []

    for (idx, row) in enumerate(eachrow(orders_df))
        if idx % 1000 == 0
            println("  Processing order $idx / $(nrow(orders_df))...")
        end

        origin_id = _row_origin_station_id(row)
        target_id = _row_destination_station_id(row)

        # Determine which stations are selected for this order
        if use_timeframes
            # Parse order time
            order_time = DateTime(row.order_time, "yyyy-mm-dd HH:MM:SS")

            # Find the matching timeframe column
            timeframe_col = get_timeframe_column(order_time, timeframe_columns)

            if timeframe_col === nothing
                error("No timeframe column found for order $(row.order_id) at time $(row.order_time)")
            end

            # Filter stations selected for this specific timeframe
            selected_stations = filter(r -> r[timeframe_col] == 1.0 || r[timeframe_col] == 1, stations_df)
        else
            # Use default selected column
            selected_stations = filter(row -> row.selected == 1, stations_df)
        end

        if nrow(selected_stations) == 0
            if use_timeframes
                error("No selected stations found for timeframe at $(row.order_time)")
            else
                error("No selected stations found in cluster file!")
            end
        end

        # Extract selected station IDs for this order
        selected_station_ids = [r.id for r in eachrow(selected_stations)]

        # Find closest selected station for pickup (from origin_id)
        assigned_pickup_id = find_closest_selected_station(
            origin_id,
            selected_station_ids,
            distance_matrix
        )

        # Find closest selected station for dropoff (from target_id)
        assigned_dropoff_id = find_closest_selected_station(
            target_id,
            selected_station_ids,
            distance_matrix
        )

        # Create transformed order row
        push!(transformed_orders, (
            order_id = row.order_id,
            pax_num = row.pax_num,
            order_time = row.order_time,
            origin_station_id = origin_id,
            destination_station_id = target_id,
            assigned_pickup_id = assigned_pickup_id,
            assigned_dropoff_id = assigned_dropoff_id
        ))
    end

    # Convert to DataFrame
    output_df = DataFrame(transformed_orders)

    println("✓ Successfully transformed $(nrow(output_df)) orders")
    println("\nSummary:")
    println("  - Mode: $(use_timeframes ? "Time-frame specific" : "Default selected")")
    println("  - Orders with assigned pickup: $(count(row -> row.assigned_pickup_id != 0, eachrow(output_df)))")
    println("  - Orders with assigned dropoff: $(count(row -> row.assigned_dropoff_id != 0, eachrow(output_df)))")
    println("  - Orders fully assigned: $(count(row -> row.assigned_pickup_id != 0 && row.assigned_dropoff_id != 0, eachrow(output_df)))")

    return output_df
end

"""
    transform_orders_from_assignments(order_file, selection_run_dir, cluster_file, method;
        start_date, end_date, use_timeframes, time_window_sec) -> DataFrame

Transform orders using x-variable assignments exported from the optimization model.

Instead of assigning each order to the closest selected station, this function uses
the actual assignment decisions (x variables) from `variable_exports/assignment_variables.csv`.
Orders without a matching assignment fall back to closest selected station.

Lookup key:
- `(scenario, origin_id, dest_id)` for `ClusteringTwoStageODModel`
- `(scenario, t_id, origin_id, dest_id)` for time-indexed route models

# Arguments
- `order_file`: Path to order CSV
- `selection_run_dir`: Path to the selection run directory (contains `variable_exports/`)
- `cluster_file`: Path to station selection results CSV (for fallback)
- `method`: e.g. `"ClusteringTwoStageODModel"` or `"RouteFleetLimitModel"`
- `start_date`: Optional start date filter
- `end_date`: Optional end date filter
- `use_timeframes`: Whether to use timeframe columns for fallback
- `time_window_sec`: Required for time-indexed route models; ignored otherwise

# Returns
- DataFrame with same schema as `transform_orders`
"""
function transform_orders_from_assignments(order_file::String,
                                            selection_run_dir::String,
                                            cluster_file::String,
                                            method::String;
                                            start_date::Union{DateTime, Nothing}=nothing,
                                            end_date::Union{DateTime, Nothing}=nothing,
                                            use_timeframes::Bool=false,
                                            time_window_sec::Union{Int, Nothing}=nothing,
                                            scenario_profile::Symbol=:four_period,
                                            segment_file::Union{String, Nothing}=nothing,
                                            max_walking_distance::Float64=Inf,
                                            in_vehicle_time_weight::Float64=1.0)

    println("Reading input files...")

    # 1. Read and filter orders by date range
    orders_df = CSV.read(order_file, DataFrame)
    println("Loaded $(nrow(orders_df)) orders from $order_file")

    if !isnothing(start_date) || !isnothing(end_date)
        println("\nFiltering orders by date range...")
        initial_count = nrow(orders_df)
        orders_df.order_time_parsed = DateTime.(orders_df.order_time, "yyyy-mm-dd HH:MM:SS")

        if !isnothing(start_date)
            orders_df = filter(row -> row.order_time_parsed >= start_date, orders_df)
            println("  After start_date filter ($start_date): $(nrow(orders_df)) orders")
        end
        if !isnothing(end_date)
            orders_df = filter(row -> row.order_time_parsed <= end_date, orders_df)
            println("  After end_date filter ($end_date): $(nrow(orders_df)) orders")
        end

        filtered_count = initial_count - nrow(orders_df)
        println("✓ Filtered out $filtered_count orders, $(nrow(orders_df)) orders remaining")
        select!(orders_df, Not(:order_time_parsed))
    end

    # 2. Read assignment_variables.csv (only active assignments)
    assignment_file = joinpath(selection_run_dir, "variable_exports", "assignment_variables.csv")
    if !isfile(assignment_file)
        error("Assignment variables file not found: $assignment_file")
    end
    assignments_df = CSV.read(assignment_file, DataFrame)
    assignments_df = filter(row -> row.value >= 0.5, assignments_df)
    println("Loaded $(nrow(assignments_df)) active assignments from $assignment_file")

    # 3. Read scenario_info.csv
    scenario_file = joinpath(selection_run_dir, "variable_exports", "scenario_info.csv")
    if !isfile(scenario_file)
        error("Scenario info file not found: $scenario_file")
    end
    scenario_df = CSV.read(scenario_file, DataFrame)
    println("Loaded $(nrow(scenario_df)) scenarios from $scenario_file")

    # 4. Read cluster_file for fallback (z-variable selected stations)
    stations_df = CSV.read(cluster_file, DataFrame)
    println("Loaded $(nrow(stations_df)) stations from $cluster_file")

    # 5. Build scenario time ranges: scenario_idx => (start_time, end_time)
    scenario_ranges = Dict{Int, Tuple{DateTime, DateTime}}()
    for row in eachrow(scenario_df)
        s_start = parse_exported_datetime(row.start_time, "start_time", row.scenario_idx)
        s_end = parse_exported_datetime(row.end_time, "end_time", row.scenario_idx)
        scenario_ranges[row.scenario_idx] = (s_start, s_end)
    end

    # 6. Build assignment lookup dict
    # Time-indexed route models use a (scenario, time_id, origin_id, dest_id) key.
    route_model = method in ("TwoStageRouteWithTimeModel", "RouteVehicleCapacityModel", "RouteFleetLimitModel")
    route_model && isnothing(time_window_sec) &&
        error("transform_orders_from_assignments requires time_window_sec for $method")

    assignment_lookup = route_model ?
        Dict{NTuple{4, Int}, Tuple{Int,Int}}() :
        Dict{NTuple{3, Int}, Tuple{Int,Int}}()

    time_col = nothing
    if route_model
        assignment_cols = names(assignments_df)
        if "t_id" in assignment_cols
            time_col = :t_id
        elseif "time_id" in assignment_cols
            time_col = :time_id
        else
            error("assignment_variables.csv missing required column 'time_id' or 't_id' for $method")
        end
    end

    for row in eachrow(assignments_df)
        key = route_model ?
            (row.scenario, row[time_col], row.origin_id, row.dest_id) :
            (row.scenario, row.origin_id, row.dest_id)
        assignment_lookup[key] = (row.pickup_id, row.dropoff_id)
    end
    println("Built $method assignment lookup with $(length(assignment_lookup)) entries")

    # 7. Precompute distances for fallback
    println("\nPrecomputing distances between all stations...")
    distance_matrix = precompute_distances(stations_df)
    println("✓ Computed $(length(distance_matrix)) pairwise distances")

    fallback_walking_costs = nothing
    fallback_routing_costs = nothing
    if !isnothing(segment_file)
        fallback_walking_costs = precompute_walking_costs(stations_df)
        fallback_routing_costs = read_routing_costs_from_segments(segment_file, stations_df)
        println("✓ Loaded fallback walking/routing costs for joint feasible (j,k) assignment")
    end

    # Identify timeframe columns for fallback
    all_columns = names(stations_df)
    timeframe_columns = filter(col -> occursin(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}_\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}", col), all_columns)

    # 8. Process each order
    println("\nProcessing orders...")
    output_df = copy(orders_df)
    origin_ids = Int[]
    dest_ids = Int[]
    assigned_pickup_ids = Int[]
    assigned_dropoff_ids = Int[]
    n_assigned = 0
    n_fallback = 0
    n_pickup_differs = 0
    n_dropoff_differs = 0
    n_either_differs = 0
    n_missing_scenario = 0
    n_missing_station_id = 0
    orders_by_period = Dict{Int, Int}()
    x_assigned_by_period = Dict{Int, Int}()
    fallback_by_period = Dict{Int, Int}()

    for (idx, row) in enumerate(eachrow(orders_df))
        if idx % 1000 == 0
            println("  Processing order $idx / $(nrow(orders_df))...")
        end

        origin_id = _row_origin_station_id(row)
        target_id = _row_destination_station_id(row)
        push!(origin_ids, origin_id)
        push!(dest_ids, target_id)

        # Parse order time
        order_time = DateTime(row.order_time, "yyyy-mm-dd HH:MM:SS")
        period_idx = _period_of_datetime(order_time, scenario_profile)
        if !isnothing(period_idx)
            orders_by_period[period_idx] = get(orders_by_period, period_idx, 0) + 1
        end

        # Find matching scenario
        matched_scenario = find_matching_scenario(order_time, scenario_ranges)

        assigned_pickup_id = 0
        assigned_dropoff_id = 0
        found_assignment = false

        if matched_scenario !== nothing && origin_id != 0 && target_id != 0
            key = if route_model
                scenario_start = scenario_ranges[matched_scenario][1]
                time_id = compute_order_time_id(order_time, scenario_start, something(time_window_sec))
                (matched_scenario, time_id, origin_id, target_id)
            else
                (matched_scenario, origin_id, target_id)
            end
            if haskey(assignment_lookup, key)
                assigned_pickup_id, assigned_dropoff_id = assignment_lookup[key]
                found_assignment = true
            elseif route_model
                error(
                    "No exact route assignment found for order $(row.order_id): " *
                    "scenario=$(matched_scenario), time_id=$(key[2]), origin_id=$origin_id, dest_id=$target_id, " *
                    "order_time=$(row.order_time)"
                )
            end
        elseif route_model
            error(
                "No matching scenario found for route assignment on order $(row.order_id) at $(row.order_time)"
            )
        end
        matched_scenario === nothing && (n_missing_scenario += 1)
        (origin_id == 0 || target_id == 0) && (n_missing_station_id += 1)

        # Fallback: closest selected station
        if !found_assignment
            selected_ids = get_selected_station_ids(
                order_time, stations_df, all_columns, timeframe_columns, use_timeframes
            )
            if !isnothing(fallback_walking_costs) && !isnothing(fallback_routing_costs)
                assigned_pickup_id, assigned_dropoff_id = find_best_feasible_station_pair(
                    origin_id,
                    target_id,
                    selected_ids,
                    fallback_walking_costs,
                    fallback_routing_costs,
                    max_walking_distance,
                    in_vehicle_time_weight,
                )
            else
                assigned_pickup_id = find_closest_selected_station(
                    origin_id, selected_ids, distance_matrix)
                assigned_dropoff_id = find_closest_selected_station(
                    target_id, selected_ids, distance_matrix)
            end
            n_fallback += 1
            if !isnothing(period_idx)
                fallback_by_period[period_idx] = get(fallback_by_period, period_idx, 0) + 1
            end
        else
            n_assigned += 1
            if !isnothing(period_idx)
                x_assigned_by_period[period_idx] = get(x_assigned_by_period, period_idx, 0) + 1
            end

            # Compare x-assignment against what closest-station would have given
            selected_ids = get_selected_station_ids(
                order_time, stations_df, all_columns, timeframe_columns, use_timeframes
            )

            closest_pickup = find_closest_selected_station(origin_id, selected_ids, distance_matrix)
            closest_dropoff = find_closest_selected_station(target_id, selected_ids, distance_matrix)

            pickup_diff = assigned_pickup_id != closest_pickup
            dropoff_diff = assigned_dropoff_id != closest_dropoff
            if pickup_diff
                n_pickup_differs += 1
            end
            if dropoff_diff
                n_dropoff_differs += 1
            end
            if pickup_diff || dropoff_diff
                n_either_differs += 1
            end
        end

        push!(assigned_pickup_ids, assigned_pickup_id)
        push!(assigned_dropoff_ids, assigned_dropoff_id)
    end

    _ensure_assignment_columns!(output_df, origin_ids, dest_ids, assigned_pickup_ids, assigned_dropoff_ids)

    println("✓ Successfully transformed $(nrow(output_df)) orders")
    println("\nSummary:")
    println("  - Mode: x-variable assignments ($method)")
    println("  - Orders with x-assignment: $n_assigned")
    println("  - Orders with fallback (closest station): $n_fallback")
    println("  - Orders with assigned pickup: $(count(row -> row.assigned_pickup_id != 0, eachrow(output_df)))")
    println("  - Orders with assigned dropoff: $(count(row -> row.assigned_dropoff_id != 0, eachrow(output_df)))")
    println("  - Orders fully assigned: $(count(row -> row.assigned_pickup_id != 0 && row.assigned_dropoff_id != 0, eachrow(output_df)))")
    # Build assignment stats
    assignment_stats = Dict{String, Any}(
        "n_x_assigned" => n_assigned,
        "n_fallback" => n_fallback,
        "n_pickup_differs_from_closest" => n_pickup_differs,
        "n_dropoff_differs_from_closest" => n_dropoff_differs,
        "n_either_differs_from_closest" => n_either_differs,
        "n_missing_scenario" => n_missing_scenario,
        "n_missing_station_id" => n_missing_station_id,
        "n_total_orders" => nrow(output_df),
        "assignment_coverage" => nrow(output_df) > 0 ? n_assigned / nrow(output_df) : 0.0,
        "pct_pickup_differs" => n_assigned > 0 ? round(100.0 * n_pickup_differs / n_assigned, digits=1) : 0.0,
        "pct_dropoff_differs" => n_assigned > 0 ? round(100.0 * n_dropoff_differs / n_assigned, digits=1) : 0.0,
        "pct_either_differs" => n_assigned > 0 ? round(100.0 * n_either_differs / n_assigned, digits=1) : 0.0,
        "orders_by_period" => _json_int_dict(orders_by_period),
        "x_assigned_by_period" => _json_int_dict(x_assigned_by_period),
        "fallback_by_period" => _json_int_dict(fallback_by_period),
    )

    if n_assigned > 0
        println("\n  Comparison vs. closest-station (z-variable) assignment:")
        println("  - Pickup differs from closest:  $n_pickup_differs / $n_assigned ($(assignment_stats["pct_pickup_differs"])%)")
        println("  - Dropoff differs from closest: $n_dropoff_differs / $n_assigned ($(assignment_stats["pct_dropoff_differs"])%)")
        println("  - Either differs from closest:  $n_either_differs / $n_assigned ($(assignment_stats["pct_either_differs"])%)")
    end

    return output_df, assignment_stats
end

"""
    split_transformed_orders_by_day(df::DataFrame, output_dir::String;
        start_date=nothing, end_date=nothing) -> DataFrame

Write one transformed order CSV per calendar day and return a manifest.
"""
function split_transformed_orders_by_day(df::DataFrame,
                                         output_dir::String;
                                         start_date::Union{DateTime, Nothing}=nothing,
                                         end_date::Union{DateTime, Nothing}=nothing)::DataFrame
    mkpath(output_dir)
    order_dates = if isempty(df)
        Date[]
    else
        Date.(DateTime.(string.(df.order_time), "yyyy-mm-dd HH:MM:SS"))
    end

    unique_dates = if !isnothing(start_date) && !isnothing(end_date)
        collect(Date(start_date):Day(1):Date(end_date))
    else
        sort(unique(order_dates))
    end
    isempty(unique_dates) && return DataFrame(
        day_index=Int[], date=String[], order_count=Int[], orders_file=String[]
    )

    rows = NamedTuple[]
    for (day_index, day) in enumerate(unique_dates)
        mask = order_dates .== day
        day_df = df[mask, :]
        day_file = joinpath(output_dir, "orders_$(Dates.format(day, "yyyy-mm-dd")).csv")
        CSV.write(day_file, day_df)
        push!(rows, (
            day_index = day_index,
            date = string(day),
            order_count = nrow(day_df),
            orders_file = day_file,
        ))
    end

    return DataFrame(rows)
end

"""
    transform_orders_for_month_backtest(order_file, selection_run_dir, cluster_file, method;
        output_dir, start_date, end_date, scenario_profile, use_timeframes, time_window_sec)
        -> (DataFrame, Dict, DataFrame)

Transform a full backtest month using solved assignments, write the month-level
CSV plus one transformed file per day, and return the transformed orders, stats,
and daily manifest.
"""
function transform_orders_for_month_backtest(order_file::String,
                                             selection_run_dir::String,
                                             cluster_file::String,
                                             method::String;
                                             output_dir::String,
                                             start_date::Union{DateTime, Nothing}=nothing,
                                             end_date::Union{DateTime, Nothing}=nothing,
                                             scenario_profile::Symbol=:four_period,
                                             use_timeframes::Bool=false,
                                             time_window_sec::Union{Int, Nothing}=nothing,
                                             segment_file::Union{String, Nothing}=nothing,
                                             max_walking_distance::Float64=Inf,
                                             in_vehicle_time_weight::Float64=1.0)
    mkpath(output_dir)

    transformed_df, stats = transform_orders_from_assignments(
        order_file,
        selection_run_dir,
        cluster_file,
        method;
        start_date=start_date,
        end_date=end_date,
        use_timeframes=use_timeframes,
        time_window_sec=time_window_sec,
        scenario_profile=scenario_profile,
        segment_file=segment_file,
        max_walking_distance=max_walking_distance,
        in_vehicle_time_weight=in_vehicle_time_weight,
    )

    month_file = joinpath(output_dir, "orders_transformed_month.csv")
    CSV.write(month_file, transformed_df)

    daily_dir = joinpath(output_dir, "daily_orders")
    manifest_df = split_transformed_orders_by_day(
        transformed_df,
        daily_dir;
        start_date=start_date,
        end_date=end_date,
    )
    manifest_file = joinpath(output_dir, "daily_manifest.csv")
    CSV.write(manifest_file, manifest_df)

    stats["month_order_file"] = month_file
    stats["daily_manifest_file"] = manifest_file
    stats["n_daily_files"] = nrow(manifest_df)
    stats["days"] = manifest_df.date

    open(joinpath(output_dir, "assignment_stats.json"), "w") do io
        JSON.print(io, stats, 2)
    end

    return transformed_df, stats, manifest_df
end


"""
    remap_order_times_stacked(df, scenario_ranges, base_date) -> (DataFrame, Float64)

Remap the `order_time` column so that all scenarios are concatenated back-to-back with
no gaps, starting from `base_date`.

Each scenario's orders are shifted by the cumulative duration of all preceding scenarios,
closing any real-time gaps between non-contiguous windows. The calendar date of
`base_date` is arbitrary — only the relative offsets within each scenario are preserved.

Returns the remapped DataFrame and the total stacked duration in seconds.

# Example

Two 4-hour scenarios with a 2-hour gap (08:00–12:00 and 14:00–18:00):
- Scenario 1 orders: keep their offset from 08:00 → placed in [0, 14400) sec
- Scenario 2 orders: gap closed → placed in [14400, 28800) sec
- Total stacked duration = 28800 sec
"""
function remap_order_times_stacked(
    df              :: DataFrame,
    scenario_ranges :: Vector{<:Dict},
    base_date       :: DateTime
)::Tuple{DataFrame, Float64}
    ranges = [(DateTime(r["start"], "yyyy-mm-dd HH:MM:SS"),
               DateTime(r["end"],   "yyyy-mm-dd HH:MM:SS")) for r in scenario_ranges]

    # Cumulative offset (seconds) at the start of each scenario after stacking
    cum_offsets = zeros(Float64, length(ranges))
    for i in 2:length(ranges)
        dur = (ranges[i-1][2] - ranges[i-1][1]).value / 1000.0
        cum_offsets[i] = cum_offsets[i-1] + dur
    end
    total_duration = cum_offsets[end] + (ranges[end][2] - ranges[end][1]).value / 1000.0

    out_df = copy(df)
    for row_i in 1:nrow(out_df)
        order_dt = DateTime(string(out_df[row_i, :order_time]), "yyyy-mm-dd HH:MM:SS")
        for j in 1:length(ranges)
            if ranges[j][1] <= order_dt <= ranges[j][2]
                within_sec = (order_dt - ranges[j][1]).value / 1000.0
                new_dt = base_date + Dates.Millisecond(round(Int, (cum_offsets[j] + within_sec) * 1000))
                out_df[row_i, :order_time] = Dates.format(new_dt, "yyyy-mm-dd HH:MM:SS")
                break
            end
        end
    end
    return out_df, total_duration
end

"""
    transform_orders_quick_extend(order_file, selection_run_dir, cluster_file,
                                  uncovered_ranges, segment_file, max_walking_distance)
        -> DataFrame

Quick Transform for (date, time_window) pairs not covered by original selection scenarios.

`uncovered_ranges` is a vector of `(window_start, window_end, scenario_idx)` tuples, where
`scenario_idx` identifies which z* active-station set to use for that window (matched by
time-of-day to an original scenario).

For each order whose `order_time` falls within an uncovered range, assign the
minimum-cost walking-feasible `(j,k)` pair:

    argmin walk(o→j) + walk(k→d) + λ·route(j→k)

subject to both walking legs being within `max_walking_distance`. The active set
Z*_s is used first, and the routine falls back to Y* (built stations) only when
no feasible active pair exists.

Reads from `selection_run_dir/variable_exports/`:
  - `scenario_activation.csv`  — z*_{js} (active stations per scenario)
"""
function transform_orders_quick_extend(
    order_file           :: String,
    selection_run_dir    :: String,
    cluster_file         :: String,
    uncovered_ranges     :: Vector{Tuple{DateTime, DateTime, Int}},
    segment_file         :: String,
    max_walking_distance :: Float64;
    in_vehicle_time_weight::Float64 = 1.0
)
    isempty(uncovered_ranges) && return DataFrame()

    # Load all orders; keep only those falling in an uncovered range
    orders_df = CSV.read(order_file, DataFrame)
    orders_df.order_time_parsed = DateTime.(orders_df.order_time, "yyyy-mm-dd HH:MM:SS")
    orders_df = filter(r -> any(w -> w[1] <= r.order_time_parsed <= w[2], uncovered_ranges),
                       orders_df)
    println("  Quick Transform: $(nrow(orders_df)) orders across $(length(uncovered_ranges)) uncovered windows")

    # Load z* (active stations per scenario)
    activation_file = joinpath(selection_run_dir, "variable_exports", "scenario_activation.csv")
    isfile(activation_file) || error("scenario_activation.csv not found: $activation_file")
    activation_df = CSV.read(activation_file, DataFrame)
    z_star = Dict{Int, Set{Int}}()
    for row in eachrow(activation_df)
        row.value >= 0.5 || continue
        push!(get!(z_star, row.scenario_idx, Set{Int}()), row.station_id)
    end

    # Load y* (built stations: selected == 1 in cluster_file)
    stations_df = CSV.read(cluster_file, DataFrame)
    y_star = Set{Int}(r.id for r in eachrow(stations_df)
                      if hasproperty(r, :selected) && r.selected >= 0.5)

    walking_costs = precompute_walking_costs(stations_df)
    routing_costs = read_routing_costs_from_segments(segment_file, stations_df)

    transformed_orders = []
    n_z = 0; n_y = 0; n_dropped = 0

    for row in eachrow(orders_df)
        dt = row.order_time_parsed
        # Find which uncovered range this order belongs to → get its scenario_idx
        range_match = findfirst(w -> w[1] <= dt <= w[2], uncovered_ranges)
        range_match === nothing && continue
        s_idx = uncovered_ranges[range_match][3]

        origin_id = _row_origin_station_id(row)
        target_id = _row_destination_station_id(row)

        z_s = get(z_star, s_idx, Set{Int}())

        assigned_pickup_id, assigned_dropoff_id = find_best_feasible_station_pair(
            origin_id,
            target_id,
            collect(z_s),
            walking_costs,
            routing_costs,
            max_walking_distance,
            in_vehicle_time_weight,
        )

        used_z_star = assigned_pickup_id != 0 && assigned_dropoff_id != 0
        if !used_z_star
            assigned_pickup_id, assigned_dropoff_id = find_best_feasible_station_pair(
                origin_id,
                target_id,
                collect(y_star),
                walking_costs,
                routing_costs,
                max_walking_distance,
                in_vehicle_time_weight,
            )
        end

        # Step 3: if still unassigned after y* fallback, drop the order entirely.
        # (origin/target is isolated — no built station reachable within max_walking_distance)
        if assigned_pickup_id == 0 || assigned_dropoff_id == 0
            n_dropped += 1
            continue
        end

        used_z_star ? (n_z += 1) : (n_y += 1)

        push!(transformed_orders, (
            order_id   = row.order_id,
            pax_num    = row.pax_num,
            order_time = row.order_time,
            origin_station_id = origin_id,
            destination_station_id = target_id,
            assigned_pickup_id  = assigned_pickup_id,
            assigned_dropoff_id = assigned_dropoff_id
        ))
    end

    println("    Assigned: $n_z fully via z* | $n_y at least one side via y* fallback | $n_dropped dropped (no built station reachable within $(max_walking_distance)s)")
    return DataFrame(transformed_orders)
end
