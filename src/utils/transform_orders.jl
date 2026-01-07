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

export transform_orders,
       parse_station_list,
       precompute_distances,
       find_closest_selected_station,
       get_timeframe_column

"""
    parse_station_list(list_str::String) -> Vector{Int}

Parse a string like "[1,2,3]" into a vector of integers.

# Arguments
- `list_str`: String representation of station list (e.g., "[1,2,3]")

# Returns
- Vector{Int} of station IDs, or empty vector if invalid
"""
function parse_station_list(list_str)
    cleaned = replace(string(list_str), "[" => "", "]" => "")
    if isempty(cleaned) || cleaned == "missing"
        return Int[]
    end
    return parse.(Int, split(cleaned, ","))
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

"""
    transform_orders(order_file::String,
                    cluster_file::String;
                    start_date::Union{DateTime, Nothing}=nothing,
                    end_date::Union{DateTime, Nothing}=nothing,
                    use_timeframes::Bool=false) -> DataFrame

Transform orders by assigning pickup and dropoff to selected stations.

This function:
1. Reads original orders with available pickup/dropoff stations
2. Reads station selection results (with 'selected' column or timeframe columns)
3. Optionally filters orders by date range
4. For each order, finds the closest selected station for pickup and dropoff
5. Returns transformed orders with assigned_pickup_id and assigned_dropoff_id

# Arguments
- `order_file`: Path to order CSV file with columns:
  - order_id, pax_num, order_time
  - available_pickup_station_list, available_pickup_walkingtime_list
  - available_dropoff_station_list, available_dropoff_walkingtime_list
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

        # Parse station lists
        available_pickup_stations = parse_station_list(string(row.available_pickup_station_list))
        available_dropoff_stations = parse_station_list(string(row.available_dropoff_station_list))

        # Determine origin_id and target_id (first available station)
        origin_id = length(available_pickup_stations) > 0 ? available_pickup_stations[1] : 0
        target_id = length(available_dropoff_stations) > 0 ? available_dropoff_stations[1] : 0

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
            origin_id = origin_id,
            target_id = target_id,
            assigned_pickup_id = assigned_pickup_id,
            assigned_dropoff_id = assigned_dropoff_id,
            available_pickup_station_list = row.available_pickup_station_list,
            available_pickup_walkingtime_list = row.available_pickup_walkingtime_list,
            available_dropoff_station_list = row.available_dropoff_station_list,
            available_dropoff_walkingtime_list = row.available_dropoff_walkingtime_list
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
