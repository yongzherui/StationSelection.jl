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
       transform_orders_from_assignments,
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

"""
    transform_orders_from_assignments(order_file, selection_run_dir, cluster_file, method;
        start_date, end_date, use_timeframes, time_window_sec) -> DataFrame

Transform orders using x-variable assignments exported from the optimization model.

Instead of assigning each order to the closest selected station, this function uses
the actual assignment decisions (x variables) from `variable_exports/assignment_variables.csv`.
Orders without a matching assignment fall back to closest selected station.

Lookup keys differ by model:
- **TSD** (`TwoStageSingleDetourModel`): `(scenario, time_id, origin_id, dest_id)`
- **Clustering** (`ClusteringTwoStageODModel`): `(scenario, origin_id, dest_id)`

# Arguments
- `order_file`: Path to order CSV
- `selection_run_dir`: Path to the selection run directory (contains `variable_exports/`)
- `cluster_file`: Path to station selection results CSV (for fallback)
- `method`: `"TwoStageSingleDetourModel"` or `"ClusteringTwoStageODModel"`
- `start_date`: Optional start date filter
- `end_date`: Optional end date filter
- `use_timeframes`: Whether to use timeframe columns for fallback
- `time_window_sec`: Time discretization in seconds (for TSD, default 120)

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
                                            time_window_sec::Int=120)
    is_tsd = (method == "TwoStageSingleDetourModel")

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
        s_start = DateTime(row.start_time)
        s_end = DateTime(row.end_time)
        scenario_ranges[row.scenario_idx] = (s_start, s_end)
    end

    # 6. Build assignment lookup dict
    if is_tsd
        assignment_lookup = Dict{Tuple{Int,Int,Int,Int}, Tuple{Int,Int}}()
        for row in eachrow(assignments_df)
            key = (row.scenario, row.time_id, row.origin_id, row.dest_id)
            assignment_lookup[key] = (row.pickup_id, row.dropoff_id)
        end
        println("Built TSD assignment lookup with $(length(assignment_lookup)) entries")
    else
        assignment_lookup = Dict{Tuple{Int,Int,Int}, Tuple{Int,Int}}()
        for row in eachrow(assignments_df)
            key = (row.scenario, row.origin_id, row.dest_id)
            assignment_lookup[key] = (row.pickup_id, row.dropoff_id)
        end
        println("Built Clustering assignment lookup with $(length(assignment_lookup)) entries")
    end

    # 7. Precompute distances for fallback
    println("\nPrecomputing distances between all stations...")
    distance_matrix = precompute_distances(stations_df)
    println("✓ Computed $(length(distance_matrix)) pairwise distances")

    # Identify timeframe columns for fallback
    all_columns = names(stations_df)
    timeframe_columns = filter(col -> occursin(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}_\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}", col), all_columns)

    # 8. Process each order
    println("\nProcessing orders...")
    transformed_orders = []
    n_assigned = 0
    n_fallback = 0
    n_pickup_differs = 0
    n_dropoff_differs = 0
    n_either_differs = 0

    for (idx, row) in enumerate(eachrow(orders_df))
        if idx % 1000 == 0
            println("  Processing order $idx / $(nrow(orders_df))...")
        end

        # Parse station lists
        available_pickup_stations = parse_station_list(string(row.available_pickup_station_list))
        available_dropoff_stations = parse_station_list(string(row.available_dropoff_station_list))

        origin_id = length(available_pickup_stations) > 0 ? available_pickup_stations[1] : 0
        target_id = length(available_dropoff_stations) > 0 ? available_dropoff_stations[1] : 0

        # Parse order time
        order_time = DateTime(row.order_time, "yyyy-mm-dd HH:MM:SS")

        # Find matching scenario
        matched_scenario = nothing
        for (s_idx, (s_start, s_end)) in scenario_ranges
            if order_time >= s_start && order_time <= s_end
                matched_scenario = s_idx
                break
            end
        end

        assigned_pickup_id = 0
        assigned_dropoff_id = 0
        found_assignment = false

        if matched_scenario !== nothing && origin_id != 0 && target_id != 0
            if is_tsd
                scenario_start = scenario_ranges[matched_scenario][1]
                time_diff_seconds = (order_time - scenario_start) / Dates.Second(1)
                time_id = floor(Int, time_diff_seconds / time_window_sec)
                key = (matched_scenario, time_id, origin_id, target_id)
                if haskey(assignment_lookup, key)
                    assigned_pickup_id, assigned_dropoff_id = assignment_lookup[key]
                    found_assignment = true
                end
            else
                key = (matched_scenario, origin_id, target_id)
                if haskey(assignment_lookup, key)
                    assigned_pickup_id, assigned_dropoff_id = assignment_lookup[key]
                    found_assignment = true
                end
            end
        end

        # Fallback: closest selected station
        if !found_assignment
            if use_timeframes && !isempty(timeframe_columns)
                timeframe_col = get_timeframe_column(order_time, timeframe_columns)
                if timeframe_col !== nothing
                    selected_ids = [r.id for r in eachrow(stations_df) if r[timeframe_col] == 1.0 || r[timeframe_col] == 1]
                else
                    selected_ids = Int[]
                end
            else
                if "selected" in all_columns
                    selected_ids = [r.id for r in eachrow(stations_df) if r.selected == 1]
                else
                    selected_ids = Int[]
                end
            end

            assigned_pickup_id = find_closest_selected_station(
                origin_id, selected_ids, distance_matrix)
            assigned_dropoff_id = find_closest_selected_station(
                target_id, selected_ids, distance_matrix)
            n_fallback += 1
        else
            n_assigned += 1

            # Compare x-assignment against what closest-station would have given
            if use_timeframes && !isempty(timeframe_columns)
                timeframe_col = get_timeframe_column(order_time, timeframe_columns)
                if timeframe_col !== nothing
                    selected_ids = [r.id for r in eachrow(stations_df) if r[timeframe_col] == 1.0 || r[timeframe_col] == 1]
                else
                    selected_ids = Int[]
                end
            else
                if "selected" in all_columns
                    selected_ids = [r.id for r in eachrow(stations_df) if r.selected == 1]
                else
                    selected_ids = Int[]
                end
            end

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

    output_df = DataFrame(transformed_orders)

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
        "pct_pickup_differs" => n_assigned > 0 ? round(100.0 * n_pickup_differs / n_assigned, digits=1) : 0.0,
        "pct_dropoff_differs" => n_assigned > 0 ? round(100.0 * n_dropoff_differs / n_assigned, digits=1) : 0.0,
        "pct_either_differs" => n_assigned > 0 ? round(100.0 * n_either_differs / n_assigned, digits=1) : 0.0,
    )

    if n_assigned > 0
        println("\n  Comparison vs. closest-station (z-variable) assignment:")
        println("  - Pickup differs from closest:  $n_pickup_differs / $n_assigned ($(assignment_stats["pct_pickup_differs"])%)")
        println("  - Dropoff differs from closest: $n_dropoff_differs / $n_assigned ($(assignment_stats["pct_dropoff_differs"])%)")
        println("  - Either differs from closest:  $n_either_differs / $n_assigned ($(assignment_stats["pct_either_differs"])%)")
    end

    return output_df, assignment_stats
end
