"""
    transform_orders(order_file::String,
                    cluster_file::String;
                    start_date::Union{DateTime, Nothing}=nothing,
                    end_date::Union{DateTime, Nothing}=nothing,
                    use_timeframes::Bool=false) -> DataFrame

Transform orders by assigning pickup and dropoff to selected stations.
"""
function transform_orders(order_file::String,
                         cluster_file::String;
                         start_date::Union{DateTime, Nothing}=nothing,
                         end_date::Union{DateTime, Nothing}=nothing,
                         use_timeframes::Bool=false)
    println("Reading input files...")

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

    stations_df = CSV.read(cluster_file, DataFrame)
    println("Loaded $(nrow(stations_df)) stations from $cluster_file")

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

        if !("selected" in all_columns)
            error("'selected' column not found in cluster file! Set use_timeframes=true or ensure the cluster file has a 'selected' column.")
        end
    end

    println("\nPrecomputing distances between all stations...")
    distance_matrix = precompute_distances(stations_df)
    println("✓ Computed $(length(distance_matrix)) pairwise distances")

    println("\nProcessing orders...")
    transformed_orders = []

    for (idx, row) in enumerate(eachrow(orders_df))
        if idx % 1000 == 0
            println("  Processing order $idx / $(nrow(orders_df))...")
        end

        origin_id = _row_origin_station_id(row)
        target_id = _row_destination_station_id(row)

        if use_timeframes
            order_time = DateTime(row.order_time, "yyyy-mm-dd HH:MM:SS")
            timeframe_col = get_timeframe_column(order_time, timeframe_columns)

            if timeframe_col === nothing
                error("No timeframe column found for order $(row.order_id) at time $(row.order_time)")
            end

            selected_stations = filter(r -> r[timeframe_col] == 1.0 || r[timeframe_col] == 1, stations_df)
        else
            selected_stations = filter(row -> row.selected == 1, stations_df)
        end

        if nrow(selected_stations) == 0
            if use_timeframes
                error("No selected stations found for timeframe at $(row.order_time)")
            else
                error("No selected stations found in cluster file!")
            end
        end

        selected_station_ids = [r.id for r in eachrow(selected_stations)]

        assigned_pickup_id = find_closest_selected_station(
            origin_id,
            selected_station_ids,
            distance_matrix
        )

        assigned_dropoff_id = find_closest_selected_station(
            target_id,
            selected_station_ids,
            distance_matrix
        )

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

    output_df = DataFrame(transformed_orders)

    println("✓ Successfully transformed $(nrow(output_df)) orders")
    println("\nSummary:")
    println("  - Mode: $(use_timeframes ? "Time-frame specific" : "Default selected")")
    println("  - Orders with assigned pickup: $(count(row -> row.assigned_pickup_id != 0, eachrow(output_df)))")
    println("  - Orders with assigned dropoff: $(count(row -> row.assigned_dropoff_id != 0, eachrow(output_df)))")
    println("  - Orders fully assigned: $(count(row -> row.assigned_pickup_id != 0 && row.assigned_dropoff_id != 0, eachrow(output_df)))")

    return output_df
end
