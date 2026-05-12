function _is_time_indexed_assignment_method(method::String)::Bool
    return method in (
        "TwoStageRouteWithTimeModel",
        "RouteVehicleCapacityModel",
        "AlphaRouteModel",
        "RouteFleetLimitModel",
    )
end

function _assignment_time_column(assignments_df::DataFrame, method::String)::Symbol
    assignment_cols = names(assignments_df)
    if "t_id" in assignment_cols
        return :t_id
    elseif "time_id" in assignment_cols
        return :time_id
    end
    error("assignment_variables.csv missing required column 'time_id' or 't_id' for $method")
end

function _build_assignment_lookup(assignments_df::DataFrame, method::String, time_window_sec)
    route_model = _is_time_indexed_assignment_method(method)
    route_model && isnothing(time_window_sec) &&
        error("transform_orders_from_assignments requires time_window_sec for $method")

    assignment_lookup = route_model ?
        Dict{NTuple{4, Int}, Tuple{Int, Int}}() :
        Dict{NTuple{3, Int}, Tuple{Int, Int}}()

    time_col = route_model ? _assignment_time_column(assignments_df, method) : nothing

    for row in eachrow(assignments_df)
        key = route_model ?
            (row.scenario, row[time_col], row.origin_id, row.dest_id) :
            (row.scenario, row.origin_id, row.dest_id)
        assignment_lookup[key] = (row.pickup_id, row.dropoff_id)
    end

    return assignment_lookup, route_model
end

"""
    transform_orders_from_assignments(order_file, selection_run_dir, cluster_file, method;
        start_date, end_date, use_timeframes, time_window_sec) -> DataFrame

Transform orders using x-variable assignments exported from the optimization model.
"""
function transform_orders_from_assignments(order_file::String,
                                            selection_run_dir::String,
                                            cluster_file::String,
                                            method::String;
                                            start_date::Union{DateTime, Nothing}=nothing,
                                            end_date::Union{DateTime, Nothing}=nothing,
                                            use_timeframes::Bool=false,
                                            time_window_sec::Union{Int, Nothing}=nothing)

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

    assignment_file = joinpath(selection_run_dir, "variable_exports", "assignment_variables.csv")
    if !isfile(assignment_file)
        error("Assignment variables file not found: $assignment_file")
    end
    assignments_df = CSV.read(assignment_file, DataFrame)
    assignments_df = filter(row -> row.value >= 0.5, assignments_df)
    println("Loaded $(nrow(assignments_df)) active assignments from $assignment_file")

    scenario_file = joinpath(selection_run_dir, "variable_exports", "scenario_info.csv")
    if !isfile(scenario_file)
        error("Scenario info file not found: $scenario_file")
    end
    scenario_df = CSV.read(scenario_file, DataFrame)
    println("Loaded $(nrow(scenario_df)) scenarios from $scenario_file")

    stations_df = CSV.read(cluster_file, DataFrame)
    println("Loaded $(nrow(stations_df)) stations from $cluster_file")

    scenario_ranges = Dict{Int, Tuple{DateTime, DateTime}}()
    for row in eachrow(scenario_df)
        s_start = parse_exported_datetime(row.start_time, "start_time", row.scenario_idx)
        s_end = parse_exported_datetime(row.end_time, "end_time", row.scenario_idx)
        scenario_ranges[row.scenario_idx] = (s_start, s_end)
    end

    assignment_lookup, route_model = _build_assignment_lookup(assignments_df, method, time_window_sec)
    println("Built $method assignment lookup with $(length(assignment_lookup)) entries")

    println("\nPrecomputing distances between all stations...")
    distance_matrix = precompute_distances(stations_df)
    println("✓ Computed $(length(distance_matrix)) pairwise distances")

    all_columns = names(stations_df)
    timeframe_columns = filter(col -> occursin(r"\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}_\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}", col), all_columns)

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

        origin_id = _row_origin_station_id(row)
        target_id = _row_destination_station_id(row)
        order_time = DateTime(row.order_time, "yyyy-mm-dd HH:MM:SS")
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

        if !found_assignment
            selected_ids = get_selected_station_ids(
                order_time, stations_df, all_columns, timeframe_columns, use_timeframes
            )

            assigned_pickup_id = find_closest_selected_station(
                origin_id, selected_ids, distance_matrix)
            assigned_dropoff_id = find_closest_selected_station(
                target_id, selected_ids, distance_matrix)
            n_fallback += 1
        else
            n_assigned += 1

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
    println("  - Mode: x-variable assignments ($method)")
    println("  - Orders with x-assignment: $n_assigned")
    println("  - Orders with fallback (closest station): $n_fallback")
    println("  - Orders with assigned pickup: $(count(row -> row.assigned_pickup_id != 0, eachrow(output_df)))")
    println("  - Orders with assigned dropoff: $(count(row -> row.assigned_dropoff_id != 0, eachrow(output_df)))")
    println("  - Orders fully assigned: $(count(row -> row.assigned_pickup_id != 0 && row.assigned_dropoff_id != 0, eachrow(output_df)))")
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
