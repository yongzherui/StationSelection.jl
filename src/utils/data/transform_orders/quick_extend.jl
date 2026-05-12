"""
    transform_orders_quick_extend(order_file, selection_run_dir, cluster_file,
                                  uncovered_ranges, segment_file, max_walking_distance)
        -> DataFrame

Quick Transform for (date, time_window) pairs not covered by original selection scenarios.
"""
function transform_orders_quick_extend(
    order_file           :: String,
    selection_run_dir    :: String,
    cluster_file         :: String,
    uncovered_ranges     :: Vector{Tuple{DateTime, DateTime, Int}},
    segment_file         :: String,
    max_walking_distance :: Float64
)
    isempty(uncovered_ranges) && return DataFrame()

    orders_df = CSV.read(order_file, DataFrame)
    orders_df.order_time_parsed = DateTime.(orders_df.order_time, "yyyy-mm-dd HH:MM:SS")
    orders_df = filter(r -> any(w -> w[1] <= r.order_time_parsed <= w[2], uncovered_ranges),
                       orders_df)
    println("  Quick Transform: $(nrow(orders_df)) orders across $(length(uncovered_ranges)) uncovered windows")

    activation_file = joinpath(selection_run_dir, "variable_exports", "scenario_activation.csv")
    isfile(activation_file) || error("scenario_activation.csv not found: $activation_file")
    activation_df = CSV.read(activation_file, DataFrame)
    z_star = Dict{Int, Set{Int}}()
    for row in eachrow(activation_df)
        row.value >= 0.5 || continue
        push!(get!(z_star, row.scenario_idx, Set{Int}()), row.station_id)
    end

    stations_df = CSV.read(cluster_file, DataFrame)
    y_star = Set{Int}(r.id for r in eachrow(stations_df)
                      if hasproperty(r, :selected) && r.selected >= 0.5)

    seg_df = CSV.read(segment_file, DataFrame)
    walk_reach = Dict{Int, Vector{Tuple{Int, Float64}}}()
    for r in eachrow(seg_df)
        r.seg_time <= max_walking_distance || continue
        push!(get!(walk_reach, r.from_station, Tuple{Int,Float64}[]), (r.to_station, Float64(r.seg_time)))
    end

    all_station_ids = unique(stations_df.id)
    for sid in all_station_ids
        push!(get!(walk_reach, sid, Tuple{Int,Float64}[]), (sid, 0.0))
    end

    function closest_reachable(origin::Int, candidates::Set{Int})
        best_id = 0
        best_time = Inf
        for (sid, t) in get(walk_reach, origin, Tuple{Int,Float64}[])
            sid ∈ candidates || continue
            if t < best_time
                best_time = t
                best_id = sid
            end
        end
        return best_id
    end

    transformed_orders = []
    n_z = 0
    n_y = 0
    n_dropped = 0

    for row in eachrow(orders_df)
        dt = row.order_time_parsed
        range_match = findfirst(w -> w[1] <= dt <= w[2], uncovered_ranges)
        range_match === nothing && continue
        s_idx = uncovered_ranges[range_match][3]

        origin_id = _row_origin_station_id(row)
        target_id = _row_destination_station_id(row)

        z_s = get(z_star, s_idx, Set{Int}())

        assigned_pickup_id = closest_reachable(origin_id, z_s)
        assigned_dropoff_id = closest_reachable(target_id, z_s)

        using_z_pickup = assigned_pickup_id != 0
        using_z_dropoff = assigned_dropoff_id != 0
        if !using_z_pickup
            assigned_pickup_id = closest_reachable(origin_id, y_star)
        end
        if !using_z_dropoff
            assigned_dropoff_id = closest_reachable(target_id, y_star)
        end

        if assigned_pickup_id == 0 || assigned_dropoff_id == 0
            n_dropped += 1
            continue
        end

        used_z_star = using_z_pickup && using_z_dropoff
        used_z_star ? (n_z += 1) : (n_y += 1)

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

    println("    Assigned: $n_z fully via z* | $n_y at least one side via y* fallback | $n_dropped dropped (no built station reachable within $(max_walking_distance)s)")
    return DataFrame(transformed_orders)
end
