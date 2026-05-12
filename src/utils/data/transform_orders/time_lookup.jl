"""
    get_timeframe_column(order_time::DateTime, columns::Vector{String}) -> Union{String, Nothing}

Find the timeframe column that matches the given order time.
"""
function get_timeframe_column(order_time::DateTime, columns::Vector{String})
    for col in columns
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

"""
    remap_order_times_stacked(df, scenario_ranges, base_date) -> (DataFrame, Float64)

Remap the `order_time` column so that all scenarios are concatenated back-to-back with
no gaps, starting from `base_date`.
"""
function remap_order_times_stacked(
    df              :: DataFrame,
    scenario_ranges :: Vector{<:Dict},
    base_date       :: DateTime
)::Tuple{DataFrame, Float64}
    ranges = [(DateTime(r["start"], "yyyy-mm-dd HH:MM:SS"),
               DateTime(r["end"],   "yyyy-mm-dd HH:MM:SS")) for r in scenario_ranges]

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
