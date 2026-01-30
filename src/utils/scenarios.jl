using Dates

export generate_scenarios
export generate_scenarios_from_ranges
export generate_scenarios_by_datetimes
export generate_scenarios_by_profile

const _SCENARIO_DATETIME_FMT = "yyyy-mm-dd HH:MM:SS"

"""
generate_scenarios(start_date::Date, end_date::Date;
                   segment_hours::Int=24,
                   weekly_cycle::Bool=false,
                   scenario_count::Union{Int, Nothing}=nothing)

Generates a vector of tuples of `(start_datetime_string, end_datetime_string)` for scenarios.

- `segment_hours` : length of each scenario in hours (1, 2, 4, 6, 24).
- `weekly_cycle` : if true, selects one scenario per week starting from `start_date`.
- `scenario_count` : if provided, extend the range forward until this many scenarios are produced.

Returns a Vector of (String, String) tuples suitable for your clustering input.
"""
function generate_scenarios(start_date::Date, end_date::Date;
                            segment_hours::Int=24,
                            weekly_cycle::Bool=false,
                            scenario_count::Union{Int, Nothing}=nothing)::Vector{Tuple{String, String}}
    return generate_scenarios_by_datetimes(
        _as_datetime(start_date),
        _as_datetime(end_date) + Hour(23) + Minute(59) + Second(59);
        segment_hours=segment_hours,
        weekly_cycle=weekly_cycle,
        scenario_count=scenario_count
    )
end

function generate_scenarios(start_dt::DateTime, end_dt::DateTime;
                            segment_hours::Int=24,
                            weekly_cycle::Bool=false,
                            scenario_count::Union{Int, Nothing}=nothing)::Vector{Tuple{String, String}}
    return generate_scenarios_by_datetimes(
        start_dt,
        end_dt;
        segment_hours=segment_hours,
        weekly_cycle=weekly_cycle,
        scenario_count=scenario_count
    )
end

"""
    generate_scenarios_from_ranges(ranges::Vector{Tuple{Any, Any}}) -> Vector{Tuple{String, String}}

Generate scenarios from explicit ranges. Each range can be a tuple of:
- `DateTime`, or
- String formatted as `yyyy-mm-dd HH:MM:SS`.
"""
function generate_scenarios_from_ranges(
    ranges::Vector{Tuple{Any, Any}}
)::Vector{Tuple{String, String}}
    scenarios = Vector{Tuple{String, String}}()
    for (start_val, end_val) in ranges
        start_dt = _as_datetime(start_val)
        end_dt = _as_datetime(end_val)
        end_dt > start_dt || throw(ArgumentError("Scenario end must be after start: $(start_val) -> $(end_val)"))
        push!(scenarios, (Dates.format(start_dt, _SCENARIO_DATETIME_FMT),
                          Dates.format(end_dt, _SCENARIO_DATETIME_FMT)))
    end
    return scenarios
end

"""
    generate_scenarios_by_datetimes(start_dt::DateTime, end_dt::DateTime;
                                    segment_hours::Int=24,
                                    weekly_cycle::Bool=false,
                                    scenario_count::Union{Int, Nothing}=nothing)

Generate scenarios by splitting the [start_dt, end_dt] range into segments.
If `scenario_count` is provided, extends the range forward until enough
scenarios are produced.
"""
function generate_scenarios_by_datetimes(start_dt::DateTime, end_dt::DateTime;
                                         segment_hours::Int=24,
                                         weekly_cycle::Bool=false,
                                         scenario_count::Union{Int, Nothing}=nothing)::Vector{Tuple{String, String}}

    segment_hours > 0 || throw(ArgumentError("segment_hours must be positive"))
    if !isnothing(scenario_count)
        scenario_count > 0 || throw(ArgumentError("scenario_count must be positive"))
    end

    scenarios = Vector{Tuple{String, String}}()
    current_start = start_dt
    current_end = end_dt

    while true
        append!(scenarios, _generate_segments(current_start, current_end; segment_hours=segment_hours, weekly_cycle=weekly_cycle))
        if isnothing(scenario_count) || length(scenarios) >= scenario_count
            break
        end
        # extend forward by the same window length (at least 1 day)
        window_days = max(1, Dates.value(Date(current_end) - Date(current_start)) + 1)
        current_start = current_end + Second(1)
        current_end = current_end + Day(window_days)
    end

    if !isnothing(scenario_count)
        return scenarios[1:scenario_count]
    end
    return scenarios
end

"""
    generate_scenarios_by_profile(start_date::Date, end_date::Date;
                                  profile::Symbol=:full_day,
                                  scenario_count::Union{Int, Nothing}=nothing,
                                  only_weekdays::Union{Bool, Nothing}=nothing)

Generate scenarios using preset daily time windows.
If `scenario_count` is provided, extends forward until enough scenarios are produced.

Supported profiles:
- `:full_day` (00:00-23:59:59)
- `:commute` (07:00-10:00, 16:00-19:00)
- `:morning` (06:00-10:00)
- `:midday` (10:00-14:00)
- `:evening` (16:00-20:00)
- `:night` (20:00-23:59:59)
"""
function generate_scenarios_by_profile(start_date::Date, end_date::Date;
                                       profile::Symbol=:full_day,
                                       scenario_count::Union{Int, Nothing}=nothing,
                                       only_weekdays::Union{Bool, Nothing}=nothing)::Vector{Tuple{String, String}}

    if !isnothing(scenario_count)
        scenario_count > 0 || throw(ArgumentError("scenario_count must be positive"))
    end

    scenarios = Vector{Tuple{String, String}}()
    current_start = start_date
    current_end = end_date

    while true
        append!(scenarios, _generate_profile_windows(current_start, current_end; profile=profile, only_weekdays=only_weekdays))
        if isnothing(scenario_count) || length(scenarios) >= scenario_count
            break
        end
        window_days = max(1, Dates.value(current_end - current_start) + 1)
        current_start = current_end + Day(1)
        current_end = current_end + Day(window_days)
    end

    if !isnothing(scenario_count)
        return scenarios[1:scenario_count]
    end
    return scenarios
end

function _generate_segments(start_dt::DateTime, end_dt::DateTime;
                            segment_hours::Int,
                            weekly_cycle::Bool)::Vector{Tuple{String, String}}
    scenarios = Vector{Tuple{String, String}}()
    current = start_dt
    while current < end_dt
        seg_end = min(current + Hour(segment_hours) - Second(1), end_dt)
        push!(scenarios, (Dates.format(current, _SCENARIO_DATETIME_FMT),
                          Dates.format(seg_end, _SCENARIO_DATETIME_FMT)))
        current += Hour(segment_hours)
    end

    if weekly_cycle
        scenarios = [s for s in scenarios if dayofweek(Date(DateTime(s[1], _SCENARIO_DATETIME_FMT))) == dayofweek(Date(start_dt))]
    end

    return scenarios
end

function _generate_profile_windows(start_date::Date, end_date::Date;
                                   profile::Symbol,
                                   only_weekdays::Union{Bool, Nothing})
    windows = _profile_windows(profile)
    scenarios = Vector{Tuple{String, String}}()

    d = start_date
    while d <= end_date
        if isnothing(only_weekdays) || (only_weekdays && dayofweek(d) in 1:5) || (!only_weekdays && dayofweek(d) in 6:7)
            for (start_h, start_m, end_h, end_m, end_s) in windows
                start_dt = DateTime(year(d), month(d), day(d), start_h, start_m, 0)
                end_dt = DateTime(year(d), month(d), day(d), end_h, end_m, end_s)
                end_dt > start_dt || continue
                push!(scenarios, (Dates.format(start_dt, _SCENARIO_DATETIME_FMT),
                                  Dates.format(end_dt, _SCENARIO_DATETIME_FMT)))
            end
        end
        d += Day(1)
    end

    return scenarios
end

function _profile_windows(profile::Symbol)
    if profile == :full_day
        return [(0, 0, 23, 59, 59)]
    elseif profile == :commute
        return [(7, 0, 10, 0, 0), (16, 0, 19, 0, 0)]
    elseif profile == :morning
        return [(6, 0, 10, 0, 0)]
    elseif profile == :midday
        return [(10, 0, 14, 0, 0)]
    elseif profile == :evening
        return [(16, 0, 20, 0, 0)]
    elseif profile == :night
        return [(20, 0, 23, 59, 59)]
    else
        throw(ArgumentError("Unknown profile: $profile"))
    end
end

function _as_datetime(val)
    if val isa DateTime
        return val
    elseif val isa String
        return DateTime(val, _SCENARIO_DATETIME_FMT)
    elseif val isa Date
        return DateTime(year(val), month(val), day(val), 0, 0, 0)
    else
        throw(ArgumentError("Unsupported scenario datetime value: $(typeof(val))"))
    end
end
