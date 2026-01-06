module GenerateScenarios
using Dates

export generate_scenarios

"""
generate_scenarios(start_date::Date, end_date::Date;
                   segment_hours::Int=24,
                   weekly_cycle::Bool=false)

Generates a vector of tuples of `(start_datetime_string, end_datetime_string)` for scenarios.

- `segment_hours` : length of each scenario in hours (1, 2, 4, 6, 24).
- `weekly_cycle` : if true, selects one scenario per week starting from `start_date`.

Returns a Vector of (String, String) tuples suitable for your clustering input.
"""
function generate_scenarios(start_date::Date, end_date::Date;
                            segment_hours::Int=24,
                            weekly_cycle::Bool=false)::Vector{Tuple{String, String}}

    @assert segment_hours > 0 "Segment hours must be positive"
    scenarios = []

    # Create all segments first
    dt_start = DateTime(year(start_date), month(start_date), day(start_date), 0, 0, 0)  # include last day until 23:59:59
    dt_end = DateTime(year(end_date), month(end_date), day(end_date), 23, 59, 59)  # include last day until 23:59:59

    current = dt_start
    while current < dt_end
        seg_end = min(current + Hour(segment_hours) - Second(1), dt_end)
        push!(scenarios, (Dates.format(current, "yyyy-mm-dd HH:MM:SS"),
                          Dates.format(seg_end, "yyyy-mm-dd HH:MM:SS")))
        current += Hour(segment_hours)
    end

    # Apply weekly cycle filter if needed
    if weekly_cycle
        # keep only one scenario per week (Monday as example)
        scenarios = [s for (i, s) in enumerate(scenarios) if dayofweek(Date(DateTime(s[1], "yyyy-mm-dd HH:MM:SS"))) == dayofweek(start_date)]
    end

    return scenarios
end

end # module