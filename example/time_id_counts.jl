"""
Analyze how many orders fall into each time_id bucket.

Usage:
    julia --project=. example/time_id_counts.jl --config <config_file>

If no config file is specified, defaults to example/config.toml
"""

using ArgParse
using StationSelection
using DataFrames: nrow
using Dates
using TOML

const PROJECT_ROOT = dirname(dirname(@__FILE__))

function parse_commandline()
    s = ArgParseSettings(
        description = "Analyze order counts per time_id",
        prog = "time_id_counts.jl"
    )

    @add_arg_table! s begin
        "--config", "-c"
            help = "Path to configuration TOML file"
            arg_type = String
            default = joinpath(PROJECT_ROOT, "example/config.toml")
    end

    return parse_args(s)
end

function parse_datetime_or_nothing(value)
    if value isa String && !isempty(value)
        return DateTime(value, "yyyy-mm-dd HH:MM:SS")
    end
    return nothing
end

function main(config_path::String)
    println("=" ^ 60)
    println("Time ID Counts")
    println("=" ^ 60)

    println("\n[1] Loading configuration from: $config_path")
    config = TOML.parsefile(config_path)

    order_file = joinpath(PROJECT_ROOT, config["paths"]["order_file"])
    model_cfg = config["model"]
    scenario_cfg = config["scenario"]

    start_dt = parse_datetime_or_nothing(get(scenario_cfg, "start_time", ""))
    end_dt = parse_datetime_or_nothing(get(scenario_cfg, "end_time", ""))

    println("\n[2] Loading requests...")
    requests = read_customer_requests(
        order_file;
        start_time=start_dt,
        end_time=end_dt
    )
    println("  - Loaded $(nrow(requests)) requests")

    time_window = floor(Int, model_cfg["time_window"])
    if time_window <= 0
        error("time_window must be positive; got $time_window")
    end

    if isnothing(start_dt) || isnothing(end_dt)
        start_dt = minimum(requests.request_time)
        end_dt = maximum(requests.request_time)
        label = "all_requests"
    else
        label = "$(scenario_cfg["start_time"])_$(scenario_cfg["end_time"])"
    end

    scenario = create_scenario_data(
        requests,
        label;
        start_time=start_dt,
        end_time=end_dt
    )

    println("\n[3] Computing time_id counts...")
    time_to_od_count = compute_time_to_od_count_mapping(scenario, time_window)
    time_counts = Dict{Int, Int}()
    for (time_id, od_counts) in time_to_od_count
        time_counts[time_id] = sum(values(od_counts))
    end

    println("  - Scenario: $(scenario.label)")
    println("  - Requests: $(nrow(scenario.requests))")
    println("  - Time window: $(time_window)s")
    println("  - Unique time_ids: $(length(time_counts))")
    println("  - time_id counts:")
    for time_id in sort(collect(keys(time_counts)))
        println("    $time_id: $(time_counts[time_id])")
    end

    return time_counts
end

if abspath(PROGRAM_FILE) == @__FILE__
    args = parse_commandline()
    main(args["config"])
end
