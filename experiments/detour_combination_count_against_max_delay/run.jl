"""
Experiment script to count detour combinations across max delay values.

Usage:
    julia --project=. experiments/detour_combination_count_against_max_delay/run.jl --config <config_file>

Optional:
    --max-delays "0,60,120"  # explicit list (seconds)
    --max-delay-start 0 --max-delay-end 1200 --max-delay-step 60
"""

using ArgParse
using StationSelection
using DataFrames: nrow, DataFrame
using CSV
using Combinatorics
using TOML

const PROJECT_ROOT = dirname(dirname(dirname(@__FILE__)))

function parse_commandline()
    s = ArgParseSettings(
        description = "Count detour combinations across max delay values",
        prog = "run.jl"
    )

    @add_arg_table! s begin
        "--config", "-c"
            help = "Path to configuration TOML file"
            arg_type = String
            default = joinpath(PROJECT_ROOT, "example/config.toml")
        "--max-delays"
            help = "Comma-separated list of max delay values in seconds (overrides start/end/step)"
            arg_type = String
            default = ""
            dest_name = "max_delays"
        "--max-delay-start"
            help = "Start value for max delay sweep in seconds"
            arg_type = Float64
            default = 0.0
            dest_name = "max_delay_start"
        "--max-delay-end"
            help = "End value for max delay sweep in seconds"
            arg_type = Float64
            default = 1200.0
            dest_name = "max_delay_end"
        "--max-delay-step"
            help = "Step size for max delay sweep in seconds"
            arg_type = Float64
            default = 60.0
            dest_name = "max_delay_step"
        "--output", "-o"
            help = "Output CSV path"
            arg_type = String
            default = joinpath(@__DIR__, "detour_counts.csv")
    end

    return parse_args(s)
end

function parse_max_delays(args, default_max_delay::Float64)
    if !isempty(args["max_delays"])
        parts = split(args["max_delays"], ",")
        return [parse(Float64, strip(p)) for p in parts if !isempty(strip(p))]
    end

    start_val = args["max_delay_start"]
    end_val = args["max_delay_end"]
    step_val = args["max_delay_step"]

    if step_val <= 0
        error("max-delay-step must be > 0")
    end
    if end_val < start_val
        error("max-delay-end must be >= max-delay-start")
    end

    return collect(start_val:step_val:end_val)
end

function main()
    args = parse_commandline()

    println("=" ^ 60)
    println("Detour Combination Count vs Max Delay")
    println("=" ^ 60)

    # Load configuration
    println("\n[1] Loading configuration from: $(args["config"]) ")
    config = TOML.parsefile(args["config"])

    # Resolve file paths
    station_file = joinpath(PROJECT_ROOT, config["paths"]["station_file"])
    order_file = joinpath(PROJECT_ROOT, config["paths"]["order_file"])
    segment_file = joinpath(PROJECT_ROOT, config["paths"]["segment_file"])

    # Load data
    println("\n[2] Loading data...")
    stations = read_candidate_stations(station_file)
    println("  - Loaded $(nrow(stations)) stations")

    requests = read_customer_requests(
        order_file;
        start_time=config["scenario"]["start_time"],
        end_time=config["scenario"]["end_time"]
    )
    println("  - Using full station list: $(nrow(stations)) stations")
    println("  - Loaded $(nrow(requests)) requests")

    # Compute costs
    println("\n[3] Computing costs...")
    walking_costs = compute_station_pairwise_costs(stations)
    println("  - Computed $(length(walking_costs)) walking cost entries")

    routing_costs = read_routing_costs_from_segments(segment_file, stations)
    println("  - Computed $(length(routing_costs)) routing cost entries")

    # Create StationSelectionData
    println("\n[4] Creating StationSelectionData...")
    scenarios = [(config["scenario"]["start_time"], config["scenario"]["end_time"])]

    data = create_station_selection_data(
        stations,
        requests,
        walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )
    println("  - Number of stations: $(data.n_stations)")
    println("  - Number of scenarios: $(n_scenarios(data))")

    mc = config["model"]
    k = mc["k"]
    l = mc["l"]
    vehicle_routing_weight = mc["vehicle_routing_weight"]
    time_window = get(mc, "time_window", nothing)
    routing_delay = get(mc, "routing_delay", nothing)
    if isnothing(time_window) || isnothing(routing_delay)
        error("Config model section must include time_window and routing_delay for detour experiments.")
    end
    default_max_delay = Float64(routing_delay)

    max_delays = parse_max_delays(args, default_max_delay)
    total_triplets = binomial(data.n_stations, 3)

    println("\n[5] Sweeping max delay values...")
    println("  - Using $(length(max_delays)) values")
    println("  - Total possible triplets (n choose 3): $total_triplets")

    results = DataFrame(
        max_delay = Float64[],
        same_source_count = Int[],
        same_dest_count = Int[],
        total_triplets = Int[],
        same_source_proportion = Float64[],
        same_dest_proportion = Float64[]
    )

    for max_delay in max_delays
        model = TwoStageSingleDetourModel(
            k, l, vehicle_routing_weight,
            time_window, max_delay;
            in_vehicle_time_weight=vehicle_routing_weight,
            tight_constraints=true
        )

        same_source = find_same_source_detour_combinations(model, data)
        same_dest = find_same_dest_detour_combinations(model, data)

        same_source_count = length(same_source)
        same_dest_count = length(same_dest)
        same_source_prop = total_triplets == 0 ? 0.0 : same_source_count / total_triplets
        same_dest_prop = total_triplets == 0 ? 0.0 : same_dest_count / total_triplets

        push!(results, (
            max_delay,
            same_source_count,
            same_dest_count,
            total_triplets,
            same_source_prop,
            same_dest_prop
        ))

        println("  - max_delay=$(max_delay)s: same_source=$(same_source_count) ($(round(same_source_prop, digits=6))), same_dest=$(same_dest_count) ($(round(same_dest_prop, digits=6)))")
    end

    output_path = args["output"]
    CSV.write(output_path, results)
    println("\n✓ Wrote results to: $output_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
