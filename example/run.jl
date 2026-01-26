"""
Example script to run StationSelection optimization.

Usage:
    julia --project=. example/run.jl [config_file]

If no config file is specified, defaults to example/config.toml
"""

using StationSelection
using DataFrames: nrow
using TOML

const PROJECT_ROOT = dirname(dirname(@__FILE__))

function main(config_path::String)
    println("=" ^ 60)
    println("StationSelection Optimization")
    println("=" ^ 60)

    # Load configuration
    println("\n[1] Loading configuration from: $config_path")
    config = TOML.parsefile(config_path)

    # Resolve file paths
    station_file = joinpath(PROJECT_ROOT, config["paths"]["station_file"])
    order_file = joinpath(PROJECT_ROOT, config["paths"]["order_file"])
    segment_file = joinpath(PROJECT_ROOT, config["paths"]["segment_file"])

    # Load data using module functions
    println("\n[2] Loading data...")
    stations = read_candidate_stations(station_file)
    println("  - Loaded $(nrow(stations)) stations")

    requests = read_customer_requests(
        order_file;
        start_time=config["scenario"]["start_time"],
        end_time=config["scenario"]["end_time"]
    )
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

    # Create model
    println("\n[5] Creating TwoStageSingleDetourModel...")
    mc = config["model"]
    model = TwoStageSingleDetourModel(
        mc["k"], mc["l"], mc["routing_weight"],
        mc["time_window"], mc["routing_delay"]
    )
    println("  - k=$(model.k), l=$(model.l), γ=$(model.routing_weight)")
    println("  - time_window=$(model.time_window)s, routing_delay=$(model.routing_delay)s")

    # Run optimization
    println("\n[6] Running optimization...")
    term_status, obj_value, solution = run_opt(
        model, data;
        silent=get(config["solver"], "silent", true)
    )

    # Report results
    println("\n" * "=" ^ 60)
    println("RESULTS")
    println("=" ^ 60)
    println("Termination status: $term_status")

    if !isnothing(obj_value)
        println("Objective value: $obj_value")
        if !isnothing(solution)
            x_val, y_val = solution
            selected = findall(y_val .> 0.5)
            println("Selected stations ($(length(selected))): $selected")
        end
    end

    return term_status, obj_value, solution
end

if abspath(PROGRAM_FILE) == @__FILE__
    config_file = length(ARGS) > 0 ? ARGS[1] : joinpath(PROJECT_ROOT, "example/config.toml")
    main(config_file)
end
