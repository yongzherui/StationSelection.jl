"""
Example script to run StationSelection optimization.

Usage:
    julia --project=. example/run.jl --config <config_file>
    julia --project=. example/run.jl -c <config_file>

If no config file is specified, defaults to example/config.toml
"""

using ArgParse
using StationSelection
using DataFrames: nrow, combine, groupby, select
using JSON
using JuMP
using Gurobi
using TOML

const PROJECT_ROOT = dirname(dirname(@__FILE__))

function total_num_constraints(m::Model)
    total = 0
    for (F, S) in list_of_constraint_types(m)
        total += num_constraints(m, F, S)
    end
    return total
end

function parse_commandline()
    s = ArgParseSettings(
        description = "Run StationSelection optimization",
        prog = "run.jl"
    )

    @add_arg_table! s begin
        "--config", "-c"
            help = "Path to configuration TOML file"
            arg_type = String
            default = joinpath(PROJECT_ROOT, "example/config.toml")
        "--station-limit", "-n"
            help = "Limit number of stations for a small test run (0 = no limit)"
            arg_type = Int
            default = 0
            dest_name = "station_limit"
        "--no-optimize"
            help = "Build model and print counts without running optimize!"
            action = :store_true
    end

    return parse_args(s)
end

function main(config_path::String, station_limit::Int, no_optimize::Bool)
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
    if station_limit > 0
        station_counts = combine(
            groupby(
                vcat(
                    select(requests, :start_station_id => :station_id),
                    select(requests, :end_station_id => :station_id)
                ),
                :station_id
            ),
            nrow => :request_count
        )
        station_counts = sort(station_counts, :request_count, rev=true)
        top_station_ids = Set(first(station_counts.station_id, min(station_limit, nrow(station_counts))))
        stations = stations[in.(stations.id, Ref(top_station_ids)), :]
        station_ids = Set(stations.id)
        requests = requests[
            in.(requests.start_station_id, Ref(station_ids)) .&
            in.(requests.end_station_id, Ref(station_ids)),
            :
        ]
    end
    println("  - Filtered to $(nrow(stations)) stations")
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
    optimizer_env = Gurobi.Env()
    silent = get(config["solver"], "silent", true)
    term_status, obj_value, solution, total_runtime_sec, m, variable_counts, constraint_counts, detour_combo_counts =
        run_opt(
            model,
            data;
            optimizer_env=optimizer_env,
            silent=silent,
            show_counts=true,
            return_model=true,
            return_counts=true,
            do_optimize=!no_optimize
        )
    println("  - Variables: $(num_variables(m))")
    println("  - Constraints: $(total_num_constraints(m))")
    solve_time_sec = try
        MOI.get(m, MOI.SolveTimeSec())
    catch
        nothing
    end

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

    metadata = Dict(
        "model" => Dict(
            "name" => "TwoStageSingleDetourModel",
            "k" => model.k,
            "l" => model.l,
            "routing_weight" => model.routing_weight,
            "time_window" => model.time_window,
            "routing_delay" => model.routing_delay
        ),
        "data" => Dict(
            "n_stations" => data.n_stations,
            "n_requests" => nrow(requests),
            "n_scenarios" => n_scenarios(data),
            "n_walking_costs" => length(walking_costs),
            "n_routing_costs" => length(routing_costs)
        ),
        "solve" => Dict(
            "termination_status" => string(term_status),
            "objective_value" => obj_value,
            "solve_time_sec" => solve_time_sec,
            "total_runtime_sec" => total_runtime_sec
        ),
        "variables" => Dict(
            "total" => num_variables(m),
            "by_type" => variable_counts
        ),
        "constraints" => Dict(
            "total" => total_num_constraints(m),
            "by_type" => constraint_counts
        ),
        "detour_combinations" => detour_combo_counts
    )

    metadata_path = joinpath(dirname(abspath(config_path)), "metadata.json")
    open(metadata_path, "w") do io
        JSON.print(io, metadata, 4)
    end
    println("  ✓ Exported metadata: $metadata_path")

    return term_status, obj_value, solution
end

if abspath(PROGRAM_FILE) == @__FILE__
    args = parse_commandline()
    main(args["config"], args["station_limit"], get(args, "no_optimize", false))
end
