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

    # Create model based on type specified in config
    mc = config["model"]
    model_type = get(mc, "type", "TwoStageSingleDetourModel")

    println("\n[5] Creating $model_type...")

    if model_type == "TwoStageSingleDetourModel"
        use_walking_distance_limit = get(mc, "use_walking_distance_limit", false)
        max_walking_distance = get(mc, "max_walking_distance", nothing)
        tight_constraints = get(mc, "tight_constraints", true)
        detour_use_flow_bounds = get(mc, "detour_use_flow_bounds", false)
        model = TwoStageSingleDetourModel(
            mc["k"], mc["l"], mc["routing_weight"],
            mc["time_window"], mc["routing_delay"];
            use_walking_distance_limit=use_walking_distance_limit,
            max_walking_distance=max_walking_distance,
            tight_constraints=tight_constraints,
            detour_use_flow_bounds=detour_use_flow_bounds
        )
        println("  - k=$(model.k), l=$(model.l), γ=$(model.routing_weight)")
        println("  - time_window=$(model.time_window)s, routing_delay=$(model.routing_delay)s")
        println("  - walking_limit=$(model.use_walking_distance_limit), max_walking_distance=$(model.max_walking_distance)")
        println("  - tight_constraints=$(model.tight_constraints)")
        println("  - detour_use_flow_bounds=$(model.detour_use_flow_bounds)")
    elseif model_type == "ClusteringTwoStageODModel"
        use_walking_distance_limit = get(mc, "use_walking_distance_limit", false)
        max_walking_distance = get(mc, "max_walking_distance", nothing)
        variable_reduction = get(mc, "variable_reduction", true)
        tight_constraints = get(mc, "tight_constraints", true)
        model = ClusteringTwoStageODModel(
            mc["k"], mc["l"], mc["routing_weight"];
            use_walking_distance_limit=use_walking_distance_limit,
            max_walking_distance=max_walking_distance,
            variable_reduction=variable_reduction,
            tight_constraints=tight_constraints
        )
        println("  - k=$(model.k), l=$(model.l), λ=$(model.routing_weight)")
        println("  - walking_limit=$(model.use_walking_distance_limit), max_walking_distance=$(model.max_walking_distance)")
        println("  - variable_reduction=$(model.variable_reduction)")
        println("  - tight_constraints=$(model.tight_constraints)")
    elseif model_type == "ClusteringBaseModel"
        model = ClusteringBaseModel(mc["k"])
        println("  - k=$(model.k) (stations to select)")
    else
        error("Unknown model type: $model_type. Supported: TwoStageSingleDetourModel, ClusteringTwoStageODModel, ClusteringBaseModel")
    end

    # Run optimization
    println("\n[6] Running optimization...")
    optimizer_env = Gurobi.Env()
    silent = get(config["solver"], "silent", true)
    result = run_opt(
        model,
        data;
        optimizer_env=optimizer_env,
        silent=silent,
        show_counts=true,
        do_optimize=!no_optimize
    )
    variable_counts = isnothing(result.counts) ? Dict{String, Int}() : result.counts.variables
    constraint_counts = isnothing(result.counts) ? Dict{String, Int}() : result.counts.constraints
    total_vars = isempty(variable_counts) ? num_variables(result.model) : sum(values(variable_counts))
    total_constraints = isempty(constraint_counts) ? total_num_constraints(result.model) : sum(values(constraint_counts))
    println("  - Variables: $total_vars")
    println("  - Constraints: $total_constraints")

    solve_time_sec = result.runtime_sec
    if no_optimize
        println("  - Optimization skipped (--no-optimize)")
    end

    # Report results
    println("\n" * "=" ^ 60)
    println("RESULTS")
    println("=" ^ 60)
    println("Termination status: $(result.termination_status)")

    if !isnothing(result.objective_value)
        println("Objective value: $(result.objective_value)")
        if !isnothing(result.solution)
            x_val, y_val = result.solution
            selected = findall(y_val .> 0.5)
            println("Selected stations ($(length(selected))): $selected")
        end
    end

    # Build model metadata based on type
    model_metadata = Dict{String, Any}(
        "name" => model_type,
        "k" => model.k
    )
    if model_type == "TwoStageSingleDetourModel"
        model_metadata["l"] = model.l
        model_metadata["routing_weight"] = model.routing_weight
        model_metadata["time_window"] = model.time_window
        model_metadata["routing_delay"] = model.routing_delay
        model_metadata["use_walking_distance_limit"] = model.use_walking_distance_limit
        model_metadata["max_walking_distance"] = model.max_walking_distance
    elseif model_type == "ClusteringTwoStageODModel"
        model_metadata["l"] = model.l
        model_metadata["routing_weight"] = model.routing_weight
        model_metadata["use_walking_distance_limit"] = model.use_walking_distance_limit
        model_metadata["max_walking_distance"] = model.max_walking_distance
        model_metadata["variable_reduction"] = model.variable_reduction
    end
    # ClusteringBaseModel only has k, which is already added

    metadata = Dict(
        "model" => model_metadata,
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
            "total" => total_vars,
            "by_type" => variable_counts
        ),
        "constraints" => Dict(
            "total" => total_constraints,
            "by_type" => constraint_counts
        ),
        "extra_counts" => detour_combo_counts
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
