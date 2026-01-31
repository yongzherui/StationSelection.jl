"""
Experiment to measure variable/constraint growth vs walking distance limit.

Runs StationSelection model builds with varying max_walking_distance to count
variables and constraints.

Usage:
    julia --project=. experiments/variable_and_constraint_growth_walking_limit/run.jl --config <config_file>
"""

using ArgParse
using StationSelection
using Gurobi
using DataFrames: nrow, combine, groupby, select, DataFrame
using CSV
using TOML
using Random
using Logging

const PROJECT_ROOT = dirname(dirname(dirname(@__FILE__)))

function parse_commandline()
    s = ArgParseSettings(
        description = "Measure variable/constraint growth vs walking distance limit",
        prog = "run.jl"
    )

    @add_arg_table! s begin
        "--config", "-c"
            help = "Path to configuration TOML file"
            arg_type = String
            default = joinpath(PROJECT_ROOT, "experiments/variable_and_constraint_growth_walking_limit/config.toml")
        "--output", "-o"
            help = "Output CSV path"
            arg_type = String
            default = joinpath(@__DIR__, "walking_limit_growth.csv")
    end

    return parse_args(s)
end

function main()
    global_logger(SimpleLogger(stderr, Logging.Error))

    args = parse_commandline()

    println("=" ^ 60)
    println("Walking Distance Limit Growth Experiment")
    println("=" ^ 60)

    # Load configuration
    println("\n[1] Loading configuration from: $(args["config"])")
    config = TOML.parsefile(args["config"])

    # Resolve file paths
    station_file = joinpath(PROJECT_ROOT, config["paths"]["station_file"])
    order_file = joinpath(PROJECT_ROOT, config["paths"]["order_file"])
    segment_file = joinpath(PROJECT_ROOT, config["paths"]["segment_file"])

    # Load data
    println("\n[2] Loading data...")
    stations_all = read_candidate_stations(station_file)
    println("  - Loaded $(nrow(stations_all)) stations")

    requests_all = read_customer_requests(
        order_file;
        start_time=config["scenario"]["start_time"],
        end_time=config["scenario"]["end_time"]
    )
    println("  - Loaded $(nrow(requests_all)) requests")

    # Select stations (use all by default; allow optional limit)
    exp_cfg = config["experiment"]
    station_limit = get(exp_cfg, "n_stations_limit", 0)

    stations = stations_all
    requests = requests_all

    if station_limit > 0
        Random.seed!(42)
        station_counts = combine(
            groupby(
                vcat(
                    select(requests_all, :start_station_id => :station_id),
                    select(requests_all, :end_station_id => :station_id)
                ),
                :station_id
            ),
            nrow => :request_count
        )
        station_counts = sort(station_counts, :request_count, rev=true)
        top_station_ids = Set(first(station_counts.station_id, min(station_limit, nrow(station_counts))))

        stations = stations_all[in.(stations_all.id, Ref(top_station_ids)), :]
        station_ids = Set(stations.id)
        requests = requests_all[
            in.(requests_all.start_station_id, Ref(station_ids)) .&
            in.(requests_all.end_station_id, Ref(station_ids)),
            :
        ]
    end

    println("  - Using $(nrow(stations)) stations and $(nrow(requests)) requests")

    # Compute costs
    walking_costs = compute_station_pairwise_costs(stations)
    routing_costs = read_routing_costs_from_segments(segment_file, stations)

    # Find maximum walking distance needed
    max_walking_cost = maximum(values(walking_costs))
    println("  - Maximum walking distance in data: $(max_walking_cost)m")

    scenarios = [(config["scenario"]["start_time"], config["scenario"]["end_time"])]

    data = create_station_selection_data(
        stations,
        requests,
        walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    model_cfg = config["model"]
    walking_start = Float64(exp_cfg["walking_distance_start"])
    walking_step = Float64(exp_cfg["walking_distance_step"])

    # Generate walking distance values from start to max in increments
    max_limit = max(walking_start, ceil(max_walking_cost / walking_step) * walking_step)
    walking_distances = collect(walking_start:walking_step:max_limit)

    println("\n[3] Computing growth across walking distance limits...")
    println("  - Testing $(length(walking_distances)) walking distance values")
    println("  - Range: $(walking_start)m to $(last(walking_distances))m in $(walking_step)m increments")

    rows = Vector{Dict{Symbol, Any}}()

    env = Gurobi.Env()

    for walking_dist in walking_distances
        print("  - max_walking_distance=$(walking_dist)m ... ")

        model = TwoStageSingleDetourModel(
            model_cfg["k"],
            model_cfg["l"],
            model_cfg["routing_weight"],
            model_cfg["time_window"],
            model_cfg["routing_delay"];
            max_walking_distance=walking_dist,
            tight_constraints=get(model_cfg, "tight_constraints", true)
        )

        start_time = time()
        result = run_opt(
            model,
            data;
            optimizer_env=env,
            silent=true,
            show_counts=false,
            do_optimize=false
        )
        elapsed = time() - start_time

        var_counts = isnothing(result.counts) ? Dict{String, Int}() : result.counts.variables
        con_counts = isnothing(result.counts) ? Dict{String, Int}() : result.counts.constraints
        detour_counts = isnothing(result.counts) ? Dict{String, Int}() : result.counts.extras
        total_vars = isempty(var_counts) ? 0 : sum(values(var_counts))
        total_constraints = isempty(con_counts) ? 0 : sum(values(con_counts))

        assignment_vars = get(var_counts, "assignment", 0)
        detour_vars = get(var_counts, "detour", 0)
        assignment_cons = get(con_counts, "assignment_to_active", 0) + get(con_counts, "assignment_to_flow", 0)
        detour_cons = get(con_counts, "assignment_to_same_source_detour", 0) + get(con_counts, "assignment_to_same_dest_detour", 0)

        row = Dict{Symbol, Any}(
            :walking_distance => walking_dist,
            :n_stations => data.n_stations,
            :n_requests => nrow(requests),
            :total_variables => total_vars,
            :total_constraints => total_constraints,
            :assignment_vars => assignment_vars,
            :detour_vars => detour_vars,
            :assignment_constraints => assignment_cons,
            :detour_constraints => detour_cons,
            :build_time_sec => elapsed
        )

        for (name, count) in var_counts
            row[Symbol("var_" * name)] = count
        end

        for (name, count) in con_counts
            row[Symbol("con_" * name)] = count
        end

        for (name, count) in detour_counts
            row[Symbol("detour_combo_" * name)] = count
        end

        push!(rows, row)

        println("vars=$(total_vars), cons=$(total_constraints), time=$(round(elapsed, digits=2))s")
    end

    results = DataFrame(rows)
    for name in names(results)
        if startswith(String(name), "var_") ||
           startswith(String(name), "con_") ||
           startswith(String(name), "detour_combo_")
            replace!(results[!, name], missing => 0)
        end
    end

    output_path = args["output"]
    CSV.write(output_path, results)
    println("\n✓ Wrote results to: $output_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
