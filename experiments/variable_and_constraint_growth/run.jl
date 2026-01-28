"""
Local experiment to measure variable/constraint growth vs station count.

Runs StationSelection model builds with --no-optimize to count variables and constraints.

Usage:
    julia --project=. experiments/variable_and_constraint_growth/run.jl --config <config_file>

Options:
    --station-limits "10,20,30"   # explicit list
    --output <path>                # CSV output
"""

using ArgParse
using StationSelection
using Gurobi
using DataFrames: nrow, combine, groupby, select, DataFrame
using CSV
using TOML
using Random

const PROJECT_ROOT = dirname(dirname(dirname(@__FILE__)))

function parse_commandline()
    s = ArgParseSettings(
        description = "Measure variable/constraint growth vs station count",
        prog = "run.jl"
    )

    @add_arg_table! s begin
        "--config", "-c"
            help = "Path to configuration TOML file"
            arg_type = String
            default = joinpath(PROJECT_ROOT, "experiments/variable_and_constraint_growth/config.toml")
        "--n-stations"
            help = "Comma-separated list of station counts"
            arg_type = String
            default = "10,20,30,40,50,60,70,84"
        "--output", "-o"
            help = "Output CSV path"
            arg_type = String
            default = joinpath(@__DIR__, "variable_constraint_growth.csv")
    end

    return parse_args(s)
end

function parse_station_limits(arg::String)
    parts = split(arg, ",")
    return [parse(Int, strip(p)) for p in parts if !isempty(strip(p))]
end

function main()
    args = parse_commandline()

    println("=" ^ 60)
    println("Variable/Constraint Growth Experiment")
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
    stations_all = read_candidate_stations(station_file)
    println("  - Loaded $(nrow(stations_all)) stations")

    requests_all = read_customer_requests(
        order_file;
        start_time=config["scenario"]["start_time"],
        end_time=config["scenario"]["end_time"]
    )
    println("  - Loaded $(nrow(requests_all)) requests")

    station_limits = parse_station_limits(args["n-stations"])

    println("\n[3] Computing growth across station limits...")
    Random.seed!(42)
    results = DataFrame(
        n_stations = Int[],
        n_requests = Int[],
        total_variables = Int[],
        total_constraints = Int[],
        build_time_sec = Float64[]
    )

    for station_limit in station_limits
        println("  - n_stations_target=$station_limit")

        stations = stations_all
        requests = requests_all

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
            if length(top_station_ids) < station_limit
                remaining_ids = setdiff(Set(stations_all.id), top_station_ids)
                needed = min(station_limit - length(top_station_ids), length(remaining_ids))
                if needed > 0
                    extra_ids = rand(collect(remaining_ids), needed)
                    union!(top_station_ids, extra_ids)
                end
            end
            stations = stations_all[in.(stations_all.id, Ref(top_station_ids)), :]
            station_ids = Set(stations.id)
            requests = requests[
                in.(requests.start_station_id, Ref(station_ids)) .&
                in.(requests.end_station_id, Ref(station_ids)),
                :
            ]
        end

        walking_costs = compute_station_pairwise_costs(stations)
        routing_costs = read_routing_costs_from_segments(segment_file, stations)

        scenarios = [(config["scenario"]["start_time"], config["scenario"]["end_time"])]

        data = create_station_selection_data(
            stations,
            requests,
            walking_costs;
            routing_costs=routing_costs,
            scenarios=scenarios
        )

        model_cfg = config["model"]
        model_type = get(model_cfg, "type", "TwoStageSingleDetourModel")
        if model_type != "TwoStageSingleDetourModel"
            error("This experiment expects model.type = TwoStageSingleDetourModel (got $model_type)")
        end

        model = TwoStageSingleDetourModel(
            model_cfg["k"],
            model_cfg["l"],
            model_cfg["routing_weight"],
            model_cfg["time_window"],
            model_cfg["routing_delay"]
        )

        start_time = time()
        term_status, obj_value, solution, runtime_sec, m, var_counts, con_counts, detour_counts = run_opt(
            model,
            data;
            optimizer_env=Gurobi.Env(),
            silent=true,
            show_counts=false,
            return_model=true,
            return_counts=true,
            do_optimize=false
        )
        elapsed = time() - start_time

        total_vars = isempty(var_counts) ? 0 : sum(values(var_counts))
        total_constraints = isempty(con_counts) ? 0 : sum(values(con_counts))

        push!(results, (
            data.n_stations,
            nrow(requests),
            total_vars,
            total_constraints,
            elapsed
        ))
    end

    output_path = args["output"]
    CSV.write(output_path, results)
    println("\n✓ Wrote results to: $output_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
