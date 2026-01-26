#!/usr/bin/env julia
"""
Run station selection job for scalability pipeline.

Usage:
    julia 03_run_selection.jl <job_id>
"""

using TOML, Dates, JSON, SHA, Logging
using Pkg

STUDY_DIR = abspath(joinpath(@__DIR__, ".."))
PROJECT_ROOT = abspath(joinpath(STUDY_DIR, "../../.."))

println("="^80)
println("Pipeline Stage: Station Selection")
println("="^80)
println("Study: $STUDY_DIR")
println("Project: $PROJECT_ROOT")
println()

if length(ARGS) < 1
    error("Missing job ID. Usage: julia 03_run_selection.jl <job_id>")
end

job_id = parse(Int, ARGS[1])
println("Job ID: $job_id")
println()

config_file = joinpath(STUDY_DIR, "config", "selection", "job_$(job_id).toml")
if !isfile(config_file)
    error("Configuration file not found: $config_file")
end

cfg = TOML.parsefile(config_file)
println("Loaded configuration from: $config_file")

if haskey(cfg, "parameters")
    println("\nParameters:")
    for (k, v) in cfg["parameters"]
        println("  $k = $v")
    end
end
println()

println("Activating project...")
Pkg.activate(PROJECT_ROOT)

using StationSelection
using Gurobi
using DataFrames: nrow, combine, groupby, select

# Generate run directory
timestamp = Dates.format(now(), "yyyy-mm-dd_HH-MM-SS")
config_str = JSON.json(cfg)
hash = bytes2hex(sha1(Vector{UInt8}(config_str)))[1:6]
name = get(cfg, "name", "selection")

runs_dir = joinpath(STUDY_DIR, "runs")
run_dir = joinpath(runs_dir, "$(timestamp)_job$(job_id)_$(name)_$(hash)")
mkpath(run_dir)

# Progress tracking
progress_file = joinpath(STUDY_DIR, "progress", "job_$(job_id)_progress.json")
progress_data = isfile(progress_file) ? JSON.parsefile(progress_file) : Dict("job_id" => job_id)
progress_data["selection"] = get(progress_data, "selection", Dict{String, Any}())
progress_data["selection"]["status"] = "running"
progress_data["selection"]["started"] = string(now())
progress_data["selection"]["run_dir"] = run_dir
open(progress_file, "w") do f
    JSON.print(f, progress_data, 2)
end

# Save config snapshot
config_out = joinpath(run_dir, "config.toml")
open(config_out, "w") do f
    TOML.print(f, cfg)
end
println("✓ Saved config: $config_out")

# Extract config fields
paths = cfg["paths"]
scenario = cfg["scenario"]
model_cfg = cfg["model"]
solver_cfg = get(cfg, "solver", Dict{String, Any}())
params = get(cfg, "parameters", Dict{String, Any}())

station_file = joinpath(PROJECT_ROOT, paths["station_file"])
order_file = joinpath(PROJECT_ROOT, paths["order_file"])
segment_file = joinpath(PROJECT_ROOT, paths["segment_file"])

station_limit = get(params, "station_limit", 0)

try
    println("\nLoading data...")
    stations = read_candidate_stations(station_file)
    requests = read_customer_requests(
        order_file;
        start_time=scenario["start_time"],
        end_time=scenario["end_time"]
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

    println("  - Stations: $(nrow(stations))")
    println("  - Requests: $(nrow(requests))")

    println("\nComputing costs...")
    walking_costs = compute_station_pairwise_costs(stations)
    routing_costs = read_routing_costs_from_segments(segment_file, stations)

    println("\nCreating StationSelectionData...")
    scenarios = [(scenario["start_time"], scenario["end_time"])]
    data = create_station_selection_data(
        stations,
        requests,
        walking_costs;
        routing_costs=routing_costs,
        scenarios=scenarios
    )

    model_type = get(model_cfg, "type", "TwoStageSingleDetourModel")
    if model_type != "TwoStageSingleDetourModel"
        error("This pipeline expects model.type = TwoStageSingleDetourModel (got $model_type)")
    end

    model = TwoStageSingleDetourModel(
        model_cfg["k"],
        model_cfg["l"],
        model_cfg["routing_weight"],
        model_cfg["time_window"],
        model_cfg["routing_delay"]
    )

    silent = get(solver_cfg, "silent", true)
    gurobi_env = Gurobi.Env()

    println("\nRunning optimization...")
    start_time = now()

    term_status, obj_value, solution, runtime_sec, m, var_counts, con_counts, detour_counts = run_opt(
        model,
        data;
        optimizer_env=gurobi_env,
        silent=silent,
        show_counts=true,
        return_model=true,
        return_counts=true,
        do_optimize=true
    )

    elapsed = Dates.value(now() - start_time) / 1000

    total_vars = isempty(var_counts) ? 0 : sum(values(var_counts))
    total_constraints = isempty(con_counts) ? 0 : sum(values(con_counts))

    metadata = Dict(
        "job_id" => job_id,
        "station_limit" => station_limit,
        "n_stations" => data.n_stations,
        "n_requests" => nrow(requests),
        "n_scenarios" => n_scenarios(data),
        "termination_status" => string(term_status),
        "objective_value" => obj_value,
        "solve_time_sec" => runtime_sec,
        "total_runtime_sec" => elapsed,
        "variables" => Dict("by_type" => var_counts),
        "constraints" => Dict("by_type" => con_counts),
        "detour_combinations" => detour_counts,
        "model" => model_type,
        "timestamp" => string(timestamp)
    )

    open(joinpath(run_dir, "metadata.json"), "w") do f
        JSON.print(f, metadata, 2)
    end

    progress_data["selection"]["status"] = "completed"
    progress_data["selection"]["completed"] = string(now())
    progress_data["selection"]["run_dir"] = run_dir
    progress_data["selection"]["variables_total"] = total_vars
    progress_data["selection"]["constraints_total"] = total_constraints
    progress_data["selection"]["runtime_sec"] = elapsed
    open(progress_file, "w") do f
        JSON.print(f, progress_data, 2)
    end

    println("\n✓ Completed job $job_id")
    println("Results saved to: $run_dir")
    exit(0)

catch e
    @error "Selection failed" exception=e
    for (exc, bt) in Base.catch_stack()
        showerror(stdout, exc, bt)
        println()
    end

    error_file = joinpath(run_dir, "error.txt")
    open(error_file, "w") do f
        println(f, "Error: $e")
        for (exc, bt) in Base.catch_stack()
            showerror(f, exc, bt)
            println(f)
        end
    end

    progress_data["selection"]["status"] = "failed"
    progress_data["selection"]["failed"] = string(now())
    progress_data["selection"]["error_file"] = error_file
    open(progress_file, "w") do f
        JSON.print(f, progress_data, 2)
    end

    println("\n✗ Selection failed - see $error_file")
    exit(1)
end
