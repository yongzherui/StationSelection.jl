"""
    scripts/sample09_route100x_ms56_task.jl

Single-(n_stations, method)-pair task runner for the sample_09 route100x/ms5-ms6
experiment (scripts/sample09_route100x_ms56.jl), one SLURM array task per
combination (see scripts/sbatch_sample09_route100x_ms56_task.sh).

Usage:
    julia --project=. scripts/sample09_route100x_ms56_task.jl <outdir> <n_stations> <method_label>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_route100x_ms56.jl"))

function main_task()
    length(ARGS) >= 3 || error("Usage: sample09_route100x_ms56_task.jl <outdir> <n_stations> <method_label>")
    outdir = ARGS[1]
    n_stations = parse(Int, ARGS[2])
    method_label = ARGS[3]

    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    run_one(n_stations, method_label, results_dir, iters_dir)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main_task()
end
