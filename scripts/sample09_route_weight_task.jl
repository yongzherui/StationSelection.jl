"""
    scripts/sample09_route_weight_task.jl

Single-(n_stations, route_reg_weight)-pair task runner for the sample_09
route_reg_weight sensitivity experiment
(scripts/sample09_route_weight_sensitivity.jl), so each combination runs as its
own SLURM array task on its own node (see
scripts/sbatch_sample09_route_weight_task.sh /
scripts/submit_sample09_route_weight_task.sh).

Usage:
    julia --project=. scripts/sample09_route_weight_task.jl <outdir> <n_stations> <route_weight>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_route_weight_sensitivity.jl"))

function main_task()
    length(ARGS) >= 3 || error("Usage: sample09_route_weight_task.jl <outdir> <n_stations> <route_weight>")
    outdir = ARGS[1]
    n_stations = parse(Int, ARGS[2])
    route_weight = parse(Float64, ARGS[3])

    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    run_one_weight(n_stations, route_weight, results_dir, iters_dir)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main_task()
end
