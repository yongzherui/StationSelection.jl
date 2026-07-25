"""
    scripts/sample09_task.jl

Single-(n_stations, method)-pair task runner for the sample_09 MW-vs-Direct
experiment (scripts/sample09_mw_vs_direct.jl), so each combination runs as its
own SLURM array task on its own node instead of all 33 combinations running
sequentially in one job (see scripts/sbatch_sample09_task.sh /
scripts/submit_sample09_task.sh).

Usage:
    julia --project=. scripts/sample09_task.jl <outdir> <n_stations> <method_label>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

function main_task()
    length(ARGS) >= 3 || error("Usage: sample09_task.jl <outdir> <n_stations> <method_label>")
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
