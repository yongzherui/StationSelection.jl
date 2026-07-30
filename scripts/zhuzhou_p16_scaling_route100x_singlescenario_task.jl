"""
    scripts/zhuzhou_p16_scaling_route100x_singlescenario_task.jl

Single-(n_stations, seed, method)-triple task runner for the single-scenario
zhuzhou p16 route100x scaling study
(scripts/zhuzhou_p16_scaling_route100x_singlescenario.jl), one SLURM array
task per combination (see sbatch_zhuzhou_p16_scaling_route100x_singlescenario_task.sh).

Usage:
    julia --project=. scripts/zhuzhou_p16_scaling_route100x_singlescenario_task.jl <outdir> <n_stations> <seed> <method_label>
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "zhuzhou_p16_scaling_route100x_singlescenario.jl"))

function main_task()
    length(ARGS) >= 4 || error("Usage: zhuzhou_p16_scaling_route100x_singlescenario_task.jl <outdir> <n_stations> <seed> <method_label>")
    outdir = ARGS[1]
    n_stations = parse(Int, ARGS[2])
    seed = parse(Int, ARGS[3])
    method_label = ARGS[4]

    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    run_one(n_stations, seed, method_label, results_dir, iters_dir)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main_task()
end
