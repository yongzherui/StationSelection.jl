"""
    scripts/generate_sample09_walk_weight_job_list.jl

Writes the tab-separated job list for the sample_09 walk_cost_weight
sensitivity experiment (one row per (n_stations, walk_cost_weight) pair --
N_STATIONS_FOR_WALK_SWEEP x WALK_WEIGHTS from
scripts/sample09_walk_weight_sensitivity.jl), consumed by
scripts/sbatch_sample09_walk_weight_task.sh as one SLURM array task per row.

Usage:
    julia --project=. scripts/generate_sample09_walk_weight_job_list.jl [outpath]

Default output: experiments/sample09_walk_weight_sensitivity/jobs.txt
"""

include(joinpath(@__DIR__, "sample09_walk_weight_sensitivity.jl"))

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "sample09_walk_weight_sensitivity", "jobs.txt")
    mkpath(dirname(outpath))

    n_jobs = 0
    open(outpath, "w") do io
        println(io, "n_stations\twalk_weight")
        for n_stations in N_STATIONS_FOR_WALK_SWEEP, walk_weight in WALK_WEIGHTS
            println(io, "$n_stations\t$walk_weight")
            n_jobs += 1
        end
    end
    println("Wrote $n_jobs jobs to $outpath")
    println("  n_stations       : $(join(N_STATIONS_FOR_WALK_SWEEP, ", "))")
    println("  walk_cost_weight : $(join(WALK_WEIGHTS, ", "))")
    println("  route_reg_weight : $FIXED_ROUTE_WEIGHT (fixed)")
    println("  method           : $WALK_WEIGHT_METHOD_LABEL")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
