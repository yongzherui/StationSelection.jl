"""
    scripts/generate_sample09_route_weight_job_list.jl

Writes the tab-separated job list for the sample_09 route_reg_weight
sensitivity experiment (one row per (n_stations, route_reg_weight) pair --
N_STATIONS_FOR_WEIGHT_SWEEP x ROUTE_WEIGHTS from
scripts/sample09_route_weight_sensitivity.jl), consumed by
scripts/sbatch_sample09_route_weight_task.sh as one SLURM array task per row.

Usage:
    julia --project=. scripts/generate_sample09_route_weight_job_list.jl [outpath]

Default output: experiments/sample09_route_weight_sensitivity/jobs.txt
"""

include(joinpath(@__DIR__, "sample09_route_weight_sensitivity.jl"))

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "sample09_route_weight_sensitivity", "jobs.txt")
    mkpath(dirname(outpath))

    n_jobs = 0
    open(outpath, "w") do io
        println(io, "n_stations\troute_weight")
        for n_stations in N_STATIONS_FOR_WEIGHT_SWEEP, route_weight in ROUTE_WEIGHTS
            println(io, "$n_stations\t$route_weight")
            n_jobs += 1
        end
    end
    println("Wrote $n_jobs jobs to $outpath")
    println("  n_stations       : $(join(N_STATIONS_FOR_WEIGHT_SWEEP, ", "))")
    println("  route_reg_weight : $(join(ROUTE_WEIGHTS, ", "))")
    println("  method           : $WEIGHT_METHOD_LABEL")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
