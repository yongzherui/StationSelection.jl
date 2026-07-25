"""
    scripts/generate_sample09_route100x_ms56_job_list.jl

Writes the tab-separated job list for the sample_09 route100x/ms5-ms6 experiment
(one row per (n_stations, method) pair -- N_STATIONS_TO_RUN x METHOD_LABELS from
scripts/sample09_route100x_ms56.jl), consumed by
scripts/sbatch_sample09_route100x_ms56_task.sh as one SLURM array task per row.

Usage:
    julia --project=. scripts/generate_sample09_route100x_ms56_job_list.jl [outpath]

Default output: experiments/sample09_route100x_ms56/jobs.txt
"""

include(joinpath(@__DIR__, "sample09_route100x_ms56.jl"))

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "sample09_route100x_ms56", "jobs.txt")
    mkpath(dirname(outpath))

    n_jobs = 0
    open(outpath, "w") do io
        println(io, "n_stations\tmethod")
        for n_stations in N_STATIONS_TO_RUN, method_label in METHOD_LABELS
            println(io, "$n_stations\t$method_label")
            n_jobs += 1
        end
    end
    println("Wrote $n_jobs jobs to $outpath")
    println("  n_stations : $(join(N_STATIONS_TO_RUN, ", "))")
    println("  methods    : $(join(METHOD_LABELS, ", "))")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
