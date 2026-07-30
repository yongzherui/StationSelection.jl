"""
    scripts/generate_zhuzhou_p16_scaling_route100x_job_list.jl

Writes the tab-separated job list for the zhuzhou p16 route100x scaling study
(one row per (n_stations, seed, method) triple -- N_STATIONS_LIST x SEEDS x
METHOD_LABELS from scripts/zhuzhou_p16_scaling_route100x.jl), consumed by
scripts/sbatch_zhuzhou_p16_scaling_route100x_task.sh as one SLURM array task
per row.

Usage:
    julia --project=. scripts/generate_zhuzhou_p16_scaling_route100x_job_list.jl [outpath]

Default output: experiments/zhuzhou_p16_scaling_route100x/jobs.txt
"""

include(joinpath(@__DIR__, "zhuzhou_p16_scaling_route100x.jl"))

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "zhuzhou_p16_scaling_route100x", "jobs.txt")
    mkpath(dirname(outpath))

    n_jobs = 0
    open(outpath, "w") do io
        println(io, "n_stations\tseed\tmethod")
        for n_stations in N_STATIONS_LIST, seed in SEEDS, method_label in METHOD_LABELS
            println(io, "$n_stations\t$seed\t$method_label")
            n_jobs += 1
        end
    end
    println("Wrote $n_jobs jobs to $outpath")
    println("  n_stations : $(join(N_STATIONS_LIST, ", "))")
    println("  seeds      : $(join(SEEDS, ", "))")
    println("  methods    : $(join(METHOD_LABELS, ", "))")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
