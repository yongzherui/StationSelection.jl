"""
    scripts/generate_zhuzhou_p16_cg_ms45_singlescenario_job_list.jl

Writes the tab-separated job list for scripts/zhuzhou_p16_cg_ms45_singlescenario.jl
(one row per (n_stations, seed, method) triple -- N_STATIONS_LIST_CG x SEEDS x
METHOD_LABELS), consumed by
scripts/sbatch_zhuzhou_p16_cg_ms45_singlescenario_task.sh as one SLURM array
task per row.

Usage:
    julia --project=. scripts/generate_zhuzhou_p16_cg_ms45_singlescenario_job_list.jl [outpath]

Default output: experiments/zhuzhou_p16_cg_ms45_singlescenario/jobs.txt
"""

include(joinpath(@__DIR__, "zhuzhou_p16_cg_ms45_singlescenario.jl"))

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ?
        ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "zhuzhou_p16_cg_ms45_singlescenario", "jobs.txt")
    mkpath(dirname(outpath))

    n_jobs = 0
    open(outpath, "w") do io
        println(io, "n_stations\tseed\tmethod")
        for n_stations in N_STATIONS_LIST_CG, seed in SEEDS, method_label in METHOD_LABELS
            println(io, "$n_stations\t$seed\t$method_label")
            n_jobs += 1
        end
    end
    println("Wrote $n_jobs jobs to $outpath")
    println("  n_stations : $(join(N_STATIONS_LIST_CG, ", "))")
    println("  seeds      : $(join(SEEDS, ", "))")
    println("  methods    : $(join(METHOD_LABELS, ", "))")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
