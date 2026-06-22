"""
    scripts/generate_job_list.jl

Generate the tab-separated job list for the compatibility-set scaling sweep.
Each row encodes one `(nx, ny, n_requests, seed)` instance by its generation
parameters, so the sbatch task can synthesize the input data on the fly.

Usage:
    julia --project=. scripts/generate_job_list.jl [outpath]

Default output:
    experiments/compatibility_set_scaling/jobs.txt
"""

const GRIDS = [(4, 4), (6, 6), (8, 8), (10, 10)]
const N_REQUESTS = [20, 40, 80, 160]
const SEEDS = [42, 123, 999]

function main()
    outpath = length(ARGS) >= 1 ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "compatibility_set_scaling", "jobs.txt")

    mkpath(dirname(outpath))

    open(outpath, "w") do io
        println(io, "nx\tny\tn_requests\tseed")
        for (nx, ny) in GRIDS, n_requests in N_REQUESTS, seed in SEEDS
            println(io, "$nx\t$ny\t$n_requests\t$seed")
        end
    end

    n_jobs = length(GRIDS) * length(N_REQUESTS) * length(SEEDS)
    println("Wrote $n_jobs jobs to $outpath")
    println("  Grids:      $(join(["$(nx)×$(ny)" for (nx, ny) in GRIDS], ", "))")
    println("  Requests:   $(join(N_REQUESTS, ", "))")
    println("  Seeds:      $(join(SEEDS, ", "))")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
