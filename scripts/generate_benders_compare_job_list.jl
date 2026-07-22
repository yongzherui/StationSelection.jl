"""
    scripts/generate_benders_compare_job_list.jl

Generate the tab-separated job list for the BendersY/BendersYZ/BendersYZH convergence
comparison (pilot grid). Each row encodes one (n_stations, l, n_pairs, endpoint_overlap,
seed) instance; the SLURM array task runs all three decompositions on that one instance
via compare_benders_decompositions.jl.

Usage:
    julia --project=. scripts/generate_benders_compare_job_list.jl [outpath]

Default output:
    experiments/benders_decomposition_compare/jobs.txt
"""

# Pilot sweep axes
const N_STATIONS_LIST   = [20, 40, 60]
const N_PAIRS_LIST      = [16]
const ENDPOINT_OVERLAPS = [2.0]
const SEEDS             = [42, 123]

# l = stations to build; set to ceil(n_stations / 2) for each grid point
_l_for(n::Int) = ceil(Int, n / 2)

function main()
    outpath = length(ARGS) >= 1 ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "benders_decomposition_compare", "jobs.txt")
    mkpath(dirname(outpath))

    n_jobs = 0
    open(outpath, "w") do io
        println(io, "n_stations\tl\tn_pairs\tendpoint_overlap\tseed")
        for n_st in N_STATIONS_LIST,
                n_p in N_PAIRS_LIST,
                ov in ENDPOINT_OVERLAPS,
                seed in SEEDS
            l = _l_for(n_st)
            println(io, "$n_st\t$l\t$n_p\t$ov\t$seed")
            n_jobs += 1
        end
    end

    println("Wrote $n_jobs jobs to $outpath")
    println("  n_stations  : $(join(N_STATIONS_LIST, ", "))")
    println("  l (built)   : $(join(_l_for.(N_STATIONS_LIST), ", "))  (ceil(n/2))")
    println("  n_pairs     : $(join(N_PAIRS_LIST, ", "))")
    println("  ov          : $(join(ENDPOINT_OVERLAPS, ", "))")
    println("  seeds       : $(join(SEEDS, ", "))")
    println("  (submit_benders_compare.sh submits BendersY/BendersYZ/BendersYZH as three")
    println("   independent SLURM arrays over this instance grid, each with its own time budget)")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
