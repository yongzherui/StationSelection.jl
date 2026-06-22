"""
    scripts/generate_zhuzhou_job_list.jl

Generate the tab-separated job list for the Zhuzhou CompatibilitySetModel scaling sweep.
Each row encodes one (n_stations, l, n_pairs, endpoint_overlap, seed) instance so the
SLURM array task can generate the data on the fly via run_zhuzhou_instance.jl.

Usage:
    julia --project=. scripts/generate_zhuzhou_job_list.jl [outpath]

Default output:
    experiments/zhuzhou_scaling/jobs.txt
"""

# Sweep axes
const N_STATIONS_LIST   = [20, 40, 60, 80]
const N_PAIRS_LIST      = [8, 16, 32]
const ENDPOINT_OVERLAPS = [1.0, 2.0]
const SEEDS             = [42, 123, 999]

# l = stations to build; set to ceil(n_stations / 2) for each grid point
_l_for(n::Int) = ceil(Int, n / 2)

function main()
    outpath = length(ARGS) >= 1 ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "zhuzhou_scaling", "jobs.txt")
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
    println("  l (built)   : $(join(_l_for.(N_STATIONS_LIST), ", "))  (⌈n/2⌉)")
    println("  n_pairs     : $(join(N_PAIRS_LIST, ", "))")
    println("  ov          : $(join(ENDPOINT_OVERLAPS, ", "))")
    println("  seeds       : $(join(SEEDS, ", "))")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
