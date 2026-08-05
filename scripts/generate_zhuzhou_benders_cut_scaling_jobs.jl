"""Generate the focused synthetic-Zhuzhou Benders/direct scaling matrix."""

const NS = [10, 15, 20, 25, 30, 35, 40]
const PS = [16, 32]
const SEEDS = [42, 123, 999]
const SCENARIO_COUNTS = [1, 3]
const SMALL_METHODS = [
    "direct_ms5",
    "bendersY_std_reprice_ms5", "bendersY_zerocomp_ms5", "bendersY_mw_ms5",
    "bendersYZ_std_reprice_ms5", "bendersYZ_zerocomp_ms5", "bendersYZ_mw_ms5",
]
const LARGE_METHODS = ["direct_ms5", "bendersYZ_mw_ms5"]

function write_jobs(path, ns, methods)
    mkpath(dirname(path))
    open(path, "w") do io
        println(io, "family\tn_stations\tl\tn_pairs\tseed\tmethod\tn_scenarios")
        for n in ns, p in PS, seed in SEEDS, method in methods, q in SCENARIO_COUNTS
            println(io, "zhuzhou\t$n\t$(ceil(Int, n / 2))\t$p\t$seed\t$method\t$q")
        end
    end
    return length(ns) * length(PS) * length(SEEDS) * length(methods) * length(SCENARIO_COUNTS)
end

function main()
    outdir = length(ARGS) == 1 ? abspath(ARGS[1]) :
        abspath(joinpath(@__DIR__, "..", "experiments", "zhuzhou_benders_cut_scaling_ms5"))
    small = write_jobs(joinpath(outdir, "jobs_n_le_20.tsv"), filter(<=(20), NS), SMALL_METHODS)
    large = write_jobs(joinpath(outdir, "jobs_n_gt_20.tsv"), filter(>(20), NS), LARGE_METHODS)
    smoke = write_jobs(joinpath(outdir, "jobs_n10.tsv"), [10], SMALL_METHODS)
    remaining_small = write_jobs(
        joinpath(outdir, "jobs_n15_n20.tsv"), [15, 20], SMALL_METHODS,
    )
    open(joinpath(outdir, "manifest.tsv"), "w") do io
        println(io, "group\twall_limit\tmemory\tn_jobs\tjobs_file")
        println(io, "n_le_20\t01:30:00\t16G\t$small\tjobs_n_le_20.tsv")
        println(io, "n_gt_20\t03:00:00\t16G\t$large\tjobs_n_gt_20.tsv")
        println(io, "n10_smoke\t01:30:00\t16G\t$smoke\tjobs_n10.tsv")
        println(io, "n15_n20\t01:30:00\t16G\t$remaining_small\tjobs_n15_n20.tsv")
        println(io, "total\t\t\t$(small + large)\t")
    end
    println("Wrote $small small jobs + $large large jobs = $(small + large) total to $outdir")
end

main()
