const NS = [10, 15, 20]
const PS = [16, 32]
const SEEDS = [42, 123, 999]
const QS = [1, 3]
const VARIANTS = ["common_od", "common_od_fractional"]

outpath = length(ARGS) == 1 ? abspath(ARGS[1]) :
    abspath(joinpath(@__DIR__, "..", "experiments", "zhuzhou_branch_benders_mcf_increment_ms5", "jobs.tsv"))
mkpath(dirname(outpath))
open(outpath, "w") do io
    println(io, "n_stations\tn_pairs\tseed\tn_scenarios\ttime_class\tmcf_variant")
    # Keep each n block contiguous so Slurm time limits can be assigned by row range.
    for n in NS, variant in VARIANTS, p in PS, seed in SEEDS, q in QS
        println(io, "$n\t$p\t$seed\t$q\t$(n == 10 ? "small" : "large")\t$variant")
    end
end
println("Wrote $(length(NS) * length(VARIANTS) * length(PS) * length(SEEDS) * length(QS)) jobs to $outpath")
