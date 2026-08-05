const NS = [10, 15, 20]
const PS = [16, 32]
const SEEDS = [42, 123, 999]
const QS = [1, 3]
const METHOD = "bendersYZ_mw_common_od_ms5"

outpath = length(ARGS) == 1 ? abspath(ARGS[1]) :
    abspath(joinpath(@__DIR__, "..", "experiments", "zhuzhou_classical_benders_common_od_ms5", "jobs.tsv"))
mkpath(dirname(outpath))
open(outpath, "w") do io
    println(io, "family\tn_stations\tl\tn_pairs\tseed\tmethod\tn_scenarios")
    for n in NS, p in PS, seed in SEEDS, q in QS
        println(io, "zhuzhou\t$n\t$(ceil(Int, n / 2))\t$p\t$seed\t$METHOD\t$q")
    end
end
println("Wrote $(length(NS) * length(PS) * length(SEEDS) * length(QS)) jobs to $outpath")
