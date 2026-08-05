const NS_BB = [10, 15, 20]
const PS_BB = [16, 32]
const SEEDS_BB = [42, 123, 999]
const QS_BB = [1, 3]

outpath = length(ARGS) == 1 ? abspath(ARGS[1]) :
    abspath(joinpath(@__DIR__, "..", "experiments", "zhuzhou_branch_benders_yz_mw_no_mcf_ms5", "jobs.tsv"))
mkpath(dirname(outpath))
open(outpath, "w") do io
    println(io, "n_stations\tn_pairs\tseed\tn_scenarios\ttime_class")
    for n in NS_BB, p in PS_BB, seed in SEEDS_BB, q in QS_BB
        println(io, "$n\t$p\t$seed\t$q\t$(n == 10 ? "small" : "large")")
    end
end
println("Wrote $(length(NS_BB) * length(PS_BB) * length(SEEDS_BB) * length(QS_BB)) jobs to $outpath")
