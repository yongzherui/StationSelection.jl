const OUT = length(ARGS) == 1 ? ARGS[1] : "experiments/free_assignment_cg_direct_ms5/jobs_cg.tsv"
mkpath(dirname(OUT))
open(OUT, "w") do io
    println(io, "n\tp\tseed\tq\tmethod")
    for n in (10,15,20), p in (16,32), seed in (42,123,999), q in (1,3)
        println(io, "$n\t$p\t$seed\t$q\tcg")
    end
end
println(OUT)
