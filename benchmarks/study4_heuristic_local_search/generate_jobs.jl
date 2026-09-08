"""Generate 10 paired n=20, p=16, s=3 cells for three independent-process arms."""
const SEEDS = 42:51
const ARMS = ("exact", "station_simple", "cluster_guide")
const N, P, S, MAX_STOPS = 20, 16, 3, 10
const N_THREADS = S
const CLUSTER_COUNT, GUIDE_ROUTES = 9, 5
const PRICING_LIMIT = 300.0
const CERTIFYING_LIMIT = 3600.0
const TOTAL_LIMIT = 14400.0
outdir = joinpath(@__DIR__, "config")
mkpath(outdir)
path = joinpath(outdir, "jobs.tsv")
open(path, "w") do io
    println(io, join(("job_id", "cell_id", "arm", "n", "p", "s", "seed", "max_stops",
                     "n_threads", "K", "guide_routes", "pricing_limit",
                     "certifying_limit", "total_limit"), '\t'))
    job_id = 0
    for arm in ARMS, seed in SEEDS
        job_id += 1
        cell_id = "n$(N)_p$(P)_s$(S)_ms$(MAX_STOPS)_seed$(seed)"
        println(io, join((job_id, cell_id, arm, N, P, S, seed, MAX_STOPS, N_THREADS,
                          CLUSTER_COUNT, GUIDE_ROUTES, PRICING_LIMIT,
                          CERTIFYING_LIMIT, TOTAL_LIMIT), '\t'))
    end
end
println("Wrote $(length(ARMS) * length(SEEDS)) jobs to $path " *
        "($(length(ARMS)) arms x $(length(SEEDS)) seeds)")
