"""Generate Study 6's paired Base exact-CG versus enumeration grid."""

const N_STATIONS = (10, 15, 20)
const N_PAIRS = 16
const N_SCENARIOS = 3
const SEEDS = 42:51
const METHODS = ("cg_exact", "enumeration")
const MAX_STOPS = 4
const TIME_LIMIT_SEC = 1800.0
const MAX_ROUTES = 2_000_000

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
outpath = joinpath(config_dir, "jobs.tsv")
header = ("job_id", "instance_id", "method", "n_stations", "n_pairs",
    "n_scenarios", "seed", "max_stops", "time_limit_sec", "max_routes")

open(outpath, "w") do io
    println(io, join(header, '\t'))
    job_id = 0
    for n in N_STATIONS, seed in SEEDS, method in METHODS
        job_id += 1
        instance_id = "zhuzhou_n$(n)_p$(N_PAIRS)_s$(N_SCENARIOS)_seed$(seed)_ms$(MAX_STOPS)"
        println(io, join((job_id, instance_id, method, n, N_PAIRS, N_SCENARIOS,
            seed, MAX_STOPS, TIME_LIMIT_SEC, MAX_ROUTES), '\t'))
    end
end
println("Wrote $(length(N_STATIONS) * length(SEEDS) * length(METHODS)) jobs to $outpath")
