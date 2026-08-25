"""Generate the independent-process Study 3 dominance-ablation grid."""

const N_STATIONS = 20
const N_PAIRS = 16
const N_SCENARIOS = 1
const SEEDS = collect(42:51)
const MAX_STOPS = 10
const PRICING_TIME_LIMIT_SEC = 900.0
const DOMINANCE_MODES = [true, false]

outpath = isempty(ARGS) ? joinpath(@__DIR__, "config", "jobs.tsv") : abspath(ARGS[1])
mkpath(dirname(outpath))
open(outpath, "w") do io
    println(io, join(("job_id", "instance_id", "compensated_dominance", "n_stations",
        "n_pairs", "n_scenarios", "seed", "max_stops", "pricing_time_limit_sec"), '\t'))
    job_id = 0
    for seed in SEEDS, compensated in DOMINANCE_MODES
        job_id += 1
        instance_id = "zhuzhou_n$(N_STATIONS)_p$(N_PAIRS)_sc$(N_SCENARIOS)_seed$(seed)_ms$(MAX_STOPS)"
        println(io, join((job_id, instance_id, compensated, N_STATIONS, N_PAIRS,
            N_SCENARIOS, seed, MAX_STOPS, PRICING_TIME_LIMIT_SEC), '\t'))
    end
end
println("Wrote $(length(SEEDS) * length(DOMINANCE_MODES)) jobs to $outpath")
