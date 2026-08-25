"""Generate the independent-process Study 3 dominance-ablation grid.

Usage: `julia --project=. benchmarks/study3_dominance_ablation/generate_jobs.jl [output.tsv]`

Every row is one `(instance, compensated_dominance)` job, so the two dominance arms
never share a Julia process or its JIT/cache state.

`N_PAIRS` is held fixed while `N_STATIONS` sweeps, so the size axis isolates the
candidate-station count `|J|` rather than moving demand and stations together.
Note that `k = max(2, ceil(n/2))` (see `../lib/cg_benchmark.jl`) still tracks `n`,
so the larger cells build more stations for the same 16 OD pairs.
"""

const N_STATIONS = [15, 20, 25]
const N_PAIRS = [16]
const N_SCENARIOS = 1
const SEEDS = collect(42:51)
const MAX_STOPS = [10]
const PRICING_TIME_LIMIT_SEC = 900.0
const DOMINANCE_MODES = [true, false]

outpath = isempty(ARGS) ? joinpath(@__DIR__, "config", "jobs.tsv") : abspath(ARGS[1])
mkpath(dirname(outpath))
open(outpath, "w") do io
    println(io, join(("job_id", "instance_id", "compensated_dominance", "n_stations",
        "n_pairs", "n_scenarios", "seed", "max_stops", "pricing_time_limit_sec"), '\t'))
    job_id = 0
    for n in N_STATIONS, p in N_PAIRS, seed in SEEDS, max_stops in MAX_STOPS, compensated in DOMINANCE_MODES
        job_id += 1
        instance_id = "zhuzhou_n$(n)_p$(p)_sc$(N_SCENARIOS)_seed$(seed)_ms$(max_stops)"
        println(io, join((job_id, instance_id, compensated, n, p,
            N_SCENARIOS, seed, max_stops, PRICING_TIME_LIMIT_SEC), '\t'))
    end
end

n_jobs = length(N_STATIONS) * length(N_PAIRS) * length(SEEDS) * length(MAX_STOPS) * length(DOMINANCE_MODES)
println("Wrote $n_jobs jobs to $outpath")
