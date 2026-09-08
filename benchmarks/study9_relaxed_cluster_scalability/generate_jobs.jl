"""Generate the validation and staged frontier tables for Study 9."""

const VALIDATION_SIZES = (20, 25)
const FRONTIER_SIZES = (30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 84)
const VALIDATION_SEEDS = 42:51
const FRONTIER_SEEDS = 42:51
const N_PAIRS = 16
const N_SCENARIOS = 3
const MAX_STOPS = 10
const GUIDE_ROUTES = 5
const PRICING_LIMIT_SEC = 300.0
const CERTIFYING_LIMIT_SEC = 3600.0

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "phase", "arm", "n_stations", "n_pairs", "n_scenarios",
    "seed", "max_stops", "n_threads", "cluster_count", "cluster_max_count",
    "guide_routes", "barren_cache", "cut_management", "pricing_limit_sec",
    "certifying_limit_sec", "total_limit_sec")

function write_table(path, rows)
    open(path, "w") do io
        println(io, join(header, '\t'))
        for row in rows
            println(io, join(row, '\t'))
        end
    end
    println("Wrote $(length(rows)) jobs to $path")
end

validation = Tuple[]
job_id = Ref(0)
for n in VALIDATION_SIZES
    total = n == 20 ? 14_400.0 : 18_000.0
    for (arm, k0) in (
            ("exact", 0),
            ("relaxed_k60", round(Int, 0.6n)),
            ("relaxed_k80", round(Int, 0.8n)),
        ), seed in VALIDATION_SEEDS
        job_id[] += 1
        push!(validation, (job_id[], "validation", arm, n, N_PAIRS, N_SCENARIOS,
            seed, MAX_STOPS, N_SCENARIOS, k0, 0, GUIDE_ROUTES, false, false,
            PRICING_LIMIT_SEC, CERTIFYING_LIMIT_SEC, total))
    end
end
write_table(joinpath(config_dir, "validation.tsv"), validation)

for n in FRONTIER_SIZES
    # Fixed six-hour CG window at every size: the frontier is the largest n for which a
    # confirmed fixed-K arm certifies >90% of seeds inside the same wall budget.
    total = 21_600.0
    rows = Tuple[]
    row_id = 0
    for (arm, frac) in (("relaxed_k60", 0.6), ("relaxed_k80", 0.8)),
            seed in FRONTIER_SEEDS
        row_id += 1
        push!(rows, (row_id, "frontier", arm, n, N_PAIRS, N_SCENARIOS,
            seed, MAX_STOPS, N_SCENARIOS, round(Int, frac*n), 0,
            GUIDE_ROUTES, false, false, PRICING_LIMIT_SEC, CERTIFYING_LIMIT_SEC, total))
    end
    write_table(joinpath(config_dir, "n$(n).tsv"), rows)
end
