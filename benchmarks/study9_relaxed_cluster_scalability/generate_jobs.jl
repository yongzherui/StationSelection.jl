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

# ── two-tier tables ─────────────────────────────────────────────────────────────
#
# Written separately, with an 18th `macro_count` column, so the single-tier tables above
# keep the exact 17-field shape the n40/n50 arrays already in flight parse on a preemption
# requeue. `run_benchmark.jl` accepts either width.
#
# K2 matches the single-tier arm it is compared against, so a pair differs ONLY by the
# macro tier.
#
# K1 is an ABSOLUTE NODE COUNT, not a fraction of K2. This was `round(0.6 * k2)`, which
# gives 11 at K2=18 and 19 at K2=32 -- both outside the working band. MEASURED at n=40: the
# band is K1 = 12-16 (`relaxed_cluster_two_tier_guide.jl` -- K1 <= 8 is nearly worthless,
# K1 >= 18 pays real time in the macro sweep itself), and it does not move with K2 because
# the macro sweep's cost depends on its own graph size, not on the meso graph's. The 0.6
# rule also made K2=32 look bad for a reason that was really K1=19: two-tier at K2=32/K1=16
# beat K2=32/K1=19 arms and lost 8-15x to K2=24/K1=14 on every shared seed.
const TWO_TIER_SIZES = (30, 40, 50)
# The measured working band for the macro node count; see the comment above.
const TWO_TIER_K1_MIN = 12
const TWO_TIER_K1_MAX = 16
two_tier_header = (header..., "macro_count")

function write_two_tier_table(path, rows)
    open(path, "w") do io
        println(io, join(two_tier_header, '\t'))
        for row in rows
            println(io, join(row, '\t'))
        end
    end
    println("Wrote $(length(rows)) jobs to $path")
end

for n in TWO_TIER_SIZES
    rows = Tuple[]
    row_id = 0
    for (arm, frac) in (("twotier_k60", 0.6), ("twotier_k80", 0.8)), seed in FRONTIER_SEEDS
        k2 = round(Int, frac * n)
        k1 = clamp(round(Int, 0.6 * k2), TWO_TIER_K1_MIN, TWO_TIER_K1_MAX)
        row_id += 1
        push!(rows, (row_id, "frontier", arm, n, N_PAIRS, N_SCENARIOS, seed, MAX_STOPS,
                     N_SCENARIOS, k2, 0, GUIDE_ROUTES, false, false,
                     PRICING_LIMIT_SEC, CERTIFYING_LIMIT_SEC, 21_600.0, k1))
    end
    write_two_tier_table(joinpath(config_dir, "n$(n)_twotier.tsv"), rows)
end

# One tiny row for a pipeline smoke test of the mode, mirroring smoke.tsv.
write_two_tier_table(joinpath(config_dir, "smoke_twotier.tsv"),
    Tuple[(1, "smoke", "twotier_k60", 8, 4, 3, 42, 6, 3, 5, 0, 5, false, false,
           30.0, 120.0, 600.0, 3)])
