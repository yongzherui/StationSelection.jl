"""Generate Study 7's jobs: certified-optimal CG solves at n=20, one row per instance.

The study asks a structural question about the *answer*, not about solver cost: are the
route columns in the final optimal selection elementary in their station set -- does an
optimal route visit each station at most once? The pricer's labels are revisit-tolerant
(`JointRoutingAssignmentPricingLabel`, unlike `station_simple/`'s elementary label), so a
non-elementary route is representable and priceable; whether the optimum actually uses one
is what this measures.

Design consequences of that question:

- **n=20 fixed.** Every cell must CERTIFY, because "the optimal route" is only meaningful
  once pricing has exhausted -- a budget-stopped incumbent is an upper bound whose columns
  carry no optimality claim. n=20/s=3 certified 40/40 at p<=16 and 8/10 at p=24 in Study 5
  (`experiments/2026-08-30_study5_scaling_exact_cg`), and uncertified cells are reported
  separately rather than pooled.
- **n_pairs is the axis.** Route length is what gives revisiting a chance to pay: more
  passengers per scenario means more assignments a single column can absorb. p=8/16/24
  spans short-route to long-route regimes while staying inside the certified frontier.
  p=32 is excluded -- it certified 1/10 in Study 5, so it would contribute almost no
  optimal columns at high queue cost.
- **max_stops=10 against 20 stations.** The cap has to exceed the station count before a
  route *must* revisit to keep growing; at 10 vs 20 an elementary route is always available,
  so a non-elementary optimum is a real preference, not a cap artifact.
- **Parallel pricing, 1 Gurobi thread.** Study 5's measured-faster arm. Nothing here
  compares arms; parallel is chosen only because it certifies more cells per wall-hour.
"""

const SEEDS = 42:51
const N_STATIONS = 20
const N_SCENARIOS = 3
const N_PAIRS_VALUES = (8, 16, 24)
const MAX_STOPS = 10
const PRICING_TIME_LIMIT_SEC = 300.0
const CERTIFYING_TIME_LIMIT_SEC = 3600.0
# 4 h, from Study 5's measured walls on exactly these cells: every one of the 8 p=24 cells
# that certified did so within 12,099 s, and the 2 that did not were still running at
# 21,600 s. A 6 h cap would therefore buy no extra certifications, only queue time.
const TOTAL_TIME_LIMIT_SEC = 14400.0

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "cell_id", "n_stations", "n_pairs", "n_scenarios", "seed", "max_stops",
    "n_threads", "time_limit_sec", "certifying_time_limit_sec", "total_time_limit_sec")

outpath = joinpath(config_dir, "jobs.tsv")
job_id = Ref(0)  # Ref, not a bare Int: the writer below is a closure over it.
open(outpath, "w") do io
    println(io, join(header, '\t'))
    for p in N_PAIRS_VALUES, seed in SEEDS
        job_id[] += 1
        cell_id = "n$(N_STATIONS)_p$(p)_s$(N_SCENARIOS)_ms$(MAX_STOPS)_seed$(seed)"
        println(io, join((job_id[], cell_id, N_STATIONS, p, N_SCENARIOS, seed, MAX_STOPS,
            N_SCENARIOS, PRICING_TIME_LIMIT_SEC, CERTIFYING_TIME_LIMIT_SEC,
            TOTAL_TIME_LIMIT_SEC), '\t'))
    end
end
println("Wrote $(job_id[]) jobs to $outpath " *
        "($(length(N_PAIRS_VALUES)) n_pairs values x $(length(SEEDS)) seeds)")
