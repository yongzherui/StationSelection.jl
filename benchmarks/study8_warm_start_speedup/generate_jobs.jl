"""Generate Study 8's jobs: the `warm_start` arm only, on Study 7's exact grid.

Two questions, both answered by this one arm:

1. **Is elementary-first faster end to end?** Measured against **Study 7's already-completed
   `exact` runs**, which used this identical grid, budgets, formulation and thread count.
   Study 8 therefore runs 30 jobs, not 60 -- the baseline is not re-run.
2. **How quickly does the elementary phase exhaust?** From `cg_warm_start_sec` /
   `cg_warm_start_iterations` (recorded by `CGSolver` at the handoff) plus the per-phase
   split of `cg_iteration_log`, whose rows now carry `pricing_mode`.

The grid is Study 7's verbatim: n=20, s=3, p ∈ {8,16,24}, seeds 42-51, ms=10, 300 s
regular / 3600 s certifying / 14400 s total, 3 threads with parallel scenario pricing,
`recover_integer_solution=true`. `cell_id` is formatted identically so `analyze.jl` can
join the two studies on it directly.

**Two confounds this cross-study comparison carries**, neither correctable without
re-running the baseline, both believed small:
- *Build drift.* Study 7's `exact` runs predate recording assignment positions in route
  replay. That work lands in `_replay_joint_routing_assignment_route`, which runs once per
  accepted column, not per label -- and the label-setting dominance scan is ~90% of pricing
  wall -- so the delta should sit far below run-to-run noise.
- *Node variance.* Both studies run on `mit_preemptable`, so cells land on whatever
  hardware is free. Paired per-cell ratios (same instance, same seed) absorb instance
  difficulty but not hardware, which is why `analyze.jl` reports the distribution of
  per-cell speedups rather than a single ratio of totals.

p=24 is retained despite costing the most queue time: Study 7 measured 88.9% elementary
optima there against 100% at p<=16, making it the regime where the elementary universe is
likeliest to exhaust with real work still to do -- exactly where a speedup could fail to
appear.
"""

const SEEDS = 42:51
const N_STATIONS = 20
const N_SCENARIOS = 3
const N_PAIRS_VALUES = (8, 16, 24)
const MAX_STOPS = 10
const PRICING_TIME_LIMIT_SEC = 300.0
const CERTIFYING_TIME_LIMIT_SEC = 3600.0
const TOTAL_TIME_LIMIT_SEC = 14400.0
# Only the warm-start arm is run here; the `exact` baseline is Study 7's completed runs
# (see the module docstring). The arm column is kept in the schema so a baseline arm can be
# added later without changing the row format or the analysis join.
# `warm_start` = CGSolver(warm_start_pricing_mode=:station_simple): elementary pricing until
# that universe exhausts, then the formulation's own exact pricer, which is the phase that
# certifies. It must reach the SAME certified optimum as Study 7's exact run on the same
# cell -- analyze.jl checks that per cell and treats a mismatch as a correctness failure,
# not a result.
const ARMS = ("warm_start",)

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "cell_id", "arm", "n_stations", "n_pairs", "n_scenarios", "seed",
    "max_stops", "n_threads", "time_limit_sec", "certifying_time_limit_sec",
    "total_time_limit_sec")

outpath = joinpath(config_dir, "jobs.tsv")
job_id = Ref(0)
open(outpath, "w") do io
    println(io, join(header, '\t'))
    # Arm varies slowest so each arm is one contiguous --array range, submittable alone.
    for arm in ARMS, p in N_PAIRS_VALUES, seed in SEEDS
        job_id[] += 1
        cell_id = "n$(N_STATIONS)_p$(p)_s$(N_SCENARIOS)_ms$(MAX_STOPS)_seed$(seed)"
        println(io, join((job_id[], cell_id, arm, N_STATIONS, p, N_SCENARIOS, seed,
            MAX_STOPS, N_SCENARIOS, PRICING_TIME_LIMIT_SEC, CERTIFYING_TIME_LIMIT_SEC,
            TOTAL_TIME_LIMIT_SEC), '\t'))
    end
end
println("Wrote $(job_id[]) jobs to $outpath ($(length(ARMS)) arms x " *
        "$(length(N_PAIRS_VALUES)) n_pairs x $(length(SEEDS)) seeds)")
