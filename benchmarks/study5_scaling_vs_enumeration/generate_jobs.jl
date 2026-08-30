"""Generate Study 5's three scalability sub-studies, each with a serial and a parallel arm.

Every `(instance, arm)` pair is one row and therefore one SLURM task and one Julia
process, so the two arms never share a process's JIT state or caches.

The two arms differ in exactly one setting, `CGSolver(parallel_scenario_pricing=...)`:

- `serial`   -- 1 CPU, `JULIA_NUM_THREADS=1`, scenarios priced one after another.
- `parallel` -- `n_scenarios` CPUs and threads, scenarios priced concurrently.

Gurobi is pinned to one thread in BOTH arms, so scenario pricing is the only thing that
parallelizes and the comparison isolates it.

`PRICING_TIME_LIMIT_SEC` is the **wall-clock budget for one pricing round** and is the
same 300 s on both arms -- that is what makes the comparison apples to apples, and it is
deliberately NOT scaled by `n_scenarios`. What differs is how much search fits inside it:
serial splits the round `n_scenarios` ways (each scenario gets `300/s`), while parallel
runs the scenarios concurrently so each gets the full 300 s. The parallel arm therefore
does up to `n_scenarios` x more label search per round for the same round wall, and the
hypothesis under test is that this lets it certify cells the serial arm cannot -- with
runtime staying roughly flat in `s` where serial's degrades.
"""

const SEEDS = 42:51
const MAX_STOPS = 10
# Wall budget for ONE pricing round, identical on both arms. Serial splits it across
# scenarios (each gets this / n_scenarios); parallel gives each scenario the whole thing.
const PRICING_TIME_LIMIT_SEC = 300.0
# The longer per-round budget the loop escalates to when a regular round returns nothing
# without exhausting; only a certifying round can prove optimality.
const CERTIFYING_TIME_LIMIT_SEC = 3600.0
# Strict cap on the CG loop. On expiry the job still writes a row flagged uncertified
# (cg_stop_reason="total_budget") rather than being killed with nothing. The recovery MIP
# and a final master re-solve add at most 300 s each, so SLURM walltime is 6 h 30 m.
const TOTAL_TIME_LIMIT_SEC = 21600.0
const MAX_ROUTES = 2_000_000
const ARMS = ("serial", "parallel")

# n=40 dropped 2026-08-30. The archived serial run
# (experiments/2026-08-30_..._SUPERSEDED_serial_only) gave each scenario 300 s -- exactly
# what this study's PARALLEL arm gives each scenario -- and certified 0/10 at n=40 with
# 9/10 not even writing a row. It is the only cell in the grid with no certification at
# all, so it is out of reach for both arms here and buys nothing but queue time. Every
# other value certified at least partially and is retained: the frontier cells (stations
# 30, passengers 24/32, scenarios 6/9/12) are precisely where the parallel arm has
# something to prove, since the serial arm gets only 300/n_scenarios per scenario there.
const SUBSTUDIES = (
    (name="stations", values=(10, 20, 30)),
    (name="passengers", values=(8, 16, 24, 32)),
    (name="scenarios", values=(3, 6, 9, 12)),
)

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "cell_id", "substudy", "axis_value", "arm", "method", "n_stations",
    "n_pairs", "n_scenarios", "seed", "max_stops", "n_threads", "time_limit_sec",
    "certifying_time_limit_sec", "total_time_limit_sec", "max_routes")
job_id = Ref(0)

for spec in SUBSTUDIES
    outpath = joinpath(config_dir, "$(spec.name)_jobs.tsv")
    n_written = Ref(0)
    open(outpath, "w") do io
        println(io, join(header, '\t'))
        # Arm varies slowest so a whole arm is one contiguous --array range: the parallel
        # arm needs its own --cpus-per-task, which sbatch fixes per submission.
        for arm in ARMS, value in spec.values, seed in SEEDS
            n = spec.name == "stations" ? value : 20
            p = spec.name == "passengers" ? value : 16
            s = spec.name == "scenarios" ? value : 3
            # Serial is single-threaded by definition; parallel gets one thread per
            # scenario, which is the allocation the flat-runtime hypothesis is about.
            n_threads = arm == "parallel" ? s : 1
            cell_id = "$(spec.name)_$(value)_n$(n)_p$(p)_s$(s)_ms$(MAX_STOPS)_seed$(seed)"
            job_id[] += 1
            n_written[] += 1
            println(io, join((job_id[], cell_id, spec.name, value, arm, "cg_exact", n, p, s,
                seed, MAX_STOPS, n_threads, PRICING_TIME_LIMIT_SEC,
                CERTIFYING_TIME_LIMIT_SEC, TOTAL_TIME_LIMIT_SEC, MAX_ROUTES), '\t'))
        end
    end
    println("Wrote $(n_written[]) jobs to $outpath " *
            "($(length(ARMS)) arms x $(length(spec.values)) values x $(length(SEEDS)) seeds)")
end
