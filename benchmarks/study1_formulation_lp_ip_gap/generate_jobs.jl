"""Generate Study 1's four LP/IP-gap comparison grids.

Usage: `julia --project=. benchmarks/study1_formulation_lp_ip_gap/generate_jobs.jl [config_dir]`

Every row is an independent Julia/SLURM job. Repeated baseline parameterizations are
retained in their comparison group so each comparison is self-contained.
"""

const N_STATIONS = 10
const N_PAIRS = 8
const N_SCENARIOS = 1
const SEEDS = collect(42:51)
const PRICING_TIME_LIMIT_SEC = 900.0

const VARIANTS = [
    # max_stops=4 (not the Study 2 baseline of 10): keeps both formulations' exhaustive
    # column enumeration tractable so each one's direct-solve LP/IP pair is a genuine
    # global bound, not a truncated one (see run_benchmark.jl's "formulation" branch and
    # this study's README).
    (comparison="formulation", variant="base", formulation="base", max_stops=4, max_wait_time=900.0, detour_factor=2.0),
    (comparison="formulation", variant="joint", formulation="joint", max_stops=4, max_wait_time=900.0, detour_factor=2.0),
    (comparison="max_stops", variant="max_stops_3", formulation="joint", max_stops=3, max_wait_time=900.0, detour_factor=2.0),
    (comparison="max_stops", variant="max_stops_5", formulation="joint", max_stops=5, max_wait_time=900.0, detour_factor=2.0),
    (comparison="max_stops", variant="max_stops_7", formulation="joint", max_stops=7, max_wait_time=900.0, detour_factor=2.0),
    (comparison="max_wait_time", variant="max_wait_time_600", formulation="joint", max_stops=10, max_wait_time=600.0, detour_factor=2.0),
    (comparison="max_wait_time", variant="max_wait_time_900", formulation="joint", max_stops=10, max_wait_time=900.0, detour_factor=2.0),
    (comparison="max_wait_time", variant="max_wait_time_1200", formulation="joint", max_stops=10, max_wait_time=1200.0, detour_factor=2.0),
    (comparison="detour_factor", variant="detour_factor_1_5", formulation="joint", max_stops=10, max_wait_time=900.0, detour_factor=1.5),
    (comparison="detour_factor", variant="detour_factor_2_0", formulation="joint", max_stops=10, max_wait_time=900.0, detour_factor=2.0),
    (comparison="detour_factor", variant="detour_factor_2_5", formulation="joint", max_stops=10, max_wait_time=900.0, detour_factor=2.5),
]

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = join(("job_id", "instance_id", "comparison", "variant", "formulation",
    "n_stations", "n_pairs", "n_scenarios", "seed", "max_stops",
    "max_wait_time", "detour_factor", "pricing_time_limit_sec"), '\t')

job_id = Ref(0)
for comparison in ("formulation", "max_stops", "max_wait_time", "detour_factor")
    specs = filter(spec -> spec.comparison == comparison, VARIANTS)
    outpath = joinpath(config_dir, "$(comparison)_jobs.tsv")
    open(outpath, "w") do io
        println(io, header)
        for seed in SEEDS, spec in specs
            job_id[] += 1
            instance_id = "zhuzhou_n$(N_STATIONS)_p$(N_PAIRS)_sc$(N_SCENARIOS)_seed$(seed)"
            println(io, join((job_id[], instance_id, spec.comparison, spec.variant,
                spec.formulation, N_STATIONS, N_PAIRS, N_SCENARIOS, seed,
                spec.max_stops, spec.max_wait_time, spec.detour_factor,
                PRICING_TIME_LIMIT_SEC), '\t'))
        end
    end
    println("Wrote $(length(SEEDS) * length(specs)) jobs to $outpath")
end
