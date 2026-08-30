"""Generate Study 5's three independent scalability sub-studies."""

const SEEDS = 42:51
const MAX_STOPS = 10
const TIME_LIMIT_SEC = 1800.0
const MAX_ROUTES = 2_000_000

const SUBSTUDIES = (
    (name="stations", values=(10, 20, 30, 40)),
    (name="passengers", values=(8, 16, 24, 32)),
    (name="scenarios", values=(3, 6, 9, 12)),
)

config_dir = isempty(ARGS) ? joinpath(@__DIR__, "config") : abspath(ARGS[1])
mkpath(config_dir)
header = ("job_id", "cell_id", "substudy", "axis_value", "method", "n_stations",
    "n_pairs", "n_scenarios", "seed", "max_stops", "time_limit_sec", "max_routes")
job_id = Ref(0)

for spec in SUBSTUDIES
    outpath = joinpath(config_dir, "$(spec.name)_jobs.tsv")
    n_written = Ref(0)
    open(outpath, "w") do io
        println(io, join(header, '\t'))
        for value in spec.values, seed in SEEDS
            n = spec.name == "stations" ? value : 20
            p = spec.name == "passengers" ? value : 16
            s = spec.name == "scenarios" ? value : 3
            cell_id = "$(spec.name)_$(value)_n$(n)_p$(p)_s$(s)_ms$(MAX_STOPS)_seed$(seed)"
            job_id[] += 1
            n_written[] += 1
            println(io, join((job_id[], cell_id, spec.name, value, "cg_exact", n, p, s,
                seed, MAX_STOPS, TIME_LIMIT_SEC, MAX_ROUTES), '\t'))
        end
    end
    println("Wrote $(n_written[]) jobs to $outpath")
end
