"""
    scripts/generate_station_simple_compare_job_list.jl

Generate the tab-separated job list for the station-simple vs normal pricer
comparison: just BendersYZ restricted_mw_fixed_pi (MW cut) at max_stops=5, each
run with `use_station_simple` false and true (its `_ss` twin) -- see
`aggregate_od_route_method_grid.jl`'s `METHODS` for `bendersYZ_mw_ms5[_ss]`.

Crossed with the SAME n_stations/seed/family grid as the full method-compare
experiment (`N_STATIONS_LIST`, `SEEDS`, `FAMILIES`), but narrowed to
n_pairs=16 only (not the full `N_PAIRS_LIST`) since this comparison is scoped
to a single demand-density point, not a full sweep.

Each row is one (instance, method) pair = one SLURM array task, consumed by
the (reused, unmodified) sbatch_method_compare.sh via
submit_station_simple_compare.sh.

Usage:
    julia --project=. scripts/generate_station_simple_compare_job_list.jl [outpath]

Default output:
    experiments/aggregate_od_route_station_simple_compare/jobs.txt
    experiments/aggregate_od_route_station_simple_compare/batch_manifest.txt
"""

include(joinpath(@__DIR__, "aggregate_od_route_method_grid.jl"))

const STATION_SIMPLE_COMPARE_METHOD_LABELS = [
    "bendersYZ_mw_ms5",
    "bendersYZ_mw_ms5_ss",
]
const STATION_SIMPLE_COMPARE_N_PAIRS_LIST = [16]

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "aggregate_od_route_station_simple_compare", "jobs.txt")
    mkpath(dirname(outpath))
    manifest_path = joinpath(dirname(outpath), "batch_manifest.txt")

    methods = [method_by_label(label) for label in STATION_SIMPLE_COMPARE_METHOD_LABELS]

    n_jobs = 0
    batch_bounds = Tuple{Int,Int,Int}[]  # (n_stations, start_row, end_row), 1-indexed data rows
    open(outpath, "w") do io
        println(io, "family\tn_stations\tl\tn_pairs\tseed\tmethod")
        for n_st in N_STATIONS_LIST
            start_row = n_jobs + 1
            for family in FAMILIES,
                    n_p in STATION_SIMPLE_COMPARE_N_PAIRS_LIST,
                    seed in SEEDS,
                    method in methods
                l = _l_for(n_st)
                println(io, "$family\t$n_st\t$l\t$n_p\t$seed\t$(method.label)")
                n_jobs += 1
            end
            push!(batch_bounds, (n_st, start_row, n_jobs))
        end
    end

    open(manifest_path, "w") do io
        println(io, "n_stations\tstart_row\tend_row\tn_jobs")
        for (n_st, start_row, end_row) in batch_bounds
            println(io, "$n_st\t$start_row\t$end_row\t$(end_row - start_row + 1)")
        end
    end

    n_instances = length(FAMILIES) * length(N_STATIONS_LIST) * length(STATION_SIMPLE_COMPARE_N_PAIRS_LIST) * length(SEEDS)
    println("Wrote $n_jobs jobs to $outpath")
    println("Wrote batch manifest to $manifest_path")
    println("  families    : $(join(FAMILIES, ", "))")
    println("  n_stations  : $(join(N_STATIONS_LIST, ", "))")
    println("  n_pairs     : $(join(STATION_SIMPLE_COMPARE_N_PAIRS_LIST, ", "))")
    println("  seeds       : $(join(SEEDS, ", "))")
    println("  methods     : $(join(STATION_SIMPLE_COMPARE_METHOD_LABELS, ", "))")
    println("  instances   : $n_instances  x  methods: $(length(methods))  =  $n_jobs jobs")
    println("  batches (per n_stations, $(div(n_jobs, length(N_STATIONS_LIST))) jobs each):")
    for (n_st, start_row, end_row) in batch_bounds
        println("    n_stations=$n_st  ->  rows $start_row-$end_row")
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
