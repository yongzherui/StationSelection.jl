"""
    scripts/generate_method_compare_job_list.jl

Generate the tab-separated job list for the full AggregateODRouteModel method
comparison grid: Direct solve, plain Column Generation, and Benders
Y/YZ/YZH (each with standard/zero-completion/MW-cut cut derivations x
repriced/not-repriced x max_stops in {4, uncapped}, where applicable -- see
aggregate_od_route_method_grid.jl's `METHODS` for the exact list) crossed with
an instance grid over synthetic `grid` and real-data `zhuzhou` families.

Each row is one (instance, method) pair = one SLURM array task, consumed by
sbatch_method_compare.sh via a single job array (submit_method_compare.sh).

`n_stations` is the OUTERMOST loop, so all jobs for a given n_stations value
form one contiguous block of rows (family x n_pairs x seed x method =
2 x 3 x 3 x 25 = 450 jobs). This lets submit_method_compare.sh submit one
n_stations value at a time as a single SLURM array -- needed both because the
cluster caps a single submission at 500 jobs, and because we want to roll the
grid out incrementally starting from n_stations=10 rather than launching the
full 3150-job grid at once. A companion `batch_manifest.txt` records the exact
row range for each n_stations value so the submit script never has to
re-derive (and risk drifting from) the block size.

Usage:
    julia --project=. scripts/generate_method_compare_job_list.jl [outpath]

Default output:
    experiments/aggregate_od_route_method_compare/jobs.txt
    experiments/aggregate_od_route_method_compare/batch_manifest.txt
"""

include(joinpath(@__DIR__, "aggregate_od_route_method_grid.jl"))

function main()
    outpath = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "aggregate_od_route_method_compare", "jobs.txt")
    mkpath(dirname(outpath))
    manifest_path = joinpath(dirname(outpath), "batch_manifest.txt")

    n_jobs = 0
    batch_bounds = Tuple{Int,Int,Int}[]  # (n_stations, start_row, end_row), 1-indexed data rows
    open(outpath, "w") do io
        println(io, "family\tn_stations\tl\tn_pairs\tseed\tmethod")
        for n_st in N_STATIONS_LIST
            start_row = n_jobs + 1
            for family in FAMILIES,
                    n_p in N_PAIRS_LIST,
                    seed in SEEDS,
                    method in METHODS
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

    n_instances = length(FAMILIES) * length(N_STATIONS_LIST) * length(N_PAIRS_LIST) * length(SEEDS)
    println("Wrote $n_jobs jobs to $outpath")
    println("Wrote batch manifest to $manifest_path")
    println("  families    : $(join(FAMILIES, ", "))")
    println("  n_stations  : $(join(N_STATIONS_LIST, ", "))")
    println("  n_pairs     : $(join(N_PAIRS_LIST, ", "))")
    println("  seeds       : $(join(SEEDS, ", "))")
    println("  methods     : $(length(METHODS))")
    println("  instances   : $n_instances  x  methods: $(length(METHODS))  =  $n_jobs jobs")
    println("  batches (per n_stations, $(div(n_jobs, length(N_STATIONS_LIST))) jobs each):")
    for (n_st, start_row, end_row) in batch_bounds
        println("    n_stations=$n_st  ->  rows $start_row-$end_row")
    end
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
