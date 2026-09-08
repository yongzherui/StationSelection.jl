using CSV
using DataFrames
using Printf
using Statistics

length(ARGS) >= 1 || error("usage: analyze.jl <output-dir> [report-dir]")
root = abspath(ARGS[1])
report_dir = length(ARGS) >= 2 ? abspath(ARGS[2]) :
    joinpath(@__DIR__, "..", "results", basename(root))
mkpath(report_dir)
files = filter(f -> occursin(r"^n\d+_seed\d+_[a-z_]+\.csv$", f), readdir(root))
isempty(files) && error("no Study 9 summary rows in $root")
rows = reduce((a,b) -> vcat(a,b; cols=:union),
    [CSV.read(joinpath(root, f), DataFrame) for f in files])
sort!(rows, [:n_stations, :seed, :arm])
CSV.write(joinpath(report_dir, "all_rows.csv"), rows)

println("Study 9: $(nrow(rows)) rows from $root")
println("certified $(count(==("certified"), rows.status))/$(nrow(rows))")
bad = rows[.!isempty.(coalesce.(rows.error_message, "")), :]
println("errors $(nrow(bad))")

validation = rows[rows.phase .== "validation", :]
paired_rows = DataFrame()
if nrow(validation) > 0
    exact = select(validation[validation.arm .== "exact", :], :n_stations, :seed,
        :wall_sec => :exact_wall, :cg_iterations => :exact_iters,
        :objective_value => :exact_objective, :status => :exact_status,
        :exact_certifying_rounds => :exact_escalations)
    arms = validation[validation.arm .!= "exact", :]
    paired_rows = innerjoin(arms, exact; on=[:n_stations, :seed])
    paired_rows.speedup = paired_rows.exact_wall ./ paired_rows.wall_sec
    paired_rows.objective_abs_error = abs.(paired_rows.objective_value .- paired_rows.exact_objective)
    CSV.write(joinpath(report_dir, "validation_paired.csv"), paired_rows)
    println("\nValidation (only pairs where both runs certified):")
    @printf("%-4s %-15s %5s %10s %10s %8s %8s %10s %10s\n",
        "n", "arm", "pairs", "exact s", "arm s", "speedup", "CG iter", "cert sec", "cuts")
    usable = paired_rows[(paired_rows.status .== "certified") .&
        (paired_rows.exact_status .== "certified"), :]
    for g in groupby(usable, [:n_stations, :arm])
        @printf("%-4d %-15s %5d %10.1f %10.1f %8.2f %8.1f %10.1f %10.1f\n",
            first(g.n_stations), first(g.arm), nrow(g), median(g.exact_wall),
            median(g.wall_sec), median(g.speedup), median(g.cg_iterations),
            median(g.certification_sec), median(g.nogood_total_cuts))
    end
    mismatch = usable[usable.objective_abs_error .>
        1e-6 .* max.(1.0, abs.(usable.exact_objective)), :]
    println("objective mismatches: $(nrow(mismatch))")
end

frontier = rows[rows.phase .== "frontier", :]
frontier_summary = DataFrame()
if nrow(frontier) > 0
    summaries = NamedTuple[]
    println("\nFull-pipeline frontier:")
    @printf("%-4s %5s %9s %10s %8s %10s %10s %10s\n",
        "n", "rows", "certified", "median s", "CG iter", "cert sec", "cuts", "final K")
    for g in groupby(frontier, :n_stations)
        certified = count(==("certified"), g.status)
        final_k = [mean(parse.(Int, split(x, ';'))) for x in g.final_cluster_counts if !isempty(x)]
        item = (n_stations=first(g.n_stations), rows=nrow(g), certified=certified,
            certification_rate=certified/nrow(g), median_wall_sec=median(g.wall_sec),
            median_cg_iterations=median(g.cg_iterations),
            median_certification_sec=median(g.certification_sec),
            median_nogood_cuts=median(g.nogood_total_cuts),
            median_final_cluster_count=isempty(final_k) ? missing : median(final_k))
        push!(summaries, item)
        @printf("%-4d %5d %9s %10.1f %8.1f %10.1f %10.1f %10.1f\n",
            item.n_stations, item.rows, "$(item.certified)/$(item.rows)",
            item.median_wall_sec, item.median_cg_iterations,
            item.median_certification_sec, item.median_nogood_cuts,
            something(item.median_final_cluster_count, NaN))
    end
    frontier_summary = DataFrame(summaries)
    CSV.write(joinpath(report_dir, "frontier_summary.csv"), frontier_summary)
end

println("\nWrote reports to $report_dir")

