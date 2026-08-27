"""Aggregate and pair Study 6 exact-CG and exhaustive-enumeration results."""

using CSV
using DataFrames
using Dates
using Statistics

1 <= length(ARGS) <= 2 || error("usage: analyze.jl RAW_DIR [RESULTS_DIR]")
raw_dir = abspath(ARGS[1])
project_root = normpath(joinpath(@__DIR__, "..", ".."))
results_dir = length(ARGS) == 2 ? abspath(ARGS[2]) : joinpath(
    project_root, "benchmarks", "results", "$(Dates.today())_study6_exact_cg_vs_enumeration",
)
jobs = CSV.read(joinpath(@__DIR__, "config", "jobs.tsv"), DataFrame; delim='\t')
files = sort(filter(p -> endswith(p, ".csv"), readdir(raw_dir; join=true)))
isempty(files) && error("no result CSVs found in $raw_dir")
results = reduce(vcat, (CSV.read(path, DataFrame) for path in files))

metrics = select(results, :job_id, :status, :termination_status, :objective_value,
    :lp_objective_value, :wall_sec, :n_columns, :cg_iterations, :cg_converged,
    :cg_pricing_exhausted, :error_message)
cases = leftjoin(jobs, metrics; on=:job_id)
cases.status = coalesce.(cases.status, "missing_result")
sort!(cases, :job_id)

keys = [:instance_id, :n_stations, :n_pairs, :n_scenarios, :seed, :max_stops]
function method_frame(method, prefix)
    frame = select(cases[cases.method .== method, :], keys..., :status,
        :objective_value, :wall_sec, :n_columns)
    rename!(frame, :status => Symbol(prefix, "_status"),
        :objective_value => Symbol(prefix, "_objective"),
        :wall_sec => Symbol(prefix, "_wall_sec"), :n_columns => Symbol(prefix, "_n_columns"))
    return frame
end
paired = innerjoin(method_frame("cg_exact", "cg"),
    method_frame("enumeration", "enum"); on=keys)
paired.both_exhausted = (paired.cg_status .== "exhausted") .&
    (paired.enum_status .== "exhausted")
paired.objective_delta = paired.cg_objective .- paired.enum_objective
paired.objective_match = [row.both_exhausted && !ismissing(row.cg_objective) &&
    !ismissing(row.enum_objective) &&
    isapprox(row.cg_objective, row.enum_objective; atol=1e-6, rtol=1e-8)
    for row in eachrow(paired)]
paired.runtime_ratio_enum_over_cg = paired.enum_wall_sec ./ paired.cg_wall_sec

summary_rows = NamedTuple[]
for group in groupby(cases, [:n_stations, :method])
    good = group[group.status .== "exhausted", :]
    push!(summary_rows, (
        n_stations=group.n_stations[1], method=group.method[1], n_jobs=nrow(group),
        n_exhausted=nrow(good), n_missing=count(==("missing_result"), group.status),
        mean_wall_sec=isempty(good) ? missing : mean(good.wall_sec),
        median_wall_sec=isempty(good) ? missing : median(good.wall_sec),
        mean_n_columns=isempty(good) ? missing : mean(skipmissing(good.n_columns)),
    ))
end
summary = DataFrame(summary_rows)
sort!(summary, [:n_stations, :method])

mkpath(results_dir)
CSV.write(joinpath(results_dir, "case_results.csv"), cases)
CSV.write(joinpath(results_dir, "paired_comparison.csv"), paired)
CSV.write(joinpath(results_dir, "variant_summary.csv"), summary)
CSV.write(joinpath(results_dir, "missing_results.csv"),
    cases[cases.status .== "missing_result", :])
println("Read $(nrow(results))/$(nrow(jobs)) jobs; wrote $results_dir")
