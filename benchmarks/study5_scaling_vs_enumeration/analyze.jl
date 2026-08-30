"""Aggregate Study 5 result rows, retaining un-run jobs as censored placeholders.

Every configured job appears in `case_comparison.csv`. A job that never wrote a row is
carried as `status="missing_result"` with **all measurement fields blank** -- it is a
right-censored observation (it did not certify within the walltime), never an imputed
result. `censored_cells.csv` records which limit bound each non-certified cell and the
evidence for it; see `../notes/2026-08-30_compute_budgets_of_record.md`.
"""

# The `sbatch --time` the run being analysed received. Only used to label censored rows;
# verify against `sacct -j <array> --format=Timelimit` rather than trusting the script,
# which can be overridden on the command line.
const WALLTIME_LIMIT = get(ENV, "STUDY5_WALLTIME_LIMIT", "06:30:00")

using CSV
using DataFrames
using Dates
using Statistics

1 <= length(ARGS) <= 2 || error("usage: analyze.jl RAW_DIR [RESULTS_DIR]")
raw_dir = abspath(ARGS[1])
project_root = normpath(joinpath(@__DIR__, "..", ".."))
results_dir = length(ARGS) == 2 ? abspath(ARGS[2]) : joinpath(
    project_root, "benchmarks", "results", "$(Dates.today())_study5_scaling_exact_cg",
)
job_tables = [CSV.read(joinpath(@__DIR__, "config", "$(name)_jobs.tsv"), DataFrame; delim='\t')
    for name in ("stations", "passengers", "scenarios")]
jobs = reduce(vcat, job_tables)
files = sort(filter(p -> endswith(p, ".csv"), readdir(raw_dir; join=true)))
isempty(files) && error("no result CSVs found in $raw_dir")
results = reduce(vcat, (CSV.read(path, DataFrame) for path in files))

duplicates = combine(groupby(results, :job_id), nrow => :n)
duplicates = duplicates[duplicates.n .!= 1, :]
isempty(duplicates) || error("duplicate result job IDs: $(join(duplicates.job_id, ", "))")
extra = setdiff(results.job_id, jobs.job_id)
isempty(extra) || error("unexpected result job IDs: $(join(extra, ", "))")

metric_cols = [:job_id, :status, :termination_status, :objective_value, :wall_sec,
    :n_columns, :cg_iterations, :cg_converged, :cg_pricing_exhausted, :error_message]
for extra in (:julia_threads, :scenario_search_sec_sum, :scenario_search_sec_max,
              :scenario_search_sec_min, :n_scenario_searches)
    extra in propertynames(results) && push!(metric_cols, extra)
end
# Present only for runs made after the two-tier/total-budget change; tolerated as absent
# so this analyzer still reads older raw directories.
for optional in (:cg_stop_reason, :cg_total_budget_exhausted, :cg_certifying_rounds)
    optional in propertynames(results) && push!(metric_cols, optional)
end
metrics = select(results, metric_cols)
cases = leftjoin(jobs, metrics; on=:job_id)
cases.status = coalesce.(cases.status, "missing_result")

agreement = Dict{String, Union{Missing, Bool}}()
for group in groupby(cases, :cell_id)
    comparable = group[(group.status .== "exhausted") .&
        in.(group.method, Ref(("cg_exact", "cg_darp"))), :]
    values = collect(skipmissing(comparable.objective_value))
    agreement[string(group.cell_id[1])] = length(values) < 2 ? missing :
        all(isapprox(v, values[1]; atol=1e-6, rtol=1e-8) for v in values[2:end])
end
cases.objective_agreement = [agreement[string(id)] for id in cases.cell_id]
sort!(cases, :job_id)

summary_rows = NamedTuple[]
group_keys = :arm in propertynames(cases) ?
    [:substudy, :axis_value, :max_stops, :arm] : [:substudy, :axis_value, :max_stops, :method]
for group in groupby(cases, group_keys)
    successful = group[group.status .== "exhausted", :]
    agreements = collect(skipmissing(group.objective_agreement))
    push!(summary_rows, (
        substudy=group.substudy[1], axis_value=group.axis_value[1],
        max_stops=group.max_stops[1],
        arm=(:arm in propertynames(group) ? group.arm[1] : "n/a"),
        method=group.method[1], n_jobs=nrow(group),
        n_exhausted=count(==("exhausted"), group.status),
        n_timed_out=count(==("timed_out"), group.status),
        n_route_limit=count(==("route_limit"), group.status),
        n_budget_exhausted=count(==("budget_exhausted"), group.status),
        n_incomplete=count(==("incomplete"), group.status),
        n_error=count(==("error"), group.status),
        n_missing_result=count(==("missing_result"), group.status),
        mean_wall_sec=isempty(successful) ? missing : mean(successful.wall_sec),
        median_wall_sec=isempty(successful) ? missing : median(successful.wall_sec),
        mean_n_columns=isempty(successful) ? missing : mean(skipmissing(successful.n_columns)),
        # Total label-search work actually performed, and the longest single search. A
        # parallel round fits ~n_scenarios x the search of a serial one into the same wall,
        # so `mean_search_sum` is the direct measure of that, pooled over ALL rows (not
        # just certified ones) because the point holds for censored runs too.
        mean_search_sum_sec=(:scenario_search_sec_sum in propertynames(group) &&
            !isempty(collect(skipmissing(group.scenario_search_sec_sum)))) ?
            mean(skipmissing(group.scenario_search_sec_sum)) : missing,
        mean_search_max_sec=(:scenario_search_sec_max in propertynames(group) &&
            !isempty(collect(skipmissing(group.scenario_search_sec_max)))) ?
            mean(skipmissing(group.scenario_search_sec_max)) : missing,
        objective_agreement=isempty(agreements) ? missing : all(agreements),
    ))
end
summary = DataFrame(summary_rows)
sort!(summary, [:substudy, :axis_value, :max_stops, :arm])

mkpath(results_dir)
CSV.write(joinpath(results_dir, "case_comparison.csv"), cases)
CSV.write(joinpath(results_dir, "variant_summary.csv"), summary)
missing_cases = cases[cases.status .== "missing_result", :]
CSV.write(joinpath(results_dir, "missing_results.csv"), missing_cases)

# Every cell that did not certify, with the limit that bound it and the evidence. Causes
# are distinct because their remedies differ: a walltime kill loses the row entirely, a
# total-budget stop keeps a feasible incumbent, and an inconclusive stop had budget left.
censored_rows = NamedTuple[]
for row in eachrow(cases)
    row.status == "exhausted" && continue
    cause, limit, evidence = if row.status == "missing_result"
        ("slurm_walltime", WALLTIME_LIMIT,
         "killed by scheduler, no row written; did not certify within walltime. " *
         "Clock 2 could not stop it first -- see compute-budgets record SS3.")
    elseif row.status == "budget_exhausted"
        ("total_solve_budget", "$(row.total_time_limit_sec)s",
         "CG loop stopped at the total budget; feasible incumbent, not certified; " *
         "z_lp is not a valid bound. wall_sec=$(round(something(row.wall_sec, NaN); digits=1))")
    elseif row.status == "incomplete"
        ("pricing_inconclusive", "$(row.certifying_time_limit_sec)s certifying",
         "certifying round did not resolve; loop stopped with budget remaining. " *
         "wall_sec=$(round(something(row.wall_sec, NaN); digits=1)), " *
         "stop_reason=$(get(row, :cg_stop_reason, missing))")
    else
        (row.status, "n/a", "see case_comparison.csv")
    end
    push!(censored_rows, (
        job_id=row.job_id, cell_id=row.cell_id, substudy=row.substudy,
        axis_value=row.axis_value, seed=row.seed, status=row.status,
        cause=cause, limit=limit, certified=false, evidence=evidence,
    ))
end
censored = DataFrame(censored_rows)
CSV.write(joinpath(results_dir, "censored_cells.csv"), censored)

# Serial vs parallel, paired per (substudy, axis_value): the headline of the rewrite.
if :arm in propertynames(cases)
    arm_rows = NamedTuple[]
    for group in groupby(summary, [:substudy, :axis_value])
        ser = group[group.arm .== "serial", :]
        par = group[group.arm .== "parallel", :]
        (nrow(ser) == 1 && nrow(par) == 1) || continue
        ratio(a, b) = (ismissing(a) || ismissing(b) || b == 0) ? missing : a / b
        push!(arm_rows, (
            substudy=group.substudy[1], axis_value=group.axis_value[1],
            n_certified_serial=ser.n_exhausted[1], n_certified_parallel=par.n_exhausted[1],
            certified_delta=par.n_exhausted[1] - ser.n_exhausted[1],
            search_sum_serial=ser.mean_search_sum_sec[1],
            search_sum_parallel=par.mean_search_sum_sec[1],
            # >1 means the parallel arm performed that many times more label search for the
            # same per-round wall budget -- the mechanism the study is testing.
            search_work_ratio=ratio(par.mean_search_sum_sec[1], ser.mean_search_sum_sec[1]),
            runtime_serial=ser.mean_wall_sec[1], runtime_parallel=par.mean_wall_sec[1],
        ))
    end
    CSV.write(joinpath(results_dir, "arm_comparison.csv"), DataFrame(arm_rows))
end

open(joinpath(results_dir, "slides_results.tex"), "w") do io
    println(io, "% Generated by Study 5 analyze.jl")
    for (i, row) in enumerate(eachrow(summary))
        runtime = ismissing(row.mean_wall_sec) ? "--" : string(round(row.mean_wall_sec; digits=2))
        columns = ismissing(row.mean_n_columns) ? "--" : string(round(row.mean_n_columns; digits=0))
        println(io, "\\newcommand{\\StudyFiveRow$(i)}{$(row.substudy) & $(row.axis_value) & " *
            "$(row.max_stops) & $(row.arm) & $(row.n_exhausted)/$(row.n_jobs) & " *
            "$runtime & $columns}")
    end
end

println("Read $(nrow(results)) results for $(nrow(jobs)) jobs; " *
    "$(nrow(missing_cases)) with no row (censored at walltime $WALLTIME_LIMIT); " *
    "$(nrow(censored)) cells did not certify; wrote $results_dir")
