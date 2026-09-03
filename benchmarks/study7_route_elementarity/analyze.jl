"""Study 7 analysis: are the route columns in the certified optimum elementary?

Reads the per-job metrics rows and the per-job `routes/job_NNNN/variable_exports/`
directories written by `run_benchmark.jl`, and answers the study question at three levels:
per column, per instance, and per `n_pairs` regime.

**Only `status == "exhausted"` jobs count toward the headline.** A budget-stopped run's
columns come from a restricted master that pricing never exhausted: the selection is a
feasible upper bound, and asking whether *the optimum* revisits stations is not a question
its columns can answer. Uncertified jobs are loaded and reported separately, never pooled.

Usage: julia --project=<root> analyze.jl [<experiment_dir>] [<output_dir>]
"""

using CSV
using DataFrames
using JSON
using Printf
using Statistics

const DEFAULT_EXPERIMENT_GLOB = "study7_route_elementarity"

function _default_experiment_dir()
    root = normpath(joinpath(@__DIR__, "..", "experiments"))
    isdir(root) || error("no benchmarks/experiments directory at $root")
    candidates = sort!(filter(d -> occursin(DEFAULT_EXPERIMENT_GLOB, d), readdir(root)))
    isempty(candidates) && error("no $DEFAULT_EXPERIMENT_GLOB run under $root")
    return joinpath(root, last(candidates))  # newest by date-prefixed name
end

experiment_dir = length(ARGS) >= 1 ? abspath(ARGS[1]) : _default_experiment_dir()
output_dir = length(ARGS) >= 2 ? abspath(ARGS[2]) :
    joinpath(normpath(joinpath(@__DIR__, "..", "results")), basename(experiment_dir))
mkpath(output_dir)
println("Reading  $experiment_dir")
println("Writing  $output_dir\n")

# ── load metrics rows ────────────────────────────────────────────────────────
metric_files = filter(f -> occursin(r"^job_\d+\.csv$", f), readdir(experiment_dir))
isempty(metric_files) && error("no job_NNNN.csv metrics rows in $experiment_dir")
metrics = reduce(vcat, [DataFrame(CSV.File(joinpath(experiment_dir, f))) for f in metric_files])
sort!(metrics, :job_id)
println("Loaded $(nrow(metrics)) job rows")

# ── load exported route columns, one frame per job ───────────────────────────
function _load_columns(experiment_dir, job_id)
    dir = joinpath(experiment_dir, "routes", "job_$(lpad(job_id, 4, '0'))", "variable_exports")
    path = joinpath(dir, "route_activations.csv")
    isfile(path) || return nothing, nothing
    meta_path = joinpath(dir, "variable_export_metadata.json")
    meta = isfile(meta_path) ? JSON.parsefile(meta_path) : Dict{String, Any}()
    return DataFrame(CSV.File(path)), meta
end

column_frames = DataFrame[]
meta_rows = NamedTuple[]
for row in eachrow(metrics)
    cols, meta = _load_columns(experiment_dir, row.job_id)
    cols === nothing && continue
    cols[!, :job_id] .= row.job_id
    cols[!, :n_pairs] .= row.n_pairs
    cols[!, :seed] .= row.seed
    cols[!, :status] .= row.status
    push!(column_frames, cols)
    push!(meta_rows, (
        job_id = row.job_id,
        status = row.status,
        coverage_shortfall_max = get(meta, "coverage_shortfall_max", missing),
        coverage_max = get(meta, "coverage_max", missing),
        objective_residual = get(meta, "objective_residual", missing),
        theta_relaxed = get(meta, "theta_relaxed", missing),
        n_pool = get(meta, "n_route_columns_in_pool", missing),
    ))
end
isempty(column_frames) && error("no route_activations.csv found under $experiment_dir/routes")
columns = reduce(vcat, column_frames)
checks = DataFrame(meta_rows)
println("Loaded $(nrow(columns)) selected route columns from $(length(column_frames)) jobs\n")

# ── export self-checks first: a failure here invalidates everything below ─────
cov = collect(skipmissing(checks.coverage_shortfall_max))
obj = collect(skipmissing(checks.objective_residual))
@printf("Export self-checks   coverage shortfall max %.3e   objective residual max %.3e\n",
        isempty(cov) ? 0.0 : maximum(cov), isempty(obj) ? 0.0 : maximum(obj))
if !isempty(cov) && maximum(cov) > 1e-6
    println("  !! coverage shortfall exceeds 1e-6 -- exported assignments leave demand uncovered")
end
if !isempty(obj) && maximum(obj) > 1e-4
    println("  !! objective residual exceeds 1e-4 -- exported rows do not reproduce the objective")
end
if any(skipmissing(checks.theta_relaxed))
    println("  !! some jobs exported a RELAXED (fractional) theta -- integer recovery did not run")
end
println()

certified = filter(:status => ==("exhausted"), columns)
uncertified = filter(:status => !=("exhausted"), columns)
@printf("Certified columns %d   uncertified (reported separately) %d\n\n",
        nrow(certified), nrow(uncertified))

# ── the study question ───────────────────────────────────────────────────────
function _summarize(df::DataFrame, label::AbstractString)
    n = nrow(df)
    n == 0 && return (; scope=label, n_columns=0, n_elementary=0, pct_elementary=missing,
        n_multi_stop=0, pct_elementary_multi_stop=missing, mean_stops=missing,
        max_revisits=missing, mean_distinct=missing)
    revisits = df.n_stops .- df.n_distinct_stations
    # A 2-stop column cannot revisit and is elementary by construction, so pooling it
    # inflates the headline. `multi_stop` restates the rate over columns that *could*
    # have revisited -- the only ones where elementarity is a choice the optimum made.
    multi = df[df.n_stops .>= 3, :]
    return (;
        scope = label,
        n_columns = n,
        n_elementary = count(df.is_elementary),
        pct_elementary = 100 * count(df.is_elementary) / n,
        n_multi_stop = nrow(multi),
        pct_elementary_multi_stop = nrow(multi) == 0 ? missing :
            100 * count(multi.is_elementary) / nrow(multi),
        mean_stops = mean(df.n_stops),
        max_revisits = maximum(revisits),
        mean_distinct = mean(df.n_distinct_stations),
    )
end

summary_rows = NamedTuple[_summarize(certified, "certified (all)")]
for p in sort(unique(certified.n_pairs))
    push!(summary_rows, _summarize(filter(:n_pairs => ==(p), certified), "certified n_pairs=$p"))
end
nrow(uncertified) > 0 && push!(summary_rows, _summarize(uncertified, "uncertified"))
summary = DataFrame(summary_rows)

println("Elementarity of selected route columns")
println("-"^92)
for r in eachrow(summary)
    @printf("%-22s  n=%4d  elementary %5.1f%%   multi-stop(>=3) n=%4d elementary %s   mean stops %.2f  max revisits %d\n",
        r.scope, r.n_columns,
        r.pct_elementary === missing ? NaN : r.pct_elementary,
        r.n_multi_stop,
        r.pct_elementary_multi_stop === missing ? "  n/a" :
            @sprintf("%5.1f%%", r.pct_elementary_multi_stop),
        r.mean_stops === missing ? NaN : r.mean_stops,
        r.max_revisits === missing ? 0 : r.max_revisits)
end
println()

# Per-instance view: is non-elementarity concentrated in a few instances or spread thin?
per_instance = combine(groupby(certified, [:job_id, :n_pairs, :seed]),
    nrow => :n_columns,
    :is_elementary => count => :n_elementary,
    :is_elementary => (v -> count(!, v)) => :n_non_elementary,
    :n_stops => maximum => :max_stops_used,
    [:n_stops, :n_distinct_stations] => ((a, b) -> maximum(a .- b)) => :max_revisits,
)
n_clean = count(==(0), per_instance.n_non_elementary)
@printf("Instances whose entire certified selection is elementary: %d / %d (%.1f%%)\n\n",
        n_clean, nrow(per_instance), 100 * n_clean / max(nrow(per_instance), 1))

# Stop-count distribution, since elementarity is only at stake once routes are long.
println("Selected-column stop-count distribution (certified)")
dist = nrow(certified) == 0 ?
    DataFrame(n_stops=Int[], n_columns=Int[], pct_elementary=Float64[]) :
    sort(combine(groupby(certified, :n_stops), nrow => :n_columns,
    :is_elementary => (v -> 100 * count(v) / length(v)) => :pct_elementary), :n_stops)
for r in eachrow(dist)
    @printf("  %2d stops: %4d columns   %5.1f%% elementary\n",
            r.n_stops, r.n_columns, r.pct_elementary)
end
println()

CSV.write(joinpath(output_dir, "elementarity_summary.csv"), summary)
CSV.write(joinpath(output_dir, "per_instance_elementarity.csv"), per_instance)
CSV.write(joinpath(output_dir, "stop_count_distribution.csv"), dist)
CSV.write(joinpath(output_dir, "selected_columns.csv"), columns)
CSV.write(joinpath(output_dir, "export_self_checks.csv"), checks)
println("Wrote 5 CSVs to $output_dir")
