"""Study 9 analysis: does the relaxed-cluster relaxation certify, and at which K?

All arms live in one run directory here (unlike Study 8's cross-study join), so the
analysis is a within-study group-by on `n_clusters`, paired against the `baseline` arm
(`n_clusters == 0`) cell by cell.

Three questions, in the order they should be answered:

1. **Correctness gate.** Every arm must reach the same certified optimum on a cell. The
   relaxation only decides *when to stop pricing*; it never changes which columns are
   priced, so a certified objective that moves is a bug, not a finding. Reported first,
   and affected cells are excluded from every timing summary below.

2. **Certification rate per K** -- the go/no-go. The share of cells where
   `certified_by_relaxation` fired, per cluster count, followed by the failure mode of
   every attempt that did not. That second table is what makes an all-zero rate readable:
   *refuted* means the relaxation was too loose (raise K), *inconclusive* means the attempt
   ran out of its own budget (raise `certification_time_limit_sec`). The `K = n` arm is the
   ceiling control -- the tightest possible relaxation -- so whatever it achieves bounds
   what any smaller K can.

3. **What it cost and what it saved, per K.** Paired against baseline on the same cell:
   `wall_sec` speedup, plus the two numbers that explain it -- `certification_sec` (what
   the attempts cost, successes and failures together) and `failed_certification_sec`
   (pure overhead, the attempts that proved nothing). An arm can certify often and still
   lose on wall if its per-round attempts are expensive, which is exactly the tradeoff K
   controls, so both are reported rather than a single net number.

Only cells where BOTH the arm and baseline certified (`status == "exhausted"`) enter the
timing summaries: a budget-stopped run's wall measures the budget, not the work.

Usage: julia --project=<root> analyze.jl [<study9_dir>] [<output_dir>]
"""

using CSV
using DataFrames
using Printf
using Statistics

function _newest(root, needle)
    isdir(root) || error("no experiments directory at $root")
    c = sort!(filter(d -> occursin(needle, d) && !startswith(d, "_"), readdir(root)))
    isempty(c) && error("no $needle run under $root")
    return joinpath(root, last(c))
end

experiments = normpath(joinpath(@__DIR__, "..", "experiments"))
study_dir = length(ARGS) >= 1 ? abspath(ARGS[1]) :
    _newest(experiments, "study9_relaxed_cluster_certification")
output_dir = length(ARGS) >= 2 ? abspath(ARGS[2]) :
    joinpath(normpath(joinpath(@__DIR__, "..", "results")), basename(study_dir))
mkpath(output_dir)
println("study9   $study_dir")
println("writing  $output_dir\n")

files = filter(f -> occursin(r"^job_\d+\.csv$", f), readdir(study_dir))
isempty(files) && error("no job_NNNN.csv rows in $study_dir")
rows = reduce(vcat, [DataFrame(CSV.File(joinpath(study_dir, f))) for f in files])
println("loaded $(nrow(rows)) rows across $(length(unique(rows.arm))) arms")

errored = filter(r -> !ismissing(r.error_message) && !isempty(string(r.error_message)), rows)
if nrow(errored) > 0
    println("\n!! $(nrow(errored)) job(s) errored:")
    for r in eachrow(errored)
        println("   job $(r.job_id) ($(r.arm), $(r.cell_id)): $(first(string(r.error_message), 160))")
    end
end
rows = filter(r -> ismissing(r.error_message) || isempty(string(r.error_message)), rows)

baseline = filter(r -> r.n_clusters == 0, rows)
arms = filter(r -> r.n_clusters != 0, rows)
nrow(baseline) > 0 || error("no baseline (n_clusters == 0) rows -- nothing to pair against")

paired = innerjoin(
    select(arms, :cell_id, :arm, :n_clusters, :n_pairs, :seed, :status,
           :wall_sec, :objective_value, :cg_iterations, :n_columns,
           :certified_by_relaxation, :certification_rounds,
           :certification_refuted_rounds, :certification_inconclusive_rounds,
           :certification_sec, :failed_certification_sec, :certifying_rounds, :cluster_sizes,
           :total_pricing_sec, :cg_stop_reason),
    select(baseline, :cell_id, :status => :base_status, :wall_sec => :base_wall,
           :objective_value => :base_obj, :cg_iterations => :base_iters,
           :n_columns => :base_cols, :certifying_rounds => :base_certifying_rounds,
           :total_pricing_sec => :base_pricing_sec),
    on = :cell_id,
)
println("paired $(nrow(paired)) arm rows against baseline\n")

# ── 1. correctness gate ─────────────────────────────────────────────────────
function _obj_mismatch(r)
    (ismissing(r.objective_value) || ismissing(r.base_obj)) && return false
    r.status == "exhausted" && r.base_status == "exhausted" || return false
    return abs(r.objective_value - r.base_obj) > 1e-6 * max(1.0, abs(r.base_obj))
end
mismatched = filter(_obj_mismatch, paired)
println("=== 1. correctness gate ===")
if nrow(mismatched) == 0
    println("PASS: every certified arm/baseline pair agrees on the objective")
else
    println("FAIL: $(nrow(mismatched)) certified pair(s) disagree -- these are bugs, not results:")
    for r in eachrow(mismatched)
        @printf("   %-16s %-34s arm=%.6f base=%.6f\n", r.arm, r.cell_id,
                r.objective_value, r.base_obj)
    end
end
paired = filter(r -> !_obj_mismatch(r), paired)

# ── 2. certification rate per K ─────────────────────────────────────────────
println("\n=== 2. certification rate per cluster count ===")
@printf("%-6s %-6s %8s %8s %10s %10s\n", "K", "cells", "certified", "rate", "med rounds", "cluster sizes")
# `median`/`first` on an all-missing column would throw, and an arm that errored on every
# cell is a real (if unhappy) state to report rather than crash on.
_median_or(values, fallback) = (v = collect(skipmissing(values));
                                isempty(v) ? fallback : median(v))
_first_or(values, fallback) = (v = collect(skipmissing(values));
                               isempty(v) ? fallback : first(v))

rate_rows = NamedTuple[]
for gdf in sort(collect(groupby(paired, :n_clusters)), by = g -> first(g.n_clusters))
    k = first(gdf.n_clusters)
    certified = count(x -> x === true, gdf.certified_by_relaxation)
    rate = certified / nrow(gdf)
    sizes = _first_or(gdf.cluster_sizes, "?")
    @printf("%-6d %-6d %8d %7.1f%% %10.1f   %s\n", k, nrow(gdf), certified, 100 * rate,
            _median_or(gdf.certification_rounds, NaN), string(sizes))
    push!(rate_rows, (n_clusters = k, cells = nrow(gdf), certified = certified,
                      certification_rate = rate, cluster_sizes = string(sizes)))
end
CSV.write(joinpath(output_dir, "certification_rate_by_k.csv"), DataFrame(rate_rows))

# Why the failures failed. This is what makes an all-zero certification column readable:
# "refuted" means the relaxation found an improving cluster route, so it was too loose and
# the fix is a larger K; "inconclusive" means the attempt ran out of its own budget, so the
# fix is a larger certification_time_limit_sec and no K would have helped. The K = n arm is
# the control here -- it is the tightest possible relaxation, so if even it is refuted
# something is wrong with the relaxation itself rather than with K.
println("\n--- failure mode of the attempts that did not certify ---")
@printf("%-6s %10s %12s %14s\n", "K", "attempts", "refuted", "inconclusive")
mode_rows = NamedTuple[]
for gdf in sort(collect(groupby(paired, :n_clusters)), by = g -> first(g.n_clusters))
    k = first(gdf.n_clusters)
    attempts = sum(skipmissing(gdf.certification_rounds); init = 0)
    refuted = sum(skipmissing(gdf.certification_refuted_rounds); init = 0)
    inconclusive = sum(skipmissing(gdf.certification_inconclusive_rounds); init = 0)
    @printf("%-6d %10d %12d %14d\n", k, attempts, refuted, inconclusive)
    push!(mode_rows, (n_clusters = k, attempts = attempts, refuted = refuted,
                      inconclusive = inconclusive))
end
CSV.write(joinpath(output_dir, "certification_failure_modes.csv"), DataFrame(mode_rows))

# ── 3. cost and saving per K ────────────────────────────────────────────────
println("\n=== 3. wall-clock effect per cluster count (both arms certified only) ===")
timed = filter(r -> r.status == "exhausted" && r.base_status == "exhausted", paired)
@printf("%-6s %-6s %10s %10s %10s %12s %12s\n",
        "K", "cells", "med base", "med arm", "med x", "med cert s", "med waste s")
speedup_rows = NamedTuple[]
for gdf in sort(collect(groupby(timed, :n_clusters)), by = g -> first(g.n_clusters))
    k = first(gdf.n_clusters)
    nrow(gdf) == 0 && continue
    speedups = gdf.base_wall ./ gdf.wall_sec
    @printf("%-6d %-6d %10.1f %10.1f %10.2f %12.1f %12.1f\n", k, nrow(gdf),
            median(gdf.base_wall), median(gdf.wall_sec), median(speedups),
            _median_or(gdf.certification_sec, NaN),
            _median_or(gdf.failed_certification_sec, NaN))
    push!(speedup_rows, (
        n_clusters = k, cells = nrow(gdf),
        median_base_wall_sec = median(gdf.base_wall),
        median_arm_wall_sec = median(gdf.wall_sec),
        median_speedup = median(speedups),
        min_speedup = minimum(speedups), max_speedup = maximum(speedups),
        median_certification_sec = _median_or(gdf.certification_sec, missing),
        median_failed_certification_sec = _median_or(gdf.failed_certification_sec, missing),
        median_certifying_rounds = _median_or(gdf.certifying_rounds, missing),
        median_base_certifying_rounds = _median_or(gdf.base_certifying_rounds, missing),
    ))
end
CSV.write(joinpath(output_dir, "speedup_by_k.csv"), DataFrame(speedup_rows))

# The mechanism check: certification is supposed to REPLACE the two-tier certifying round,
# so a working arm should show fewer of those than its baseline.
println("\n--- certifying-round escalations replaced (median per cell) ---")
for gdf in sort(collect(groupby(timed, :n_clusters)), by = g -> first(g.n_clusters))
    @printf("K=%-4d baseline %.1f -> arm %.1f\n", first(gdf.n_clusters),
            _median_or(gdf.base_certifying_rounds, NaN),
            _median_or(gdf.certifying_rounds, NaN))
end

CSV.write(joinpath(output_dir, "paired_rows.csv"), paired)
println("\nwrote certification_rate_by_k.csv, certification_failure_modes.csv, " *
        "speedup_by_k.csv, paired_rows.csv to $output_dir")
