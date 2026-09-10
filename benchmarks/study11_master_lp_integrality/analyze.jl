"""
Study 11 analysis: is the CG master's LP solution close to binary at INTERMEDIATE
iterations, or only at the end?

    julia --startup-file=no --project=../.. analyze.jl ../experiments/<run>_study11_master_lp_integrality

Writes `case_results.csv`, `per_third.csv` and `ip_gap_trajectory.csv` into
`../results/<run>_study11_master_lp_integrality/` and prints the same tables.

Iterations are bucketed into thirds by ITERATION INDEX, not by wall time. A relaxed-cluster
run spends wildly uneven time per iteration (early rounds exhaust in milliseconds, late
ones burn the full pricing budget), so a time split would put almost every iteration in the
first bucket and answer a different question.
"""

using CSV
using DataFrames
using Printf
using Statistics

length(ARGS) == 1 || error("usage: analyze.jl <experiment_dir>")
const EXPDIR = ARGS[1]
isdir(EXPDIR) || error("not a directory: $EXPDIR")

resultsdir = joinpath(@__DIR__, "..", "results", basename(rstrip(EXPDIR, '/')))
mkpath(resultsdir)

case_files = sort(filter(f -> occursin(r"^n\d+_seed\d+_[a-z0-9_]+\.csv$", basename(f)),
                         readdir(EXPDIR; join=true)))
isempty(case_files) && error("no case result CSVs under $EXPDIR")
cases = reduce(vcat, [CSV.read(f, DataFrame) for f in case_files]; cols=:union)
sort!(cases, [:arm, :seed])

# Which runs' numbers are statements about the MODEL, and which only about a pool.
#
# `ip_gap_pct` is always the integrality gap of the RESTRICTED master -- the pool that
# existed at that iteration. Only when pricing proved the pool complete for the full route
# universe does the final one also equal the integrality gap of the actual problem. A
# budget-stopped run's LP is an upper bound over a truncated pool, so its gap is not
# comparable with a certified run's and must not be pooled with one, or cited against an
# independent implementation of the same model. Computing the flag here rather than
# recalling it at reading time is the whole point: the distinction is invisible in the
# number itself.
cases.pool_certified = coalesce.(cases.converged, false) .&
    (coalesce.(cases.optimality_scope, "") .== "full_route_universe")
CSV.write(joinpath(resultsdir, "case_results.csv"), cases)

println("\n", "="^108)
println("CASE RESULTS  ($(nrow(cases)) runs)")
println("="^108)
show(cases[:, intersect(names(cases), [
    "arm", "seed", "iterations", "termination_status", "stop_reason",
    "integral_iterations_tight", "integral_iterations_loose", "mean_y_frac", "max_y_frac",
    "mean_theta_frac_share", "mean_ip_gap_pct", "max_ip_gap_pct", "final_ip_gap_pct",
    "final_gap_pct", "runtime_sec", "ip_snapshot_sec", "pool_certified"])];
     allrows=true, allcols=true, truncate=40)
println()
@printf("\npool_certified: %d/%d runs proved their pool complete for the full route universe.\n",
        count(cases.pool_certified), nrow(cases))
if !all(cases.pool_certified)
    stopped = cases[.!cases.pool_certified, :]
    println("  NOT certified (final gap is over a TRUNCATED pool, not the model's own " *
            "integrality gap):")
    for r in eachrow(stopped)
        @printf("    %-12s seed %-4d  stop_reason=%s\n", r.arm, r.seed, r.stop_reason)
    end
end

iter_files = sort(filter(f -> endswith(f, ".csv"),
                         readdir(joinpath(EXPDIR, "iterations"); join=true)))
isempty(iter_files) && error("no per-iteration CSVs under $EXPDIR/iterations")
iters = reduce(vcat, [CSV.read(f, DataFrame) for f in iter_files]; cols=:union)
# Carry the pool-completeness flag down to the per-iteration rows, so the gap trajectory
# says which runs' final numbers are about the model rather than about a truncated pool.
# Keep ONLY runs that finished. An in-flight run has an iterations file but no result row,
# and its newest iteration is wherever it happens to have got to -- bucketing that as the
# "late" third would describe a truncated prefix as if it were the end of a run.
done_keys = Set(zip(cases.arm, cases.seed))
iters = iters[[(a, sd) in done_keys for (a, sd) in zip(iters.arm, iters.seed)], :]
nrow(iters) == 0 && error("no per-iteration rows belong to a completed run yet")
iters = leftjoin(iters, cases[:, [:arm, :seed, :pool_certified]]; on=[:arm, :seed])
iters.pool_certified = coalesce.(iters.pool_certified, false)

function third_label(i, n)
    n <= 2 && return "all"
    i <= cld(n, 3) && return "early"
    i <= 2 * cld(n, 3) && return "mid"
    return "late"
end

# `transform` per run rather than a global column: the thirds are relative to each run's own
# iteration count, and runs here range from 9 to 25+ iterations.
iters = combine(groupby(iters, [:arm, :seed])) do sub
    sub = sort(sub, :iteration)
    sub.bucket = [third_label(i, nrow(sub)) for i in 1:nrow(sub)]
    sub.run_iterations .= nrow(sub)
    sub
end

per_third = combine(groupby(iters, [:arm, :bucket]),
    nrow => :iterations,
    :n_columns => mean => :columns,
    :y_n_frac => mean => :y_frac,
    :y_sum_dist => mean => :y_dist,
    :walk_n_frac => mean => :walk_frac,
    :walk_n_support => mean => :walk_support,
    :walk_sum_dist => mean => :walk_dist,
    :theta_n_frac => mean => :theta_frac,
    :theta_n_support => mean => :theta_support,
    :theta_sum_dist => mean => :theta_dist,
    :worst_dist => maximum => :worst_dist,
    :all_integral_tight => (x -> count(x)) => :n_integral_tight,
    :all_integral_loose => (x -> count(x)) => :n_integral_loose,
)
const BUCKET_ORDER = Dict("early" => 1, "mid" => 2, "late" => 3, "all" => 4)
bucket_rank(b) = get(BUCKET_ORDER, b, 99)
sort!(per_third, [:arm, :bucket]; by = x -> x isa AbstractString ? bucket_rank(x) : x)
CSV.write(joinpath(resultsdir, "per_third.csv"), per_third)

println("\n", "="^116)
println("PER-THIRD MEANS  (fractional = distance to {0,1} above 1e-6; thirds by iteration index)")
println("="^116)
@printf("%-13s %-6s %6s %8s | %6s %8s | %6s %8s %9s | %7s %7s %10s | %9s %8s\n",
        "arm", "third", "iters", "columns",
        "y#frac", "y dist", "wlk#fr", "wlk sup", "wlk dist",
        "th#frac", "th sup", "theta dist", "worst", "integral")
for r in eachrow(per_third)
    @printf("%-13s %-6s %6d %8.0f | %6.2f %8.4f | %6.2f %8.1f %9.4f | %7.2f %7.1f %10.4f | %9.6f %4d/%-4d\n",
            r.arm, r.bucket, r.iterations, r.columns,
            r.y_frac, r.y_dist, r.walk_frac, r.walk_support, r.walk_dist,
            r.theta_frac, r.theta_support, r.theta_dist,
            r.worst_dist, r.n_integral_tight, r.iterations)
end

# LP vs IP on the SAME restricted pool, per iteration: the integrality gap of the master as
# columns arrive. This is the direct answer to "how tight is the relaxation", where the
# fractionality columns above answer "how binary are the variables" -- a fractional LP
# point can still cost exactly what the best integer point over the same pool costs, and
# only this table can tell the two apart.
gaps = iters[.!isnan.(coalesce.(iters.ip_gap_pct, NaN)), :]
if nrow(gaps) > 0
    CSV.write(joinpath(resultsdir, "ip_gap_trajectory.csv"),
              gaps[:, intersect(names(gaps), ["arm", "seed", "iteration", "run_iterations",
                   "bucket", "n_columns", "lp_objective", "ip_objective", "ip_gap_pct",
                   "ip_status", "ip_sec", "pool_certified", "worst_dist"])])
    per_third_gap = combine(groupby(gaps, [:arm, :bucket]),
        nrow => :snapshots,
        :ip_gap_pct => mean => :mean_gap_pct,
        :ip_gap_pct => maximum => :max_gap_pct,
        :ip_gap_pct => (x -> count(v -> v <= 1e-6, x)) => :n_zero_gap)
    sort!(per_third_gap, [:arm, :bucket];
          by = x -> x isa AbstractString ? bucket_rank(x) : x)
    println("\n", "="^116)
    println("LP vs IP ON THE SAME RESTRICTED POOL  (integrality gap of the master)")
    println("="^116)
    @printf("%-13s %-6s %10s %12s %12s %14s\n",
            "arm", "third", "snapshots", "mean gap %", "max gap %", "exact (0 gap)")
    for r in eachrow(per_third_gap)
        @printf("%-13s %-6s %10d %12.4f %12.4f %9d/%-4d\n",
                r.arm, r.bucket, r.snapshots, r.mean_gap_pct, r.max_gap_pct,
                r.n_zero_gap, r.snapshots)
    end
    # The headline number, and the only one that is a statement about the problem rather
    # than about a column pool: the gap at the LAST iteration of a run whose pricing
    # certified. Everything else here describes the trajectory getting there.
    final_rows = combine(groupby(gaps[gaps.pool_certified, :], [:arm, :seed])) do sub
        sub[[argmax(sub.iteration)], :]
    end
    if nrow(final_rows) > 0
        println("\n", "="^116)
        println("FINAL-ITERATION INTEGRALITY GAP, certified runs only (pool provably complete)")
        println("="^116)
        @printf("%-13s %6s %6s %14s %14s %10s %9s\n",
                "arm", "seed", "iters", "lp", "ip", "gap %", "worst")
        for r in eachrow(sort(final_rows, [:arm, :seed]))
            @printf("%-13s %6d %6d %14.4f %14.4f %10.5f %9.6f\n",
                    r.arm, r.seed, r.iteration, r.lp_objective, r.ip_objective,
                    r.ip_gap_pct, r.worst_dist)
        end
        @printf("\nmean %.5f%%   max %.5f%%   exactly zero: %d/%d\n",
                mean(final_rows.ip_gap_pct), maximum(final_rows.ip_gap_pct),
                count(<=(1e-6), final_rows.ip_gap_pct), nrow(final_rows))
    end
else
    println("\nno MIP snapshots in this run (ip_every=0, or every snapshot failed)")
end

println("\nwrote $resultsdir/{case_results,per_third,ip_gap_trajectory}.csv")
