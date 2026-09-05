"""Study 10 analysis: does no-good-cut certification fire at n=20 and n=30?

Structurally Study 9's analysis, with one deliberate difference that matters at n=30.

**Study 9 required both the arm and the baseline to have certified before reporting
anything about a cell.** That is right when the baseline certifies -- the question there is
"is the relaxation cheaper" -- and it is exactly wrong at n=30, which sits past the
measured CG certification frontier (p=16 certifies through n<=20 at every scenario count,
n=30 only at s=1). If the baseline never certifies, that filter discards the entire
finding. So the headline table here is the **certification outcome cross-tab**: how many
cells the arm certified that the baseline did not, and vice versa. The paired speedup is
reported second, still only over cells where both finished, because a budget-stopped run's
wall measures the budget rather than the work.

Everything is grouped by `(n_stations, n_clusters)`. `K` is swept as a fraction of `n`, so
`k_fraction` is what makes the two sizes comparable; the absolute `K` is printed alongside
because that is what a run is actually configured with.

Four blocks, in the order they should be read:

1. **Correctness gate.** Any two runs of the same cell that both certified must agree on
   the objective. Certification only decides *when to stop pricing*; a moved certified
   objective is a bug, not a finding. Affected cells are dropped from everything below.
2. **Certification outcome per size and K** -- the go/no-go, as a cross-tab against what
   the baseline managed on the same cell.
3. **The no-good trace** -- rounds, cuts and subset sizes. This says *how* an arm got its
   answer, and separates the three failure modes (refuted / round cap / out of wall). It
   also recovers Study 9's one-shot question for free via `certified_at_round_1`.
4. **Cost and saving**, paired against baseline where both finished.

Usage: julia --project=<root> analyze.jl [<study10_dir>] [<output_dir>]
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
    _newest(experiments, "study10_nogood_certification_scaling")
output_dir = length(ARGS) >= 2 ? abspath(ARGS[2]) :
    joinpath(normpath(joinpath(@__DIR__, "..", "results")), basename(study_dir))
mkpath(output_dir)
println("study10  $study_dir")
println("writing  $output_dir\n")

files = filter(f -> occursin(r"^job_\d+\.csv$", f), readdir(study_dir))
isempty(files) && error("no job_NNNN.csv rows in $study_dir")
# `cols=:union` so a run whose rows span a schema change (a renamed or newly added
# metric) still loads, with the absent column filled as `missing` rather than erroring.
rows = reduce((a, b) -> vcat(a, b; cols=:union),
              [DataFrame(CSV.File(joinpath(study_dir, f))) for f in files])
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
    select(arms, :cell_id, :arm, :n_clusters, :k_fraction, :n_stations, :n_pairs, :seed,
           :status, :wall_sec, :objective_value, :cg_iterations, :n_columns,
           :certified_by_relaxation, :certification_rounds,
           :certification_refuted_rounds, :certification_inconclusive_rounds,
           :certification_sec, :non_certifying_certification_sec, :certification_harvested_columns,
           :certifying_rounds, :cluster_sizes,
           :certification_max_rounds,
           :nogood_attempts, :nogood_max_rounds, :nogood_median_rounds, :nogood_total_cuts,
           :nogood_median_subset_size, :nogood_certified_at_round_1, :nogood_outcomes,
           :total_pricing_sec, :cg_stop_reason),
    select(baseline, :cell_id, :status => :base_status, :wall_sec => :base_wall,
           :objective_value => :base_obj, :cg_iterations => :base_iters,
           :n_columns => :base_cols, :certifying_rounds => :base_certifying_rounds,
           :total_pricing_sec => :base_pricing_sec, :cg_stop_reason => :base_stop_reason),
    on = :cell_id,
)
println("paired $(nrow(paired)) arm rows against baseline\n")

_median_or(values, fallback) = (v = collect(skipmissing(values));
                                isempty(v) ? fallback : median(v))
_first_or(values, fallback) = (v = collect(skipmissing(values));
                               isempty(v) ? fallback : first(v))
_by_size_and_k(df) = sort(collect(groupby(df, [:n_stations, :n_clusters])),
                          by = g -> (first(g.n_stations), first(g.n_clusters)))

# ── 1. correctness gate ─────────────────────────────────────────────────────
# Only comparable when BOTH runs actually finished: a budget-stopped run's objective is an
# upper bound on a partial column pool, so it is expected to differ and is not evidence.
function _obj_mismatch(r)
    (ismissing(r.objective_value) || ismissing(r.base_obj)) && return false
    r.status == "exhausted" && r.base_status == "exhausted" || return false
    return abs(r.objective_value - r.base_obj) > 1e-6 * max(1.0, abs(r.base_obj))
end
mismatched = filter(_obj_mismatch, paired)
println("=== 1. correctness gate ===")
if nrow(mismatched) == 0
    println("PASS: every finished arm/baseline pair agrees on the objective")
else
    println("FAIL: $(nrow(mismatched)) finished pair(s) disagree -- these are bugs, not results:")
    for r in eachrow(mismatched)
        @printf("   %-16s %-34s arm=%.6f base=%.6f\n", r.arm, r.cell_id,
                r.objective_value, r.base_obj)
    end
end
paired = filter(r -> !_obj_mismatch(r), paired)

# ── 2. certification outcome cross-tab ──────────────────────────────────────
# The headline. `arm_only` is the whole point of the study at n=30: cells where the
# relaxation proved optimality and baseline CG could not.
println("\n=== 2. certification outcome per size and cluster count ===")
println("(baseline 'certified' = its own pricing exhausted, i.e. status == exhausted)")
@printf("%-5s %-5s %-6s %6s %10s %10s %10s %10s\n",
        "n", "K", "K/n", "cells", "arm cert", "base cert", "arm only", "base only")
outcome_rows = NamedTuple[]
for gdf in _by_size_and_k(paired)
    n, k = first(gdf.n_stations), first(gdf.n_clusters)
    arm_cert = [x === true for x in gdf.certified_by_relaxation]
    base_cert = [s == "exhausted" for s in gdf.base_status]
    arm_only = count(i -> arm_cert[i] && !base_cert[i], eachindex(arm_cert))
    base_only = count(i -> !arm_cert[i] && base_cert[i], eachindex(arm_cert))
    @printf("%-5d %-5d %-6.2f %6d %10d %10d %10d %10d\n", n, k, first(gdf.k_fraction),
            nrow(gdf), count(arm_cert), count(base_cert), arm_only, base_only)
    push!(outcome_rows, (
        n_stations = n, n_clusters = k, k_fraction = first(gdf.k_fraction),
        cells = nrow(gdf), arm_certified = count(arm_cert),
        baseline_certified = count(base_cert),
        arm_certified_only = arm_only, baseline_certified_only = base_only,
        arm_certification_rate = count(arm_cert) / nrow(gdf),
        cluster_sizes = string(_first_or(gdf.cluster_sizes, "?")),
    ))
end
CSV.write(joinpath(output_dir, "certification_outcome_by_size_and_k.csv"), DataFrame(outcome_rows))

# ── 3. the no-good trace ────────────────────────────────────────────────────
# How the answer was reached, and which of the three failure modes applies when it was not.
# `max rounds` at the configured `certification_max_rounds` means the ROUND CAP bound;
# below it with inconclusive attempts means `certification_time_limit_sec` bound; refuted
# attempts are true negatives and indict neither.
println("\n=== 3. no-good trace: rounds, cuts, and the size of each exhaustive subset ===")
# Cut counts are meaningful only PER ATTEMPT. `cluster_sets` starts empty on every
# (CG iteration x scenario) call and is discarded when it returns -- it has to be, since a
# cut is only valid at the duals it was derived under (see nogood_certify.jl). So a solve's
# summed cut count measures how many times the loop RAN, not how deep any one loop went.
# The depth of a single loop is `max rnds`, which is what the round cap bounds.
@printf("%-5s %-5s %9s %10s %10s %11s %10s %12s %10s\n",
        "n", "K", "attempts", "med rnds", "max rnds", "cuts/attmpt", "cap", "med |S|", "cert@r1")
trace_rows = NamedTuple[]
for gdf in _by_size_and_k(paired)
    n, k = first(gdf.n_stations), first(gdf.n_clusters)
    attempts = sum(skipmissing(gdf.nogood_attempts); init = 0)
    cuts = sum(skipmissing(gdf.nogood_total_cuts); init = 0)
    @printf("%-5d %-5d %9d %10.1f %10.0f %11.1f %10s %12.1f %10d\n", n, k, attempts,
            _median_or(gdf.nogood_median_rounds, NaN),
            _median_or(gdf.nogood_max_rounds, NaN),
            attempts == 0 ? NaN : cuts / attempts,
            string(_first_or(gdf.certification_max_rounds, "?")),
            _median_or(gdf.nogood_median_subset_size, NaN),
            sum(skipmissing(gdf.nogood_certified_at_round_1); init = 0))
    push!(trace_rows, (
        n_stations = n, n_clusters = k,
        attempts = attempts,
        median_rounds = _median_or(gdf.nogood_median_rounds, missing),
        # Deepest single loop across this arm's runs -- the number to compare against
        # `round_cap`, and the only cut-depth figure that means anything.
        max_rounds = _median_or(gdf.nogood_max_rounds, missing),
        round_cap = _first_or(gdf.certification_max_rounds, missing),
        cuts_per_attempt = attempts == 0 ? missing : cuts / attempts,
        # Summed over every attempt of every run in this group. NOT a cut-set size:
        # it scales with how many times the loop ran. Kept only as a raw total.
        cuts_all_attempts = cuts,
        median_subset_size = _median_or(gdf.nogood_median_subset_size, missing),
        # Attempts that certified with NO cut placed -- i.e. exactly what the one-shot
        # `:relaxed_cluster` mode (Study 9's arm, 0/31 at n=15) would have certified.
        certified_at_round_1 = sum(skipmissing(gdf.nogood_certified_at_round_1); init = 0),
        refuted_rounds = sum(skipmissing(gdf.certification_refuted_rounds); init = 0),
        inconclusive_rounds = sum(skipmissing(gdf.certification_inconclusive_rounds); init = 0),
    ))
end
CSV.write(joinpath(output_dir, "nogood_trace_by_size_and_k.csv"), DataFrame(trace_rows))

println("\n--- failure mode of the attempts that did not certify ---")
@printf("%-5s %-5s %10s %12s %14s\n", "n", "K", "attempts", "refuted", "inconclusive")
for gdf in _by_size_and_k(paired)
    @printf("%-5d %-5d %10d %12d %14d\n", first(gdf.n_stations), first(gdf.n_clusters),
            sum(skipmissing(gdf.certification_rounds); init = 0),
            sum(skipmissing(gdf.certification_refuted_rounds); init = 0),
            sum(skipmissing(gdf.certification_inconclusive_rounds); init = 0))
end

# ── 4. cost and saving ──────────────────────────────────────────────────────
# Restricted to cells where BOTH runs finished, since a budget-stopped run's wall measures
# the budget. At n=30 this table is expected to be thin or empty -- that is the finding,
# not a gap, and block 2 is where the n=30 answer lives.
println("\n=== 4. wall-clock effect (cells where both arm and baseline finished) ===")
timed = filter(r -> r.status == "exhausted" && r.base_status == "exhausted", paired)
@printf("%-5s %-5s %6s %10s %10s %8s %12s %12s\n",
        "n", "K", "cells", "med base", "med arm", "med x", "med cert s", "med non-cert s")
speedup_rows = NamedTuple[]
for gdf in _by_size_and_k(timed)
    nrow(gdf) == 0 && continue
    n, k = first(gdf.n_stations), first(gdf.n_clusters)
    speedups = gdf.base_wall ./ gdf.wall_sec
    @printf("%-5d %-5d %6d %10.1f %10.1f %8.2f %12.1f %12.1f\n", n, k, nrow(gdf),
            median(gdf.base_wall), median(gdf.wall_sec), median(speedups),
            _median_or(gdf.certification_sec, NaN),
            _median_or(gdf.non_certifying_certification_sec, NaN))
    push!(speedup_rows, (
        n_stations = n, n_clusters = k, cells = nrow(gdf),
        median_base_wall_sec = median(gdf.base_wall),
        median_arm_wall_sec = median(gdf.wall_sec),
        median_speedup = median(speedups),
        min_speedup = minimum(speedups), max_speedup = maximum(speedups),
        median_certification_sec = _median_or(gdf.certification_sec, missing),
        median_non_certifying_sec = _median_or(gdf.non_certifying_certification_sec, missing),
        median_harvested_columns = _median_or(gdf.certification_harvested_columns, missing),
        median_certifying_rounds = _median_or(gdf.certifying_rounds, missing),
        median_base_certifying_rounds = _median_or(gdf.base_certifying_rounds, missing),
    ))
end
nrow(timed) == 0 && println("(no cell had both arm and baseline finish -- see block 2)")
CSV.write(joinpath(output_dir, "speedup_by_size_and_k.csv"), DataFrame(speedup_rows))

# ── 4b. speedup against instance difficulty ─────────────────────────────────
# A median over this column is misleading and was measured to be: at n=20, K=12 ran
# 0.68x-2.28x, and the spread is not noise -- it orders almost perfectly by how long the
# BASELINE took. Certification replaces the final exhaustive pricing round, which dominates
# a hard cell's solve and is already cheap on an easy one, so the feature pays exactly where
# the cost is. Reporting the median alone hides that and understates the case; this block
# is what the conclusion should rest on.
println("\n--- speedup vs baseline difficulty (per cell, sorted by baseline wall) ---")
if nrow(timed) > 0
    for sdf in sort(collect(groupby(timed, :n_stations)), by = g -> first(g.n_stations))
        n = first(sdf.n_stations)
        ks = sort(unique(sdf.n_clusters))
        @printf("n=%d\n", n)
        @printf("  %-26s %9s", "cell", "base s")
        for k in ks
            @printf("%12s", "K=$k")
        end
        println()
        cells = unique(sdf.cell_id)
        for cell in sort(cells; by = c -> -maximum(r.base_wall for r in eachrow(sdf) if r.cell_id == c))
            sub = filter(r -> r.cell_id == cell, sdf)
            @printf("  %-26s %9.0f", cell, first(sub.base_wall))
            for k in ks
                m = filter(r -> r.n_clusters == k, sub)
                nrow(m) == 0 ? @printf("%12s", "-") :
                    @printf("%11.2fx", first(m.base_wall) / first(m.wall_sec))
            end
            println()
        end
    end
    # One number for the trend: Spearman-style rank correlation between the baseline wall
    # and the speedup, per size and K. Positive means "helps more on harder cells".
    println("\n  rank correlation(baseline wall, speedup) -- positive = pays off on hard cells")
    function _spearman(x, y)
        length(x) < 3 && return NaN
        rank(v) = (p = sortperm(v); r = similar(p, Float64); r[p] = 1:length(v); r)
        rx, ry = rank(collect(x)), rank(collect(y))
        mx, my = sum(rx) / length(rx), sum(ry) / length(ry)
        num = sum((rx .- mx) .* (ry .- my))
        den = sqrt(sum((rx .- mx) .^ 2) * sum((ry .- my) .^ 2))
        return den == 0 ? NaN : num / den
    end
    for gdf in _by_size_and_k(timed)
        nrow(gdf) < 3 && continue
        @printf("    n=%-4d K=%-4d rho=%+.2f  (%d cells)\n",
                first(gdf.n_stations), first(gdf.n_clusters),
                _spearman(gdf.base_wall, gdf.base_wall ./ gdf.wall_sec), nrow(gdf))
    end
end

# The mechanism check: certification is supposed to REPLACE the two-tier certifying round,
# so a working arm should show fewer of those than its baseline.
println("\n--- certifying-round escalations replaced (median per cell) ---")
for gdf in _by_size_and_k(timed)
    @printf("n=%-4d K=%-4d baseline %.1f -> arm %.1f\n",
            first(gdf.n_stations), first(gdf.n_clusters),
            _median_or(gdf.base_certifying_rounds, NaN),
            _median_or(gdf.certifying_rounds, NaN))
end

CSV.write(joinpath(output_dir, "paired_rows.csv"), paired)
println("\nwrote certification_outcome_by_size_and_k.csv, nogood_trace_by_size_and_k.csv, " *
        "speedup_by_size_and_k.csv, paired_rows.csv to $output_dir")
