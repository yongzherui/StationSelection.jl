"""Study 8 analysis: warm-start speedup, and how fast the elementary phase exhausts.

Joins Study 8's `warm_start` rows to **Study 7's completed `exact` rows** on `cell_id`
(same grid, budgets, allocation and formulation), then answers the two study questions:

1. Speedup: paired per-cell `exact_wall / warm_wall`. Reported as a distribution (median
   and range), not a ratio of totals -- totals are dominated by the slowest cells and both
   studies run on `mit_preemptable`, so hardware varies per cell and a single pooled ratio
   would hide that.
2. Phase-1 exhaustion: `warm_start_sec` absolute, and as a share of the warm run's wall.

**Correctness gate first.** Both arms must reach the same certified optimum on a cell --
warm start changes only which pricer runs first, and phase 2 is the full pricer either way.
A mismatch is a bug, not a finding, so it is reported before any timing number and the
affected cells are excluded from the speedup summary.

Only cells where BOTH arms certified (`status == "exhausted"`) enter the speedup: a
budget-stopped run stopped on the clock, so its wall measures the budget rather than the
work, and pairing it with a certified run compares different quantities.

Usage: julia --project=<root> analyze.jl [<study8_dir>] [<study7_dir>] [<output_dir>]
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
study8_dir = length(ARGS) >= 1 ? abspath(ARGS[1]) : _newest(experiments, "study8_warm_start_speedup")
study7_dir = length(ARGS) >= 2 ? abspath(ARGS[2]) : _newest(experiments, "study7_route_elementarity")
output_dir = length(ARGS) >= 3 ? abspath(ARGS[3]) :
    joinpath(normpath(joinpath(@__DIR__, "..", "results")), basename(study8_dir))
mkpath(output_dir)
println("warm_start  $study8_dir")
println("exact       $study7_dir")
println("writing     $output_dir\n")

function _load(dir)
    files = filter(f -> occursin(r"^job_\d+\.csv$", f), readdir(dir))
    isempty(files) && error("no job_NNNN.csv rows in $dir")
    return reduce(vcat, [DataFrame(CSV.File(joinpath(dir, f))) for f in files])
end

warm = _load(study8_dir)
exact = _load(study7_dir)
println("loaded $(nrow(warm)) warm_start rows, $(nrow(exact)) exact rows")

paired = innerjoin(
    select(warm, :cell_id, :n_pairs, :seed, :status => :warm_status,
           :wall_sec => :warm_wall, :objective_value => :warm_obj,
           :cg_iterations => :warm_iters, :n_columns => :warm_cols,
           :warm_start_sec, :warm_start_iterations,
           :phase1_iterations, :phase2_iterations,
           :phase1_pricing_sec, :phase2_pricing_sec,
           :phase1_columns, :phase2_columns, :cg_optimality_scope),
    select(exact, :cell_id, :status => :exact_status, :wall_sec => :exact_wall,
           :objective_value => :exact_obj, :cg_iterations => :exact_iters,
           :n_columns => :exact_cols),
    on = :cell_id,
)
println("paired on cell_id: $(nrow(paired)) cells\n")

# ── correctness gate ─────────────────────────────────────────────────────────
both_certified = (paired.warm_status .== "exhausted") .& (paired.exact_status .== "exhausted")
obj_ok = map(eachrow(paired)) do r
    (ismissing(r.warm_obj) || ismissing(r.exact_obj)) && return false
    isapprox(r.warm_obj, r.exact_obj; rtol = 1e-6, atol = 1e-6)
end
bad = paired[both_certified .& .!obj_ok, :]
scope_bad = paired[paired.cg_optimality_scope .!= "full_route_universe", :]

println("Correctness gate")
println("-"^70)
@printf("  cells where both arms certified: %d / %d\n", count(both_certified), nrow(paired))
if nrow(bad) > 0
    println("  !! OBJECTIVE MISMATCH on $(nrow(bad)) certified cells -- this is a BUG:")
    for r in eachrow(bad)
        @printf("     %s  warm=%.6f  exact=%.6f\n", r.cell_id, r.warm_obj, r.exact_obj)
    end
else
    println("  ✓ every certified cell agrees on the objective (rtol 1e-6)")
end
if nrow(scope_bad) > 0
    println("  !! $(nrow(scope_bad)) warm runs did not end in the full route universe:")
    for r in eachrow(scope_bad)
        println("     $(r.cell_id): scope=$(r.cg_optimality_scope)")
    end
else
    println("  ✓ every warm run certified against the full route universe")
end
println()

usable = paired[both_certified .& obj_ok, :]
usable.speedup = usable.exact_wall ./ usable.warm_wall
usable.warm_start_share = usable.warm_start_sec ./ usable.warm_wall

# ── Q1: speedup ──────────────────────────────────────────────────────────────
function _summ(df, label)
    n = nrow(df)
    n == 0 && return (; scope=label, n=0, median_speedup=missing, min_speedup=missing,
        max_speedup=missing, n_faster=0, median_exact_wall=missing, median_warm_wall=missing)
    (; scope=label, n=n,
       median_speedup=median(df.speedup), min_speedup=minimum(df.speedup),
       max_speedup=maximum(df.speedup), n_faster=count(>(1.0), df.speedup),
       median_exact_wall=median(df.exact_wall), median_warm_wall=median(df.warm_wall))
end
rows = [_summ(usable, "all certified")]
for p in sort(unique(usable.n_pairs))
    push!(rows, _summ(filter(:n_pairs => ==(p), usable), "n_pairs=$p"))
end
speedup = DataFrame(rows)

println("Q1  Warm-start speedup (exact_wall / warm_wall; >1 means warm start is faster)")
println("-"^94)
for r in eachrow(speedup)
    r.n == 0 && continue
    @printf("  %-16s n=%2d  median %.2fx  range %.2f-%.2fx  faster on %d/%d  median wall %.0fs -> %.0fs\n",
        r.scope, r.n, r.median_speedup, r.min_speedup, r.max_speedup,
        r.n_faster, r.n, r.median_exact_wall, r.median_warm_wall)
end
println()

# ── Q2: how fast does the elementary phase exhaust ───────────────────────────
function _phase(df, label)
    n = nrow(df)
    n == 0 && return (; scope=label, n=0, median_warm_start_sec=missing,
        median_share=missing, median_p1_iters=missing, median_p2_iters=missing,
        median_p1_cols=missing, median_p2_cols=missing)
    (; scope=label, n=n,
       median_warm_start_sec=median(df.warm_start_sec),
       median_share=median(df.warm_start_share),
       median_p1_iters=median(df.phase1_iterations),
       median_p2_iters=median(df.phase2_iterations),
       median_p1_cols=median(df.phase1_columns),
       median_p2_cols=median(df.phase2_columns))
end
prows = [_phase(usable, "all certified")]
for p in sort(unique(usable.n_pairs))
    push!(prows, _phase(filter(:n_pairs => ==(p), usable), "n_pairs=$p"))
end
phases = DataFrame(prows)

println("Q2  Elementary (phase 1) exhaustion")
println("-"^94)
for r in eachrow(phases)
    r.n == 0 && continue
    @printf("  %-16s n=%2d  phase1 %.0fs (%.0f%% of warm wall)  iters %g->%g  columns %g->%g\n",
        r.scope, r.n, r.median_warm_start_sec, 100 * r.median_share,
        r.median_p1_iters, r.median_p2_iters, r.median_p1_cols, r.median_p2_cols)
end
println()

# A warm start that never handed off did not measure what this study is about.
never = usable[usable.warm_start_iterations .== 0, :]
if nrow(never) > 0
    println("  !! $(nrow(never)) cells never completed phase 1 (warm_start_iterations==0);")
    println("     their wall reflects an unfinished elementary phase, not a handoff.")
    println()
end

CSV.write(joinpath(output_dir, "speedup_summary.csv"), speedup)
CSV.write(joinpath(output_dir, "phase_summary.csv"), phases)
CSV.write(joinpath(output_dir, "paired_cells.csv"), usable)
println("Wrote 3 CSVs to $output_dir")
