"""Study 10: does the certification frontier scale in `n`, or in `n x s`?

Three questions, in the order that can invalidate the run:

1. SAFETY. Every arm solves the identical model at a given (n, s, seed), so two arms that
   both certify must agree on the objective. Disagreement is a false certificate, not a
   tuning result, and nothing below means anything until it is ruled out.
2. FRONTIER. How many seeds certified at each (n, s), and WHICH. A different count on the
   same seed set is a speed story; a different SET is a frontier story.
3. WHY. Where the failures stop -- budget, `pricing_inconclusive`, or unexhausted station
   searches -- and whether that differs from s=3.

Reports CUT YIELD PER ROUND, never summed over attempts. Cut sets are local to an attempt
(`two_tier.jl` initialises them empty each time), so a sum over attempts measures mostly
how many rounds ran. Getting this wrong is what made Study 9's cap table say the opposite
of the truth -- see `notes/2026-09-09_n40_certification_frontier_5_of_10.md`.

Usage: julia --project=../.. analyze.jl <output-dir>
"""

using CSV
using DataFrames
using Printf
using Statistics

length(ARGS) == 1 || error("usage: analyze.jl <output-dir>")
root = abspath(ARGS[1])
isdir(root) || error("missing output directory $root")

files = filter(f -> occursin(r"^n\d+_seed\d+_[a-z_0-9]+\.csv$", f), readdir(root))
isempty(files) && error("no result rows in $root")
rows = reduce((a, b) -> vcat(a, b; cols = :union),
    (CSV.read(joinpath(root, f), DataFrame) for f in files))

val(r, c, d = missing) = hasproperty(r, c) ? getproperty(r, c) : d
short(a) = startswith(String(a), "twotier") ? "twotier" : "single"

# ── 1. SAFETY ───────────────────────────────────────────────────────────────
println("\n=== 1. SAFETY: arms that both certify must agree on the objective")
bad = 0
for g in groupby(filter(r -> r.status == "certified", rows), [:n_stations, :n_scenarios, :seed])
    objs = collect(skipmissing(g.objective_value))
    length(objs) < 2 && continue
    if maximum(objs) - minimum(objs) > 1e-6
        bad += 1
        @printf("  MISMATCH n=%d s=%d seed=%d: %s\n", g.n_stations[1], g.n_scenarios[1],
                g.seed[1], join(string.(objs), " vs "))
    end
end
println(bad == 0 ? "  OK -- every multiply-certified instance agrees" :
                   "  $bad MISMATCHES -- stop here, the rest is meaningless")

# ── 2. FRONTIER ─────────────────────────────────────────────────────────────
println("\n=== 2. FRONTIER: certified seeds by (n, s, arm)")
@printf("%6s %4s %9s %10s   %s\n", "n", "s", "arm", "certified", "seeds")
for g in sort(groupby(rows, [:n_stations, :n_scenarios, :arm]) |> collect,
              by = x -> (x.n_stations[1], x.n_scenarios[1], String(x.arm[1])))
    c = filter(r -> r.status == "certified", g)
    @printf("%6d %4d %9s %6d/%-3d   %s\n", g.n_stations[1], g.n_scenarios[1],
            short(g.arm[1]), nrow(c), nrow(g), join(sort(c.seed), ", "))
end

# ── 3. WHY ──────────────────────────────────────────────────────────────────
println("\n=== 3. WHERE THE FAILURES STOP")
@printf("%6s %4s %9s %6s  %22s %8s %9s\n",
        "n", "s", "arm", "seed", "stop", "wall", "unused")
for r in sort(filter(r -> r.status != "certified", rows),
              [:n_stations, :n_scenarios, :arm, :seed]) |> eachrow
    lim = something(val(r, :total_limit_sec), 7200.0)
    @printf("%6d %4d %9s %6d  %22s %8.0f %9.0f\n", r.n_stations, r.n_scenarios,
            short(r.arm), r.seed, val(r, :cg_stop_reason, "?"), r.wall_sec,
            lim - r.wall_sec)
end

# ── per-round cut yield, if the two-tier dumps are present ──────────────────
adir = joinpath(root, "attempts")
if isdir(adir)
    println("\n=== 4. CUT YIELD PER ROUND (never summed over attempts)")
    @printf("%6s %4s %6s %9s %11s %10s %12s %11s\n", "n", "s", "seed",
            "macroRnd", "macroCut/rnd", "stnSrch", "mesoCut/srch", "unexh/srch")
    for f in sort(readdir(adir))
        endswith(f, ".csv") || continue
        a = CSV.read(joinpath(adir, f), DataFrame)
        (hasproperty(a, :macro_rounds) && nrow(a) > 0) || continue
        mr, sr = sum(a.macro_rounds), sum(a.station_rounds)
        (mr > 0 && sr > 0) || continue
        @printf("%6d %4d %6d %9d %11.3f %10d %12.3f %11.3f\n",
                a.n_stations[1], get(a, :n_scenarios, [missing])[1] |> x -> ismissing(x) ? 0 : x,
                a.seed[1], mr, sum(a.macro_cuts) / mr, sr,
                sum(a.meso_cuts) / sr, sum(a.station_unexhausted) / sr)
    end
end
println()
