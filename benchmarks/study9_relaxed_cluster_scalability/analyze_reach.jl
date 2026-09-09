"""Did the budget-schedule and alignment fixes move the n=40 certification frontier?

The baseline to beat is fixed in advance: **4/10 seeds certified** at n=40, which is what
BOTH `relaxed_k60` (single-tier, K2=24) and `twotier_k60m14` reached over 10 seeds at a
21600 s budget, on the identical seed set {44, 46, 48, 49}. The reach run asks for >= 6/10
at 7200 s, so a win here is a win at one third of the budget.

Three questions, in the order that can invalidate the run:

1. SAFETY. A pricer is a search algorithm: every arm solves the identical model, so two arms
   that both certify a seed must agree on the objective. Disagreement is a false certificate,
   not a tuning result, and nothing else in the output means anything until it is ruled out.
2. REACH. How many seeds certified, and WHICH -- a different count on the same seed set is a
   speed story, a different SET is a frontier story.
3. WHY. Whether the schedule was the binding constraint. `meso_escalations` and
   `station_escalations` count sweeps that a scheduled slice truncated and the attempt's real
   remaining budget then finished; every one of those used to be a round that returned
   `:inconclusive` and threw the attempt away. `guides_used_mean` below `guide_routes` says
   alignment shrinking is active, and `align_skipped` says how often it still gave up.

Usage: julia --project=../.. analyze_reach.jl <output-dir>
"""

using CSV
using DataFrames
using Printf
using Statistics

length(ARGS) == 1 || error("usage: analyze_reach.jl <output-dir>")
root = abspath(ARGS[1])
isdir(root) || error("missing output directory $root")

files = filter(f -> occursin(r"^n40_seed\d+_[a-z_0-9]+\.csv$", f), readdir(root))
isempty(files) && error("no n40 result rows in $root")
rows = reduce((a, b) -> vcat(a, b; cols=:union),
    (CSV.read(joinpath(root, f), DataFrame) for f in files))

val(r, c, d=missing) = hasproperty(r, c) ? getproperty(r, c) : d
num(x) = (ismissing(x) || x === nothing) ? missing : Float64(x)

# ── 1. safety ───────────────────────────────────────────────────────────────────
println("="^78)
println("1. SAFETY: certified objectives must agree across arms on the same seed")
println("="^78)
mismatches = 0
for g in groupby(rows, :seed)
    certs = g[g.status .== "certified", :]
    nrow(certs) < 2 && continue
    objs = [num(val(r, :objective_value)) for r in eachrow(certs)]
    spread = maximum(objs) - minimum(objs)
    if spread > 1e-4
        mismatches += 1
        @printf("  MISMATCH seed %d spread %.6f across %s\n",
                first(g.seed), spread, join(String.(certs.arm), ", "))
    end
end
if mismatches == 0
    println("  OK -- every seed certified by >1 arm agrees to 1e-4")
else
    println("\n  STOP. A disagreement is a FALSE CERTIFICATE (a cut applied in the wrong")
    println("  index space), not a parameter effect. Do not read the sections below as")
    println("  results until this is resolved.")
end

# ── 2. reach ────────────────────────────────────────────────────────────────────
println("\n" * "="^78)
println("2. REACH: baseline to beat is 4/10 on {44, 46, 48, 49} at 21600 s")
println("="^78)
const BASELINE_SET = Set([44, 46, 48, 49])
@printf("\n%-18s %7s %7s  %-28s %s\n", "arm", "done", "cert", "certified seeds", "vs baseline")
for g in groupby(rows, :arm)
    arm = String(first(g.arm))
    cert = sort(Int.(g[g.status .== "certified", :seed]))
    cset = Set(cert)
    gained = sort(collect(setdiff(cset, BASELINE_SET)))
    lost = sort(collect(setdiff(BASELINE_SET, cset)))
    verdict = if cset == BASELINE_SET
        "same set"
    else
        strs = String[]
        isempty(gained) || push!(strs, "GAINED " * join(gained, ","))
        isempty(lost) || push!(strs, "lost " * join(lost, ","))
        join(strs, " / ")
    end
    @printf("%-18s %7d %7d  %-28s %s\n",
            arm, nrow(g), length(cert), join(cert, ","), verdict)
end

# Median wall on the seeds an arm certified -- the speed story, kept separate from reach.
println("\nwall on certified seeds (median, and per-seed):")
for g in groupby(rows, :arm)
    certs = g[g.status .== "certified", :]
    nrow(certs) == 0 && continue
    walls = [num(val(r, :wall_sec)) for r in eachrow(certs)]
    @printf("  %-18s median %8.0fs   %s\n", String(first(g.arm)), median(walls),
            join([@sprintf("s%d=%.0fs", r.seed, num(val(r, :wall_sec)))
                  for r in eachrow(certs)], "  "))
end

# ── 3. why ──────────────────────────────────────────────────────────────────────
println("\n" * "="^78)
println("3. WHY: was the SCHEDULE binding, or the instance hard?")
println("="^78)
println("  escalations = sweeps a scheduled slice truncated that the real remaining budget")
println("  finished. Each one used to be an attempt thrown away as `:inconclusive`.")
@printf("\n%-18s %5s %6s %7s %7s %8s %8s %7s %s\n", "arm", "seed", "status",
        "mesoEsc", "statEsc", "algSkip", "algDown", "guides", "exit reasons")
for r in eachrow(sort(rows, [:arm, :seed]))
    startswith(String(r.arm), "twotier") || continue
    @printf("%-18s %5d %6s %7s %7s %8s %8s %7s %s\n",
            String(r.arm), r.seed, first(String(r.status), 5),
            string(val(r, :two_tier_meso_escalations, "-")),
            string(val(r, :two_tier_station_escalations, "-")),
            string(val(r, :two_tier_align_skipped, "-")),
            string(val(r, :two_tier_align_downgraded, "-")),
            (v = num(val(r, :two_tier_guides_used_mean));
             ismissing(v) ? "-" : @sprintf("%.2f", v)),
            string(val(r, :two_tier_exit_reasons, "")))
end

# ── per-round cost, from the long-format dump ───────────────────────────────────
rdir = joinpath(root, "rounds")
if isdir(rdir)
    rfiles = filter(f -> startswith(f, "n40_seed") && endswith(f, ".csv"), readdir(rdir))
    if !isempty(rfiles)
        rr = reduce((a, b) -> vcat(a, b; cols=:union),
            (CSV.read(joinpath(rdir, f), DataFrame) for f in rfiles))
        println("\n" * "="^78)
        println("PER-ROUND COST BY CG ITERATION (rounds/*.csv)")
        println("="^78)
        println("  `sat` = share of rounds that used >=95% of their granted slice, i.e. were")
        println("  cut off by the schedule rather than finishing inside it. A high `sat` in")
        println("  late iterations is the starvation this run set out to fix.")
        rr = rr[.!ismissing.(rr.sec) .& .!ismissing.(rr.slice), :]
        rr.bucket = map(it -> it <= 5 ? "it 1-5" : it <= 15 ? "it 6-15" :
                              it <= 30 ? "it 16-30" : "it 31+", rr.iteration)
        @printf("\n%-10s %-8s %6s %9s %9s %9s %6s\n",
                "bucket", "tier", "n", "med sec", "p90 sec", "med slice", "sat")
        for b in ("it 1-5", "it 6-15", "it 16-30", "it 31+")
            for t in ("macro", "meso", "station")
                sub = rr[(rr.bucket .== b) .& (rr.tier .== t), :]
                nrow(sub) == 0 && continue
                secs = Float64.(sub.sec)
                sat = count(i -> Float64(sub.sec[i]) >= 0.95 * Float64(sub.slice[i]),
                            1:nrow(sub)) / nrow(sub)
                @printf("%-10s %-8s %6d %9.2f %9.2f %9.2f %5.0f%%\n",
                        b, t, nrow(sub), median(secs), quantile(secs, 0.9),
                        median(Float64.(sub.slice)), 100 * sat)
            end
        end
    end
end

println()
