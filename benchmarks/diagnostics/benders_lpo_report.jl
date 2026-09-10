"""Aggregate the per-arm TSV rows `benders_lpo_arm.jl` writes, and apply the cross-arm gates.

Each arm runs as its own job so it cannot be checked against the others in-process. This
reads whatever rows exist and does the three things a single arm cannot:

1. **Correctness gate.** Every oracle solves the IDENTICAL model, so any objective
   disagreement among converged arms means a wrong cut -- and for the activated family
   specifically it means the dual completion is unsound. This must pass before any
   cut-count claim means anything.
2. **Paired comparison** per instance, since all arms ran the same instance.
3. **The two verdicts**: does the Pareto completion fix the activated cut blowup, and does a
   built-only phase 1 cheapen full-universe certification?

Missing arms are reported as missing rather than silently skipped -- a comparison drawn over
whichever jobs happened to finish is how a direction gets claimed from noise.

Usage: julia --project=. benchmarks/diagnostics/benders_lpo_report.jl [results_dir]
"""

using Printf

const DIR = length(ARGS) >= 1 ? ARGS[1] :
    joinpath(@__DIR__, "results", "benders_lpo")
const TOL = 1e-6

isdir(DIR) || error("no results directory at $DIR")

rows = Dict{String, String}[]
for f in sort(readdir(DIR; join=true))
    endswith(f, ".tsv") || continue
    lines = readlines(f)
    length(lines) >= 2 || continue
    header = split(lines[1], '\t')
    vals = split(lines[2], '\t')
    length(header) == length(vals) || continue
    push!(rows, Dict(String(h) => String(v) for (h, v) in zip(header, vals)))
end
isempty(rows) && error("no arm rows found in $DIR")

num(r, key, default=NaN) = something(tryparse(Float64, get(r, key, "")), default)
int(r, key, default=-1) = something(tryparse(Int, get(r, key, "")), default)
inst(r) = "n=$(r["n"]) s=$(r["s"]) p=$(r["p"]) seed=$(r["seed"]) ms=$(r["max_stops"])"

const ARMS = ["direct_enumeration", "column_generation", "column_generation_activated",
              "column_generation_activated_lpo",
              "column_generation_activated_warm_start"]
short = Dict("direct_enumeration" => "enumeration",
             "column_generation" => "plain CG",
             "column_generation_activated" => "activated",
             "column_generation_activated_lpo" => "activated_lpo",
             "column_generation_activated_warm_start" => "warm_start")

instances = unique(inst.(rows))
checks = Tuple{String, Bool, String}[]

for key in sort(instances)
    here = [r for r in rows if inst(r) == key]
    byarm = Dict(r["oracle"] => r for r in here)
    @printf("\n%s\n=== %s ===\n", repeat("=", 100), key)
    @printf("%-30s %9s %5s %5s %8s %8s %8s %8s %10s %7s\n",
            "oracle", "status", "it", "cuts", "price", "restr", "full", "sep",
            "objective", "pool")
    for a in ARMS
        haskey(byarm, a) || continue
        r = byarm[a]
        @printf("%-30s %9s %5d %5d %8.1f %8.1f %8.1f %8.1f %10.2f %7d\n",
                short[a], r["status"], int(r, "iters"), int(r, "cuts"),
                num(r, "price_total", 0.0), num(r, "price_restricted", 0.0),
                num(r, "price_full", 0.0), num(r, "price_separation", 0.0),
                num(r, "objective"), int(r, "pool", 0))
    end
    missing_arms = [a for a in ARMS if !haskey(byarm, a) &&
                    a != "column_generation_activated"]
    isempty(missing_arms) ||
        @printf("  MISSING: %s\n", join([short[a] for a in missing_arms], ", "))

    # --- gate 1: every converged arm must agree on the objective ---
    ok = [r for r in here if r["status"] == "OPTIMAL"]
    if length(ok) >= 2
        ref = ok[1]
        for r in ok[2:end]
            d = num(r, "objective") - num(ref, "objective")
            push!(checks, ("$key: $(short[r["oracle"]]) obj == $(short[ref["oracle"]])",
                abs(d) <= max(TOL, TOL * abs(num(ref, "objective"))),
                @sprintf("%.6f vs %.6f (diff %.3e)", num(r, "objective"),
                         num(ref, "objective"), d)))
        end
    elseif length(ok) == 1
        push!(checks, ("$key: at least two arms converged", false,
                       "only $(short[ok[1]["oracle"]]) did"))
    else
        push!(checks, ("$key: any arm converged", false, "none"))
    end

    # --- verdict A: does the Pareto completion fix the activated blowup? ---
    if haskey(byarm, "column_generation_activated") &&
       haskey(byarm, "column_generation_activated_lpo")
        a, l = byarm["column_generation_activated"], byarm["column_generation_activated_lpo"]
        @printf("  LPO vs activated: cuts %d -> %d (%.2fx), iters %d -> %d, wall %.1fs -> %.1fs\n",
                int(a, "cuts"), int(l, "cuts"),
                int(a, "cuts") == 0 ? NaN : int(l, "cuts") / int(a, "cuts"),
                int(a, "iters"), int(l, "iters"), num(a, "wall"), num(l, "wall"))
    end

    # --- verdict B: the warm start, against plain CG ---
    # Phase 1 already attains the EXACT Q_s(yhat) -- a column touching an unbuilt station is
    # pinned to theta=0 by its own `theta - y_j <= 0` row -- so all of plain CG's
    # full-station pricing buys DUALS, not value. Does warming the pool cheapen it?
    if haskey(byarm, "column_generation") &&
       haskey(byarm, "column_generation_activated_warm_start")
        c, w = byarm["column_generation"], byarm["column_generation_activated_warm_start"]
        pc, pw = num(c, "price_total"), num(w, "price_total")
        @printf("  warm_start vs plain CG: pricing %.1fs -> %.1fs (%.2fx), cuts %+d, iters %+d\n",
                pc, pw, pc <= 0 ? NaN : pw / pc,
                int(w, "cuts") - int(c, "cuts"), int(w, "iters") - int(c, "iters"))
        # The decisive split: plain CG's pricing is ALL full-universe. If the warm start's
        # own full-universe time is much smaller, the warm pool genuinely shortened the
        # expensive search; if it is comparable and only the restricted phase was added, the
        # win is elsewhere (a smaller pool, fewer master iterations) and the search itself
        # was never the lever.
        fc, fw = num(c, "price_full", 0.0), num(w, "price_full", 0.0)
        @printf("    full-universe search only: %.1fs -> %.1fs (%.2fx) | warm start also spent %.1fs restricted\n",
                fc, fw, fc <= 0 ? NaN : fw / fc, num(w, "price_restricted", 0.0))
    end
end

println("\n", repeat("=", 100))
println("=== checks ===")
for (name, good, detail) in checks
    @printf("%-64s %s   %s\n", name, good ? "PASS" : "FAIL", detail)
end
n_fail = count(c -> !c[2], checks)
@printf("\n%d checks, %d failed\n", length(checks), n_fail)

# Cross-instance summary, only over instances where BOTH arms of a pair converged -- a
# ratio pooled over partial data is how a direction gets claimed from noise.
for (label, arm_a, arm_b) in
        (("warm_start vs plain CG", "column_generation",
          "column_generation_activated_warm_start"),
         ("activated_lpo vs plain CG", "column_generation",
          "column_generation_activated_lpo"))
    pairs = Tuple{String, Float64, Float64, Int, Int}[]
    for key in sort(instances)
        here = Dict(r["oracle"] => r for r in rows if inst(r) == key)
        (haskey(here, arm_a) && haskey(here, arm_b)) || continue
        (here[arm_a]["status"] == "OPTIMAL" && here[arm_b]["status"] == "OPTIMAL") || continue
        push!(pairs, (key, num(here[arm_a], "price_total"), num(here[arm_b], "price_total"),
                       int(here[arm_a], "cuts"), int(here[arm_b], "cuts")))
    end
    isempty(pairs) && continue
    @printf("\nPAIRED: %s (%d instance(s), both converged)\n", label, length(pairs))
    for (key, pa, pb, ca, cb) in pairs
        @printf("  %-40s pricing %8.1fs -> %8.1fs (%.2fx) | cuts %3d -> %3d (%+d)\n",
                key, pa, pb, pa <= 0 ? NaN : pb / pa, ca, cb, cb - ca)
    end
end

n_fail == 0 || error("verification failed")
println("\nALL CHECKS PASSED")
