"""Anytime comparison: given a fixed wall budget, which pricer has the better bound?

This asks a different question from `check_gate.jl`/`compare_two_tier.jl`, which score a run
by whether it CERTIFIED. At n=40 almost nothing certifies (study 9: `relaxed_k60` 3/6,
`relaxed_k80` 0/7), so certification-rate scoring cannot rank the arms there at all. What
still separates them is how far each pushes the bounds inside a budget, which is what the
per-iteration log records.

# The two bounds are not the same kind of object

`master_objective` is the restricted master LP over a partial column pool, so it is an UPPER
bound on `z_LP` that descends as columns arrive. It measures pool quality and nothing else --
it says how good a solution the pricer has found, never how far from optimal that is.

`relaxed_rc_bound` is a valid LOWER bound on the minimum reduced cost over the whole real
route universe (see `RelaxedClusterCertificationResult.relaxed_rc_bound`). It is the only
quantity here that bounds from below, and it is the ingredient a genuine `z_LP` lower bound
would be built from -- that construction additionally needs a bound on the number of columns
in an optimal LP solution, which this master does not currently carry, so it is NOT reported
as an `z_LP` bound here. It is `NaN` for `exact` (which never attempts one) and for any
iteration whose attempt came back inconclusive.

It is also NOT monotone across iterations: each iteration's bound is a statement about THAT
iteration's duals, so a later, smaller value is not a regression. Compare arms at the same
point in the solve, never a max over the window.

# Why the window is truncated rather than the budget being set to it

Every arm runs a longer `total_limit_sec` than the window analysed here, because CG's own
budget clamp (`certification_limit = min(pricing_time_limit_sec, remaining_budget())`)
squeezes the last rounds of a run as its total budget runs out. A run whose total IS the
window would therefore be handicapped exactly where we want to read it. Running long and
truncating in analysis keeps every in-window round at its full pricing slice.

An output directory holds every size a run touched, and pooling them is meaningless -- an
n=20 run and an n=50 run are different problems, and a median over both tracks the mix of
sizes present rather than anything about the pricers. So `n` is filtered, not grouped, and
it defaults to nothing being filtered ONLY when the directory holds a single size.

Usage: julia --project=../.. analyze_anytime.jl <output-dir> [n] [window_sec]
"""

using CSV
using DataFrames
using Printf
using Statistics

length(ARGS) in (1, 2, 3) ||
    error("usage: analyze_anytime.jl <output-dir> [n] [window_sec]")
root = abspath(ARGS[1])
isdir(root) || error("missing output directory $root")
want_n = length(ARGS) >= 2 ? parse(Int, ARGS[2]) : nothing
window = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : 1200.0

iter_dir = joinpath(root, "iterations")
isdir(iter_dir) || error("missing $iter_dir -- no per-iteration logs to read")

# Wall time actually spent inside the CG loop, which is what the budget clamps. Read
# defensively: `certification_sec` is absent from logs written before the mode existed.
col(df, name) = hasproperty(df, name) ? coalesce.(getproperty(df, name), 0.0) : zeros(nrow(df))
iteration_sec(df) = col(df, :master_sec) .+ col(df, :pricing_sec) .+
                    col(df, :add_columns_sec) .+ col(df, :certification_sec)

col_outcome(df) = hasproperty(df, :certification_outcome) ?
    getproperty(df, :certification_outcome) : fill("none", nrow(df))


snapshots = NamedTuple[]
for f in readdir(iter_dir)
    m = match(r"^n(\d+)_seed(\d+)_([a-z_0-9]+)\.csv$", f)
    isnothing(m) && continue
    df = CSV.read(joinpath(iter_dir, f), DataFrame)
    nrow(df) == 0 && continue
    elapsed = cumsum(iteration_sec(df))
    inside = findall(<=(window), elapsed)
    isempty(inside) && continue          # not even one iteration finished in the window
    last_i = inside[end]
    row = df[last_i, :]
    # The bound as of the last iteration that FINISHED inside the window.
    rc = hasproperty(df, :relaxed_rc_bound) ? row.relaxed_rc_bound : NaN
    push!(snapshots, (
        n=parse(Int, m[1]), seed=parse(Int, m[2]), arm=String(m[3]),
        iterations=last_i, elapsed=elapsed[last_i],
        objective=ismissing(row.master_objective) ? NaN : Float64(row.master_objective),
        columns=Int(coalesce(row.cumulative_columns_added, 0)),
        rc_bound=ismissing(rc) ? NaN : Float64(rc),
        # How much of the window this arm spent with a bound it could not use.
        inconclusive=count(==("inconclusive"),
            string.(coalesce.(col_outcome(df), "none"))[inside]),
    ))
end
isempty(snapshots) && error("no iteration logs with a completed iteration inside $(window)s")

snap = DataFrame(snapshots)
sizes = sort(unique(snap.n))
if isnothing(want_n)
    # Refuse rather than silently average across sizes -- the failure this guards against is
    # a plausible-looking table that is really a report on which sizes happened to finish.
    length(sizes) == 1 || error(
        "directory holds sizes $(sizes); pass one as the 2nd argument -- pooling them " *
        "would report the mix of sizes present, not the pricers")
else
    want_n in sizes || error("no runs at n=$want_n; directory holds $(sizes)")
    snap = snap[snap.n .== want_n, :]
end
@printf("\n=== ANYTIME SNAPSHOT n=%d at %.0fs -- %d runs\n\n",
        first(snap.n), window, nrow(snap))

@printf("%-14s %5s  %10s  %14s  %9s  %12s  %6s\n",
        "arm", "runs", "iters", "master obj", "columns", "rc bound", "inconc")
for g in groupby(sort(snap, :arm), :arm)
    finite = filter(isfinite, g.rc_bound)
    @printf("%-14s %5d  %10.1f  %14.1f  %9.0f  %12s  %6.1f\n",
            first(g.arm), nrow(g), median(g.iterations), median(g.objective),
            median(g.columns),
            isempty(finite) ? "--" : @sprintf("%.1f", median(finite)),
            median(g.inconclusive))
end

# ── seed-matched deltas ─────────────────────────────────────────────────────────
#
# Instance difficulty varies enough between seeds that an across-seed mean can invert a
# per-seed result, so every comparison is paired.
function paired(a::String, b::String)
    out = NamedTuple[]
    for g in groupby(snap, [:n, :seed])
        ra = g[g.arm .== a, :]; rb = g[g.arm .== b, :]
        (nrow(ra) == 1 && nrow(rb) == 1) || continue
        push!(out, (seed=first(g.seed), a=first(eachrow(ra)), b=first(eachrow(rb))))
    end
    return out
end

# Every two-tier arm against the single-tier arm sharing its K2, then each against the
# unrelaxed control. Derived rather than hardcoded: the K2 an arm carries is what makes a
# pair differ ONLY by the macro tier, and a hardcoded list silently drops a new arm.
present = Set(snap.arm)
comparisons = Tuple{String, String}[]
for arm in sort(collect(present))
    startswith(arm, "twotier") || continue
    base = replace(arm, "twotier_" => "relaxed_")
    base in present && push!(comparisons, (arm, base))
end
for arm in sort(collect(present))
    arm == "exact" || !("exact" in present) || push!(comparisons, (arm, "exact"))
end
for (a, b) in comparisons
    ps = paired(a, b)
    isempty(ps) && continue
    @printf("\n--- %s vs %s (%d seed-matched pairs)\n", a, b, length(ps))
    wins = count(p -> p.a.objective < p.b.objective - 1e-9, ps)
    @printf("    better master objective: %d/%d\n", wins, length(ps))
    @printf("    median objective delta : %+.1f  (negative = %s ahead)\n",
            median([p.a.objective - p.b.objective for p in ps]), a)
    @printf("    median iterations      : %.1f vs %.1f\n",
            median([p.a.iterations for p in ps]), median([p.b.iterations for p in ps]))
    @printf("    median columns         : %.0f vs %.0f\n",
            median([p.a.columns for p in ps]), median([p.b.columns for p in ps]))
    rc = [(p.a.rc_bound, p.b.rc_bound) for p in ps
          if isfinite(p.a.rc_bound) && isfinite(p.b.rc_bound)]
    if !isempty(rc)
        # Higher is tighter: the bound rises toward 0 as cuts carve the relaxed graph down.
        @printf("    median rc bound        : %.1f vs %.1f  (%d pairs, higher = tighter)\n",
                median(first.(rc)), median(last.(rc)), length(rc))
    end
end
println()
