"""Is `:relaxed_cluster_two_tier` SAFE and is it FASTER than the one-tier mode?

Safety is the first question and the only one that can invalidate the mode: a pricer is a
search algorithm, so both arms solve the identical model and a certified optimum from
either must be the same number. Any disagreement beyond solver tolerance on a pair where
the one-tier arm certified is a bug in the two-tier loop, not a tuning result -- the shape
it would take is a false certificate from a cut applied in the wrong index space.

Pairs are seed-matched within one size and one K2, so `twotier_k60` is compared against
`relaxed_k60` (same K2, same instance, same budgets) and never against `relaxed_k80`.

Usage: julia --project=../.. compare_two_tier.jl <output-dir> [n]
"""

using CSV
using DataFrames
using Printf
using Statistics

length(ARGS) in (1, 2) || error("usage: compare_two_tier.jl <output-dir> [n]")
root = abspath(ARGS[1])
isdir(root) || error("missing output directory $root")
want_n = length(ARGS) == 2 ? parse(Int, ARGS[2]) : nothing

files = filter(f -> occursin(r"^n\d+_seed\d+_[a-z_0-9]+\.csv$", f), readdir(root))
rows = reduce((a, b) -> vcat(a, b; cols=:union),
    (CSV.read(joinpath(root, f), DataFrame) for f in files); init=DataFrame())
isnothing(want_n) || (rows = rows[rows.n_stations .== want_n, :])
nrow(rows) > 0 || error("no rows found")

value(row, col) = hasproperty(row, col) ? getproperty(row, col) : missing
tier_of(arm) = startswith(String(arm), "twotier") ? :two : :one
# `twotier_k60m14` and `twotier_k60` both compare against `relaxed_k60`: the trailing
# `m<K1>` names the macro layer, which the single-tier reference does not have.
base_of(arm) = replace(replace(String(arm), r"m\d+$" => ""), "twotier_" => "relaxed_")

pairs = NamedTuple[]
for g in groupby(rows, [:n_stations, :seed])
    for tt in eachrow(g[tier_of.(g.arm) .== :two, :])
        base = g[String.(g.arm) .== base_of(tt.arm), :]
        nrow(base) == 1 || continue
        ot = first(eachrow(base))
        push!(pairs, (
            n=tt.n_stations, seed=tt.seed, arm=String(tt.arm),
            k2=tt.cluster_count, k1=value(tt, :macro_count),
            one_status=ot.status, two_status=tt.status,
            one_obj=ot.objective_value, two_obj=tt.objective_value,
            one_lp=value(ot, :lp_objective_value), two_lp=value(tt, :lp_objective_value),
            one_wall=ot.wall_sec, two_wall=tt.wall_sec,
            one_iters=ot.cg_iterations, two_iters=tt.cg_iterations,
            macro_cuts=value(tt, :two_tier_macro_cuts),
            meso_cuts=value(tt, :two_tier_meso_cuts),
            aligned=value(tt, :two_tier_aligned_rounds),
            subset=value(tt, :nogood_median_subset_size),
        ))
    end
end
isempty(pairs) && error("no seed-matched one-tier/two-tier pairs in $root")

# ── safety ──────────────────────────────────────────────────────────────────────
mismatches = NamedTuple[]
for p in pairs
    # Only a pair whose one-tier arm CERTIFIED carries a proven optimum to compare against.
    p.one_status == "certified" || continue
    scale = max(1.0, abs(p.one_obj))
    ismissing(p.two_obj) && (push!(mismatches, p); continue)
    abs(p.two_obj - p.one_obj) <= 1e-6 * scale || push!(mismatches, p)
end
@printf("\n=== SAFETY: %d seed-matched pairs, %d with a certified one-tier reference\n",
        length(pairs), count(p -> p.one_status == "certified", pairs))
if isempty(mismatches)
    println("PASS objectives agree to 1e-6 relative on every certified reference pair")
else
    println("FAIL objective mismatch on $(length(mismatches)) pair(s):")
    for p in mismatches
        @printf("  n=%d seed=%d %s: one-tier %.6f (%s) vs two-tier %s (%s)\n",
                p.n, p.seed, p.arm, p.one_obj, p.one_status, string(p.two_obj),
                p.two_status)
    end
end

# ── certification and speed ─────────────────────────────────────────────────────
println("\n=== PER PAIR")
@printf("%-4s %-5s %-12s %-4s %-4s %-11s %-11s %9s %9s %7s %6s %6s %6s\n",
        "n", "seed", "arm", "K2", "K1", "one_status", "two_status", "one_wall",
        "two_wall", "speedup", "mcuts", "scuts", "align")
for p in sort(pairs; by=q -> (q.n, q.arm, q.seed))
    sp = (ismissing(p.two_wall) || p.two_wall <= 0) ? NaN : p.one_wall / p.two_wall
    @printf("%-4d %-5d %-12s %-4d %-4s %-11s %-11s %9.1f %9.1f %7.1fx %6s %6s %6s\n",
            p.n, p.seed, p.arm, p.k2, string(p.k1), p.one_status, p.two_status,
            p.one_wall, p.two_wall, sp, string(p.macro_cuts), string(p.meso_cuts),
            string(p.aligned))
end

println("\n=== BY ARM")
for arm in sort(unique(p.arm for p in pairs))
    sel = [p for p in pairs if p.arm == arm]
    speed = [p.one_wall / p.two_wall for p in sel if !ismissing(p.two_wall) && p.two_wall > 0]
    @printf("%-12s pairs=%2d  one-tier certified %d/%d  two-tier certified %d/%d  " *
            "median speedup %.2fx  median subset %s\n",
            arm, length(sel),
            count(p -> p.one_status == "certified", sel), length(sel),
            count(p -> p.two_status == "certified", sel), length(sel),
            isempty(speed) ? NaN : median(speed),
            string(median(skipmissing([p.subset for p in sel]))))
end

# A macro cut is the mode's whole reason to exist: without one the loop is the one-tier
# loop with an extra sweep in front of it.
tt_rows = rows[tier_of.(rows.arm) .== :two, :]
if hasproperty(tt_rows, :two_tier_macro_cuts)
    mc = collect(skipmissing(tt_rows.two_tier_macro_cuts))
    al = collect(skipmissing(tt_rows.two_tier_aligned_rounds))
    @printf("\nmacro cuts per run: median %s, zero on %d of %d runs\n",
            isempty(mc) ? "n/a" : string(median(mc)), count(==(0), mc), length(mc))
    @printf("aligned station searches per run: median %s\n",
            isempty(al) ? "n/a" : string(median(al)))
end
println()
