"""
    scripts/analyze_pfa_scaling_grid.jl

Aggregate the (n_stations x n_pairs) scaling grid produced by
`scripts/sbatch_pfa_scaling_grid.sh` and fit how solve time scales.

Each array cell writes its own `combined_results.csv`; this collects them into
one table, prints wall-time and label-count grids, and fits a power law
`t ~ C * n^a` in stations (at fixed p) and `t ~ C * p^b` in pairs (at fixed n).

**Only cells that reached `optimality_proven` are used in the fits.** A cell that
hit the 3h budget reports the time it was cut off at, not the time it needed, so
including it would bias every exponent downward. Truncated cells are still
listed, marked, and counted -- if many cells truncate, the exponents describe
only the easy corner of the grid and should be read that way.

Usage:
    julia --project=. scripts/analyze_pfa_scaling_grid.jl [study_dir]
"""

using CSV, DataFrames, Printf, Statistics

const DEFAULT_STUDY = normpath(joinpath(
    @__DIR__, "..", "experiments", "2026-07-30_pfa_scaling_grid",
))

function load_grid(study::AbstractString)
    rows = DataFrame[]
    for entry in sort(readdir(study))
        f = joinpath(study, entry, "combined_results.csv")
        isfile(f) || continue
        push!(rows, CSV.read(f, DataFrame))
    end
    isempty(rows) && error("no combined_results.csv found under $study")
    return reduce(vcat, rows; cols=:union)
end

_num(x) = ismissing(x) || x === nothing ? missing :
    (x isa Number ? Float64(x) : something(tryparse(Float64, string(x)), missing))

"""Print one metric as an n (rows) x p (columns) grid."""
function print_grid(df::DataFrame, col::Symbol, title::AbstractString; fmt="%10.1f")
    ns = sort(unique(df.n_stations))
    ps = sort(unique(df.n_pairs))
    println("\n## $title")
    @printf("%6s", "n\\p")
    for p in ps
        @printf("%11d", p)
    end
    println()
    for n in ns
        @printf("%6d", n)
        for p in ps
            sub = df[(df.n_stations .== n) .& (df.n_pairs .== p), :]
            if nrow(sub) == 0
                @printf("%11s", "-")
            else
                v = _num(sub[1, col])
                proven = String(string(sub[1, :cg_stop_reason])) == "optimality_proven"
                if ismissing(v)
                    @printf("%11s", "n/a")
                else
                    s = Printf.format(Printf.Format(fmt), v)
                    # `*` marks a cell that hit its budget: the number is a floor.
                    @printf("%11s", proven ? s : s * "*")
                end
            end
        end
        println()
    end
end

"""Least-squares slope of log(y) on log(x): the exponent of `y ~ C x^a`."""
function power_fit(xs::Vector{Float64}, ys::Vector{Float64})
    length(xs) >= 3 || return (missing, missing)
    lx, ly = log.(xs), log.(ys)
    mx, my = mean(lx), mean(ly)
    denom = sum((lx .- mx) .^ 2)
    denom <= 0 && return (missing, missing)
    a = sum((lx .- mx) .* (ly .- my)) / denom
    resid = ly .- (my .+ a .* (lx .- mx))
    r2 = 1 - sum(resid .^ 2) / max(sum((ly .- my) .^ 2), eps())
    return (a, r2)
end

function fit_axis(df::DataFrame, along::Symbol, fixed::Symbol, metric::Symbol)
    println("\n## $(metric) scaling in $(along) (proven cells only)")
    for fv in sort(unique(df[!, fixed]))
        sub = df[(df[!, fixed] .== fv) .&
                 (string.(df.cg_stop_reason) .== "optimality_proven"), :]
        xs = Float64[]; ys = Float64[]
        for r in eachrow(sub)
            x, y = _num(r[along]), _num(r[metric])
            (ismissing(x) || ismissing(y) || y <= 0) && continue
            push!(xs, x); push!(ys, y)
        end
        a, r2 = power_fit(xs, ys)
        if ismissing(a)
            @printf("  %s=%-4d  (only %d proven cell(s) -- no fit)\n", fixed, fv, length(xs))
        else
            @printf("  %s=%-4d  exponent=%.2f  R^2=%.3f  (%d proven cells)\n",
                    fixed, fv, a, r2, length(xs))
        end
    end
end

function main()
    study = isempty(ARGS) ? DEFAULT_STUDY : abspath(ARGS[1])
    df = load_grid(study)
    sort!(df, [:n_stations, :n_pairs])

    n_cells = nrow(df)
    n_proven = count(==("optimality_proven"), string.(df.cg_stop_reason))
    n_err = count(!=("ok"), string.(df.status))
    @printf("study: %s\n%d cells, %d proven optimal, %d errored\n",
            study, n_cells, n_proven, n_err)
    println("(* marks a cell that hit its budget -- its time is a lower bound)")

    print_grid(df, :wall_time_sec, "wall time (s)")
    print_grid(df, :total_pricing_seconds, "pricing time (s)")
    print_grid(df, :total_labels_generated, "labels generated"; fmt="%10.0f")
    print_grid(df, :n_columns, "columns generated"; fmt="%10.0f")
    print_grid(df, :n_cg_iters, "CG iterations"; fmt="%10.0f")
    print_grid(df, :lp_mip_gap_pct, "LP-MIP gap (%)"; fmt="%10.3f")

    fit_axis(df, :n_stations, :n_pairs, :wall_time_sec)
    fit_axis(df, :n_pairs, :n_stations, :wall_time_sec)

    println("\n## non-proven cells")
    bad = df[string.(df.cg_stop_reason) .!= "optimality_proven", :]
    if nrow(bad) == 0
        println("  none -- every cell certified optimality")
    else
        for r in eachrow(bad)
            @printf("  n=%-3d p=%-3d status=%s stop=%s wall=%.0fs\n",
                    r.n_stations, r.n_pairs, r.status, r.cg_stop_reason,
                    something(_num(r.wall_time_sec), NaN))
        end
    end

    out = joinpath(study, "grid_combined.csv")
    CSV.write(out, df)
    println("\nWrote $out")
end

main()
