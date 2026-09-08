using CSV
using DataFrames
using Statistics

length(ARGS) == 2 || error("usage: check_gate.jl <output-dir> <validation|n>")
root, gate = abspath(ARGS[1]), ARGS[2]
isdir(root) || error("missing output directory $root")
files = filter(f -> occursin(r"^n\d+_seed\d+_[a-z_]+\.csv$", f), readdir(root))
rows = reduce((a,b) -> vcat(a,b; cols=:union),
    (CSV.read(joinpath(root, f), DataFrame) for f in files); init=DataFrame())

if gate == "validation"
    expected = 2 * 3 * 10
    nrow(rows) == expected || error("validation incomplete: $(nrow(rows))/$expected rows")
    all(rows.status .== "certified") || error("validation has non-certified rows")
    for g in groupby(rows, [:n_stations, :seed])
        nrow(g) == 3 || error("missing arm for n=$(g.n_stations[1]) seed=$(g.seed[1])")
        maximum(g.objective_value) - minimum(g.objective_value) <=
            1e-6 * max(1.0, abs(first(g.objective_value))) || error("objective mismatch")
    end
    println("PASS validation: 60/60 certified, objectives agree")
    for g in groupby(rows, [:n_stations, :arm])
        println("n=$(first(g.n_stations)) arm=$(first(g.arm)) median_wall=$(round(median(g.wall_sec);digits=1))s median_iters=$(median(g.cg_iterations))")
    end
else
    n = parse(Int, gate)
    sub = rows[rows.n_stations .== n, :]
    nrow(sub) == 20 || error("frontier n=$n incomplete: $(nrow(sub))/20 rows")
    any(.!isempty.(coalesce.(sub.error_message, ""))) && error("frontier n=$n has errors")
    rates = Dict(first(g.arm) => count(==("certified"), g.status) / nrow(g)
                 for g in groupby(sub, :arm))
    best = first(sort!(collect(rates); by=last, rev=true))
    best_arm, best_rate = first(best), last(best)
    best_rate > 0.9 || error("frontier reached at n=$n: best arm $best_arm certified " *
        "$(round(100best_rate; digits=1))%, not more than 90%")
    println("PASS frontier n=$n: $best_arm certified $(round(100best_rate;digits=1))%; " *
        "median wall=$(round(median(sub[sub.arm .== best_arm, :].wall_sec);digits=1))s")
end
