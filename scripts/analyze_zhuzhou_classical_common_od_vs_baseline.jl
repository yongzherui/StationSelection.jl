using CSV, DataFrames, Statistics

const ROOT = normpath(joinpath(@__DIR__, ".."))
const BASE = joinpath(ROOT, "experiments", "zhuzhou_benders_cut_scaling_ms5", "results")
const MCF = joinpath(ROOT, "experiments", "zhuzhou_classical_benders_common_od_ms5", "results")

function read_suffix(dir, suffix)
    frames = [CSV.read(joinpath(dir, f), DataFrame) for f in readdir(dir) if endswith(f, suffix)]
    return vcat(frames...; cols=:union)
end

b = read_suffix(BASE, "__bendersYZ_mw_ms5.csv")
m = read_suffix(MCF, "__bendersYZ_mw_common_od_ms5.csv")

select!(b, :instance, :n_stations, :n_pairs, :seed, :n_scenarios,
        :objective_value => :base_obj, :wall_time_sec => :base_wall,
        :n_iterations => :base_iters, :optimality_cuts_added => :base_cuts,
        :final_outer_gap => :base_gap)
select!(m, :instance, :objective_value => :mcf_obj, :wall_time_sec => :mcf_wall,
        :n_iterations => :mcf_iters, :optimality_cuts_added => :mcf_cuts,
        :final_outer_gap => :mcf_gap)
x = innerjoin(b, m; on=:instance)
x.obj_diff = abs.(x.mcf_obj .- x.base_obj)
x.speedup = x.base_wall ./ x.mcf_wall

println("matched=$(nrow(x)) objective_matches=$(count(<=(1e-6), x.obj_diff))")
println("mcf_faster=$(count(>(1), x.speedup))/$(nrow(x)) geomean_speedup=$(exp(mean(log.(x.speedup))))")
println("base_gap_closed=$(count(<=(1e-6), x.base_gap))/$(nrow(x)) mcf_gap_closed=$(count(<=(1e-6), x.mcf_gap))/$(nrow(x))")

g = combine(groupby(x, [:n_stations, :n_pairs, :n_scenarios]),
    nrow => :runs,
    :base_wall => mean => :base_wall_mean,
    :mcf_wall => mean => :mcf_wall_mean,
    :speedup => (v -> exp(mean(log.(v)))) => :speedup_geomean,
    :base_iters => mean => :base_iters_mean,
    :mcf_iters => mean => :mcf_iters_mean,
    :base_cuts => mean => :base_cuts_mean,
    :mcf_cuts => mean => :mcf_cuts_mean,
    :base_gap => maximum => :base_gap_max,
    :mcf_gap => maximum => :mcf_gap_max,
    :obj_diff => maximum => :max_obj_diff)
sort!(g, [:n_stations, :n_pairs, :n_scenarios])
show(stdout, MIME("text/csv"), g); println()

println("NONMATCHES")
show(stdout, MIME("text/csv"), x[x.obj_diff .> 1e-6,
    [:instance, :base_obj, :mcf_obj, :base_gap, :mcf_gap, :base_wall, :mcf_wall]]); println()
