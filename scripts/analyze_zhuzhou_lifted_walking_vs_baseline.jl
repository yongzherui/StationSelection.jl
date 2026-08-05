using CSV, DataFrames, Statistics

const ROOT = normpath(joinpath(@__DIR__, ".."))
const BASE = joinpath(ROOT, "experiments", "zhuzhou_benders_cut_scaling_ms5", "results")
const LIFT = joinpath(ROOT, "experiments", "zhuzhou_classical_benders_lifted_walking_ms5", "results")

function read_suffix(dir, suffix)
    frames = [CSV.read(joinpath(dir, f), DataFrame) for f in readdir(dir) if endswith(f, suffix)]
    return vcat(frames...; cols=:union)
end

b = read_suffix(BASE, "__bendersYZ_mw_ms5.csv")
l = read_suffix(LIFT, "__bendersYZ_mw_lifted_walking_ms5.csv")
b = b[in.(b.n_stations, Ref([10, 15])), :]

select!(b, :instance, :n_stations, :n_pairs, :seed, :n_scenarios,
        :status => :base_status, :objective_value => :base_obj,
        :wall_time_sec => :base_wall, :n_iterations => :base_iters,
        :optimality_cuts_added => :base_cuts, :final_outer_gap => :base_gap)
select!(l, :instance, :status => :lift_status, :objective_value => :lift_obj,
        :wall_time_sec => :lift_wall, :n_iterations => :lift_iters,
        :optimality_cuts_added => :lift_cuts, :final_outer_gap => :lift_gap)
x = innerjoin(b, l; on=:instance)
x.obj_diff = abs.(x.lift_obj .- x.base_obj)
x.speedup = x.base_wall ./ x.lift_wall
x.iter_ratio = x.base_iters ./ x.lift_iters
x.cut_ratio = x.base_cuts ./ x.lift_cuts

println("matched=$(nrow(x)) objective_matches=$(count(<=(1e-6), x.obj_diff)) max_obj_diff=$(maximum(x.obj_diff))")
println("lift_faster=$(count(>(1), x.speedup))/$(nrow(x)) geomean_speedup=$(exp(mean(log.(x.speedup))))")
println("base_gap_closed=$(count(<=(1e-6), x.base_gap))/$(nrow(x)) lift_gap_closed=$(count(<=(1e-6), x.lift_gap))/$(nrow(x))")

g = combine(groupby(x, [:n_stations, :n_pairs, :n_scenarios]),
    nrow => :runs,
    :base_wall => mean => :base_wall_mean,
    :lift_wall => mean => :lift_wall_mean,
    :speedup => (v -> exp(mean(log.(v)))) => :speedup_geomean,
    :base_iters => mean => :base_iters_mean,
    :lift_iters => mean => :lift_iters_mean,
    :base_cuts => mean => :base_cuts_mean,
    :lift_cuts => mean => :lift_cuts_mean,
    :base_gap => maximum => :base_gap_max,
    :lift_gap => maximum => :lift_gap_max,
    :obj_diff => maximum => :max_obj_diff)
sort!(g, [:n_stations, :n_pairs, :n_scenarios])
show(stdout, MIME("text/csv"), g); println()
