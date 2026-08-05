using CSV
using DataFrames
using Statistics

const ROOT = normpath(joinpath(@__DIR__, ".."))
const BB_DIR = joinpath(ROOT, "experiments", "zhuzhou_branch_benders_yz_mw_no_mcf_ms5", "results")
const B_DIR = joinpath(ROOT, "experiments", "zhuzhou_benders_cut_scaling_ms5", "results")

function read_matching(dir, suffix)
    frames = DataFrame[]
    for path in readdir(dir; join=true)
        endswith(path, suffix) || continue
        push!(frames, CSV.read(path, DataFrame))
    end
    isempty(frames) ? DataFrame() : vcat(frames...; cols=:union)
end

bb = read_matching(BB_DIR, "__branch_bendersYZ_mw_no_mcf_ms5.csv")
b = read_matching(B_DIR, "__bendersYZ_mw_ms5.csv")
direct = read_matching(B_DIR, "__direct_ms5.csv")
b_group_source = copy(b)
select!(b, :instance, :status => :b_status, :termination_status => :b_term,
        :objective_value => :b_obj, :wall_time_sec => :b_wall,
        :n_iterations => :b_iterations, :optimality_cuts_added => :b_cuts,
        :final_outer_gap => :b_gap)
select!(bb, :instance, :n_stations, :n_pairs, :seed, :n_scenarios,
        :status => :bb_status, :termination_status => :bb_term,
        :objective_value => :bb_obj, :wall_time_sec => :bb_wall,
        :callback_count => :bb_callbacks, :unique_exact_evaluations => :bb_unique_y,
        :cuts_submitted => :bb_cuts, :relative_gap => :bb_gap)
x = innerjoin(bb, b; on=:instance)
x.obj_abs_diff = abs.(x.bb_obj .- x.b_obj)
x.obj_rel_diff = x.obj_abs_diff ./ max.(abs.(x.b_obj), 1.0)
x.speedup = x.b_wall ./ x.bb_wall
x.bb_faster = x.bb_wall .< x.b_wall

println("MATCHED_ROWS=", nrow(x))
println("MAX_OBJ_ABS_DIFF=", maximum(x.obj_abs_diff))
println("MAX_OBJ_REL_DIFF=", maximum(x.obj_rel_diff))
println("OBJECTIVE_MATCH_1E6=", count(<=(1e-6), x.obj_abs_diff), "/", nrow(x))
println("BB_FASTER=", count(x.bb_faster), "/", nrow(x))
println("GEOMEAN_SPEEDUP=", exp(mean(log.(x.speedup))))
println("MEDIAN_SPEEDUP=", median(x.speedup))
matched_obj = x[x.obj_abs_diff .<= 1e-6, :]
println("MATCHED_OBJ_BB_FASTER=", count(matched_obj.bb_faster), "/", nrow(matched_obj))
println("MATCHED_OBJ_GEOMEAN_SPEEDUP=", exp(mean(log.(matched_obj.speedup))))
println("MATCHED_OBJ_MEDIAN_SPEEDUP=", median(matched_obj.speedup))
println("B_GAP_LE_1E6=", count(<=(1e-6), x.b_gap), "/", nrow(x))
println("BB_GAP_LE_1E6=", count(<=(1e-6), x.bb_gap), "/", nrow(x))

g = combine(groupby(x, [:n_stations, :n_pairs, :n_scenarios]),
    nrow => :matched,
    :b_wall => mean => :b_wall_mean,
    :bb_wall => mean => :bb_wall_mean,
    :speedup => (v -> exp(mean(log.(v)))) => :speedup_geomean,
    :b_cuts => mean => :b_cuts_mean,
    :bb_cuts => mean => :bb_cuts_mean,
    :b_iterations => mean => :b_iterations_mean,
    :bb_callbacks => mean => :bb_callbacks_mean,
    :obj_abs_diff => maximum => :max_obj_diff)
sort!(g, [:n_stations, :n_pairs, :n_scenarios])
show(stdout, MIME("text/csv"), g); println()

println("DETAIL")
detail = select(x, :instance, :b_obj, :bb_obj, :obj_abs_diff, :b_wall, :bb_wall,
                :speedup, :b_cuts, :bb_cuts, :b_iterations, :bb_callbacks)
sort!(detail, :instance)
show(stdout, MIME("text/csv"), detail); println()

println("MISMATCH_WITH_DIRECT")
d = select(direct, :instance, :status => :direct_status,
           :termination_status => :direct_term, :objective_value => :direct_obj,
           :wall_time_sec => :direct_wall)
xd = leftjoin(x[x.obj_abs_diff .> 1e-6, :], d; on=:instance)
xd.b_vs_direct = abs.(xd.b_obj .- xd.direct_obj)
xd.bb_vs_direct = abs.(xd.bb_obj .- xd.direct_obj)
show(stdout, MIME("text/csv"), select(xd, :instance, :b_obj, :bb_obj, :direct_obj,
    :b_vs_direct, :bb_vs_direct, :b_gap, :bb_gap, :b_wall, :bb_wall, :direct_wall)); println()

println("BENDERS_ROWS_BY_GROUP")
bg = combine(groupby(b_group_source, [:n_stations, :n_pairs, :n_scenarios]), nrow => :rows,
             :wall_time_sec => mean => :wall_mean,
             :status => (v -> join(unique(v), ";")) => :statuses)
sort!(bg, [:n_stations, :n_pairs, :n_scenarios])
show(stdout, MIME("text/csv"), bg); println()
