using CSV
using DataFrames
using Statistics

const ROOT = normpath(joinpath(@__DIR__, ".."))

function read_suffix(dir, suffix)
    rows = DataFrame[]
    for path in readdir(dir; join=true)
        endswith(path, suffix) && push!(rows, CSV.read(path, DataFrame))
    end
    return vcat(rows...; cols=:union)
end

base_dir = joinpath(ROOT, "experiments", "zhuzhou_benders_cut_scaling_ms5", "results")
bb0_dir = joinpath(ROOT, "experiments", "zhuzhou_branch_benders_yz_mw_no_mcf_ms5", "results")
mcf_dir = joinpath(ROOT, "experiments", "zhuzhou_branch_benders_mcf_increment_ms5", "results")
classical_mcf_dir = joinpath(ROOT, "experiments", "zhuzhou_classical_benders_common_od_ms5", "results")

function classical_frame(dir, suffix, label)
    d = read_suffix(dir, suffix)
    d = d[d.n_stations .== 10, :]
    d.method .= label
    select!(d, :instance, :n_pairs, :seed, :n_scenarios, :method, :status,
            :termination_status, :objective_value, :wall_time_sec,
            :n_iterations => :iterations, :optimality_cuts_added => :exact_cuts,
            :final_outer_gap => :gap)
    d.callbacks = fill(missing, nrow(d)); d.unique_y = fill(missing, nrow(d))
    d.mcf_separations = fill(0, nrow(d)); d.mcf_cuts = fill(0, nrow(d));
    d.mcf_seconds = fill(0.0, nrow(d)); d.common_od_count = fill(0, nrow(d))
    return d
end

function branch_frame(dir, suffix, label)
    d = read_suffix(dir, suffix)
    d = d[d.n_stations .== 10, :]
    d.method .= label
    defaults = (
        projected_mcf_separations=0, projected_mcf_cuts=0,
        projected_mcf_seconds=0.0, mcf_common_od_count=0,
    )
    for (name, value) in pairs(defaults)
        name in propertynames(d) || (d[!, name] = fill(value, nrow(d)))
    end
    select!(d, :instance, :n_pairs, :seed, :n_scenarios, :method, :status,
            :termination_status, :objective_value, :wall_time_sec,
            :callback_count => :callbacks, :unique_exact_evaluations => :unique_y,
            :cuts_submitted => :exact_cuts, :relative_gap => :gap,
            :projected_mcf_separations => :mcf_separations,
            :projected_mcf_cuts => :mcf_cuts,
            :projected_mcf_seconds => :mcf_seconds,
            :mcf_common_od_count => :common_od_count)
    d.iterations = fill(missing, nrow(d))
    return d
end

b0 = classical_frame(base_dir, "__bendersYZ_mw_ms5.csv", "Classical baseline")
bc = classical_frame(classical_mcf_dir, "__bendersYZ_mw_common_od_ms5.csv", "Classical + common OD")
bb0 = branch_frame(bb0_dir, "__branch_bendersYZ_mw_no_mcf_ms5.csv", "B&B baseline")
bbc = branch_frame(mcf_dir, "__branch_bendersYZ_mw_common_od_ms5.csv", "B&B + common OD")
bbf = branch_frame(mcf_dir, "__branch_bendersYZ_mw_common_od_fractional_mcf_ms5.csv", "B&B + common OD + fractional MCF")

all = vcat(b0, bc, bb0, bbc, bbf; cols=:union)
all = all[all.n_pairs .∈ Ref([16, 32]), :]
base_obj = select(b0, :instance, :objective_value => :baseline_obj)
all = leftjoin(all, base_obj; on=:instance)
all.obj_abs_diff = abs.(all.objective_value .- all.baseline_obj)

summary = combine(groupby(all, [:method, :n_pairs, :n_scenarios]),
    nrow => :runs,
    :wall_time_sec => mean => :wall_mean,
    :wall_time_sec => median => :wall_median,
    :exact_cuts => mean => :exact_cuts_mean,
    :callbacks => (x -> isempty(skipmissing(x)) ? missing : mean(skipmissing(x))) => :callbacks_mean,
    :unique_y => (x -> isempty(skipmissing(x)) ? missing : mean(skipmissing(x))) => :unique_y_mean,
    :mcf_separations => mean => :mcf_separations_mean,
    :mcf_cuts => mean => :mcf_cuts_mean,
    :mcf_seconds => mean => :mcf_seconds_mean,
    :common_od_count => mean => :common_od_mean,
    :obj_abs_diff => maximum => :max_obj_diff)
sort!(summary, [:n_pairs, :n_scenarios, :method])
show(stdout, MIME("text/csv"), summary); println()

println("OVERALL")
overall = combine(groupby(all, :method), nrow => :runs,
    :wall_time_sec => mean => :wall_mean,
    :exact_cuts => mean => :exact_cuts_mean,
    :mcf_separations => mean => :mcf_separations_mean,
    :mcf_cuts => sum => :mcf_cuts_total,
    :mcf_seconds => sum => :mcf_seconds_total,
    :obj_abs_diff => maximum => :max_obj_diff)
sort!(overall, :method)
show(stdout, MIME("text/csv"), overall); println()

println("PAIRWISE_SPEEDUPS")
wide = unstack(select(all, :instance, :method, :wall_time_sec), :instance, :method, :wall_time_sec)
for (new, old) in [
    ("Classical + common OD", "Classical baseline"),
    ("B&B + common OD", "B&B baseline"),
    ("B&B + common OD + fractional MCF", "B&B + common OD"),
    ("B&B + common OD + fractional MCF", "B&B baseline"),
]
    ratio = wide[!, old] ./ wide[!, new]
    println("$old -> $new: geomean_speedup=$(exp(mean(log.(ratio)))) faster=$(count(>(1), ratio))/$(length(ratio))")
end
