"""
    analyze_theta_rho_heuristic_compare.jl <output_dir> <label>=<run_dir> ...

Reconstruct the theta/rho restricted-pricing comparison entirely from durable
per-run CSV logs. The first labeled directory is the baseline. Writes
`case_comparison.csv` and `variant_summary.csv` to `output_dir`.
"""

using CSV, DataFrames, Printf, Statistics

length(ARGS) >= 3 || error(
    "usage: analyze_theta_rho_heuristic_compare.jl <output_dir> " *
    "<baseline_label>=<run_dir> <variant_label>=<run_dir> ...",
)

const OUTDIR = ARGS[1]
const SPECS = [let parts = split(arg, '='; limit=2)
    length(parts) == 2 || error("expected label=directory, got $arg")
    (label=parts[1], dir=parts[2])
end for arg in ARGS[2:end]]
const BASELINE = SPECS[1].label

parse_set(value, separator=';') =
    Set(parse.(Int, filter(x -> !isempty(x), split(String(value), separator))))

function milestone(remaining, threshold)
    index = findfirst(<=(threshold), remaining)
    return isnothing(index) ? length(remaining) + 1 : index
end

function analyze_case(label, dir, summary_path)
    summary = CSV.read(summary_path, DataFrame)[1, :]
    case = String(summary.case)
    prefix = joinpath(dir, case)
    y = CSV.read(prefix * "_ysupport.csv", DataFrame)
    theta = CSV.read(prefix * "_theta.csv", DataFrame)
    columns = CSV.read(prefix * "_columns.csv", DataFrame)

    routes = Dict(Int(row.column_id) => parse_set(row.route, '-') for row in eachrow(columns))
    theta_by_solve = Dict{Int, Vector{DataFrameRow}}()
    for row in eachrow(theta)
        push!(get!(() -> DataFrameRow[], theta_by_solve, Int(row.solve_sequence)), row)
    end
    support_by_solve = Dict(Int(row.solve_sequence) => parse_set(row.topl_indices)
                            for row in eachrow(y))

    objective = Float64.(y.lp_bound)
    total_improvement = max(objective[1] - objective[end], 1e-12)
    remaining = max.(0.0, (objective .- objective[end]) ./ total_improvement)
    early_improvement = 0.0
    certification_improvement = 0.0
    for i in 1:(nrow(y) - 1)
        improvement = max(0.0, objective[i] - objective[i + 1])
        if y.phase[i] == "certification"
            certification_improvement += improvement
        else
            early_improvement += improvement
        end
    end

    raw_shifts = 0
    meaningful_shifts = 0
    total_y_l1 = 0.0
    for row in eachrow(y)
        !ismissing(row.l1_move) && (total_y_l1 += Float64(row.l1_move))
        q = Int(row.solve_sequence)
        q > 1 && haskey(support_by_solve, q - 1) || continue
        entered = setdiff(support_by_solve[q], support_by_solve[q - 1])
        isempty(entered) && continue
        raw_shifts += 1
        active = filter(t -> Float64(t.theta_value) > 1e-7,
                        get(theta_by_solve, q, DataFrameRow[]))
        any(!isempty(intersect(routes[Int(t.column_id)], entered)) for t in active) &&
            (meaningful_shifts += 1)
    end

    final_q = maximum(keys(theta_by_solve))
    final_active = filter(t -> Float64(t.theta_value) > 1e-7, theta_by_solve[final_q])
    effective_support = isempty(final_active) ? Set{Int}() :
        union((routes[Int(t.column_id)] for t in final_active)...)
    theta_l1 = collect(skipmissing(CSV.read(prefix * "_theta_summary.csv", DataFrame).theta_l1_move))

    return (
        variant=label, case=case,
        n_scenarios=Int(summary.n_scenarios), seed=Int(summary.seed),
        theta_rho_core_size=Int(summary.theta_rho_core_size),
        theta_rho_outsiders=Int(summary.theta_rho_outsiders),
        status=String(summary.status), cg_stop_reason=String(summary.cg_stop_reason),
        lp_bound=Float64(summary.lp_bound),
        lp_bound_certified=Bool(summary.lp_bound_certified),
        mip_objective=ismissing(summary.mip_objective) ? missing : Float64(summary.mip_objective),
        total_seconds=Float64(summary.total_seconds),
        total_pricing_seconds=Float64(summary.total_pricing_seconds),
        total_lp_seconds=Float64(summary.total_lp_seconds),
        certification_seconds=Float64(summary.certification_seconds),
        total_labels_generated=Int(summary.total_labels_generated),
        n_cg_iters=Int(summary.n_cg_iters), n_rounds=Int(summary.n_rounds),
        n_columns=Int(summary.n_columns),
        early_routes_priced=Int(summary.early_routes_priced),
        certification_routes_priced=Int(summary.certification_routes_priced),
        solve_at_50pct=milestone(remaining, 0.5),
        solve_at_90pct=milestone(remaining, 0.1),
        solve_at_99pct=milestone(remaining, 0.01),
        solve_at_final=milestone(remaining, 1e-7),
        normalized_gap_auc=mean(remaining),
        early_objective_improvement=early_improvement,
        certification_objective_improvement=certification_improvement,
        certification_improvement_share=
            certification_improvement / max(early_improvement + certification_improvement, 1e-12),
        raw_y_shifts=raw_shifts, meaningful_y_shifts=meaningful_shifts,
        meaningful_y_shift_share=meaningful_shifts / max(raw_shifts, 1),
        total_y_l1_move=total_y_l1,
        total_theta_l1_move=sum(Float64.(theta_l1)),
        effective_support=join(sort!(collect(effective_support)), ";"),
        effective_support_size=length(effective_support),
    )
end

rows = NamedTuple[]
for spec in SPECS
    paths = sort!(filter(path -> endswith(path, "_run_summary.csv"),
                         readdir(spec.dir; join=true)))
    isempty(paths) && error("no *_run_summary.csv files found in $(spec.dir)")
    append!(rows, [analyze_case(spec.label, spec.dir, path) for path in paths])
end

baseline = Dict(row.case => row for row in rows if row.variant == BASELINE)
comparison_rows = [let base = baseline[row.case]
    base_support = parse_set(base.effective_support)
    support = parse_set(row.effective_support)
    union_size = length(union(base_support, support))
    (
        row...,
        mip_delta_vs_baseline=ismissing(row.mip_objective) || ismissing(base.mip_objective) ?
            missing : row.mip_objective - base.mip_objective,
        elapsed_ratio_vs_baseline=row.total_seconds / base.total_seconds,
        pricing_ratio_vs_baseline=row.total_pricing_seconds / max(base.total_pricing_seconds, 1e-12),
        label_ratio_vs_baseline=row.total_labels_generated / max(base.total_labels_generated, 1),
        effective_support_jaccard=union_size == 0 ? 1.0 :
            length(intersect(base_support, support)) / union_size,
    )
end for row in rows]

variant_rows = NamedTuple[]
for spec in SPECS
    selected = filter(row -> row.variant == spec.label, comparison_rows)
    push!(variant_rows, (
        variant=spec.label, n_cases=length(selected),
        all_lp_certified=all(row.lp_bound_certified for row in selected),
        max_abs_lp_delta=maximum(abs(row.lp_bound - baseline[row.case].lp_bound) for row in selected),
        max_mip_delta=maximum(skipmissing(
            (row.mip_delta_vs_baseline for row in selected)); init=0.0),
        mean_total_seconds=mean(row.total_seconds for row in selected),
        mean_pricing_seconds=mean(row.total_pricing_seconds for row in selected),
        mean_lp_seconds=mean(row.total_lp_seconds for row in selected),
        mean_certification_seconds=mean(row.certification_seconds for row in selected),
        mean_labels=mean(row.total_labels_generated for row in selected),
        mean_iterations=mean(row.n_cg_iters for row in selected),
        mean_columns=mean(row.n_columns for row in selected),
        mean_solve_at_90pct=mean(row.solve_at_90pct for row in selected),
        mean_solve_at_99pct=mean(row.solve_at_99pct for row in selected),
        mean_certification_improvement_share=
            mean(row.certification_improvement_share for row in selected),
        mean_meaningful_y_shift_share=mean(row.meaningful_y_shift_share for row in selected),
        mean_effective_support_jaccard=mean(row.effective_support_jaccard for row in selected),
    ))
end

mkpath(OUTDIR)
CSV.write(joinpath(OUTDIR, "case_comparison.csv"), DataFrame(comparison_rows))
CSV.write(joinpath(OUTDIR, "variant_summary.csv"), DataFrame(variant_rows))

println("wrote $(joinpath(OUTDIR, "case_comparison.csv"))")
println("wrote $(joinpath(OUTDIR, "variant_summary.csv"))")
for row in variant_rows
    @printf("%-12s time=%8.2fs pricing=%8.2fs labels=%10.1f iters=%6.1f max_mip_delta=%8.3f\n",
        row.variant, row.mean_total_seconds, row.mean_pricing_seconds,
        row.mean_labels, row.mean_iterations, row.max_mip_delta)
end
