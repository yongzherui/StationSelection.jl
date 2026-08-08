"""
    scripts/lib/aggregate_od_route_run_instance.jl

Shared "solve one AggregateODRouteModel instance via plain column generation
and write the standard five-file output" runner, used by
`run_single_instance.jl` (synthetic grid data) and `run_zhuzhou_instance.jl`
(real Zhuzhou data) -- those two scripts were ~90% identical, differing only
in how `data`/`model` get built. That part stays in each script; everything
after "solve and report" lives here.

`using CSV, DataFrames, Printf, StationSelection` must happen before this file
is included.
"""

struct AorInstancePaths
    results_dir::String
    iters_dir::String
    columns_dir::String
    duals_dir::String
    selected_dir::String
    csv_path::String
    iter_path::String
    column_path::String
    dual_path::String
    selected_path::String
end

"Standard `<base_outdir>/{results,iters,columns,duals,selected}/<inst_name>...` layout."
function aor_instance_paths(base_outdir::AbstractString, inst_name::AbstractString)
    results_dir  = joinpath(base_outdir, "results")
    iters_dir    = joinpath(base_outdir, "iters")
    columns_dir  = joinpath(base_outdir, "columns")
    duals_dir    = joinpath(base_outdir, "duals")
    selected_dir = joinpath(base_outdir, "selected")
    return AorInstancePaths(
        results_dir, iters_dir, columns_dir, duals_dir, selected_dir,
        joinpath(results_dir, "$(inst_name).csv"),
        joinpath(iters_dir, "$(inst_name)_iters.csv"),
        joinpath(columns_dir, "$(inst_name)_columns.csv"),
        joinpath(duals_dir, "$(inst_name)_duals.csv"),
        joinpath(selected_dir, "$(inst_name)_selected.csv"),
    )
end

"""
Fields like `best_reduced_cost` are `nothing` on rows that didn't compute them
(e.g. an iteration that priced no columns). CSV.jl can serialize `missing` but
not `nothing`, so convert column-by-column before writing -- the same fix
`passenger_free_assignment_cg_scaling.jl` applies to its own iteration rows.
"""
function aor_write_namedtuple_rows(path::AbstractString, rows::Vector{<:NamedTuple})
    df = isempty(rows) ? DataFrame() : DataFrame(rows)
    for col in names(df)
        if any(x -> x === nothing, df[!, col])
            df[!, col] = [x === nothing ? missing : x for x in df[!, col]]
        end
    end
    CSV.write(path, df)
end

aor_write_summary(path::AbstractString, row::NamedTuple) = CSV.write(path, DataFrame([row]))

function aor_write_selected_columns(path::AbstractString, result)
    column_by_id = Dict(col.id => col for col in result.final_result.mapping.columns)
    rows = NamedTuple[]
    for column_id in result.selected_column_ids
        col = get(column_by_id, column_id, nothing)
        col === nothing && continue
        push!(rows, (
            column_id=col.id, n_pairs=length(col.od_pairs), tau=col.tau,
            metadata=string(col.metadata), pairs=string(Tuple(col.od_pairs)),
        ))
    end
    aor_write_namedtuple_rows(path, rows)
end

"""
    aor_run_and_report(inst_name, paths, model, data;
                        max_cg_iters, max_new_cols, pricing_time, ip_time_limit,
                        mip_gap, extra_summary_fields=(;))
        -> (result, wall_time)  (or (nothing, 0.0) if skipped)

Solve `(model, data)` via `run_aggregate_od_route_column_generation`, write
the five standard output files, print the standard report block, and skip
entirely (returning `nothing`) if `paths.csv_path` already has a result row --
so a re-submitted array job doesn't redo finished cells.

`extra_summary_fields` is merged in FIRST, so caller-specific columns (e.g.
`nx`/`ny` for the synthetic grid, `endpoint_overlap` for Zhuzhou) appear
before the common ones in the output CSV.
"""
function aor_run_and_report(
    inst_name::AbstractString, paths::AorInstancePaths, model, data;
    max_cg_iters::Int, max_new_cols::Int, pricing_time::Float64,
    ip_time_limit::Float64, mip_gap::Float64, extra_summary_fields::NamedTuple=(;),
)
    mkpath.((paths.results_dir, paths.iters_dir, paths.columns_dir, paths.duals_dir, paths.selected_dir))
    if isfile(paths.csv_path) && countlines(paths.csv_path) >= 2
        println("=== Skipping $inst_name — result already exists ===")
        return nothing, 0.0
    end

    t0 = time()
    result = run_aggregate_od_route_column_generation(
        model, data;
        verbose=false, cg_log_path=paths.iter_path, column_log_path=paths.column_path,
        dual_log_path=paths.dual_path, max_cg_iters=max_cg_iters, max_new_columns=max_new_cols,
        n_candidates=max(max_new_cols, 20), reduced_cost_tol=1e-6,
        pricing_time_limit_sec=pricing_time, ip_time_limit_sec=ip_time_limit, mip_gap=mip_gap,
        silent=true,
    )
    wall_time = time() - t0

    ip_obj = result.final_result.objective_value
    lp_bnd = result.lp_bound
    gap_pct = if !isnothing(ip_obj) && isfinite(ip_obj) && isfinite(lp_bnd) && ip_obj > 1e-10
        100.0 * (ip_obj - lp_bnd) / ip_obj
    else
        NaN
    end

    @printf("  Status       : %s\n", result.status)
    @printf("  IP objective : %s\n", isnothing(ip_obj) ? "n/a" : @sprintf("%.4f", ip_obj))
    @printf("  LP bound     : %s\n", isfinite(lp_bnd) ? @sprintf("%.4f", lp_bnd) : "n/a")
    @printf("  Gap %%        : %s\n", isnan(gap_pct) ? "n/a" : @sprintf("%.2f%%", gap_pct))
    @printf("  CG iters     : %d  (%s)\n", result.n_cg_iters, result.cg_stop_reason)
    @printf("  Wall time    : %.1fs\n", wall_time)
    println()

    summary_row = merge(extra_summary_fields, (
        instance=inst_name,
        status=string(result.status),
        termination_status=string(result.final_result.termination_status),
        objective_value=isnothing(ip_obj) ? "" : string(ip_obj),
        lp_bound=string(lp_bnd),
        integrality_gap_pct=isnan(gap_pct) ? "" : string(gap_pct),
        n_cg_iters=result.n_cg_iters,
        cg_stop_reason=string(result.cg_stop_reason),
        n_generated_columns=length(result.generated_columns),
        n_selected_columns=length(result.selected_column_ids),
        wall_time_sec=wall_time,
    ))
    aor_write_summary(paths.csv_path, summary_row)
    aor_write_namedtuple_rows(paths.iter_path, result.iteration_rows)
    aor_write_namedtuple_rows(paths.column_path, result.column_log_rows)
    aor_write_namedtuple_rows(paths.dual_path, result.dual_log_rows)
    aor_write_selected_columns(paths.selected_path, result)

    println("Written: $(paths.csv_path)")
    println("Written: $(paths.iter_path)")
    println("Written: $(paths.column_path)")
    println("Written: $(paths.dual_path)")
    println("Written: $(paths.selected_path)")

    return result, wall_time
end
