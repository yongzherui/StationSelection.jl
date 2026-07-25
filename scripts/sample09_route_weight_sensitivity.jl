"""
    scripts/sample09_route_weight_sensitivity.jl

Follow-up to scripts/sample09_mw_vs_direct.jl: tests whether `route_reg_weight`
(route_regularization_weight) explains why the big n=10..60 method-compare sweep
shows a smaller/noisier BendersY(mw) speedup than the original hopeful result in
notes/2026-07-17_restricted_mw_cut_benders_y.md.

The two differ specifically on this knob: the original check script
(scripts/check_zhuzhou_restricted_mw_timing.jl) used
`route_regularization_weight=0.1` with the model's default walk weight, while the
big sweep (scripts/run_method_compare_task.jl's CFG) uses `route_reg_weight=10.0`
/ `walk_cost_weight=0.1` -- a 100x-vs-1x ratio difference. This script holds
everything else fixed at the big sweep's values (matching
sample09_mw_vs_direct.jl) and varies ONLY route_reg_weight across
{0.0, 0.1, 1.0, 10.0}, on BendersY(cut_derivation=:restricted_mw_fixed_pi) only
(method label `bendersY_mw_ms4`) -- the method/cut mode this whole investigation
is about -- to see whether convergence speed (iterations, wall time) is sensitive
to this weight on its own.

Reuses load_sample09/build_model/build_solver/MethodSpec/METHODS from
sample09_mw_vs_direct.jl (which itself reuses run_method_compare_task.jl) by
`include`-ing it as a library; its own `main()` is guarded by
`abspath(PROGRAM_FILE) == @__FILE__`, which is false here, so nothing there runs.

Usage:
    julia --project=. scripts/sample09_route_weight_sensitivity.jl [outdir]

Default output dir: experiments/sample09_route_weight_sensitivity/
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const ROUTE_WEIGHTS = haskey(ENV, "SAMPLE09_ROUTE_WEIGHTS") ?
    parse.(Float64, split(ENV["SAMPLE09_ROUTE_WEIGHTS"], ",")) : [0.0, 0.1, 1.0, 10.0]
const N_STATIONS_FOR_WEIGHT_SWEEP = haskey(ENV, "SAMPLE09_N_STATIONS") ?
    parse.(Int, split(ENV["SAMPLE09_N_STATIONS"], ",")) : [10, 15]
const WEIGHT_METHOD_LABEL = "bendersY_mw_ms4"

function run_one_weight(n_stations::Int, route_weight::Float64, results_dir::String, iters_dir::String)
    l = L_FOR[n_stations]
    method = method_by_label(WEIGHT_METHOD_LABEL)
    max_stops = resolve_max_stops(method.max_stops_mode, n_stations)
    inst_name = "sample09_n$(n_stations)"
    rw_tag = replace(string(route_weight), "." => "p")
    summary_path = joinpath(results_dir, "$(inst_name)__$(WEIGHT_METHOD_LABEL)__rw$(rw_tag).csv")

    @printf("  [%s / rw=%.2f] l=%d max_stops=%d ... ", inst_name, route_weight, l, max_stops)
    flush(stdout)

    cfg = merge(CFG, (route_reg_weight = route_weight,))

    data = load_sample09(n_stations)
    model = build_model(l, max_stops, MAX_WALK, cfg)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(WEIGHT_METHOD_LABEL)__rw$(rw_tag)")
    mkpath(log_dir)
    solver = build_solver(method, cfg, log_dir)

    t0 = time()
    status = "ok"
    result = nothing
    try
        result = StationSelection.run_opt(data, model, solver)
    catch err
        status = "error: $(sprint(showerror, err))"
    end
    wall_time = time() - t0

    selected_stations = Int[]
    if !isnothing(result)
        try
            StationSelection.export_variables(result, log_dir)
            selected_stations = _selected_station_ids_sample09(result, method)
        catch err
            @warn "failed to export variables for rw=$route_weight on $inst_name" exception=(err, catch_backtrace())
        end
    end

    n_iterations = _last_n_iterations(log_dir, method)

    summary = (
        instance           = inst_name,
        n_stations         = n_stations,
        l                  = l,
        route_reg_weight   = route_weight,
        walk_cost_weight   = cfg.walk_cost_weight,
        method             = WEIGHT_METHOD_LABEL,
        status             = status,
        termination_status = isnothing(result) ? "" : string(result.termination_status),
        objective_value    = isnothing(result) || isnothing(result.objective_value) ? "" : string(result.objective_value),
        wall_time_sec      = wall_time,
        n_iterations       = n_iterations,
        n_stations_selected = length(selected_stations),
        selected_stations  = string(selected_stations),
        iters_log_path     = log_dir,
    )
    CSV.write(summary_path, DataFrame([summary]))

    @printf("status=%s obj=%s wall=%.1fs iters=%s\n",
        status, summary.objective_value, wall_time, isempty(n_iterations) ? "n/a" : n_iterations)
    flush(stdout)
    return summary
end

function main_weight_sweep()
    outdir = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "sample09_route_weight_sensitivity")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== sample_09: route_reg_weight sensitivity, $(WEIGHT_METHOD_LABEL), n_stations=$(N_STATIONS_FOR_WEIGHT_SWEEP) ===")
    println("route_reg_weight values: ", join(ROUTE_WEIGHTS, ", "))
    println()

    rows = NamedTuple[]
    for n_stations in N_STATIONS_FOR_WEIGHT_SWEEP, route_weight in ROUTE_WEIGHTS
        push!(rows, run_one_weight(n_stations, route_weight, results_dir, iters_dir))
    end

    combined = DataFrame(rows)
    combined_path = joinpath(outdir, "combined_results.csv")
    CSV.write(combined_path, combined)
    println("\nWrote combined results to $combined_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main_weight_sweep()
end
