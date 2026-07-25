"""
    scripts/sample09_walk_weight_sensitivity.jl

Follow-up to scripts/sample09_route_weight_sensitivity.jl: that experiment held
walk_cost_weight fixed at 0.1 and varied route_reg_weight across
{0.0, 0.1, 1.0, 10.0}, finding that BendersY(mw_ms4)'s convergence at n=15
degrades sharply as route_reg_weight increases (30 iters at 0.0 -> 379 iters at
0.1 -> non-convergent (500-iter cap) at 1.0 and 10.0), while n=10 was
essentially unaffected. That leaves open whether it's route_reg_weight in
isolation, or the RATIO of route_reg_weight to walk_cost_weight, that drives
this -- since the sweep's default (route_reg_weight=10.0, walk_cost_weight=0.1)
is a 100x ratio, same order as route_reg_weight=1.0 with walk_cost_weight=0.01.

This script holds route_reg_weight FIXED at 1.0 and instead varies
walk_cost_weight across {1.0, 0.1, 0.01} (ratios 1x, 10x, 100x), same method
(bendersY_mw_ms4), same n_stations in {10,15}, everything else identical to
sample09_route_weight_sensitivity.jl's CFG.

Reuses load_sample09/build_model/build_solver/MethodSpec/METHODS from
sample09_mw_vs_direct.jl by `include`-ing it as a library; its own `main()` is
guarded by `abspath(PROGRAM_FILE) == @__FILE__`, which is false here.

Usage:
    julia --project=. scripts/sample09_walk_weight_sensitivity.jl [outdir]

Default output dir: experiments/sample09_walk_weight_sensitivity/
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const WALK_WEIGHTS = haskey(ENV, "SAMPLE09_WALK_WEIGHTS") ?
    parse.(Float64, split(ENV["SAMPLE09_WALK_WEIGHTS"], ",")) : [1.0, 0.1, 0.01]
const N_STATIONS_FOR_WALK_SWEEP = haskey(ENV, "SAMPLE09_N_STATIONS") ?
    parse.(Int, split(ENV["SAMPLE09_N_STATIONS"], ",")) : [10, 15]
const FIXED_ROUTE_WEIGHT = 1.0
const WALK_WEIGHT_METHOD_LABEL = "bendersY_mw_ms4"

function run_one_walk_weight(n_stations::Int, walk_weight::Float64, results_dir::String, iters_dir::String)
    l = L_FOR[n_stations]
    method = method_by_label(WALK_WEIGHT_METHOD_LABEL)
    max_stops = resolve_max_stops(method.max_stops_mode, n_stations)
    inst_name = "sample09_n$(n_stations)"
    ww_tag = replace(string(walk_weight), "." => "p")
    summary_path = joinpath(results_dir, "$(inst_name)__$(WALK_WEIGHT_METHOD_LABEL)__ww$(ww_tag).csv")

    @printf("  [%s / ww=%.3f] l=%d max_stops=%d route_reg_weight=%.1f ... ",
        inst_name, walk_weight, l, max_stops, FIXED_ROUTE_WEIGHT)
    flush(stdout)

    cfg = merge(CFG, (route_reg_weight = FIXED_ROUTE_WEIGHT, walk_cost_weight = walk_weight))

    data = load_sample09(n_stations)
    model = build_model(l, max_stops, MAX_WALK, cfg)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(WALK_WEIGHT_METHOD_LABEL)__ww$(ww_tag)")
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
            @warn "failed to export variables for ww=$walk_weight on $inst_name" exception=(err, catch_backtrace())
        end
    end

    n_iterations = _last_n_iterations(log_dir, method)

    summary = (
        instance           = inst_name,
        n_stations         = n_stations,
        l                  = l,
        route_reg_weight   = FIXED_ROUTE_WEIGHT,
        walk_cost_weight   = walk_weight,
        method             = WALK_WEIGHT_METHOD_LABEL,
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

function main_walk_weight_sweep()
    outdir = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "sample09_walk_weight_sensitivity")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== sample_09: walk_cost_weight sensitivity (route_reg_weight fixed at $FIXED_ROUTE_WEIGHT), $(WALK_WEIGHT_METHOD_LABEL), n_stations=$(N_STATIONS_FOR_WALK_SWEEP) ===")
    println("walk_cost_weight values: ", join(WALK_WEIGHTS, ", "))
    println()

    rows = NamedTuple[]
    for n_stations in N_STATIONS_FOR_WALK_SWEEP, walk_weight in WALK_WEIGHTS
        push!(rows, run_one_walk_weight(n_stations, walk_weight, results_dir, iters_dir))
    end

    combined = DataFrame(rows)
    combined_path = joinpath(outdir, "combined_results.csv")
    CSV.write(combined_path, combined)
    println("\nWrote combined results to $combined_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main_walk_weight_sweep()
end
