"""
    scripts/sample09_route_weight_ramp_ground_truth.jl

Independent ground truth for `sample09_route_weight_ramp_experiment.jl`: solves the same
instance/model with `DirectSolver` (compact enumeration, method label "direct_ms4" from
`run_method_compare_task.jl`'s METHODS) instead of BendersY, to confirm the direct/ramp
BendersY objective values are actually correct and not just consistent with each other.

Usage:
    julia --project=. scripts/sample09_route_weight_ramp_ground_truth.jl <outdir>

Env overrides:
    RAMP_N_STATIONS   number of candidate stations from the sample_09 fixture (default 15)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const RAMP_N_STATIONS = parse(Int, get(ENV, "RAMP_N_STATIONS", "15"))

function main()
    length(ARGS) >= 1 || error("Usage: julia sample09_route_weight_ramp_ground_truth.jl <outdir>")
    outdir = ARGS[1]
    results_dir = joinpath(outdir, "results")
    mkpath(results_dir)

    l = L_FOR[RAMP_N_STATIONS]
    max_stops = resolve_max_stops(:ms4, RAMP_N_STATIONS)
    data = load_sample09(RAMP_N_STATIONS)
    model = build_model(l, max_stops, MAX_WALK, CFG)

    println("=== ground truth (DirectSolver) β=$(CFG.route_reg_weight), n_stations=$RAMP_N_STATIONS l=$l ===")
    flush(stdout)

    method = method_by_label("direct_ms4")
    log_dir = joinpath(outdir, "iters", "ground_truth")
    mkpath(log_dir)
    solver = build_solver(method, CFG, log_dir)

    t0 = time()
    result = run_opt(data, model, solver)
    wall = time() - t0
    @printf("[ground_truth] status=%s obj=%.4f wall=%.1fs\n", result.termination_status, result.objective_value, wall)
    flush(stdout)

    row = DataFrame([(
        variant="ground_truth", n_stations=RAMP_N_STATIONS, l=l,
        objective_value=result.objective_value,
        termination_status=string(result.termination_status),
        wall_time_sec=wall,
    )])
    result_path = joinpath(results_dir, "ground_truth.csv")
    CSV.write(result_path, row)
    println("Wrote $result_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
