"""
    scripts/sample09_route_weight_ramp_experiment.jl

Compares solving `BendersY(lifted_walking_objective=true)` directly at
`route_regularization_weight=β_target` against ramping up to the same `β_target` via
`route_regularization_weight_schedule` -- i.e. warm-starting the master/cuts/pool from a sequence
of cheaper, lower-β solves instead of deriving every cut from scratch at the target weight.

Reuses `scripts/sample09_mw_vs_direct.jl`'s data loading (`load_sample09`), `CFG` (whose
`route_reg_weight` default of `10.0` is exactly the requested target β), `MAX_WALK`, `L_FOR`, and
`build_model`/`resolve_max_stops` (from `run_method_compare_task.jl`, included transitively) --
safe to `include` since its `main()` is guarded by `_RUN_AS_MAIN = abspath(PROGRAM_FILE) ==
@__FILE__`, which is false here.

Runs ONE variant per invocation (`direct` or `ramp`), each writing its own result row to
`<outdir>/results/<variant>.csv` -- meant to be run as two separate, parallel sbatch jobs (one per
instance/method, matching this repo's `sbatch_sample09_task.sh` convention) rather than solved
sequentially in one job, since the two variants are independent and each solve can run long.

Usage:
    julia --project=. scripts/sample09_route_weight_ramp_experiment.jl <outdir> <direct|ramp>

Env overrides:
    RAMP_N_STATIONS    number of candidate stations from the sample_09 fixture (default 15)
    RAMP_SCHEDULE      ';'-separated β schedule (default "0.01;0.1;1.0;10.0")
    RAMP_DECOMPOSITION "bendersY" or "bendersYZ" (default "bendersY")
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "sample09_mw_vs_direct.jl"))

const RAMP_N_STATIONS = parse(Int, get(ENV, "RAMP_N_STATIONS", "15"))
# ';'-separated, not ','  -- sbatch's own --export=NAME=VALUE,... parsing splits on commas even
# inside a quoted VALUE, so a comma-separated schedule silently truncates at the first comma when
# passed via --export (observed: "0.01,0.03,0.1,..." arrived as just "0.01").
const RAMP_SCHEDULE = parse.(Float64, split(get(ENV, "RAMP_SCHEDULE", "0.01;0.1;1.0;10.0"), ";"))
const RAMP_DECOMPOSITION = get(ENV, "RAMP_DECOMPOSITION", "bendersY")
RAMP_DECOMPOSITION in ("bendersY", "bendersYZ") ||
    error("RAMP_DECOMPOSITION must be \"bendersY\" or \"bendersYZ\", got $(repr(RAMP_DECOMPOSITION))")

function run_variant(outdir::String, variant::String)
    variant in ("direct", "ramp") || error("variant must be \"direct\" or \"ramp\", got $(repr(variant))")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters", variant)
    mkpath.((results_dir, iters_dir))

    l = L_FOR[RAMP_N_STATIONS]
    max_stops = resolve_max_stops(:ms4, RAMP_N_STATIONS)
    data = load_sample09(RAMP_N_STATIONS)
    model = build_model(l, max_stops, MAX_WALK, CFG)
    schedule = variant == "ramp" ? RAMP_SCHEDULE : nothing

    decomposition = RAMP_DECOMPOSITION == "bendersYZ" ? BendersYZ() : BendersY()

    println(
        "=== [$RAMP_DECOMPOSITION/$variant] β=$(CFG.route_reg_weight)",
        variant == "ramp" ? " via schedule $(RAMP_SCHEDULE)" : " direct",
        ", n_stations=$RAMP_N_STATIONS l=$l max_stops=$max_stops, n_orders=$(nrow(data.scenarios[1].requests)) ===",
    )
    flush(stdout)

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        decomposition=decomposition,
        inner_solver=ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
            max_iterations=CFG.inner_cg_max_iters, max_columns_per_iteration=20, n_candidates=20,
            pricing_time_limit_sec=CFG.inner_pricing_time, final_ip_time_limit_sec=CFG.inner_ip_time_limit,
        ),
        max_iterations=max(CFG.benders_max_iters, 3000),
        max_reprice_rounds=CFG.max_reprice_rounds,
        reprice_subproblem=true,
        cut_derivation=:restricted_mw_fixed_pi,
        lifted_walking_objective=true,
        route_regularization_weight_schedule=schedule,
        log_dir=iters_dir,
    )
    t0 = time()
    result = run_opt(data, model, solver)
    wall = time() - t0
    @printf(
        "[%s/%s] obj=%.4f iters=%d cuts=%d wall=%.1fs\n",
        RAMP_DECOMPOSITION, variant, result.objective_value, result.metadata["benders_iterations"],
        result.metadata["optimality_cuts_added"], wall,
    )
    variant == "ramp" && println("[$RAMP_DECOMPOSITION/ramp] stage log: ", result.metadata["route_regularization_weight_stage_log"])
    flush(stdout)

    row = DataFrame([(
        decomposition=RAMP_DECOMPOSITION, variant=variant, n_stations=RAMP_N_STATIONS, l=l,
        objective_value=result.objective_value,
        iterations=result.metadata["benders_iterations"],
        optimality_cuts=result.metadata["optimality_cuts_added"],
        wall_time_sec=wall,
        route_regularization_weight_schedule=variant == "ramp" ? string(RAMP_SCHEDULE) : "",
    )])
    result_path = joinpath(results_dir, "$variant.csv")
    CSV.write(result_path, row)
    println("Wrote $result_path")
end

function main()
    length(ARGS) >= 2 || error("Usage: julia sample09_route_weight_ramp_experiment.jl <outdir> <direct|ramp>")
    outdir, variant = ARGS[1], ARGS[2]
    run_variant(outdir, variant)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
