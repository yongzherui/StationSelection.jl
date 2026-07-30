"""
    scripts/zhuzhou_route_weight_ramp_experiment.jl

Larger-n counterpart to sample09_route_weight_ramp_experiment.jl: compares solving
`BendersYZ(lifted_walking_objective=true, cut_derivation=:restricted_mw_fixed_pi)` directly at
`route_regularization_weight=β_target` against ramping up to the same `β_target` via
`route_regularization_weight_schedule` -- same comparison, but on synthetic zhuzhou-family
instances built via `aggregate_od_route_method_grid.jl`'s `build_instance`/`_l_for` (as used by
the n=10..60 method-compare sweep), since sample_09's real fixture tops out at 19 candidate
stations (see sample09_mw_vs_direct.jl's docstring) while the base zhuzhou data has 84 stations
available by request volume.

Reuses `build_model`/`resolve_max_stops` (from `run_method_compare_task.jl`) and
`build_instance`/`_l_for` (from `aggregate_od_route_method_grid.jl`, included transitively) --
safe to `include` since `run_method_compare_task.jl`'s ARGS-parsing block is guarded by
`_RUN_AS_MAIN = abspath(PROGRAM_FILE) == @__FILE__`, which is false here. Defines its own `CFG`
(mirroring sample09_mw_vs_direct.jl's shape) rather than reusing that file's, since this script's
route_reg_weight is driven by the schedule target, not a fixed sweep default.

Runs ONE variant per invocation (`direct` or `ramp`), each writing its own result row to
`<outdir>/results/<variant>.csv` -- meant to be run as separate, parallel sbatch jobs.

Usage:
    julia --project=. scripts/zhuzhou_route_weight_ramp_experiment.jl <outdir> <direct|ramp>

Env overrides:
    RAMP_N_STATIONS     top-N zhuzhou stations by request volume to use (default 20)
    RAMP_N_PAIRS        distinct OD demand pairs per scenario (default 16)
    RAMP_SEED           instance seed (default 42)
    RAMP_SCHEDULE       ';'-separated β schedule (default "0.1;0.2;0.3;0.4;0.5;0.6;0.7;0.8;0.9;1.0")
    RAMP_TARGET_WEIGHT  target route_regularization_weight; schedule (if given) must end here
                        (default 1.0)
    RAMP_MAX_STOPS_MODE  max_stops_mode passed to resolve_max_stops (default "ms4")
    RAMP_DATA_DIR        zhuzhou base-data dir (default ../../Data/base_data relative to here)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

const RAMP_N_STATIONS    = parse(Int, get(ENV, "RAMP_N_STATIONS", "20"))
const RAMP_N_PAIRS       = parse(Int, get(ENV, "RAMP_N_PAIRS", "16"))
const RAMP_SEED          = parse(Int, get(ENV, "RAMP_SEED", "42"))
# ';'-separated, not ',' -- sbatch's own --export=NAME=VALUE,... parsing splits on commas even
# inside a quoted VALUE (see sample09_route_weight_ramp_experiment.jl's identical note).
const RAMP_SCHEDULE      = parse.(Float64, split(get(ENV, "RAMP_SCHEDULE", "0.1;0.2;0.3;0.4;0.5;0.6;0.7;0.8;0.9;1.0"), ";"))
const RAMP_TARGET_WEIGHT = parse(Float64, get(ENV, "RAMP_TARGET_WEIGHT", "1.0"))
const RAMP_MAX_STOPS_MODE = Symbol(get(ENV, "RAMP_MAX_STOPS_MODE", "ms4"))
const RAMP_DATA_DIR      = get(ENV, "RAMP_DATA_DIR", normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data")))

const CFG = (
    mip_gap             = 1e-4,
    benders_max_iters   = 500,
    max_reprice_rounds  = 10000,
    inner_cg_max_iters  = 200,
    inner_pricing_time  = 120.0,
    inner_ip_time_limit = 60.0,
    detour_factor       = 2.0,
    max_wait_time       = 900.0,
    route_reg_weight    = RAMP_TARGET_WEIGHT,
    walk_cost_weight    = 0.1,
    repositioning_time  = 20.0,
)

function run_variant(outdir::String, variant::String)
    variant in ("direct", "ramp") || error("variant must be \"direct\" or \"ramp\", got $(repr(variant))")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters", variant)
    mkpath.((results_dir, iters_dir))

    l = _l_for(RAMP_N_STATIONS)
    max_stops = resolve_max_stops(RAMP_MAX_STOPS_MODE, RAMP_N_STATIONS)
    data, max_walk = build_instance("zhuzhou", RAMP_N_STATIONS, RAMP_N_PAIRS, RAMP_SEED, RAMP_DATA_DIR)
    model = build_model(l, max_stops, max_walk, CFG)
    schedule = variant == "ramp" ? RAMP_SCHEDULE : nothing

    println(
        "=== [bendersYZ/$variant] β=$(CFG.route_reg_weight)",
        variant == "ramp" ? " via schedule $(RAMP_SCHEDULE)" : " direct",
        ", n_stations=$RAMP_N_STATIONS l=$l n_pairs=$RAMP_N_PAIRS seed=$RAMP_SEED max_stops=$max_stops, ",
        "n_orders=$(nrow(data.scenarios[1].requests)) ===",
    )
    flush(stdout)

    solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=CFG.mip_gap),
        decomposition=BendersYZ(),
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
        "[bendersYZ/%s] obj=%.4f iters=%d cuts=%d wall=%.1fs\n",
        variant, result.objective_value, result.metadata["benders_iterations"],
        result.metadata["optimality_cuts_added"], wall,
    )
    variant == "ramp" && println("[bendersYZ/ramp] stage log: ", result.metadata["route_regularization_weight_stage_log"])
    flush(stdout)

    row = DataFrame([(
        decomposition="bendersYZ", variant=variant, n_stations=RAMP_N_STATIONS, l=l,
        n_pairs=RAMP_N_PAIRS, seed=RAMP_SEED,
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
    length(ARGS) >= 2 || error("Usage: julia zhuzhou_route_weight_ramp_experiment.jl <outdir> <direct|ramp>")
    outdir, variant = ARGS[1], ARGS[2]
    run_variant(outdir, variant)
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
