"""Run one pure Branch-and-Benders BendersYZ restricted-MW benchmark task."""

using CSV, DataFrames, Gurobi, Logging, StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

length(ARGS) in (7, 8) || error(
    "Usage: zhuzhou_branch_benders_yz_mw_no_mcf_task.jl " *
    "<outdir> <data_dir> <n> <p> <seed> <n_scenarios> <time_class> [mcf_variant]",
)

const OUTDIR = abspath(ARGS[1])
const DATA_DIR_BB = abspath(ARGS[2])
const N_BB = parse(Int, ARGS[3])
const P_BB = parse(Int, ARGS[4])
const SEED_BB = parse(Int, ARGS[5])
const Q_BB = parse(Int, ARGS[6])
const TIME_CLASS = ARGS[7]
const MCF_VARIANT = length(ARGS) == 8 ? Symbol(ARGS[8]) : :none
MCF_VARIANT in (:none, :common_od, :common_od_fractional) ||
    error("mcf_variant must be none, common_od, or common_od_fractional")
const L_BB = ceil(Int, N_BB / 2)
const METHOD_BB = MCF_VARIANT == :none ? "branch_bendersYZ_mw_no_mcf_ms5" :
    MCF_VARIANT == :common_od ? "branch_bendersYZ_mw_common_od_ms5" :
    "branch_bendersYZ_mw_common_od_fractional_mcf_ms5"
const INSTANCE_BB = "zhuzhou_n$(N_BB)_p$(P_BB)_s$(SEED_BB)_q$(Q_BB)"
const CASE_BB = "$(INSTANCE_BB)__$(METHOD_BB)"
const RESULT_PATH_BB = joinpath(OUTDIR, "results", "$(CASE_BB).csv")
const LOG_DIR_BB = joinpath(OUTDIR, "iters", CASE_BB)

metadata(result, key, default=missing) = isnothing(result) ? default : get(result.metadata, key, default)

function main()
    mkpath(dirname(RESULT_PATH_BB))
    mkpath(LOG_DIR_BB)
    global_logger(ConsoleLogger(stdout, Logging.Info))

    cfg = (
        mip_gap=1e-4, benders_max_iters=500, max_reprice_rounds=10_000,
        inner_cg_max_iters=200, inner_pricing_time=120.0,
        inner_ip_time_limit=60.0, cg_max_iters=10_000,
        cg_pricing_time=120.0, cg_ip_time_limit=300.0,
        direct_max_routes=typemax(Int), direct_time_limit=300.0,
        detour_factor=2.0, max_wait_time=900.0,
        route_reg_weight=10.0, walk_cost_weight=0.1, repositioning_time=20.0,
    )
    data, max_walk = build_instance(
        "zhuzhou", N_BB, P_BB, SEED_BB, DATA_DIR_BB; n_scenarios=Q_BB,
    )
    model = build_model(L_BB, 5, max_walk, cfg)
    inner = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        max_iterations=cfg.inner_cg_max_iters,
        max_columns_per_iteration=20,
        n_candidates=20,
        reduced_cost_tol=1e-7,
        pricing_time_limit_sec=cfg.inner_pricing_time,
        final_ip_time_limit_sec=cfg.inner_ip_time_limit,
    )
    solver = BranchAndBendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=false, mip_gap=cfg.mip_gap),
        decomposition=BendersYZ(),
        cut_derivation=:restricted_mw_fixed_pi,
        inner_solver=inner,
        initial_benders_cut_rounds=0,
        max_reprice_rounds=cfg.max_reprice_rounds,
        mcf_lower_bound_mode=MCF_VARIANT == :none ? :none : :common_od_scaled,
        projected_mcf_user_cuts=MCF_VARIANT == :common_od_fractional,
        pricing_tolerance=1e-7,
        dual_feasibility_tolerance=1e-7,
        log_dir=LOG_DIR_BB,
    )

    println("Starting $CASE_BB with restricted-MW Branch-and-Benders; MCF variant=$MCF_VARIANT")
    flush(stdout)
    result = nothing
    failure = ""
    t0 = time()
    try
        result = StationSelection.run_opt(data, model, solver)
    catch err
        failure = sprint(showerror, err)
        showerror(stderr, err, catch_backtrace())
        println(stderr)
    end
    wall = time() - t0

    row = (
        instance=INSTANCE_BB, n_stations=N_BB, l=L_BB, n_pairs=P_BB,
        seed=SEED_BB, n_scenarios=Q_BB, method=METHOD_BB,
        max_stops=5, time_class=TIME_CLASS,
        mcf_lower_bound_mode=string(metadata(result, "branch_benders_mcf_lower_bound_mode", "none")),
        projected_mcf_user_cuts=metadata(result, "branch_benders_projected_mcf_user_cuts", false),
        projected_mcf_separations=metadata(result, "branch_benders_projected_mcf_separations", 0),
        projected_mcf_cuts=metadata(result, "branch_benders_projected_mcf_cuts", 0),
        projected_mcf_seconds=metadata(result, "branch_benders_projected_mcf_seconds", 0.0),
        projected_mcf_root_skips=metadata(result, "branch_benders_projected_mcf_root_skips", 0),
        mcf_common_od_count=metadata(result, "branch_benders_mcf_common_od_count", 0),
        status=isnothing(result) ? "error" : "ok", error=failure,
        termination_status=isnothing(result) ? "" : string(result.termination_status),
        objective_value=isnothing(result) ? missing : result.objective_value,
        lower_bound=metadata(result, "branch_benders_global_lb"),
        relative_gap=metadata(result, "branch_benders_gap_relative"),
        wall_time_sec=wall,
        node_count=metadata(result, "branch_benders_node_count"),
        callback_count=metadata(result, "branch_benders_callback_count"),
        unique_exact_evaluations=metadata(result, "branch_benders_unique_y"),
        cache_hits=metadata(result, "branch_benders_cache_hits"),
        cuts_submitted=metadata(result, "branch_benders_cuts_submitted"),
        repeated_submissions=metadata(result, "branch_benders_repeated_submissions"),
        oracle_seconds=metadata(result, "branch_benders_oracle_seconds"),
        priming_cg_seconds=metadata(result, "branch_benders_priming_cg_seconds"),
        repricing_seconds=metadata(result, "branch_benders_repricing_seconds"),
        mw_completion_seconds=metadata(result, "branch_benders_mw_completion_seconds"),
        shared_pool_size=metadata(result, "branch_benders_shared_pool_size"),
    )
    CSV.write(RESULT_PATH_BB, DataFrame([row]))
    println("Wrote $RESULT_PATH_BB; status=$(row.status), cuts=$(row.cuts_submitted), wall=$(round(wall; digits=1))s")
    flush(stdout)
    isempty(failure) || exit(1)
end

main()
