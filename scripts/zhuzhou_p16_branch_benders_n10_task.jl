"""Run one Zhuzhou Branch-and-Benders test case on a compute node.

Usage:
    julia --project=. scripts/zhuzhou_p16_branch_benders_n10_task.jl \
        <base_outdir> <seed> <max_stops_mode> [y|yz]

Set `BRANCH_BENDERS_N_STATIONS` to select the instance size (default 10).
"""

using CSV, DataFrames, Gurobi, Logging, StationSelection

include(joinpath(@__DIR__, "zhuzhou_p16_scaling_route100x.jl"))

length(ARGS) in (3, 4) || error(
    "Usage: zhuzhou_p16_branch_benders_n10_task.jl " *
    "<base_outdir> <seed> <max_stops_mode> [y|yz]",
)

const BASE_OUTDIR = abspath(ARGS[1])
const SEED = parse(Int, ARGS[2])
const MAX_STOPS_MODE = Symbol(ARGS[3])
const DECOMPOSITION_NAME = length(ARGS) == 4 ? lowercase(ARGS[4]) : "yz"
const DECOMPOSITION = DECOMPOSITION_NAME == "y" ? BendersY() :
    DECOMPOSITION_NAME == "yz" ? BendersYZ() :
    error("decomposition must be y or yz, got $(ARGS[4])")
const CUT_DERIVATION = Symbol(get(ENV, "BRANCH_BENDERS_CUT_DERIVATION", "standard"))
const MCF_LOWER_BOUND_MODE = Symbol(get(ENV, "BRANCH_BENDERS_MCF_MODE", "all_scenarios"))
const MCF_SCENARIO_ID = let raw = get(ENV, "BRANCH_BENDERS_MCF_SCENARIO_ID", "")
    isempty(raw) ? nothing : parse(Int, raw)
end
const PROJECTED_MCF_USER_CUTS = lowercase(get(ENV, "BRANCH_BENDERS_PROJECTED_MCF_CUTS", "false")) in
    ("1", "true", "yes")
const PROJECTED_MCF_MAX_SEPARATIONS = parse(
    Int, get(ENV, "BRANCH_BENDERS_PROJECTED_MCF_MAX_SEPARATIONS", "8"),
)
const N_STATIONS = parse(Int, get(ENV, "BRANCH_BENDERS_N_STATIONS", "10"))
const L = _l_for(N_STATIONS)
const MAX_STOPS = resolve_max_stops(MAX_STOPS_MODE, N_STATIONS)
const MCF_MODE_SUFFIX = MCF_LOWER_BOUND_MODE == :all_scenarios ? "" : "_mcf_$(MCF_LOWER_BOUND_MODE)"
const CASE_NAME = "zhuzhou_n$(N_STATIONS)_p$(N_PAIRS)_s$(SEED)__branch_benders_$(DECOMPOSITION_NAME)_$(MAX_STOPS_MODE)$(MCF_MODE_SUFFIX)"
const RESULTS_DIR = joinpath(BASE_OUTDIR, "results")
const LOG_DIR = joinpath(BASE_OUTDIR, "iters", CASE_NAME)
const SUMMARY_PATH = joinpath(RESULTS_DIR, "$(CASE_NAME).csv")

mkpath(RESULTS_DIR)
mkpath(LOG_DIR)

function metadata_value(result, key, default=missing)
    isnothing(result) && return default
    return get(result.metadata, key, default)
end

function main()
    # Keep Julia/StationSelection progress, warnings, and Gurobi output in one
    # chronological SLURM stream. stderr is then reserved for uncaught failures.
    global_logger(ConsoleLogger(stdout, Logging.Info))
    println("Branch-and-Benders n=$N_STATIONS case: seed=$SEED max_stops=$MAX_STOPS decomposition=$DECOMPOSITION_NAME")
    flush(stdout)
    data, max_walk = build_instance(FAMILY, N_STATIONS, N_PAIRS, SEED, DATA_DIR)
    model = build_model(L, MAX_STOPS, max_walk, CFG)

    master_config = SolverConfig(
        # Keep the outer Gurobi progress log visible in the SLURM output. The many
        # callback routing solves remain silent through `inner_solver` below.
        optimizer_env=Gurobi.Env(), silent=false, mip_gap=0.01,
    )
    inner_solver = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        max_iterations=CFG.inner_cg_max_iters,
        max_columns_per_iteration=20,
        n_candidates=20,
        reduced_cost_tol=1e-7,
        pricing_time_limit_sec=CFG.inner_pricing_time,
        final_ip_time_limit_sec=CFG.inner_ip_time_limit,
    )
    solver = BranchAndBendersSolver(
        config=master_config,
        decomposition=DECOMPOSITION,
        cut_derivation=CUT_DERIVATION,
        inner_solver=inner_solver,
        max_reprice_rounds=CFG.max_reprice_rounds,
        mcf_lower_bound_mode=MCF_LOWER_BOUND_MODE,
        mcf_scenario_id=MCF_SCENARIO_ID,
        projected_mcf_user_cuts=PROJECTED_MCF_USER_CUTS,
        projected_mcf_max_separations=PROJECTED_MCF_MAX_SEPARATIONS,
        pricing_tolerance=1e-7,
        dual_feasibility_tolerance=1e-7,
        log_dir=LOG_DIR,
    )
    println("Starting Branch-and-Benders optimize call")
    flush(stdout)

    result = nothing
    failure = ""
    t0 = time()
    try
        result = StationSelection.run_opt(data, model, solver)
    catch err
        failure = sprint(showerror, err, catch_backtrace())
        showerror(stderr, err, catch_backtrace())
        println(stderr)
    end
    wall_time = time() - t0

    open_indices = metadata_value(result, "branch_benders_open_stations", Int[])
    selected_stations = if isnothing(result)
        Int[]
    else
        sort([result.mapping.array_idx_to_station_id[i] for i in open_indices])
    end

    summary = (
        instance=CASE_NAME,
        family=FAMILY,
        n_stations=N_STATIONS,
        l=L,
        n_pairs=N_PAIRS,
        seed=SEED,
        method="branch_benders_$(DECOMPOSITION_NAME)_$(CUT_DERIVATION)_mcf",
        mcf_lower_bound_mode=string(MCF_LOWER_BOUND_MODE),
        mcf_scenario_id=isnothing(MCF_SCENARIO_ID) ? missing : MCF_SCENARIO_ID,
        mcf_selected_scenario_id=metadata_value(result, "branch_benders_mcf_selected_scenario_id"),
        mcf_common_od_count=metadata_value(result, "branch_benders_mcf_common_od_count"),
        projected_mcf_separations=metadata_value(result, "branch_benders_projected_mcf_separations"),
        projected_mcf_cuts=metadata_value(result, "branch_benders_projected_mcf_cuts"),
        projected_mcf_seconds=metadata_value(result, "branch_benders_projected_mcf_seconds"),
        cut_derivation=string(CUT_DERIVATION),
        max_stops_mode=string(MAX_STOPS_MODE),
        max_stops=MAX_STOPS,
        status=isnothing(result) ? "error" : "ok",
        error=failure,
        termination_status=isnothing(result) ? "" : string(result.termination_status),
        objective_value=isnothing(result) ? missing : result.objective_value,
        certified_ub=metadata_value(result, "branch_benders_certified_ub"),
        global_lb=metadata_value(result, "branch_benders_global_lb"),
        relative_gap=metadata_value(result, "branch_benders_gap_relative"),
        node_count=metadata_value(result, "branch_benders_node_count"),
        result_runtime_sec=isnothing(result) ? missing : result.runtime_sec,
        pre_optimize_seconds=metadata_value(result, "branch_benders_pre_optimize_seconds"),
        master_optimize_seconds=metadata_value(result, "branch_benders_master_optimize_seconds"),
        run_opt_seconds=metadata_value(result, "branch_benders_run_opt_seconds"),
        unique_y=metadata_value(result, "branch_benders_unique_y"),
        callback_count=metadata_value(result, "branch_benders_callback_count"),
        cache_hits=metadata_value(result, "branch_benders_cache_hits"),
        cuts_submitted=metadata_value(result, "branch_benders_cuts_submitted"),
        repeated_submissions=metadata_value(result, "branch_benders_repeated_submissions"),
        oracle_seconds=metadata_value(result, "branch_benders_oracle_seconds"),
        priming_cg_seconds=metadata_value(result, "branch_benders_priming_cg_seconds"),
        repricing_seconds=metadata_value(result, "branch_benders_repricing_seconds"),
        mw_completion_seconds=metadata_value(result, "branch_benders_mw_completion_seconds"),
        reprice_rounds=metadata_value(result, "branch_benders_reprice_rounds"),
        reprice_columns=metadata_value(result, "branch_benders_reprice_columns"),
        shared_pool_size=metadata_value(result, "branch_benders_shared_pool_size"),
        walking_cost=metadata_value(result, "branch_benders_walking_cost"),
        recourse_by_scenario=string(metadata_value(result, "branch_benders_recourse_by_scenario")),
        selected_stations=string(selected_stations),
        n_stations_selected=length(selected_stations),
        wall_time_sec=wall_time,
        log_dir=LOG_DIR,
    )
    CSV.write(SUMMARY_PATH, DataFrame([summary]))
    println("Wrote $SUMMARY_PATH")
    println("status=$(summary.status) termination=$(summary.termination_status) " *
            "objective=$(summary.objective_value) wall=$(round(wall_time; digits=1))s")
    flush(stdout)
    flush(stderr)
    isempty(failure) || exit(1)
end

main()
