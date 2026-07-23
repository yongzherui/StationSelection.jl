"""
    scripts/run_method_compare_task.jl

Solve ONE (instance, method) pair from the full AggregateODRouteModel method
comparison grid (Direct solve / plain Column Generation / Benders Y,YZ,YZH
variants -- see aggregate_od_route_method_grid.jl for the method list) and
write a single summary row. Designed to be called as one task of a single
SLURM job array keyed by (instance, method) -- see sbatch_method_compare.sh /
submit_method_compare.sh.

Usage:
    julia --project=. scripts/run_method_compare_task.jl \\
        <base_outdir> <data_dir> <family> <n_stations> <l> <n_pairs> <seed> <method_label>

Output:
    <base_outdir>/results/<instance>__<method_label>.csv   -- one summary row
    <base_outdir>/iters/<instance>__<method_label>/...     -- Benders/CG iteration logs (if applicable)

Environment variables (all optional; defaults mirror
scripts/compare_benders_decompositions.jl and scripts/run_single_instance.jl):
    CS_MIP_GAP             = 1e-4    mip_gap for every MIP solve (final IP / master / direct);
                                      matches the relative tolerance analyze_method_compare.jl
                                      uses to flag objective disagreement across methods
    CS_BENDERS_MAX_ITERS   = 500     outer Benders master iteration cap; raised from 300 -- the
                                      n=10/n=15 batches showed several methods hitting the 300 cap
                                      exactly (particularly on grid n=15, which also had a 46.8%
                                      failure rate dominated by "did not converge within
                                      max_iterations"), so 500 gives more room before concluding a
                                      run is genuinely non-convergent vs. just iteration-starved
    CS_MAX_REPRICE_ROUNDS  = 30      cap on repricing rounds per subproblem LP
    CS_INNER_CG_MAX_ITERS  = 200     inner (priming) column-generation iteration cap
    CS_INNER_PRICING_TIME  = 120     inner CG per-iteration pricing time limit (seconds)
    CS_INNER_IP_TIME_LIMIT = 60      inner CG final MIP time limit (seconds)
    CS_CG_MAX_ITERS        = 10000   plain (non-Benders) CG iteration cap
    CS_CG_PRICING_TIME     = 120     plain CG per-iteration pricing time limit (seconds); raised
                                      from 30s since the labeling search can legitimately need
                                      more time to exhaust (or prove no improving column exists)
                                      once max_visits_per_node is unrestricted
    CS_CG_IP_TIME_LIMIT    = 300     plain CG final MIP time limit (seconds)
    CS_DIRECT_MAX_ROUTES   = 50000   max enumerated routes for Direct solve
    CS_DIRECT_TIME_LIMIT   = 300     max enumeration wall time for Direct solve (seconds)
    CS_DETOUR_FACTOR       = 2.0     allowed detour ratio over direct route time
    CS_MAX_WAIT_TIME       = 900     vehicle wait budget from depot (seconds)
    CS_ROUTE_REG_WEIGHT    = 10.0    route_regularization_weight: route travel-time penalty
                                      weight ("route weight")
    CS_WALK_COST_WEIGHT    = 0.1     walk_cost_weight: multiplies the walking-cost term in the
                                      AggregateODRouteModel objective ("walk weight") -- applied
                                      at the model/code level (every objective, subproblem, and
                                      MW-cut dual-completion LP that reads walking cost), not by
                                      rescaling the input data. Route weight is 100x this by
                                      default (10.0 / 0.1).
    CS_REPOSITIONING_TIME  = 20.0    fixed cost added to every column (seconds)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

const _RUN_AS_MAIN = abspath(PROGRAM_FILE) == @__FILE__

include(joinpath(@__DIR__, "aggregate_od_route_method_grid.jl"))

if _RUN_AS_MAIN
    length(ARGS) >= 8 || error(
        "Usage: run_method_compare_task.jl <base_outdir> <data_dir> <family> " *
        "<n_stations> <l> <n_pairs> <seed> <method_label>"
    )

    const BASE_OUTDIR  = ARGS[1]
    const DATA_DIR     = ARGS[2]
    const FAMILY       = ARGS[3]
    const N_STATIONS   = parse(Int, ARGS[4])
    const L            = parse(Int, ARGS[5])
    const N_PAIRS      = parse(Int, ARGS[6])
    const SEED         = parse(Int, ARGS[7])
    const METHOD_LABEL = ARGS[8]

    const CFG = (
        # 1e-4 matches the relative-mismatch tolerance analyze_method_compare.jl uses
        # to flag objective disagreement across methods, so a genuine discrepancy
        # (not just this gap) is what would trip that check.
        mip_gap             = parse(Float64, get(ENV, "CS_MIP_GAP",             "1e-4")),
        benders_max_iters   = parse(Int,     get(ENV, "CS_BENDERS_MAX_ITERS",   "500")),
        max_reprice_rounds  = parse(Int,     get(ENV, "CS_MAX_REPRICE_ROUNDS",  "30")),
        inner_cg_max_iters  = parse(Int,     get(ENV, "CS_INNER_CG_MAX_ITERS",  "200")),
        inner_pricing_time  = parse(Float64, get(ENV, "CS_INNER_PRICING_TIME",  "120")),
        inner_ip_time_limit = parse(Float64, get(ENV, "CS_INNER_IP_TIME_LIMIT", "60")),
        cg_max_iters        = parse(Int,     get(ENV, "CS_CG_MAX_ITERS",        "10000")),
        cg_pricing_time     = parse(Float64, get(ENV, "CS_CG_PRICING_TIME",     "120")),
        cg_ip_time_limit    = parse(Float64, get(ENV, "CS_CG_IP_TIME_LIMIT",    "300")),
        direct_max_routes   = parse(Int,     get(ENV, "CS_DIRECT_MAX_ROUTES",   "50000")),
        direct_time_limit   = parse(Float64, get(ENV, "CS_DIRECT_TIME_LIMIT",   "300")),
        detour_factor       = parse(Float64, get(ENV, "CS_DETOUR_FACTOR",       "2.0")),
        max_wait_time       = parse(Float64, get(ENV, "CS_MAX_WAIT_TIME",       "900")),
        route_reg_weight    = parse(Float64, get(ENV, "CS_ROUTE_REG_WEIGHT",    "10.0")),
        walk_cost_weight    = parse(Float64, get(ENV, "CS_WALK_COST_WEIGHT",    "0.1")),
        repositioning_time  = parse(Float64, get(ENV, "CS_REPOSITIONING_TIME",  "20.0")),
    )

    const INST_NAME    = "$(FAMILY)_n$(N_STATIONS)_p$(N_PAIRS)_s$(SEED)"
    const RESULTS_DIR  = joinpath(BASE_OUTDIR, "results")
    const ITERS_DIR    = joinpath(BASE_OUTDIR, "iters")
    const SUMMARY_PATH = joinpath(RESULTS_DIR, "$(INST_NAME)__$(METHOD_LABEL).csv")
end

function _last_iteration_row(log_path::String)
    isfile(log_path) || return nothing
    df = CSV.read(log_path, DataFrame)
    nrow(df) == 0 && return nothing
    return df[end, :]
end

function build_model(l::Int, max_stops::Int, max_walk::Float64, cfg::NamedTuple)
    # allow_walk_only left at its default (false) and unmet_demand_penalty left
    # unset (nothing): the zero_completion / restricted_mw_fixed_pi cut
    # derivations only apply under allow_walk_only=false, unmet_demand_penalty
    # === nothing (see BendersSolver docs) -- and every method in METHODS must
    # solve the SAME model for the objective-value comparison to be meaningful,
    # so this can't vary by method.
    #
    # max_visits_per_node is also left at the model's own default (typemax(Int),
    # unrestricted) rather than capped: capping it forces every label to track a
    # per-node visit count as part of its dominance signature, which -- exactly
    # like the bounded_max_stops case (see resolve_max_stops) -- makes labels
    # LESS comparable and shrinks pruning, not more. That was making cg_uncapped's
    # pricing search blow up rather than helping bound it.
    return AggregateODRouteModel(
        l;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
        route_regularization_weight=cfg.route_reg_weight,
        walk_cost_weight=cfg.walk_cost_weight,
        repositioning_time=cfg.repositioning_time,
        max_walking_distance=max_walk,
        max_wait_time=cfg.max_wait_time,
        detour_factor=cfg.detour_factor,
        max_stops=max_stops,
    )
end

function build_solver(method::MethodSpec, cfg::NamedTuple, log_dir::String)
    config = SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=cfg.mip_gap)

    if method.kind == :direct
        return DirectSolver(
            config=config,
            max_enumerated_routes=cfg.direct_max_routes,
            max_enumeration_time_sec=cfg.direct_time_limit,
        )
    elseif method.kind == :cg
        return ColumnGenerationSolver(
            config=config,
            max_iterations=cfg.cg_max_iters,
            max_columns_per_iteration=20,
            n_candidates=20,
            pricing_time_limit_sec=cfg.cg_pricing_time,
            final_ip_time_limit_sec=cfg.cg_ip_time_limit,
            log_dir=log_dir,
        )
    elseif method.kind == :benders
        inner_cg = ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=cfg.mip_gap),
            max_iterations=cfg.inner_cg_max_iters,
            max_columns_per_iteration=20,
            n_candidates=20,
            pricing_time_limit_sec=cfg.inner_pricing_time,
            final_ip_time_limit_sec=cfg.inner_ip_time_limit,
        )
        return BendersSolver(
            config=config,
            decomposition=method.decomposition,
            inner_solver=inner_cg,
            max_iterations=cfg.benders_max_iters,
            log_dir=log_dir,
            reprice_subproblem=method.reprice,
            max_reprice_rounds=cfg.max_reprice_rounds,
            cut_derivation=method.cut_derivation,
        )
    else
        error("unknown method.kind=$(method.kind)")
    end
end

function main()
    mkpath.((RESULTS_DIR, ITERS_DIR))

    method = method_by_label(METHOD_LABEL)
    max_stops = resolve_max_stops(method.max_stops_mode, N_STATIONS)

    println("===========================================")
    println("AggregateODRouteModel Method Comparison")
    println("===========================================")
    @printf("  Instance   : %s\n", INST_NAME)
    @printf("  Family     : %s\n", FAMILY)
    @printf("  n_stations : %d   l : %d   n_pairs : %d   seed : %d\n", N_STATIONS, L, N_PAIRS, SEED)
    @printf("  Method     : %s  (kind=%s max_stops=%d)\n", METHOD_LABEL, method.kind, max_stops)
    println()
    flush(stdout)  # stdout is block-buffered under sbatch's file redirection, so without this
                    # the above header (and Gurobi's own unbuffered C-level output right after)
                    # can appear wildly out of order in the log, making a slow-but-alive run look
                    # like it hung before even starting.

    data, max_walk = build_instance(FAMILY, N_STATIONS, N_PAIRS, SEED, DATA_DIR)
    L <= data.n_stations || error("l=$L exceeds n_stations=$(data.n_stations)")
    model = build_model(L, max_stops, max_walk, CFG)

    log_dir = joinpath(ITERS_DIR, "$(INST_NAME)__$(METHOD_LABEL)")
    mkpath(log_dir)
    solver = build_solver(method, CFG, log_dir)
    flush(stdout)

    t0 = time()
    status = "ok"
    result = nothing
    try
        result = StationSelection.run_opt(data, model, solver)
    catch err
        status = "error: $(sprint(showerror, err))"
        @warn "$METHOD_LABEL failed on $INST_NAME" exception=(err, catch_backtrace())
    end
    wall_time = time() - t0

    n_iterations = ""
    final_lower_bound = ""
    final_outer_gap = ""
    if method.kind == :benders
        last_row = _last_iteration_row(joinpath(log_dir, "aggregate_od_route_benders_iterations.csv"))
        if !isnothing(last_row)
            n_iterations = string(last_row.iteration)
            final_lower_bound = string(last_row.lower_bound)
            final_outer_gap = string(last_row.outer_gap)
        end
    elseif method.kind == :cg
        last_row = _last_iteration_row(joinpath(log_dir, "aggregate_od_route_cg_iterations.csv"))
        !isnothing(last_row) && (n_iterations = string(last_row.iteration))
    end

    summary = (
        instance           = INST_NAME,
        family             = FAMILY,
        n_stations         = N_STATIONS,
        l                  = L,
        n_pairs            = N_PAIRS,
        seed               = SEED,
        method             = METHOD_LABEL,
        kind               = string(method.kind),
        decomposition      = method.kind == :benders ? string(typeof(method.decomposition)) : "",
        cut_derivation     = string(method.cut_derivation),
        reprice_subproblem = method.reprice,
        max_stops_mode     = string(method.max_stops_mode),
        max_stops          = max_stops,
        status             = status,
        termination_status = isnothing(result) ? "" : string(result.termination_status),
        objective_value    = isnothing(result) || isnothing(result.objective_value) ? "" : string(result.objective_value),
        wall_time_sec      = wall_time,
        runtime_sec        = isnothing(result) ? "" : string(result.runtime_sec),
        n_iterations       = n_iterations,
        final_lower_bound  = final_lower_bound,
        final_outer_gap    = final_outer_gap,
        iters_log_path     = log_dir,
    )

    CSV.write(SUMMARY_PATH, DataFrame([summary]))
    @printf("  status=%s  obj=%s  wall=%.1fs  iters=%s\n",
            status, summary.objective_value, wall_time, isempty(n_iterations) ? "n/a" : n_iterations)
    println("Written: $SUMMARY_PATH")
end

if _RUN_AS_MAIN
    main()
end
