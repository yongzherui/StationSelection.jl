"""
    scripts/compare_benders_decompositions.jl

Solve one Zhuzhou AggregateODRouteModel instance with BendersY, BendersYZ, and/or BendersYZH
(NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)) and compare convergence behavior.

`reprice_subproblem=true` is passed to all three decompositions. It is required for BendersY and
BendersYZ to be provably optimal (see notes/2026-07-15_bendersy_stale_cut_soundness.md and
notes/2026-07-21_bendersyz_yzh_verification_gaps.md), and is also useful for BendersYZH because
the earlier "exact without repricing" argument is unproven in the presence of subproblem dual
degeneracy. This keeps the comparison apples-to-apples on the objective while still isolating
convergence-rate differences between the three master formulations.

`BendersY`'s repricing can hit real dual degeneracy and burn many repricing rounds (each up to
`CS_INNER_PRICING_TIME` seconds) per cut group per master iteration, making it far slower than
`BendersYZ`/`BendersYZH` on the same instance -- so the `<decomposition>` argument lets a SLURM
array key each task by (instance, decomposition), keeping a slow BendersY run from blocking or
sharing a walltime budget with the fast ones.

Usage:
    julia --project=. scripts/compare_benders_decompositions.jl \\
        <base_outdir> <data_dir> <n_stations> <l> <n_pairs> <endpoint_overlap> <seed> \\
        [decomposition]

    decomposition: "BendersY" | "BendersYZ" | "BendersYZH" | "all" (default "all")

Output (per instance, per decomposition):
    <base_outdir>/iters/<instance>_<decomp>_iters.csv       -- one row per Benders master
                                                                iteration (lower_bound,
                                                                incumbent_objective, outer_gap,
                                                                timings, cuts_added, ...)
    <base_outdir>/results/<instance>_<decomp-or-all>_summary.csv
                                                             -- one row per decomposition run.
                                                                Named per (instance, decomposition)
                                                                rather than shared per instance,
                                                                so concurrent SLURM tasks for the
                                                                same instance never race on the
                                                                same file; concatenate across files
                                                                for analysis.

Environment variables (with defaults):
    CS_REPRICE_SUBPROBLEM     = true    pass reprice_subproblem to BendersSolver. Required for
                                         BendersY/BendersYZ to be provably optimal -- set to
                                         false only for a deliberate diagnostic run to see how
                                         much repricing costs and whether its absence actually
                                         changes the objective on a given instance (tally the
                                         resulting "_noreprice" summary against a repriced run,
                                         e.g. BendersYZH with repricing enabled).
    CS_BENDERS_MAX_ITERS      = 100     outer Benders master iteration cap, per decomposition
    CS_BENDERS_TIME_LIMIT     = 6000    wall-clock budget per decomposition (seconds)
    CS_INNER_CG_MAX_ITERS     = 200     inner column-generation iteration cap (priming CG)
    CS_INNER_PRICING_TIME     = 30      inner CG per-iteration pricing time limit (seconds)
    CS_INNER_IP_TIME_LIMIT    = 30      inner CG final MIP time limit (seconds)
    CS_MAX_REPRICE_ROUNDS     = 10000   safety cap on repricing rounds per subproblem LP
    CS_N_SCENARIOS            = 3
    CS_MAX_WALKING_DISTANCE   = 600     walking distance cap (seconds)
    CS_MAX_WAIT_TIME          = 900     vehicle wait budget from depot (seconds)
    CS_DETOUR_FACTOR          = 2.0     allowed detour ratio over direct route time
    CS_MAX_STOPS              = ""      max stops per route; default = n_stations (uncapped)
    CS_ROUTE_REG_WEIGHT       = 1.0     route travel-time penalty weight
    CS_REPOSITIONING_TIME     = 20.0    fixed cost added to every column (seconds)
"""

using CSV, DataFrames, Dates, Printf, Gurobi, JuMP, StationSelection

const _RUN_AS_MAIN = abspath(PROGRAM_FILE) == @__FILE__

if _RUN_AS_MAIN
    length(ARGS) >= 7 || error(
        "Usage: compare_benders_decompositions.jl <base_outdir> <data_dir> " *
        "<n_stations> <l> <n_pairs> <endpoint_overlap> <seed>"
    )

    const BASE_OUTDIR      = ARGS[1]
    const DATA_DIR         = ARGS[2]
    const N_STATIONS       = parse(Int,     ARGS[3])
    const L                = parse(Int,     ARGS[4])
    const N_PAIRS          = parse(Int,     ARGS[5])
    const ENDPOINT_OVERLAP = parse(Float64, ARGS[6])
    const SEED             = parse(Int,     ARGS[7])
    const DECOMP_ARG       = length(ARGS) >= 8 ? ARGS[8] : "all"
    DECOMP_ARG in ("all", "BendersY", "BendersYZ", "BendersYZH") || error(
        "decomposition must be one of: all, BendersY, BendersYZ, BendersYZH (got $DECOMP_ARG)"
    )

    const REPRICE_SUBPROBLEM  = parse(Bool,    get(ENV, "CS_REPRICE_SUBPROBLEM",  "true"))
    const BENDERS_MAX_ITERS   = parse(Int,     get(ENV, "CS_BENDERS_MAX_ITERS",   "100"))
    const BENDERS_TIME_LIMIT  = parse(Float64, get(ENV, "CS_BENDERS_TIME_LIMIT",  "6000"))
    const INNER_CG_MAX_ITERS  = parse(Int,     get(ENV, "CS_INNER_CG_MAX_ITERS",  "200"))
    const INNER_PRICING_TIME  = parse(Float64, get(ENV, "CS_INNER_PRICING_TIME",  "30"))
    const INNER_IP_TIME_LIMIT = parse(Float64, get(ENV, "CS_INNER_IP_TIME_LIMIT", "30"))
    const MAX_REPRICE_ROUNDS  = parse(Int,     get(ENV, "CS_MAX_REPRICE_ROUNDS",  "10000"))
    const N_SCENARIOS         = parse(Int,     get(ENV, "CS_N_SCENARIOS",         "3"))
    const MAX_WALK_ENV        = get(ENV, "CS_MAX_WALKING_DISTANCE", "600")
    const MAX_WAIT_TIME       = parse(Float64, get(ENV, "CS_MAX_WAIT_TIME",       "900"))
    const DETOUR_FACTOR       = parse(Float64, get(ENV, "CS_DETOUR_FACTOR",       "2.0"))
    const MAX_STOPS_ENV       = get(ENV, "CS_MAX_STOPS", "")
    const ROUTE_REG_WEIGHT    = parse(Float64, get(ENV, "CS_ROUTE_REG_WEIGHT",    "1.0"))
    const REPOSITIONING_TIME  = parse(Float64, get(ENV, "CS_REPOSITIONING_TIME",  "20.0"))

    ov_str = replace(string(ENDPOINT_OVERLAP), "." => "p")
    # Suffix is empty (unchanged path) when repricing is on -- the default and the setting
    # every correctness-relevant run uses -- so this only ever affects the opt-in
    # reprice_subproblem=false diagnostic runs, never the paths existing repriced jobs write to.
    const REPRICE_SUFFIX = REPRICE_SUBPROBLEM ? "" : "_noreprice"
    const INST_NAME    = "zz_n$(N_STATIONS)_l$(L)_p$(N_PAIRS)_ov$(ov_str)_s$(SEED)"
    const ITERS_DIR    = joinpath(BASE_OUTDIR, "iters")
    const RESULTS_DIR  = joinpath(BASE_OUTDIR, "results")
    const SUMMARY_PATH = joinpath(RESULTS_DIR, "$(INST_NAME)_$(DECOMP_ARG)$(REPRICE_SUFFIX)_summary.csv")
end

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DECOMPOSITIONS = [
    ("BendersY",   StationSelection.BendersY()),
    ("BendersYZ",  StationSelection.BendersYZ()),
    ("BendersYZH", StationSelection.BendersYZH()),
]

function _zz_resolve_max_walking_distance(data::StationSelectionData, env_val::String)::Float64
    env_val == "auto" && return maximum(
        data.walking_costs[i, j]
        for i in 1:data.n_stations, j in 1:data.n_stations
        if i != j && isfinite(data.walking_costs[i, j]);
        init = 0.0,
    )
    return parse(Float64, env_val)
end

function _last_iteration_row(log_path::String)
    isfile(log_path) || return nothing
    df = CSV.read(log_path, DataFrame)
    nrow(df) == 0 && return nothing
    return df[end, :]
end

function run_one_decomposition(
    label::String,
    decomposition,
    data::StationSelectionData,
    model::AggregateODRouteModel,
    inst_name::String,
)
    println("--- $label$(REPRICE_SUFFIX) ---")
    log_dir = joinpath(ITERS_DIR, "$(inst_name)_$(label)$(REPRICE_SUFFIX)")
    mkpath(log_dir)

    inner_cg = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        max_iterations=INNER_CG_MAX_ITERS, max_columns_per_iteration=20, n_candidates=20,
        pricing_time_limit_sec=INNER_PRICING_TIME, final_ip_time_limit_sec=INNER_IP_TIME_LIMIT,
    )
    solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        decomposition=decomposition,
        inner_solver=inner_cg,
        max_iterations=BENDERS_MAX_ITERS,
        log_dir=log_dir,
        reprice_subproblem=REPRICE_SUBPROBLEM,
        max_reprice_rounds=MAX_REPRICE_ROUNDS,
    )

    t0 = time()
    status = "ok"
    result = nothing
    try
        result = StationSelection.run_opt(data, model, solver)
    catch err
        status = "error: $(sprint(showerror, err))"
        @warn "$label failed" exception=(err, catch_backtrace())
    end
    wall_time = time() - t0

    log_path = joinpath(log_dir, "aggregate_od_route_benders_iterations.csv")
    last_row = _last_iteration_row(log_path)
    n_iterations = isnothing(last_row) ? 0 : last_row.iteration

    summary = (
        instance             = inst_name,
        decomposition        = label,
        status               = status,
        termination_status   = isnothing(result) ? "" : string(result.termination_status),
        objective_value      = isnothing(result) || isnothing(result.objective_value) ? "" : string(result.objective_value),
        wall_time_sec        = wall_time,
        n_iterations         = n_iterations,
        final_lower_bound    = isnothing(last_row) ? "" : string(last_row.lower_bound),
        final_outer_gap      = isnothing(last_row) ? "" : string(last_row.outer_gap),
        optimality_cuts_added = isnothing(last_row) ? "" : string(last_row.optimality_cuts_added),
        inner_cg_iterations  = isnothing(last_row) ? "" : string(last_row.inner_cg_iterations),
        iters_log_path       = log_path,
    )

    @printf("  status=%s  obj=%s  wall=%.1fs  iters=%d\n",
            status, summary.objective_value, wall_time, n_iterations)
    return summary
end

function main()
    mkpath.((ITERS_DIR, RESULTS_DIR))

    println("===========================================")
    println("Benders Decomposition Comparison - Zhuzhou")
    println("===========================================")
    @printf("  Instance         : %s\n", INST_NAME)
    @printf("  n_stations       : %d\n", N_STATIONS)
    @printf("  l (to build)     : %d\n", L)
    @printf("  n_pairs          : %d\n", N_PAIRS)
    @printf("  endpoint_overlap : %.2f\n", ENDPOINT_OVERLAP)
    @printf("  Seed             : %d\n", SEED)
    @printf("  Decomposition(s) : %s\n", DECOMP_ARG)
    @printf("  reprice_subproblem: %s\n", REPRICE_SUBPROBLEM)
    @printf("  Benders max iters: %d  (per decomposition)\n", BENDERS_MAX_ITERS)
    println()

    data, meta = generate_zhuzhou_data(
        DATA_DIR, N_STATIONS, N_PAIRS;
        n_scenarios=N_SCENARIOS, endpoint_overlap=ENDPOINT_OVERLAP, seed=SEED,
    )
    print_zhuzhou_data_summary(data, meta)

    L <= data.n_stations || error("l=$L exceeds n_stations=$(data.n_stations)")
    max_walking_distance = _zz_resolve_max_walking_distance(data, MAX_WALK_ENV)
    max_stops = isempty(MAX_STOPS_ENV) ? data.n_stations : parse(Int, MAX_STOPS_ENV)

    model = AggregateODRouteModel(
        L;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
        route_regularization_weight=ROUTE_REG_WEIGHT,
        repositioning_time=REPOSITIONING_TIME,
        max_walking_distance=max_walking_distance,
        max_wait_time=MAX_WAIT_TIME,
        detour_factor=DETOUR_FACTOR,
        max_stops=max_stops,
        max_visits_per_node=2,
        allow_walk_only=true,
    )

    selected_decompositions = DECOMP_ARG == "all" ?
        DECOMPOSITIONS : filter(((label, _),) -> label == DECOMP_ARG, DECOMPOSITIONS)

    rows = NamedTuple[]
    for (label, decomposition) in selected_decompositions
        row = run_one_decomposition(label, decomposition, data, model, INST_NAME)
        push!(rows, row)
    end

    summary_df = DataFrame(rows)
    CSV.write(SUMMARY_PATH, summary_df)
    println()
    println("Written: $SUMMARY_PATH")

    objs = [r.objective_value for r in rows if r.status == "ok" && r.objective_value != ""]
    parsed = parse.(Float64, objs)
    if length(parsed) >= 2 && (maximum(parsed) - minimum(parsed)) > 1e-4 * max(1.0, maximum(abs, parsed))
        @warn "Objective mismatch across decompositions on $INST_NAME -- correctness check failed" parsed
    end
end

if _RUN_AS_MAIN
    main()
end
