"""
    scripts/run_zhuzhou_instance.jl

Solve one Zhuzhou AggregateODRouteModel instance and write result CSVs.

Usage:
    julia --project=. scripts/run_zhuzhou_instance.jl \\
        <base_outdir> <data_dir> <n_stations> <l> <n_pairs> <endpoint_overlap> <seed>

Arguments:
    base_outdir       output root directory
    data_dir          path to base_data/ directory with station_request_counts.csv,
                      segment.csv, order.csv
    n_stations        number of stations to load (top-N by request volume)
    l                 stations to build (first-stage budget)
    n_pairs           distinct OD demand pairs to sample
    endpoint_overlap  Zipf exponent for demand concentration (higher → more hub-like)
    seed              random seed for OD pair sampling

Output:
    <base_outdir>/results/<instance>.csv
    <base_outdir>/iters/<instance>_iters.csv
    <base_outdir>/columns/<instance>_columns.csv
    <base_outdir>/duals/<instance>_duals.csv
    <base_outdir>/selected/<instance>_selected.csv

Environment variables (with defaults):
    CS_TIME_LIMIT          = 10800   overall CG wall-time budget (seconds)
    CS_PRICING_TIME        = 300     per-iteration pricing time limit (seconds)
    CS_MAX_CG_ITERS        = 10000
    CS_MAX_NEW_COLS        = 20
    CS_IP_TIME_LIMIT       = 1200    final MIP time limit (seconds)
    CS_MIP_GAP             = 1e-4
    CS_MAX_WALKING_DISTANCE = 600    walking distance cap (seconds); set to "auto"
                                     to use the max pairwise walking time (can be large)
    CS_MAX_WAIT_TIME       = 900     vehicle wait budget from depot (seconds)
    CS_DETOUR_FACTOR       = 2.0     allowed detour ratio over direct route time
    CS_MAX_STOPS           = ""      max stops per route; default = n_stations (uncapped)
    CS_ROUTE_REG_WEIGHT    = 1.0     μ: route travel-time penalty weight
    CS_REPOSITIONING_TIME  = 20.0    ρ: fixed cost added to every column (seconds)
"""

using CSV, DataFrames, Dates, Printf, StationSelection

const _RUN_AS_MAIN = abspath(PROGRAM_FILE) == @__FILE__

if _RUN_AS_MAIN
    length(ARGS) >= 7 || error(
        "Usage: run_zhuzhou_instance.jl <base_outdir> <data_dir> " *
        "<n_stations> <l> <n_pairs> <endpoint_overlap> <seed>"
    )

    const BASE_OUTDIR      = ARGS[1]
    const DATA_DIR         = ARGS[2]
    const N_STATIONS       = parse(Int,     ARGS[3])
    const L                = parse(Int,     ARGS[4])
    const N_PAIRS          = parse(Int,     ARGS[5])
    const ENDPOINT_OVERLAP = parse(Float64, ARGS[6])
    const SEED             = parse(Int,     ARGS[7])

    const TIME_LIMIT         = parse(Float64, get(ENV, "CS_TIME_LIMIT",         "10800"))
    const PRICING_TIME       = parse(Float64, get(ENV, "CS_PRICING_TIME",       "300"))
    const MAX_CG_ITERS       = parse(Int,     get(ENV, "CS_MAX_CG_ITERS",       "10000"))
    const MAX_NEW_COLS       = parse(Int,     get(ENV, "CS_MAX_NEW_COLS",        "20"))
    const IP_TIME_LIMIT      = parse(Float64, get(ENV, "CS_IP_TIME_LIMIT",      "1200"))
    const MIP_GAP            = parse(Float64, get(ENV, "CS_MIP_GAP",            "1e-4"))
    const N_SCENARIOS        = parse(Int,     get(ENV, "CS_N_SCENARIOS",         "3"))
    const MAX_WALK_ENV       = get(ENV, "CS_MAX_WALKING_DISTANCE", "600")
    const MAX_WAIT_TIME      = parse(Float64, get(ENV, "CS_MAX_WAIT_TIME",      "900"))
    const DETOUR_FACTOR      = parse(Float64, get(ENV, "CS_DETOUR_FACTOR",      "2.0"))
    const MAX_STOPS_ENV      = get(ENV, "CS_MAX_STOPS", "")
    const ROUTE_REG_WEIGHT   = parse(Float64, get(ENV, "CS_ROUTE_REG_WEIGHT",   "1.0"))
    const REPOSITIONING_TIME = parse(Float64, get(ENV, "CS_REPOSITIONING_TIME", "20.0"))
    ov_str = replace(string(ENDPOINT_OVERLAP), "." => "p")
    const INST_NAME    = "zz_n$(N_STATIONS)_l$(L)_p$(N_PAIRS)_ov$(ov_str)_s$(SEED)"
    const RESULTS_DIR  = joinpath(BASE_OUTDIR, "results")
    const ITERS_DIR    = joinpath(BASE_OUTDIR, "iters")
    const COLUMNS_DIR  = joinpath(BASE_OUTDIR, "columns")
    const DUALS_DIR    = joinpath(BASE_OUTDIR, "duals")
    const SELECTED_DIR = joinpath(BASE_OUTDIR, "selected")
    const CSV_PATH      = joinpath(RESULTS_DIR,  "$(INST_NAME).csv")
    const ITER_PATH     = joinpath(ITERS_DIR,    "$(INST_NAME)_iters.csv")
    const COLUMN_PATH   = joinpath(COLUMNS_DIR,  "$(INST_NAME)_columns.csv")
    const DUAL_PATH     = joinpath(DUALS_DIR,    "$(INST_NAME)_duals.csv")
    const SELECTED_PATH = joinpath(SELECTED_DIR, "$(INST_NAME)_selected.csv")
end

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

function _zz_resolve_max_walking_distance(
    data     :: StationSelectionData,
    env_val  :: String,
)::Float64
    env_val == "auto" && return maximum(
        data.walking_costs[i, j]
        for i in 1:data.n_stations, j in 1:data.n_stations
        if i != j && isfinite(data.walking_costs[i, j]);
        init = 0.0,
    )
    return parse(Float64, env_val)
end

function _write_namedtuple_rows(path::AbstractString, rows::Vector{<:NamedTuple})
    df = isempty(rows) ? DataFrame() : DataFrame(rows)
    CSV.write(path, df)
end

function _write_summary(path::AbstractString, row::NamedTuple)
    CSV.write(path, DataFrame([row]))
end

function _write_selected_columns(
    path   :: AbstractString,
    result :: AggregateODRouteColumnGenerationResult,
)
    column_by_id = Dict(col.id => col for col in result.final_result.mapping.columns)
    rows = NamedTuple[]
    for column_id in result.selected_column_ids
        col = get(column_by_id, column_id, nothing)
        col === nothing && continue
        push!(rows, (
            column_id = col.id,
            n_pairs   = length(col.od_pairs),
            tau       = col.tau,
            metadata  = string(col.metadata),
            pairs     = string(Tuple(col.od_pairs)),
        ))
    end
    _write_namedtuple_rows(path, rows)
end

function main()
    mkpath.((RESULTS_DIR, ITERS_DIR, COLUMNS_DIR, DUALS_DIR, SELECTED_DIR))

    if isfile(CSV_PATH) && countlines(CSV_PATH) >= 2
        println("=== Skipping $INST_NAME — result already exists ===")
        return
    end

    println("===========================================")
    println("AggregateODRouteModel — Zhuzhou Experiment")
    println("===========================================")
    @printf("  Instance         : %s\n", INST_NAME)
    @printf("  n_stations       : %d\n", N_STATIONS)
    @printf("  l (to build)     : %d\n", L)
    @printf("  n_pairs          : %d\n", N_PAIRS)
    @printf("  endpoint_overlap : %.2f\n", ENDPOINT_OVERLAP)
    @printf("  Seed             : %d\n", SEED)
    @printf("  Model type       : %s\n", "AggregateODRouteModel")
    @printf("  Time limit       : %.0fs CG / %.0fs IP / %.0fs per pricing iter\n",
            TIME_LIMIT, IP_TIME_LIMIT, PRICING_TIME)
    @printf("  Detour factor    : %.2f\n", DETOUR_FACTOR)
    @printf("  Max wait time    : %.1fs\n", MAX_WAIT_TIME)
    println()

    data, meta = generate_zhuzhou_data(
        DATA_DIR, N_STATIONS, N_PAIRS;
        n_scenarios      = N_SCENARIOS,
        endpoint_overlap = ENDPOINT_OVERLAP,
        seed             = SEED,
    )
    print_zhuzhou_data_summary(data, meta)

    L <= data.n_stations || error("l=$L exceeds n_stations=$(data.n_stations)")
    max_walking_distance = _zz_resolve_max_walking_distance(data, MAX_WALK_ENV)
    max_stops = isempty(MAX_STOPS_ENV) ? data.n_stations : parse(Int, MAX_STOPS_ENV)
    @printf("  Max walk dist    : %.1fs\n", max_walking_distance)
    @printf("  Max stops        : %d\n", max_stops)
    println()

    _model_kwargs = (
        route_regularization_weight = ROUTE_REG_WEIGHT,
        repositioning_time          = REPOSITIONING_TIME,
        max_walking_distance        = max_walking_distance,
        max_wait_time               = MAX_WAIT_TIME,
        detour_factor               = DETOUR_FACTOR,
        max_stops                   = max_stops,
        max_visits_per_node         = 2,
        max_new_columns             = MAX_NEW_COLS,
        n_candidates                = max(MAX_NEW_COLS, 20),
        pricing_time_limit_sec      = PRICING_TIME,
        relax_integrality           = false,
    )
    model = AggregateODRouteModel(L; _model_kwargs...)

    t0 = time()
    result = run_aggregate_od_route_column_generation(
        model,
        data;
        verbose                = false,
        cg_log_path            = ITER_PATH,
        column_log_path        = COLUMN_PATH,
        dual_log_path          = DUAL_PATH,
        max_cg_iters           = MAX_CG_ITERS,
        max_new_columns        = MAX_NEW_COLS,
        n_candidates           = max(MAX_NEW_COLS, 20),
        reduced_cost_tol       = 1e-6,
        pricing_time_limit_sec = PRICING_TIME,
        ip_time_limit_sec      = IP_TIME_LIMIT,
        mip_gap                = MIP_GAP,
        silent                 = true,
    )
    wall_time = time() - t0

    ip_obj  = result.final_result.objective_value
    lp_bnd  = result.lp_bound
    gap_pct = if !isnothing(ip_obj) && isfinite(ip_obj) && isfinite(lp_bnd) && ip_obj > 1e-10
        100.0 * (ip_obj - lp_bnd) / ip_obj
    else
        NaN
    end

    @printf("  Status           : %s\n", result.status)
    @printf("  IP objective     : %s\n", isnothing(ip_obj) ? "n/a" : @sprintf("%.4f", ip_obj))
    @printf("  LP bound         : %s\n", isfinite(lp_bnd) ? @sprintf("%.4f", lp_bnd) : "n/a")
    @printf("  Gap %%            : %s\n", isnan(gap_pct) ? "n/a" : @sprintf("%.2f%%", gap_pct))
    @printf("  CG iters         : %d  (%s)\n", result.n_cg_iters, result.cg_stop_reason)
    @printf("  Wall time        : %.1fs\n", wall_time)
    println()

    summary_row = (
        instance             = INST_NAME,
        model_type           = "AggregateODRouteModel",
        n_stations           = meta.n_stations_actual,
        l                    = L,
        n_scenarios          = meta.n_scenarios_actual,
        n_pairs_requested    = N_PAIRS,
        n_pairs_actual       = sum(meta.pairs_per_scenario),
        endpoint_overlap     = ENDPOINT_OVERLAP,
        seed                 = SEED,
        max_walking_distance = max_walking_distance,
        max_wait_time        = MAX_WAIT_TIME,
        detour_factor        = DETOUR_FACTOR,
        max_stops            = max_stops,
        route_reg_weight     = ROUTE_REG_WEIGHT,
        repositioning_time   = REPOSITIONING_TIME,
        status               = string(result.status),
        termination_status   = string(result.final_result.termination_status),
        ip_objective         = isnothing(ip_obj) ? "" : string(ip_obj),
        lp_bound             = string(lp_bnd),
        integrality_gap_pct  = isnan(gap_pct) ? "" : string(gap_pct),
        n_cg_iters           = result.n_cg_iters,
        cg_stop_reason       = string(result.cg_stop_reason),
        n_generated_columns  = length(result.generated_columns),
        n_selected_columns   = length(result.selected_column_ids),
        wall_time_sec        = wall_time,
    )
    _write_summary(CSV_PATH, summary_row)
    _write_namedtuple_rows(COLUMN_PATH, result.column_log_rows)
    _write_namedtuple_rows(DUAL_PATH, result.dual_log_rows)
    _write_selected_columns(SELECTED_PATH, result)

    println("Written: $CSV_PATH")
    println("Written: $ITER_PATH")
    println("Written: $COLUMN_PATH")
    println("Written: $DUAL_PATH")
    println("Written: $SELECTED_PATH")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
