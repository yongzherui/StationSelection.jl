"""
    scripts/zhuzhou_p16_scaling_route100x_singlescenario.jl

Single-scenario twin of scripts/zhuzhou_p16_scaling_route100x.jl: identical grid
(n_stations in {10,15,20,30,40,50,60} x seeds in {42,123,999}), identical
route100x weights (route_regularization_weight=10.0, walk_cost_weight=0.1),
identical LOCAL_METHODS (direct/bendersYZ mw, ms4/ms5, lifted/nolifted) --
the ONLY difference is the Zhuzhou instance is generated with n_scenarios=1
instead of the base script's ZZ_N_SCENARIOS=3
(scripts/aggregate_od_route_method_grid.jl).

Motivation: the n=30 BendersYZ runs in the ms4/ms5 batches all timed out at
12h with the master MIP solve dominating wall time (88-95%), and the master's
cut count grows ~3x faster than the outer iteration count because the default
BendersSolver cut_mode=MultiCut() adds one optimality cut PER SCENARIO per
iteration (3 scenarios -> up to 3 cuts/iter). Re-running the exact same grid
at n_scenarios=1 removes that 3x multiplier at the data level (not by
switching cut_mode -- see the two options discussed with the user) so results
here are NOT directly comparable to zhuzhou_p16_scaling_route100x.jl's
3-scenario results; this is a separate, smaller stochastic problem.

Does not touch aggregate_od_route_method_grid.jl's shared ZZ_N_SCENARIOS
constant (that file is shared with the live n=10..60 method-compare sweep) --
instead defines its own `build_instance_zz_single_scenario` that calls
`generate_zhuzhou_data` directly with n_scenarios=1, bypassing the shared
`build_instance` dispatcher entirely (this study is zhuzhou-only, so the
grid/family dispatch in `build_instance` isn't needed here).

Usage:
    julia --project=. scripts/zhuzhou_p16_scaling_route100x_singlescenario.jl [outdir]

Default output dir: experiments/zhuzhou_p16_scaling_route100x_singlescenario/

Env overrides: same names as zhuzhou_p16_scaling_route100x.jl
    SAMPLE09_ROUTE_WEIGHT / SAMPLE09_WALK_WEIGHT
    ZZ_BENDERS_MAX_ITERS  default "1000000"
    ZZ_DIRECT_TIME_LIMIT  default "39600" (11h)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const FAMILY = "zhuzhou"
const N_SCENARIOS_SINGLE = 1

const CFG = (
    mip_gap             = 1e-4,
    benders_max_iters   = parse(Int,     get(ENV, "ZZ_BENDERS_MAX_ITERS", "1000000")),
    max_reprice_rounds  = 10000,
    inner_cg_max_iters  = 200,
    inner_pricing_time  = 120.0,
    inner_ip_time_limit = 60.0,
    cg_max_iters        = 10000,
    cg_pricing_time     = 120.0,
    cg_ip_time_limit    = 300.0,
    direct_max_routes   = typemax(Int),
    direct_time_limit   = parse(Float64, get(ENV, "ZZ_DIRECT_TIME_LIMIT", "39600")),
    detour_factor       = 2.0,
    max_wait_time       = 900.0,
    route_reg_weight    = parse(Float64, get(ENV, "SAMPLE09_ROUTE_WEIGHT", "10.0")),
    walk_cost_weight    = parse(Float64, get(ENV, "SAMPLE09_WALK_WEIGHT", "0.1")),
    repositioning_time  = 20.0,
)

const LOCAL_METHODS = MethodSpec[
    MethodSpec("direct_ms4",                :direct,  nothing,                          :standard,               false, :ms4),
    MethodSpec("bendersYZ_mw_ms4_nolifted",  :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms4),
    MethodSpec("bendersYZ_mw_ms4_lifted",    :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms4),
    MethodSpec("direct_ms5",                :direct,  nothing,                          :standard,               false, :ms5),
    MethodSpec("bendersYZ_mw_ms5_nolifted",  :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
    MethodSpec("bendersYZ_mw_ms5_lifted",    :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
]
local_method_by_label(label::AbstractString) = only(filter(m -> m.label == label, LOCAL_METHODS))

const LIFTED_BY_LABEL = Dict(
    "bendersYZ_mw_ms4_lifted" => true,
    "bendersYZ_mw_ms5_lifted" => true,
)

const METHOD_LABELS = [m.label for m in LOCAL_METHODS]

"""
    build_instance_zz_single_scenario(n_stations, n_pairs, seed, data_dir) -> (data, max_walking_distance)

Same as aggregate_od_route_method_grid.jl's `build_instance(FAMILY, ...)` for
family="zhuzhou", except n_scenarios is hardcoded to N_SCENARIOS_SINGLE=1
rather than reading the shared ZZ_N_SCENARIOS constant.
"""
function build_instance_zz_single_scenario(n_stations::Int, n_pairs::Int, seed::Int, data_dir::AbstractString)
    data, meta = generate_zhuzhou_data(
        data_dir, n_stations, n_pairs;
        n_scenarios=N_SCENARIOS_SINGLE, endpoint_overlap=ENDPOINT_OVERLAP, seed=seed,
    )
    print_zhuzhou_data_summary(data, meta)
    return data, 600.0
end

"""
    build_solver_zhuzhou(method, cfg, log_dir, method_label)

Same shape as run_method_compare_task.jl's shared `build_solver`, except the benders branch
also passes `lifted_walking_objective` -- kept as a LOCAL copy (see sample09_route100x_ms56.jl's
build_solver_ms56 for the identical rationale) rather than touching the shared `build_solver`.
"""
function build_solver_zhuzhou(method::MethodSpec, cfg::NamedTuple, log_dir::String, method_label::String)
    config = SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=cfg.mip_gap)

    method.kind == :direct && return DirectSolver(
        config=config,
        max_enumerated_routes=cfg.direct_max_routes,
        max_enumeration_time_sec=cfg.direct_time_limit,
    )
    method.kind == :benders || error("zhuzhou_p16_scaling_route100x_singlescenario only supports :direct/:benders methods, got $(method.kind)")

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
        log_subiteration_details=false,
        reprice_subproblem=method.reprice,
        max_reprice_rounds=cfg.max_reprice_rounds,
        cut_derivation=method.cut_derivation,
        lifted_walking_objective=get(LIFTED_BY_LABEL, method_label, false),
    )
end

function run_one(n_stations::Int, seed::Int, method_label::String, results_dir::String, iters_dir::String)
    l = _l_for(n_stations)
    method = local_method_by_label(method_label)
    max_stops = resolve_max_stops(method.max_stops_mode, n_stations)
    inst_name = "zhuzhou_n$(n_stations)_p$(N_PAIRS)_s$(seed)"
    summary_path = joinpath(results_dir, "$(inst_name)__$(method_label).csv")

    @printf("  [%s / %s] l=%d max_stops=%d lifted=%s ... ", inst_name, method_label, l, max_stops,
        get(LIFTED_BY_LABEL, method_label, false))
    flush(stdout)

    data, max_walk = build_instance_zz_single_scenario(n_stations, N_PAIRS, seed, DATA_DIR)
    model = build_model(l, max_stops, max_walk, CFG)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(method_label)")
    mkpath(log_dir)
    solver = build_solver_zhuzhou(method, CFG, log_dir, method_label)

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
            id_map = result.mapping.array_idx_to_station_id
            if method.kind == :benders
                idxs = get(result.metadata, "benders_open_stations", nothing)
                selected_stations = isnothing(idxs) ? Int[] : sort([id_map[i] for i in idxs])
            elseif haskey(result.model.obj_dict, :y)
                y = result.model[:y]
                selected_stations = sort([id_map[i] for i in 1:length(y) if round(value(y[i])) == 1])
            end
        catch err
            @warn "failed to export variables for $method_label on $inst_name" exception=(err, catch_backtrace())
        end
    end

    n_iterations = ""
    if !isnothing(result)
        log_path = joinpath(log_dir, "aggregate_od_route_benders_iterations.csv")
        row = _last_iteration_row(log_path)
        n_iterations = isnothing(row) ? "" : string(row.iteration)
    end

    summary = (
        instance           = inst_name,
        family             = FAMILY,
        n_stations         = n_stations,
        l                  = l,
        n_pairs            = N_PAIRS,
        n_scenarios        = N_SCENARIOS_SINGLE,
        seed               = seed,
        method             = method_label,
        kind               = string(method.kind),
        cut_derivation     = string(method.cut_derivation),
        reprice_subproblem = method.reprice,
        lifted_walking_objective = get(LIFTED_BY_LABEL, method_label, false),
        max_stops_mode     = string(method.max_stops_mode),
        max_stops          = max_stops,
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

function main()
    outdir = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "zhuzhou_p16_scaling_route100x_singlescenario")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== zhuzhou p16 scaling, route100x, SINGLE SCENARIO, n_stations=$(N_STATIONS_LIST) seeds=$(SEEDS) ===")
    println("methods: ", join(METHOD_LABELS, ", "))
    println()

    rows = NamedTuple[]
    for n_stations in N_STATIONS_LIST, seed in SEEDS, method_label in METHOD_LABELS
        push!(rows, run_one(n_stations, seed, method_label, results_dir, iters_dir))
    end

    combined = DataFrame(rows)
    combined_path = joinpath(outdir, "combined_results.csv")
    CSV.write(combined_path, combined)
    println("\nWrote combined results to $combined_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
