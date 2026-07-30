"""
    scripts/zhuzhou_p16_cg_ms45_singlescenario.jl

CG-heuristic vs Direct-enumeration comparison at n_stations >= 20, on the SAME
single-scenario Zhuzhou instances as
scripts/zhuzhou_p16_scaling_route100x_singlescenario.jl (n_pairs=16,
n_scenarios=1, route100x weights: route_regularization_weight=10.0,
walk_cost_weight=0.1).

Runs ONLY the plain (non-Benders) column-generation heuristic -- `cg_ms4` and
`cg_ms5` (kind=:cg in aggregate_od_route_method_grid.jl's MethodSpec sense:
joint y+route CG, no y-fixing/outer loop) -- since Direct's own
direct_ms4/direct_ms5 numbers for these same (n_stations, seed) cells already
exist in experiments/zhuzhou_p16_scaling_route100x_singlescenario/results/
from the earlier single-scenario batch (job 18993518); no need to re-run
Direct here.

Motivation: at n_stations>=30, Direct's ms5 enumeration OOMs and BendersYZ's
master MIP grows expensive as its cut count accumulates (see
notes discussed in-session on n=30 master-bloat). Plain CG (no outer Benders
loop, no y-fixing) solves the joint problem's LP relaxation via pricing and
takes one final MIP over the accumulated columns -- if it's both fast AND
close to Direct's true optimum, it's a candidate as a fast heuristic (or as a
warm start / bound for a supervised branch-and-bound on top of the CG root
relaxation).

Reports, per (n_stations, seed, ms4/ms5):
    objective_value       -- CG's final incumbent (from its own final IP solve)
    wall_time_sec
    lp_bound              -- CG's own LP relaxation bound at pricing exhaustion
                             (last row's `lp_objective` in
                             aggregate_od_route_cg_iterations.csv -- see
                             AggregateODRouteColumnGenerationResult.lp_bound in
                             pricing/column_generation.jl; run_opt for
                             ColumnGenerationSolver discards this field itself,
                             so it must be read back from the iteration log)
    integrality_gap_pct   -- 100 * (objective_value - lp_bound) / lp_bound;
                             CG's own root-relaxation-to-incumbent gap, NOT a
                             certified optimality gap against Direct/Benders'
                             true optimum (CG's final IP step is itself only
                             solved to CS_MIP_GAP, and pricing exhaustion at a
                             GIVEN restricted pool doesn't certify global
                             LP optimality the way Benders' dual proof does --
                             see the note on this in benders/yz.jl's shared_pool
                             comment)

Usage:
    julia --project=. scripts/zhuzhou_p16_cg_ms45_singlescenario.jl [outdir]

Default output dir: experiments/zhuzhou_p16_cg_ms45_singlescenario/

Env overrides:
    SAMPLE09_ROUTE_WEIGHT / SAMPLE09_WALK_WEIGHT   (default "10.0" / "0.1")
    ZZ_CG_MAX_ITERS       default "10000"
    ZZ_CG_PRICING_TIME    default "120"   (seconds, per-iteration pricing time limit)
    ZZ_CG_IP_TIME_LIMIT   default "300"   (seconds, final IP solve time limit)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const FAMILY = "zhuzhou"
const N_SCENARIOS_SINGLE = 1

const N_STATIONS_LIST_CG = [20, 30, 40, 50, 60]

const CFG = (
    mip_gap          = 1e-4,
    cg_max_iters     = parse(Int,     get(ENV, "ZZ_CG_MAX_ITERS",      "10000")),
    cg_pricing_time  = parse(Float64, get(ENV, "ZZ_CG_PRICING_TIME",   "120")),
    cg_ip_time_limit = parse(Float64, get(ENV, "ZZ_CG_IP_TIME_LIMIT",  "300")),
    route_reg_weight = parse(Float64, get(ENV, "SAMPLE09_ROUTE_WEIGHT", "10.0")),
    walk_cost_weight = parse(Float64, get(ENV, "SAMPLE09_WALK_WEIGHT", "0.1")),
    repositioning_time = 20.0,
    detour_factor    = 2.0,
    max_wait_time    = 900.0,
)

const LOCAL_METHODS = MethodSpec[
    MethodSpec("cg_ms4", :cg, nothing, :standard, false, :ms4),
    MethodSpec("cg_ms5", :cg, nothing, :standard, false, :ms5),
]
local_method_by_label(label::AbstractString) = only(filter(m -> m.label == label, LOCAL_METHODS))
const METHOD_LABELS = [m.label for m in LOCAL_METHODS]

"""
    build_instance_zz_single_scenario(n_stations, n_pairs, seed, data_dir) -> (data, max_walking_distance)

Same n_scenarios=1 override as
scripts/zhuzhou_p16_scaling_route100x_singlescenario.jl's function of the same
name -- kept as a separate local copy per this repo's convention of not
sharing small per-study helpers across study scripts.
"""
function build_instance_zz_single_scenario(n_stations::Int, n_pairs::Int, seed::Int, data_dir::AbstractString)
    data, meta = generate_zhuzhou_data(
        data_dir, n_stations, n_pairs;
        n_scenarios=N_SCENARIOS_SINGLE, endpoint_overlap=ENDPOINT_OVERLAP, seed=seed,
    )
    print_zhuzhou_data_summary(data, meta)
    return data, 600.0
end

function build_solver_cg(cfg::NamedTuple, log_dir::String)
    return ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=cfg.mip_gap),
        max_iterations=cfg.cg_max_iters,
        max_columns_per_iteration=20,
        n_candidates=20,
        pricing_time_limit_sec=cfg.cg_pricing_time,
        final_ip_time_limit_sec=cfg.cg_ip_time_limit,
        log_dir=log_dir,
    )
end

function run_one(n_stations::Int, seed::Int, method_label::String, results_dir::String, iters_dir::String)
    l = _l_for(n_stations)
    method = local_method_by_label(method_label)
    max_stops = resolve_max_stops(method.max_stops_mode, n_stations)
    inst_name = "zhuzhou_n$(n_stations)_p$(N_PAIRS)_s$(seed)"
    summary_path = joinpath(results_dir, "$(inst_name)__$(method_label).csv")

    @printf("  [%s / %s] l=%d max_stops=%d ... ", inst_name, method_label, l, max_stops)
    flush(stdout)

    data, max_walk = build_instance_zz_single_scenario(n_stations, N_PAIRS, seed, DATA_DIR)
    model = build_model(l, max_stops, max_walk, CFG)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(method_label)")
    mkpath(log_dir)
    solver = build_solver_cg(CFG, log_dir)

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
            if haskey(result.model.obj_dict, :y)
                y = result.model[:y]
                selected_stations = sort([id_map[i] for i in 1:length(y) if round(value(y[i])) == 1])
            end
        catch err
            @warn "failed to export variables for $method_label on $inst_name" exception=(err, catch_backtrace())
        end
    end

    n_iterations = ""
    lp_bound = ""
    integrality_gap_pct = ""
    if !isnothing(result)
        log_path = joinpath(log_dir, "aggregate_od_route_cg_iterations.csv")
        row = _last_iteration_row(log_path)
        if !isnothing(row)
            n_iterations = string(row.iteration)
            if !ismissing(row.lp_objective) && !isnothing(row.lp_objective)
                lp_bound = string(row.lp_objective)
                obj = result.objective_value
                if !isnothing(obj) && row.lp_objective > 0
                    integrality_gap_pct = string(100 * (obj - row.lp_objective) / row.lp_objective)
                end
            end
        end
    end

    summary = (
        instance            = inst_name,
        family              = FAMILY,
        n_stations          = n_stations,
        l                   = l,
        n_pairs             = N_PAIRS,
        n_scenarios         = N_SCENARIOS_SINGLE,
        seed                = seed,
        method              = method_label,
        kind                = string(method.kind),
        max_stops_mode      = string(method.max_stops_mode),
        max_stops           = max_stops,
        status              = status,
        termination_status  = isnothing(result) ? "" : string(result.termination_status),
        objective_value     = isnothing(result) || isnothing(result.objective_value) ? "" : string(result.objective_value),
        wall_time_sec       = wall_time,
        n_iterations        = n_iterations,
        lp_bound            = lp_bound,
        integrality_gap_pct = integrality_gap_pct,
        n_stations_selected = length(selected_stations),
        selected_stations   = string(selected_stations),
        iters_log_path      = log_dir,
    )
    CSV.write(summary_path, DataFrame([summary]))

    @printf("status=%s obj=%s wall=%.1fs lp_bound=%s gap%%=%s\n",
        status, summary.objective_value, wall_time, lp_bound,
        isempty(integrality_gap_pct) ? "n/a" : integrality_gap_pct)
    flush(stdout)
    return summary
end

function main()
    outdir = length(ARGS) >= 1 && !isempty(ARGS[1]) ? ARGS[1] :
        joinpath(@__DIR__, "..", "experiments", "zhuzhou_p16_cg_ms45_singlescenario")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== zhuzhou p16 CG-vs-Direct, single scenario, n_stations=$(N_STATIONS_LIST_CG) seeds=$(SEEDS) ===")
    println("methods: ", join(METHOD_LABELS, ", "))
    println()

    rows = NamedTuple[]
    for n_stations in N_STATIONS_LIST_CG, seed in SEEDS, method_label in METHOD_LABELS
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
