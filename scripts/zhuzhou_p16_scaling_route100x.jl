"""
    scripts/zhuzhou_p16_scaling_route100x.jl

Scaling study: synthetic zhuzhou family, n_pairs=16 fixed, route100x weights
(route_regularization_weight=10.0, walk_cost_weight=0.1 -- the sweep's own
default ratio), comparing Direct enumeration (ms4) vs BendersYZ with MW cuts,
lifted_walking_objective on and off, across n_stations in {10,15,20,30,40,50,60}
x seeds in {42,123,999} -- the big method-compare sweep's own instance grid
(scripts/aggregate_od_route_method_grid.jl), restricted to just this one
(family, n_pairs) slice and this one method triple, run at a generous
per-job time budget (see sbatch_zhuzhou_p16_scaling_route100x_task.sh) so each
job is bounded only by wall-clock, not an arbitrary iteration/route cap --
`benders_max_iters` and `direct_time_limit` below are set deliberately high so
SLURM's own time limit is the only thing that stops a run early.

max_stops=4 (not 5/6/uncapped, unlike sample09_route100x_ms56.jl): Direct
enumeration needs a cap to stay tractable up to n_stations=60 at all (see
aggregate_od_route_method_grid.jl's own docstring on why direct_ms3/direct_ms4
exist as Direct's only tractable settings at scale); ms4 is this codebase's
established standard cap, used throughout the big sweep and CLAUDE.md's own
model reference.

Defines its own small LOCAL_METHODS list rather than extending the shared
`METHODS` array in aggregate_od_route_method_grid.jl, for the same reason
scripts/sample09_route100x_ms56.jl does -- that array's length is load-bearing
for the live n=10..60 method-compare sweep's job-count arithmetic
(generate_method_compare_job_list.jl, submit_method_compare.sh's queue-cap
math, batch_manifest.txt row ranges).

Usage:
    julia --project=. scripts/zhuzhou_p16_scaling_route100x.jl [outdir]

Default output dir: experiments/zhuzhou_p16_scaling_route100x/

Env overrides:
    SAMPLE09_ROUTE_WEIGHT / SAMPLE09_WALK_WEIGHT  (same convention as
      sample09_route100x_ms56.jl, kept under the same env var names for
      consistency across the two route100x studies)
    ZZ_BENDERS_MAX_ITERS  default "1000000" (effectively unbounded --
      wall-clock, not iteration count, is meant to be the real limit)
    ZZ_DIRECT_TIME_LIMIT  default "39600" (11h, leaving ~1h margin under the
      12h sbatch wall-clock limit for module load/precompile/final CSV write)
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = 16
const FAMILY = "zhuzhou"

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
    # ms5 variants -- Direct's tractability at ms5 up to n_stations=60 is untested (ms4 was
    # chosen specifically because it's this codebase's known-tractable cap at that scale); watch
    # direct_ms5 for OOM/timeout at the larger n_stations values the way ms6 did on sample09/n=19.
    MethodSpec("direct_ms5",                :direct,  nothing,                          :standard,               false, :ms5),
    MethodSpec("bendersYZ_mw_ms5_nolifted",  :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
    MethodSpec("bendersYZ_mw_ms5_lifted",    :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
    # lifted_routing_lower_bound (objective term + residual cuts, see
    # .claude/plans/ticklish-herding-honey.md) generalizes to all cut_derivation values (the
    # weak-duality argument in lifted_routing_lower_bound.jl's module docstring covers the
    # restricted-completion modes' separate Q_bar-fed completion LP too), so the MW-cut "_lifted"
    # methods above get their own "_lifted_lb" variants directly. The :standard+reprice pair below
    # is kept too as a second, already-validated comparison point (cheaper to iterate on than MW cuts).
    MethodSpec("bendersYZ_mw_ms4_lifted_lb",  :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms4),
    MethodSpec("bendersYZ_mw_ms5_lifted_lb",  :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
    MethodSpec("bendersYZ_std_ms4_lifted",    :benders, StationSelection.BendersYZ(), :standard, true, :ms4),
    MethodSpec("bendersYZ_std_ms4_lifted_lb", :benders, StationSelection.BendersYZ(), :standard, true, :ms4),
    MethodSpec("bendersYZ_std_ms5_lifted",    :benders, StationSelection.BendersYZ(), :standard, true, :ms5),
    MethodSpec("bendersYZ_std_ms5_lifted_lb", :benders, StationSelection.BendersYZ(), :standard, true, :ms5),
]
local_method_by_label(label::AbstractString) = only(filter(m -> m.label == label, LOCAL_METHODS))

const LIFTED_BY_LABEL = Dict(
    "bendersYZ_mw_ms4_lifted" => true,
    "bendersYZ_mw_ms5_lifted" => true,
    "bendersYZ_mw_ms4_lifted_lb" => true,
    "bendersYZ_mw_ms5_lifted_lb" => true,
    "bendersYZ_std_ms4_lifted" => true,
    "bendersYZ_std_ms4_lifted_lb" => true,
    "bendersYZ_std_ms5_lifted" => true,
    "bendersYZ_std_ms5_lifted_lb" => true,
)

const LIFTED_LB_BY_LABEL = Dict(
    "bendersYZ_mw_ms4_lifted_lb" => true,
    "bendersYZ_mw_ms5_lifted_lb" => true,
    "bendersYZ_std_ms4_lifted_lb" => true,
    "bendersYZ_std_ms5_lifted_lb" => true,
)

const METHOD_LABELS = [m.label for m in LOCAL_METHODS]

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
    method.kind == :benders || error("zhuzhou_p16_scaling_route100x only supports :direct/:benders methods, got $(method.kind)")

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
        lifted_routing_lower_bound=get(LIFTED_LB_BY_LABEL, method_label, false),
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

    data, max_walk = build_instance(FAMILY, n_stations, N_PAIRS, seed, DATA_DIR)
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
        seed               = seed,
        method             = method_label,
        kind               = string(method.kind),
        cut_derivation     = string(method.cut_derivation),
        reprice_subproblem = method.reprice,
        lifted_walking_objective = get(LIFTED_BY_LABEL, method_label, false),
        lifted_routing_lower_bound = get(LIFTED_LB_BY_LABEL, method_label, false),
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
        joinpath(@__DIR__, "..", "experiments", "zhuzhou_p16_scaling_route100x")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== zhuzhou p16 scaling, route100x, n_stations=$(N_STATIONS_LIST) seeds=$(SEEDS) ===")
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
