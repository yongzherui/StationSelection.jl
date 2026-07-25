"""
    scripts/sample09_route100x_ms56.jl

sample_09 fixture, route:walk weight ratio 100x (route_regularization_weight=10.0,
walk_cost_weight=0.1 -- the big n=10..60 sweep's own default ratio; the counterpart
to scripts/sample09_walk_weight_task.jl's walk10x point at the opposite ratio),
comparing Direct enumeration vs. BendersYZ(MW cut) vs. BendersY(MW cut) at
max_stops in {5, 6} -- the two settings between the sweep's usual ms4 and fully
uncapped, motivated by notes/2026-07-25_lp_ip_gap_structural_vs_pool_completeness_final.md's
finding that uncapping max_stops closes most of the structural LP/IP gap at ms4;
this checks how much of that improvement is already visible at ms5/ms6.

Defines its own small LOCAL_METHODS list (direct_ms5/ms6, bendersYZ_mw_ms5/ms6,
bendersY_mw_ms5/ms6) rather than extending the shared `METHODS` array in
aggregate_od_route_method_grid.jl -- that array's length is load-bearing for the
live n=10..60 method-compare sweep's job-count arithmetic
(generate_method_compare_job_list.jl, submit_method_compare.sh's queue-cap math,
batch_manifest.txt row ranges for not-yet-submitted n_stations), so it must not
change out from under that sweep. Only `resolve_max_stops` itself (extended to
accept :ms5/:ms6, additive/harmless) is shared.

**Known open correctness caveat for BendersYZ** (see memory
project_bendersy_vs_bendersyz_yz_pool_gap.md /
notes/2026-07-25_lp_ip_gap_structural_vs_pool_completeness_final.md): its final
route-covering re-solve can land up to ~1% above the true optimum even when CG
reports pricing-exhaustion (a price-and-branch pool-completeness gap in
_solve_fixed_route_covering_by_cg, not specific to YZ but visible there) -- keep
this in mind when reading BendersYZ vs Direct objective gaps below 1-2%; treat
larger gaps as more likely to be genuine.

Usage:
    julia --project=. scripts/sample09_route100x_ms56.jl [outdir]

Default output dir: experiments/sample09_route100x_ms56/

Env overrides (same convention as sample09_mw_vs_direct.jl):
    SAMPLE09_N_STATIONS   comma-separated, default "10,15,19"
    SAMPLE09_ROUTE_WEIGHT default "10.0"
    SAMPLE09_WALK_WEIGHT  default "0.1"
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, StationSelection

include(joinpath(@__DIR__, "run_method_compare_task.jl"))

const SAMPLE_DIR = normpath(joinpath(
    @__DIR__, "..", "..", "Data", "real_world_test_cases",
    "zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20",
    "sample_09_2025-03-03_11_15_midday_low",
))

const N_STATIONS_TO_RUN = haskey(ENV, "SAMPLE09_N_STATIONS") ?
    parse.(Int, split(ENV["SAMPLE09_N_STATIONS"], ",")) : [10, 15, 19]
const L_FOR = Dict(10 => 5, 15 => 8, 19 => 10)  # ceil(n/2), matching _l_for in the big sweep
const MAX_WALK = 600.0  # matches the big sweep's zhuzhou-family convention (600s walk-time cap)

const CFG = (
    mip_gap             = 1e-4,
    benders_max_iters   = parse(Int, get(ENV, "SAMPLE09_BENDERS_MAX_ITERS", "500")),
    max_reprice_rounds  = 10000,
    inner_cg_max_iters  = 200,
    inner_pricing_time  = 120.0,
    inner_ip_time_limit = 60.0,
    cg_max_iters        = 10000,
    cg_pricing_time     = 120.0,
    cg_ip_time_limit    = 300.0,
    direct_max_routes   = typemax(Int),
    direct_time_limit   = 300.0,
    detour_factor       = 2.0,
    max_wait_time       = 900.0,
    route_reg_weight    = parse(Float64, get(ENV, "SAMPLE09_ROUTE_WEIGHT", "10.0")),
    walk_cost_weight    = parse(Float64, get(ENV, "SAMPLE09_WALK_WEIGHT", "0.1")),
    repositioning_time  = 20.0,
)

const LOCAL_METHODS = MethodSpec[
    MethodSpec("direct_ms5",     :direct,  nothing,                          :standard,               false, :ms5),
    MethodSpec("direct_ms6",     :direct,  nothing,                          :standard,               false, :ms6),
    MethodSpec("bendersY_mw_ms5",  :benders, StationSelection.BendersY(),  :restricted_mw_fixed_pi, false, :ms5),
    MethodSpec("bendersY_mw_ms6",  :benders, StationSelection.BendersY(),  :restricted_mw_fixed_pi, false, :ms6),
    # BendersYZ x {ms5,ms6} x {lifted_walking_objective on/off} -- MethodSpec itself has no field
    # for this (it's a BendersSolver-only knob, orthogonal to decomposition/cut_derivation/reprice),
    # so it's tracked out-of-band in LIFTED_BY_LABEL below and applied in build_solver_ms56.
    # "_nolifted" is the same solve `bendersYZ_mw_ms{5,6}` (no suffix) used to be -- kept under an
    # explicit name once a `_lifted` counterpart exists, so neither label is ambiguous about which
    # BendersSolver.lifted_walking_objective setting it used.
    MethodSpec("bendersYZ_mw_ms5_nolifted", :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
    MethodSpec("bendersYZ_mw_ms6_nolifted", :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms6),
    MethodSpec("bendersYZ_mw_ms5_lifted",   :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms5),
    MethodSpec("bendersYZ_mw_ms6_lifted",   :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :ms6),
    # Uncapped (max_stops=typemax(Int)) BendersY/BendersYZ -- the true optimum with no artificial
    # route-length cap, for comparing route lengths / station selections against the ms5/ms6-capped
    # runs above (Direct can't reach uncapped at this instance size -- combinatorial route-pool
    # blowup, see the n=19 direct_ms6 OOM). No direct_uncapped counterpart for that reason.
    MethodSpec("bendersY_mw_uncapped",  :benders, StationSelection.BendersY(),  :restricted_mw_fixed_pi, false, :uncapped),
    MethodSpec("bendersYZ_mw_uncapped", :benders, StationSelection.BendersYZ(), :restricted_mw_fixed_pi, false, :uncapped),
]
local_method_by_label(label::AbstractString) = only(filter(m -> m.label == label, LOCAL_METHODS))

const LIFTED_BY_LABEL = Dict(
    "bendersYZ_mw_ms5_lifted" => true,
    "bendersYZ_mw_ms6_lifted" => true,
)  # every other label (including bendersY_mw_*, direct_*) defaults to false via `get` below

const METHOD_LABELS = haskey(ENV, "SAMPLE09_METHODS") ? String.(split(ENV["SAMPLE09_METHODS"], ",")) :
    [m.label for m in LOCAL_METHODS]

function load_sample09(n_stations::Int)
    all_stations = StationSelection.read_candidate_stations(joinpath(SAMPLE_DIR, "station.csv"))
    n_stations <= nrow(all_stations) || error(
        "requested n_stations=$n_stations exceeds fixture's $(nrow(all_stations)) candidate stations"
    )
    stations = all_stations[1:n_stations, :]
    keep = Set(Int.(stations.id))
    requests = StationSelection.read_customer_requests(
        joinpath(SAMPLE_DIR, "order.csv");
        start_time="2025-03-03 11:00:00",
        end_time="2025-03-03 15:00:00",
    )
    requests = requests[
        in.(requests.origin_station_id, Ref(keep)) .&
        in.(requests.destination_station_id, Ref(keep)) .&
        (requests.origin_station_id .!= requests.destination_station_id),
        :,
    ]
    walking_costs = StationSelection.compute_station_pairwise_costs(stations)
    routing_costs = StationSelection.read_routing_costs_from_segments(joinpath(SAMPLE_DIR, "segment.csv"), stations)
    data = StationSelection.create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    return data
end

function _selected_station_ids_sample09(result, method::MethodSpec)::Vector{Int}
    id_map = result.mapping.array_idx_to_station_id
    if method.kind == :benders
        idxs = get(result.metadata, "benders_open_stations", nothing)
        isnothing(idxs) && return Int[]
        return sort([id_map[i] for i in idxs])
    end
    haskey(result.model.obj_dict, :y) || return Int[]
    y = result.model[:y]
    return sort([id_map[i] for i in 1:length(y) if round(value(y[i])) == 1])
end

function _last_n_iterations(log_dir::String, method::MethodSpec)
    if method.kind == :benders
        log_path = joinpath(log_dir, "aggregate_od_route_benders_iterations.csv")
        row = _last_iteration_row(log_path)
        return isnothing(row) ? "" : string(row.iteration)
    elseif method.kind == :cg
        log_path = joinpath(log_dir, "aggregate_od_route_cg_iterations.csv")
        row = _last_iteration_row(log_path)
        return isnothing(row) ? "" : string(row.iteration)
    end
    return ""
end

"""
    build_solver_ms56(method, cfg, log_dir, method_label)

Same shape as run_method_compare_task.jl's shared `build_solver`, except the benders branch
also passes `lifted_walking_objective` (looked up from `LIFTED_BY_LABEL`, default `false`) --
kept as a LOCAL copy rather than adding a `lifted::Bool` parameter to the shared `build_solver`,
since that function's signature is relied on as-is by the live n=10..60 method-compare sweep.
"""
function build_solver_ms56(method::MethodSpec, cfg::NamedTuple, log_dir::String, method_label::String)
    config = SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=cfg.mip_gap)

    method.kind == :direct && return DirectSolver(
        config=config,
        max_enumerated_routes=cfg.direct_max_routes,
        max_enumeration_time_sec=cfg.direct_time_limit,
    )
    method.kind == :benders || error("sample09_route100x_ms56 only supports :direct/:benders methods, got $(method.kind)")

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

function run_one(n_stations::Int, method_label::String, results_dir::String, iters_dir::String)
    l = L_FOR[n_stations]
    method = local_method_by_label(method_label)
    max_stops = resolve_max_stops(method.max_stops_mode, n_stations)
    inst_name = "sample09_n$(n_stations)"
    summary_path = joinpath(results_dir, "$(inst_name)__$(method_label).csv")

    @printf("  [%s / %s] l=%d max_stops=%d lifted=%s ... ", inst_name, method_label, l, max_stops,
        get(LIFTED_BY_LABEL, method_label, false))
    flush(stdout)

    data = load_sample09(n_stations)
    model = build_model(l, max_stops, MAX_WALK, CFG)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(method_label)")
    mkpath(log_dir)
    solver = build_solver_ms56(method, CFG, log_dir, method_label)

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
            selected_stations = _selected_station_ids_sample09(result, method)
        catch err
            @warn "failed to export variables for $method_label on $inst_name" exception=(err, catch_backtrace())
        end
    end

    n_iterations = _last_n_iterations(log_dir, method)

    summary = (
        instance           = inst_name,
        n_stations         = n_stations,
        l                  = l,
        n_orders           = nrow(data.scenarios[1].requests),
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
        joinpath(@__DIR__, "..", "experiments", "sample09_route100x_ms56")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== sample_09: route100x (route_reg_weight=$(CFG.route_reg_weight), walk_cost_weight=$(CFG.walk_cost_weight)), ms5/ms6, n_stations=$(N_STATIONS_TO_RUN) ===")
    println("methods: ", join(METHOD_LABELS, ", "))
    println()

    rows = NamedTuple[]
    for n_stations in N_STATIONS_TO_RUN, method_label in METHOD_LABELS
        push!(rows, run_one(n_stations, method_label, results_dir, iters_dir))
    end

    combined = DataFrame(rows)
    combined_path = joinpath(outdir, "combined_results.csv")
    CSV.write(combined_path, combined)
    println("\nWrote combined results to $combined_path")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
