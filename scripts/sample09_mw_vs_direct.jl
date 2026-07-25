"""
    scripts/sample09_mw_vs_direct.jl

Standalone replication + extension of notes/2026-07-17_restricted_mw_cut_benders_y.md's
"hopeful" BendersY(cut_derivation=:restricted_mw_fixed_pi) timing table
(scripts/check_zhuzhou_restricted_mw_timing.jl), on the SAME real fixture
(Data/real_world_test_cases/zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20/
sample_09_2025-03-03_11_15_midday_low -- 19 candidate stations, 52 real orders,
single scenario, no random subsampling), but:

  1. adds Direct enumeration (direct_ms3/direct_ms4) and plain CG (cg_ms4) as
     comparison points -- the original check script only compared BendersY cut
     derivations against each other, never against Direct;
  2. uses the SAME model/solver parameters as the big n=10..60 method-compare
     sweep (scripts/run_method_compare_task.jl's CFG defaults: route_reg_weight=10.0,
     walk_cost_weight=0.1, detour_factor=2.0, max_wait_time=900, repositioning_time=20.0,
     mip_gap=1e-4), rather than the original check script's different, ad hoc
     parameters (route_regularization_weight=0.1, no separate walk weight, mip_gap=0.0,
     max_wait_time=3600) -- this isolates DATA (real sample_09 vs. the sweep's synthetic
     grid/zhuzhou-subsample families) as the only thing that differs from the sweep
     results already collected in experiments/aggregate_od_route_method_compare/, so any
     gap in the MW speedup can be attributed to data rather than a parameter mismatch;
  3. extends n_stations from the original {10,15} to {10,15,19} (19 = every candidate
     station in the fixture), using `l = ceil(n/2)` (the sweep's `_l_for` convention:
     l=5,8,10) rather than the original script's ad hoc l=7 at n=15;
  4. skips BendersYZ/BendersYZH -- the hopeful result and the open question here are
     specifically about plain BendersY.

Reuses build_model/build_solver/MethodSpec/METHODS/resolve_max_stops from
run_method_compare_task.jl (which itself includes aggregate_od_route_method_grid.jl)
by `include`-ing it as a library -- safe because that file's ARGS-parsing block is
guarded by `_RUN_AS_MAIN = abspath(PROGRAM_FILE) == @__FILE__`, which is false here,
so nothing in it reads ARGS or touches the live n=10..60 sweep's job/result files.

Usage:
    julia --project=. scripts/sample09_mw_vs_direct.jl [outdir]

Default output dir: experiments/sample09_mw_vs_direct/
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

# Same knob values as run_method_compare_task.jl's CFG defaults, except
# route_reg_weight/walk_cost_weight are env-overridable (SAMPLE09_ROUTE_WEIGHT /
# SAMPLE09_WALK_WEIGHT) -- added so this same tested (data loading, build_model,
# build_solver) path can be reused to compare BendersY vs Direct at a different
# route:walk weight ratio (see notes/... route/walk weight sensitivity findings)
# without duplicating the whole script for each ratio point.
const CFG = (
    mip_gap             = 1e-4,
    benders_max_iters   = 500,
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

# BendersY only (std_noreprice/std_reprice/zerocomp/mw x {ms4,uncapped}), plus
# direct_ms3/direct_ms4/cg_ms4 -- see module docstring for why YZ/YZH are excluded.
const METHOD_LABELS = haskey(ENV, "SAMPLE09_METHODS") ? String.(split(ENV["SAMPLE09_METHODS"], ",")) : [
    "direct_ms3", "direct_ms4", "cg_ms4",
    filter(l -> startswith(l, "bendersY_"), [m.label for m in METHODS])...,
]

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

function run_one(n_stations::Int, method_label::String, results_dir::String, iters_dir::String)
    l = L_FOR[n_stations]
    method = method_by_label(method_label)
    max_stops = resolve_max_stops(method.max_stops_mode, n_stations)
    inst_name = "sample09_n$(n_stations)"
    summary_path = joinpath(results_dir, "$(inst_name)__$(method_label).csv")

    @printf("  [%s / %s] l=%d max_stops=%d ... ", inst_name, method_label, l, max_stops)
    flush(stdout)

    data = load_sample09(n_stations)
    model = build_model(l, max_stops, MAX_WALK, CFG)
    log_dir = joinpath(iters_dir, "$(inst_name)__$(method_label)")
    mkpath(log_dir)
    solver = build_solver(method, CFG, log_dir)

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
        joinpath(@__DIR__, "..", "experiments", "sample09_mw_vs_direct")
    results_dir = joinpath(outdir, "results")
    iters_dir = joinpath(outdir, "iters")
    mkpath.((results_dir, iters_dir))

    println("=== sample_09_2025-03-03_11_15_midday_low: MW vs Direct, n_stations=$(N_STATIONS_TO_RUN) ===")
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
