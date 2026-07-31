"""
    scripts/pfa_route_length_profile.jl

Route-length profile of the passenger free-assignment CG pool.

Reruns the same cells as `passenger_free_assignment_cg_scaling.jl` (same model,
weights, seed and budgets), but keeps the column pool and the final MIP's theta
values so we can ask: are the columns the final solution actually *uses* long
multi-stop routes, or short ones?

Writes two CSVs per cell into <outdir>:
  columns_<case>.csv   -- one row per generated column: route, stops, distinct
                          stations, passengers carried, tau, final theta
  summary.csv          -- one row per cell with pool-vs-selected length stats

Usage:
    julia --project=. scripts/pfa_route_length_profile.jl <outdir> "<n>:<p>" ...
e.g.
    julia --project=. scripts/pfa_route_length_profile.jl /tmp/out 10:8 15:16
"""

using CSV, DataFrames, Gurobi, JuMP, Printf, Statistics, StationSelection

include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_SCENARIOS = 3
const SEED = 42
const MAX_STOPS = typemax(Int)
const MAX_VISITS = 3
const N_CANDIDATES = 20
const PRICING_TIME = 120.0
const CERT_TIME = 1800.0
const IP_TIME = 900.0
const CASE_TIME = parse(Float64, get(ENV, "PFA_CASE_TIME", "10800"))
const MAX_CG_ITERS = 2000
const MAX_WALK = 600.0
const ROUTE_WEIGHT = 10.0
const WALK_WEIGHT = 0.1

const GRB_ENV = Gurobi.Env()

_l_for(n::Int) = max(2, ceil(Int, n / 2))

build_model_for(n) = AggregateODRouteModel(
    _l_for(n);
    route_regularization_weight = ROUTE_WEIGHT,
    walk_cost_weight            = WALK_WEIGHT,
    repositioning_time          = 20.0,
    max_walking_distance        = MAX_WALK,
    max_wait_time               = 900.0,
    detour_factor               = 2.0,
    max_stops                   = MAX_STOPS,
    max_visits_per_node         = MAX_VISITS,
)

function run_cell(n_stations::Int, n_pairs::Int, outdir::String)
    case = "n$(n_stations)_p$(n_pairs)"
    @printf("=== %s ===\n", case)
    flush(stdout)

    data, _ = generate_zhuzhou_data(
        DATA_DIR, n_stations, n_pairs; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    colpath = joinpath(outdir, "columns_$(case)_raw.csv")
    result = run_passenger_free_assignment_column_generation(
        build_model_for(n_stations), data;
        optimizer_env=GRB_ENV,
        max_cg_iters=MAX_CG_ITERS,
        n_candidates=N_CANDIDATES,
        max_new_columns=N_CANDIDATES,
        pricing_time_limit_sec=PRICING_TIME,
        certification_time_limit_sec=CERT_TIME,
        ip_time_limit_sec=IP_TIME,
        total_time_limit_sec=CASE_TIME,
        parallel_scenarios=true,
        station_budget_cap=false,
        compensated_dominance=true,
        verify_reduced_costs=true,
        column_log_path=colpath,
        verbose=true,
    )

    # theta from the final MIP over the accumulated pool.
    theta = if isnothing(result.final_result.solution)
        Dict{Int, Float64}()
    else
        Dict(Int(k) => Float64(v) for (k, v) in result.final_result.solution[1].route_columns)
    end

    cols = CSV.read(colpath, DataFrame)
    cols.stops = [length(split(String(r), "-")) for r in cols.route]
    cols.distinct_stations = [length(unique(split(String(r), "-"))) for r in cols.route]
    cols.n_passengers = [
        ismissing(a) || isempty(String(a)) ? 0 : length(split(String(a), ";")) for a in cols.assignments
    ]
    cols.theta = [get(theta, Int(id), 0.0) for id in cols.column_id]
    cols.selected = cols.theta .> 0.5
    CSV.write(joinpath(outdir, "columns_$(case).csv"), cols)
    rm(colpath; force=true)

    sel = cols[cols.selected, :]
    q(v, p) = isempty(v) ? missing : quantile(v, p)
    row = (
        case = case,
        n_stations = n_stations,
        n_pairs = n_pairs,
        cg_stop_reason = String(result.cg_stop_reason),
        lp_bound = result.lp_bound,
        mip_objective = isnothing(result.mip_objective) ? missing : result.mip_objective,
        n_passengers = result.n_passengers,
        pool_size = nrow(cols),
        n_selected = nrow(sel),
        pool_stops_mean = q(cols.stops, 0.5) === missing ? missing : mean(cols.stops),
        pool_stops_med = q(cols.stops, 0.5),
        pool_stops_max = isempty(cols.stops) ? missing : maximum(cols.stops),
        sel_stops_mean = isempty(sel.stops) ? missing : mean(sel.stops),
        sel_stops_min = isempty(sel.stops) ? missing : minimum(sel.stops),
        sel_stops_med = q(sel.stops, 0.5),
        sel_stops_p90 = q(sel.stops, 0.9),
        sel_stops_max = isempty(sel.stops) ? missing : maximum(sel.stops),
        sel_distinct_med = q(sel.distinct_stations, 0.5),
        sel_distinct_max = isempty(sel.distinct_stations) ? missing : maximum(sel.distinct_stations),
        sel_pax_mean = isempty(sel.n_passengers) ? missing : mean(sel.n_passengers),
        sel_pax_max = isempty(sel.n_passengers) ? missing : maximum(sel.n_passengers),
        wall_sec = result.total_seconds,
    )
    @printf("  pool=%d selected=%d  sel stops med=%s max=%s\n",
            row.pool_size, row.n_selected, string(row.sel_stops_med), string(row.sel_stops_max))
    flush(stdout)
    return row
end

function main()
    outdir = abspath(ARGS[1])
    mkpath(outdir)
    cells = [(parse(Int, s[1]), parse(Int, s[2])) for s in split.(ARGS[2:end], ':')]
    rows = NamedTuple[]
    for (n, p) in cells
        try
            push!(rows, run_cell(n, p, outdir))
            CSV.write(joinpath(outdir, "summary.csv"), DataFrame(rows))
        catch err
            showerror(stderr, err, catch_backtrace())
            println(stderr)
        end
    end
    println("\nwrote $(joinpath(outdir, "summary.csv"))")
end

main()
