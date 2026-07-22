"""
    scripts/run_single_instance.jl

Solve one synthetic aggregate OD route instance and write a single-row summary
CSV plus the CG iteration/column/dual logs.

Designed to be called as a SLURM array task via `sbatch_single_instance.sh`.

Usage:
    julia --project=. scripts/run_single_instance.jl <base_outdir> <nx> <ny> <n_requests> <seed>

Output:
    <base_outdir>/results/<instance>.csv
    <base_outdir>/iters/<instance>_iters.csv
    <base_outdir>/columns/<instance>_columns.csv
    <base_outdir>/duals/<instance>_duals.csv
    <base_outdir>/selected/<instance>_selected.csv
"""

using CSV
using DataFrames
using Dates
using Printf
using Random
using StationSelection

const _RUN_AS_MAIN = abspath(PROGRAM_FILE) == @__FILE__

if _RUN_AS_MAIN
    length(ARGS) >= 5 || error(
        "Usage: run_single_instance.jl <base_outdir> <nx> <ny> <n_requests> <seed>"
    )

    const BASE_OUTDIR = ARGS[1]
    const NX = parse(Int, ARGS[2])
    const NY = parse(Int, ARGS[3])
    const N_REQUESTS = parse(Int, ARGS[4])
    const SEED = parse(Int, ARGS[5])

    const TIME_LIMIT = parse(Float64, get(ENV, "CS_TIME_LIMIT", "10800"))
    const PRICING_TIME = parse(Float64, get(ENV, "CS_PRICING_TIME", "300"))
    const MAX_CG_ITERS = parse(Int, get(ENV, "CS_MAX_CG_ITERS", "10000"))
    const MAX_NEW_COLS = parse(Int, get(ENV, "CS_MAX_NEW_COLS", "20"))
    const IP_TIME_LIMIT = parse(Float64, get(ENV, "CS_IP_TIME_LIMIT", "1200"))
    const MIP_GAP = parse(Float64, get(ENV, "CS_MIP_GAP", "1e-4"))
    const WALK_SCALE = parse(Float64, get(ENV, "CS_WALK_SCALE", "600.0"))
    const ROUTE_SCALE = parse(Float64, get(ENV, "CS_ROUTE_SCALE", "450.0"))
    const MAX_WALKING_DISTANCE_ENV = get(ENV, "CS_MAX_WALKING_DISTANCE", "")
    const ROUTE_REGULARIZATION_WEIGHT = parse(Float64, get(ENV, "CS_ROUTE_REGULARIZATION_WEIGHT", "1.0"))
    const REPOSITIONING_TIME = parse(Float64, get(ENV, "CS_REPOSITIONING_TIME", "20.0"))

    const INST_NAME = "g$(NX)x$(NY)_r$(N_REQUESTS)_s$(SEED)"
    const RESULTS_DIR = joinpath(BASE_OUTDIR, "results")
    const ITERS_DIR = joinpath(BASE_OUTDIR, "iters")
    const COLUMNS_DIR = joinpath(BASE_OUTDIR, "columns")
    const DUALS_DIR = joinpath(BASE_OUTDIR, "duals")
    const SELECTED_DIR = joinpath(BASE_OUTDIR, "selected")
    const CSV_PATH = joinpath(RESULTS_DIR, "$(INST_NAME).csv")
    const ITER_PATH = joinpath(ITERS_DIR, "$(INST_NAME)_iters.csv")
    const COLUMN_PATH = joinpath(COLUMNS_DIR, "$(INST_NAME)_columns.csv")
    const DUAL_PATH = joinpath(DUALS_DIR, "$(INST_NAME)_duals.csv")
    const SELECTED_PATH = joinpath(SELECTED_DIR, "$(INST_NAME)_selected.csv")
end

function _station_grid(nx::Int, ny::Int)
    station_ids = Int[]
    lon = Float64[]
    lat = Float64[]
    id = 1
    for y in 1:ny, x in 1:nx
        push!(station_ids, id)
        push!(lon, Float64(x - 1))
        push!(lat, Float64(y - 1))
        id += 1
    end
    return DataFrame(id=station_ids, lon=lon, lat=lat)
end

function _grid_costs(stations::DataFrame; scale::Float64)
    costs = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:nrow(stations), j in 1:nrow(stations)
        oi = stations[i, :id]
        dj = stations[j, :id]
        dx = stations[i, :lon] - stations[j, :lon]
        dy = stations[i, :lat] - stations[j, :lat]
        costs[(oi, dj)] = hypot(dx, dy) * scale
    end
    return costs
end

function _generate_requests(station_ids::Vector{Int}, n_requests::Int, seed::Int)
    rng = MersenneTwister(seed)
    n_stations = length(station_ids)
    origin_station_id = Int[]
    destination_station_id = Int[]
    request_time = DateTime[]

    start_time = DateTime(2024, 1, 1, 8, 0, 0)
    for request_idx in 1:n_requests
        o = rand(rng, station_ids)
        d = rand(rng, station_ids)
        while d == o && n_stations > 1
            d = rand(rng, station_ids)
        end
        push!(origin_station_id, o)
        push!(destination_station_id, d)
        push!(request_time, start_time + Second(request_idx))
    end

    return DataFrame(
        id=collect(1:n_requests),
        origin_station_id=origin_station_id,
        destination_station_id=destination_station_id,
        request_time=request_time,
    )
end

function _demand_station_ids(stations::DataFrame)
    n_demand = min(9, nrow(stations))
    return collect(stations.id[1:n_demand])
end

function _max_walking_distance(
    walking_costs::Dict{Tuple{Int, Int}, Float64},
    demand_station_ids::Vector{Int},
)
    local_max = 0.0
    for i in demand_station_ids, j in demand_station_ids
        local_max = max(local_max, get(walking_costs, (i, j), 0.0))
    end
    return local_max
end

function _build_data(nx::Int, ny::Int, n_requests::Int, seed::Int)
    stations = _station_grid(nx, ny)
    walking_costs = _grid_costs(stations; scale=WALK_SCALE)
    routing_costs = _grid_costs(stations; scale=ROUTE_SCALE)
    demand_station_ids = _demand_station_ids(stations)
    requests = _generate_requests(demand_station_ids, n_requests, seed)
    data = create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    return stations, requests, data, walking_costs, demand_station_ids
end

function _write_namedtuple_rows(path::AbstractString, rows::Vector{<:NamedTuple})
    df = isempty(rows) ? DataFrame() : DataFrame(rows)
    CSV.write(path, df)
end

function _write_summary(path::AbstractString, row::NamedTuple)
    CSV.write(path, DataFrame([row]))
end

function _write_selected_columns(path::AbstractString, result::AggregateODRouteColumnGenerationResult)
    column_by_id = Dict(column.id => column for column in result.final_result.mapping.columns)
    rows = NamedTuple[]
    for column_id in result.selected_column_ids
        column = get(column_by_id, column_id, nothing)
        column === nothing && continue
        push!(rows, (
            column_id=column.id,
            n_pairs=length(column.od_pairs),
            tau=column.tau,
            metadata=string(column.metadata),
            pairs=string(Tuple(column.od_pairs)),
        ))
    end
    _write_namedtuple_rows(path, rows)
end

function main()
    mkpath.((
        RESULTS_DIR,
        ITERS_DIR,
        COLUMNS_DIR,
        DUALS_DIR,
        SELECTED_DIR,
    ))

    if isfile(CSV_PATH) && countlines(CSV_PATH) >= 2
        println("=== Skipping $INST_NAME — result already exists ===")
        return
    end

    println("===========================================")
    println("AggregateODRouteModel Scaling Experiment")
    println("===========================================")
    @printf("  Instance     : %s\n", INST_NAME)
    @printf("  Grid         : %d×%d (%d stations)\n", NX, NY, NX * NY)
    @printf("  Requests     : %d\n", N_REQUESTS)
    @printf("  Seed         : %d\n", SEED)
    @printf("  Time limit   : %.0fs CG  /  %.0fs IP  /  %.0fs per pricing iter\n",
            TIME_LIMIT, IP_TIME_LIMIT, PRICING_TIME)
    @printf("  Walk scale   : %.1f\n", WALK_SCALE)
    @printf("  Route scale  : %.1f\n", ROUTE_SCALE)
    println()

    stations, requests, data, walking_costs, demand_station_ids = _build_data(NX, NY, N_REQUESTS, SEED)
    max_walking_distance = isempty(MAX_WALKING_DISTANCE_ENV) ?
        _max_walking_distance(walking_costs, demand_station_ids) :
        parse(Float64, MAX_WALKING_DISTANCE_ENV)
    @printf("  Max walk dist: %.1f\n", max_walking_distance)
    @printf("  Demand set   : %s\n", string(demand_station_ids))
    println()

    model = AggregateODRouteModel(
        max(1, min(8, data.n_stations));
        route_regularization_weight=ROUTE_REGULARIZATION_WEIGHT,
        repositioning_time=REPOSITIONING_TIME,
        max_walking_distance=max_walking_distance,
        max_stops=2,
        max_visits_per_node=2,
        max_new_columns=MAX_NEW_COLS,
        n_candidates=max(MAX_NEW_COLS, 20),
        pricing_time_limit_sec=PRICING_TIME,
        relax_integrality=false,
    )

    t0 = time()
    result = run_aggregate_od_route_column_generation(
        model,
        data;
        verbose=false,
        cg_log_path=ITER_PATH,
        column_log_path=COLUMN_PATH,
        dual_log_path=DUAL_PATH,
        max_cg_iters=MAX_CG_ITERS,
        max_new_columns=MAX_NEW_COLS,
        n_candidates=max(MAX_NEW_COLS, 20),
        reduced_cost_tol=1e-6,
        pricing_time_limit_sec=PRICING_TIME,
        ip_time_limit_sec=IP_TIME_LIMIT,
        mip_gap=MIP_GAP,
        silent=true,
    )
    wall_time = time() - t0

    objective_value = result.final_result.objective_value
    lp_bound = result.lp_bound
    gap_pct = if !isnothing(objective_value) && isfinite(objective_value) &&
                 isfinite(lp_bound) && objective_value > 1e-10
        100.0 * (objective_value - lp_bound) / objective_value
    else
        NaN
    end

    @printf("  Status       : %s\n", result.status)
    @printf("  IP objective : %s\n", isnothing(objective_value) ? "n/a" : @sprintf("%.4f", objective_value))
    @printf("  LP bound     : %s\n", isfinite(lp_bound) ? @sprintf("%.4f", lp_bound) : "n/a")
    @printf("  Gap %%        : %s\n", isnan(gap_pct) ? "n/a" : @sprintf("%.2f%%", gap_pct))
    @printf("  CG iters     : %d  (%s)\n", result.n_cg_iters, result.cg_stop_reason)
    @printf("  Wall time    : %.1fs\n", wall_time)
    println()

    summary_row = (
        instance=INST_NAME,
        nx=NX,
        ny=NY,
        n_stations=nrow(stations),
        n_requests=nrow(requests),
        seed=SEED,
        max_walking_distance=max_walking_distance,
        route_regularization_weight=ROUTE_REGULARIZATION_WEIGHT,
        repositioning_time=REPOSITIONING_TIME,
        status=string(result.status),
        termination_status=string(result.final_result.termination_status),
        objective_value=isnothing(objective_value) ? "" : string(objective_value),
        lp_bound=string(lp_bound),
        integrality_gap_pct=isnan(gap_pct) ? "" : string(gap_pct),
        n_cg_iters=result.n_cg_iters,
        cg_stop_reason=string(result.cg_stop_reason),
        n_generated_columns=length(result.generated_columns),
        n_selected_columns=length(result.selected_column_ids),
        wall_time_sec=wall_time,
    )
    _write_summary(CSV_PATH, summary_row)
    println("Written: $CSV_PATH")

    _write_namedtuple_rows(ITER_PATH, result.iteration_rows)
    _write_namedtuple_rows(COLUMN_PATH, result.column_log_rows)
    _write_namedtuple_rows(DUAL_PATH, result.dual_log_rows)
    _write_selected_columns(SELECTED_PATH, result)
    println("Written: $ITER_PATH")
    println("Written: $COLUMN_PATH")
    println("Written: $DUAL_PATH")
    println("Written: $SELECTED_PATH")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
