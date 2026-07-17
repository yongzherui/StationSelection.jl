#!/usr/bin/env julia

using Gurobi
using JuMP
using CSV
using DataFrames
using StationSelection

const SAMPLE_DIR = normpath(joinpath(
    @__DIR__, "..", "..", "Data", "real_world_test_cases",
    "zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20",
    "sample_09_2025-03-03_11_15_midday_low",
))

const LOG_ROOT = "/tmp/zhuzhou_pair_chain_outer_gap"

function load_sample(n_stations::Int)
    all_stations = read_candidate_stations(joinpath(SAMPLE_DIR, "station.csv"))
    stations = all_stations[1:n_stations, :]
    keep = Set(Int.(stations.id))
    requests = read_customer_requests(
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
    walking_costs = compute_station_pairwise_costs(stations)
    routing_costs = read_routing_costs_from_segments(joinpath(SAMPLE_DIR, "segment.csv"), stations)
    return create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
end

function model(style::Symbol; allow_walk_only::Bool)
    return AggregateODRouteModel(
        5;
        assignment_policy=NearestOpenAggregateODAssignmentPolicy(style),
        max_walking_distance=500.0,
        route_regularization_weight=0.1,
        repositioning_time=0.0,
        max_stops=5,
        max_wait_time=3600.0,
        detour_factor=2.0,
        max_new_columns=50,
        n_candidates=50,
        pricing_time_limit_sec=30.0,
        allow_walk_only=allow_walk_only,
    )
end

function solve_case(data, style::Symbol; reprice::Bool, allow_walk_only::Bool, inner_mode::Symbol, log_dir::AbstractString)
    mkpath(log_dir)
    m = model(style; allow_walk_only=allow_walk_only)
    if inner_mode == :direct_full
        full_columns = enumerate_aggregate_od_route_columns(
            m,
            data;
            max_routes=100_000,
            time_limit_sec=60.0,
        )
        println("  full enumerated columns = ", length(full_columns))
        m = StationSelection._copy_with_initial_columns(m, full_columns)
    end
    inner = if inner_mode == :cg
        ColumnGenerationSolver(
            config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            max_iterations=500,
            max_columns_per_iteration=50,
            n_candidates=50,
            pricing_time_limit_sec=30.0,
            final_ip_time_limit_sec=300.0,
        )
    elseif inner_mode == :direct
        DirectSolver(
            optimizer_env=Gurobi.Env(),
            silent=true,
            mip_gap=0.0,
            max_enumerated_routes=100_000,
            max_enumeration_time_sec=60.0,
        )
    elseif inner_mode == :direct_full
        DirectSolver(
            optimizer_env=Gurobi.Env(),
            silent=true,
            mip_gap=0.0,
            max_enumerated_routes=100_000,
            max_enumeration_time_sec=60.0,
        )
    else
        throw(ArgumentError("unsupported inner_mode=$(inner_mode)"))
    end
    solver = BendersSolver(
        config=SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        decomposition=BendersY(),
        inner_solver=inner,
        max_iterations=120,
        log_dir=log_dir,
        reprice_subproblem=reprice,
        max_reprice_rounds=30,
    )
    return run_opt(data, m, solver)
end

function selected_ids(data, result)
    y = value.(result.model[:y])
    return [data.array_idx_to_station_id[j] for j in eachindex(y) if y[j] > 0.5]
end

function print_final_outer_gap(log_dir::AbstractString)
    path = joinpath(log_dir, "aggregate_od_route_benders_iterations.csv")
    if !isfile(path)
        println("  log       = missing: ", path)
        return nothing
    end
    rows = CSV.read(path, DataFrame)
    row = rows[end, :]
    println("  log       = ", path)
    println("  final LB  = ", row.lower_bound)
    println("  final UB  = ", row.incumbent_objective)
    println("  UB - LB   = ", row.outer_gap_absolute)
    println("  rel signed= ", row.outer_gap_relative)
    println("  rel abs   = ", row.outer_gap)
    return nothing
end

function main()
    data = load_sample(10)
    println("Loaded sample09 n=10: orders=$(sum(nrow(s.requests) for s in data.scenarios))")
    mkpath(LOG_ROOT)
    cases = (
        (:big_m_nearest, true),
        (:big_m_nearest, false),
        (:pair_chain, false),
    )
    for (style, allow_walk_only) in cases
        for (inner_mode, reprice) in ((:cg, false), (:cg, true), (:direct, false), (:direct_full, false))
            label = "$(style)_walk$(allow_walk_only)_inner$(inner_mode)_reprice$(reprice)"
            log_dir = joinpath(LOG_ROOT, label)
            result = solve_case(data, style; reprice=reprice, allow_walk_only=allow_walk_only, inner_mode=inner_mode, log_dir=log_dir)
            println("style=$(style), allow_walk_only=$(allow_walk_only), inner=$(inner_mode), reprice=$(reprice)")
            println("  objective = ", result.objective_value)
            println("  selected  = ", selected_ids(data, result))
            println("  reprice_columns = ", get(result.metadata, "total_reprice_columns_found", missing))
            println("  iterations = ", get(result.metadata, "benders_iterations", missing))
            println("  metadata UB - LB = ", get(result.metadata, "benders_outer_gap_absolute", missing))
            print_final_outer_gap(log_dir)
        end
    end
end

main()
