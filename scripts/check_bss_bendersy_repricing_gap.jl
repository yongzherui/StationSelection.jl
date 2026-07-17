#!/usr/bin/env julia

using CSV
using DataFrames
using Dates
using Gurobi
using JuMP

import StationSelection
const SS = StationSelection

const BSS_ROOT = normpath(joinpath(@__DIR__, "..", "..", "..", "exploration", "BendersStationSelection.jl"))
include(joinpath(BSS_ROOT, "src", "BendersStationSelection.jl"))
const BSS = BendersStationSelection

const SAMPLE_DIR = normpath(joinpath(
    @__DIR__, "..", "..", "Data", "real_world_test_cases",
    "zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20",
    "sample_09_2025-03-03_11_15_midday_low",
))

# BendersStationSelection.jl's BendersY path currently calls this helper but
# does not define it. Inject the missing routine for this empirical check only.
if !isdefined(BSS, :add_column_for_required_pairs!)
    @eval BendersStationSelection begin
        function add_column_for_required_pairs!(
            model::JuMP.Model,
            pool::CompatibilitySetPool{T},
            column::CompatibilitySet{T},
            link_cons::Dict{Tuple{T,T},JuMP.ConstraintRef},
            lambda_vars::Vector{JuMP.VariableRef};
            route_cost_weight::Float64,
        ) where T
            new_column = assign_compatibility_set_id(column, length(pool.sets) + 1)
            push!(pool.sets, new_column)
            λ = @variable(model, lower_bound = 0.0, upper_bound = 1.0, base_name = "λ[$(length(pool.sets))]")
            push!(lambda_vars, λ)
            for pair in new_column.pairs
                haskey(link_cons, pair) && JuMP.set_normalized_coefficient(link_cons[pair], λ, 1.0)
            end
            JuMP.set_objective_coefficient(model, λ, route_cost_weight * new_column.kappa)
            return λ
        end
    end
end

function gurobi_available()
    try
        Gurobi.Env()
        return true
    catch err
        @warn "Gurobi is unavailable; cannot run check" exception=(err, catch_backtrace())
        return false
    end
end

function load_station_selection_sample(n_stations::Int)
    all_stations = SS.read_candidate_stations(joinpath(SAMPLE_DIR, "station.csv"))
    stations = all_stations[1:n_stations, :]
    keep = Set(Int.(stations.id))
    requests = SS.read_customer_requests(
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
    walking_costs = SS.compute_station_pairwise_costs(stations)
    routing_costs = SS.read_routing_costs_from_segments(joinpath(SAMPLE_DIR, "segment.csv"), stations)
    data = SS.create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
    return data, walking_costs, routing_costs
end

function bss_problem_from_sample(data, walking_costs, routing_costs; max_walking_distance::Float64, max_stops::Int)
    nodes = Int.(data.array_idx_to_station_id)
    travel_cost = Dict{Tuple{Int,Int},Float64}()
    for i in nodes, j in nodes
        travel_cost[(i, j)] = get(routing_costs, (i, j), Inf)
    end

    request_feasible_pairs = Dict{Int,Vector{Tuple{Int,Int}}}()
    assignment_cost = Dict{Tuple{Int,Tuple{Int,Int}},Float64}()
    candidate_pair_set = Set{Tuple{Int,Int}}()
    reqs = data.scenarios[1].requests
    for (rid, row) in enumerate(eachrow(reqs))
        origin = Int(row.origin_station_id)
        dest = Int(row.destination_station_id)
        pairs = Tuple{Int,Int}[]
        for pickup in nodes, dropoff in nodes
            pickup == dropoff && continue
            walk_o = get(walking_costs, (origin, pickup), Inf)
            walk_d = get(walking_costs, (dropoff, dest), Inf)
            isfinite(walk_o) && isfinite(walk_d) || continue
            walk_o <= max_walking_distance + 1e-9 || continue
            walk_d <= max_walking_distance + 1e-9 || continue
            pair = (pickup, dropoff)
            push!(pairs, pair)
            push!(candidate_pair_set, pair)
            assignment_cost[(rid, pair)] = walk_o + walk_d
        end
        sort!(unique!(pairs), by=string)
        isempty(pairs) && error("request $rid ($origin => $dest) has no BSS feasible pairs")
        request_feasible_pairs[rid] = pairs
    end

    network = BSS.NetworkPricingData(
        nodes,
        travel_cost,
        sort!(collect(candidate_pair_set), by=string),
        0.0,
        3600.0,
        2.0,
        max_stops,
    )
    return BSS.StationSelectionProblemData(network, request_feasible_pairs; assignment_cost=assignment_cost)
end

function ss_model(; max_stops)
    return SS.AggregateODRouteModel(
        5;
        assignment_policy=SS.NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
        max_walking_distance=500.0,
        route_regularization_weight=0.1,
        repositioning_time=0.0,
        max_stops=max_stops,
        max_wait_time=3600.0,
        detour_factor=2.0,
        max_new_columns=50,
        n_candidates=50,
        pricing_time_limit_sec=30.0,
        allow_walk_only=true,
    )
end

function run_ss_benders_y(data, model; reprice::Bool)
    inner = SS.ColumnGenerationSolver(
        config=SS.SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        max_iterations=500,
        max_columns_per_iteration=50,
        n_candidates=50,
        pricing_time_limit_sec=30.0,
        final_ip_time_limit_sec=300.0,
    )
    return SS.run_opt(
        data,
        model,
        SS.BendersSolver(
            config=SS.SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            decomposition=SS.BendersY(),
            inner_solver=inner,
            max_iterations=120,
            reprice_subproblem=reprice,
            max_reprice_rounds=30,
        ),
    )
end

function run_bss(problem; mode::Symbol)
    model = BSS.NearestOpenModel(
        5;
        solve_mode=mode,
        feasibility_cut_style=:gamma_chain,
        assignment_cost_weight=1.0,
        route_cost_weight=0.1,
        max_benders_iters=120,
        max_cg_iters=500,
        max_new_columns=50,
        n_candidates=50,
        reduced_cost_tol=1e-6,
        pricing_time_limit_sec=30.0,
        pricing_escalated_time_limit_sec=60.0,
        max_visits_per_node=2,
        ip_time_limit_sec=300.0,
        mip_gap=0.0,
    )
    return BSS.run_opt(model, problem; verbose=false)
end

function selected_ss(data, result)
    vals = JuMP.value.(result.model[:y])
    ids = Int[]
    for j in eachindex(vals)
        vals[j] > 0.5 && push!(ids, data.array_idx_to_station_id[j])
    end
    return ids
end

function print_ss(label, data, result)
    println(label)
    println("  objective = ", result.objective_value)
    println("  status    = ", result.termination_status)
    println("  selected  = ", selected_ss(data, result))
    println("  metadata  = ", result.metadata)
end

function print_bss(label, result)
    println(label)
    println("  objective = ", result.raw_result.objective_value)
    println("  status    = ", result.status)
    println("  selected  = ", sort(collect(result.raw_result.selected_stations)))
    println("  metadata  = ", result.metadata)
end

function main()
    gurobi_available() || return 77
    data, walking_costs, routing_costs = load_station_selection_sample(10)
    println("Loaded Zhuzhou sample09 n=10: n_orders=$(nrow(data.scenarios[1].requests)), n_stations=$(data.n_stations)")

    # Existing no-max-stops run has the same objective split as max_stops=5;
    # BSS requires an integer max_stops, so this is the comparable setting.
    current_model = ss_model(max_stops=5)
    ss_plain = run_ss_benders_y(data, current_model; reprice=false)
    ss_repriced = run_ss_benders_y(data, current_model; reprice=true)

    bss_problem = bss_problem_from_sample(data, walking_costs, routing_costs; max_walking_distance=500.0, max_stops=5)
    bss_y = run_bss(bss_problem; mode=:BendersY)
    bss_xy = run_bss(bss_problem; mode=:BendersXY)

    println("\n=== Current StationSelection.jl ===")
    print_ss("BendersY plain", data, ss_plain)
    print_ss("BendersY + repricing", data, ss_repriced)

    println("\n=== ../../exploration/BendersStationSelection.jl ===")
    print_bss("BSS BendersY no-repricing", bss_y)
    print_bss("BSS BendersXY", bss_xy)

    ref = ss_repriced.objective_value
    println("\nObjective deltas against current repriced BendersY:")
    println("  current plain BendersY = ", abs(ss_plain.objective_value - ref))
    println("  BSS BendersY           = ", abs(bss_y.raw_result.objective_value - ref))
    println("  BSS BendersXY          = ", abs(bss_xy.raw_result.objective_value - ref))
    return 0
end

if abspath(PROGRAM_FILE) == @__FILE__
    exit(main())
end
