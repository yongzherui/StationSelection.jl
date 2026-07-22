#!/usr/bin/env julia
#
# Wall-time and iteration-count comparison between BendersY(cut_derivation=:standard,
# reprice_subproblem=true) (the known-good, previously the only trustworthy no-repricing-bug
# path), BendersY(cut_derivation=:zero_completion, reprice_subproblem=false), and
# BendersY(cut_derivation=:restricted_mw_fixed_pi, reprice_subproblem=false), on the 10- and
# 15-station Zhuzhou samples. allow_walk_only forced off (restricted-completion cut modes don't
# support it -- see notes/2026-07-17_restricted_mw_cut_benders_y.md).

using DataFrames
using Gurobi
using JuMP

import StationSelection
const SS = StationSelection

const SAMPLE_DIR = normpath(joinpath(
    @__DIR__, "..", "..", "Data", "real_world_test_cases",
    "zhuzhou_kmedoid4_2025-05-05_16_20_top10_plus_c3top10_cap20",
    "sample_09_2025-03-03_11_15_midday_low",
))

function gurobi_available()
    try
        Gurobi.Env()
        return true
    catch err
        @warn "Gurobi is unavailable; cannot run check" exception = (err, catch_backtrace())
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
    return data
end

function ss_model(; l::Int, max_stops::Int)
    return SS.AggregateODRouteModel(
        l;
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
        allow_walk_only=false,
    )
end

function run_ss_benders_y(data, model; reprice::Bool, cut_derivation::Symbol)
    inner = SS.ColumnGenerationSolver(
        config=SS.SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
        max_iterations=500,
        max_columns_per_iteration=50,
        n_candidates=50,
        pricing_time_limit_sec=30.0,
        final_ip_time_limit_sec=300.0,
    )
    elapsed = @elapsed result = SS.run_opt(
        data,
        model,
        SS.BendersSolver(
            config=SS.SolverConfig(optimizer_env=Gurobi.Env(), silent=true, mip_gap=0.0),
            decomposition=SS.BendersY(),
            inner_solver=inner,
            max_iterations=120,
            reprice_subproblem=reprice,
            max_reprice_rounds=30,
            cut_derivation=cut_derivation,
        ),
    )
    return result, elapsed
end

function selected_ss(data, result)
    vals = JuMP.value.(result.model[:y])
    ids = Int[]
    for j in eachindex(vals)
        vals[j] > 0.5 && push!(ids, data.array_idx_to_station_id[j])
    end
    return sort(ids)
end

function report(label, data, result, elapsed)
    println(rpad(label, 46),
        " obj=", round(result.objective_value; digits=4),
        "  iters=", get(result.metadata, "benders_iterations", nothing),
        "  wall_sec=", round(elapsed; digits=3),
        "  selected=", selected_ss(data, result))
end

function main()
    gurobi_available() || return 77
    for n_stations in (10, 15)
        l = n_stations == 10 ? 5 : 7
        data = load_station_selection_sample(n_stations)
        println("\n#### n_stations=$(n_stations), l=$(l), n_orders=$(nrow(data.scenarios[1].requests)) ####")
        for max_stops in (4,)
            println("--- max_stops=$(max_stops) ---")
            model = ss_model(l=l, max_stops=max_stops)

            r_std_reprice, t_std_reprice = run_ss_benders_y(data, model; reprice=true, cut_derivation=:standard)
            r_zero, t_zero = run_ss_benders_y(data, model; reprice=false, cut_derivation=:zero_completion)
            r_mw, t_mw = run_ss_benders_y(data, model; reprice=false, cut_derivation=:restricted_mw_fixed_pi)

            report("standard + repricing (ground truth)", data, r_std_reprice, t_std_reprice)
            report("zero_completion, no repricing", data, r_zero, t_zero)
            report("restricted_mw_fixed_pi, no repricing", data, r_mw, t_mw)

            ref = r_std_reprice.objective_value
            println("  objective match: zero_completion=", isapprox(r_zero.objective_value, ref; atol=1e-6),
                "  restricted_mw=", isapprox(r_mw.objective_value, ref; atol=1e-6))
            println("  wall-time ratio vs repricing: zero_completion=", round(t_zero / t_std_reprice; digits=3),
                "x  restricted_mw=", round(t_mw / t_std_reprice; digits=3), "x")
            println("  wall-time ratio: restricted_mw / zero_completion=", round(t_mw / t_zero; digits=3), "x")
        end
    end
    return 0
end

if abspath(PROGRAM_FILE) == @__FILE__
    exit(main())
end
