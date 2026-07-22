#!/usr/bin/env julia
#
# Checks whether the restricted-MW-fixed-pi cut (cut_derivation=:restricted_mw_fixed_pi),
# used WITHOUT subproblem repricing, closes the same BendersY premature-convergence gap that
# notes/2026-07-15_bendersy_stale_cut_soundness.md documents plain BendersY (cut_derivation=
# :standard, reprice_subproblem=false) falling into on the 10-station Zhuzhou sample -- i.e.
# whether the restricted-MW cut is, on this fixture, a genuine alternative fix to repricing
# rather than just a variant that still needs repricing to be correct.
#
# allow_walk_only is forced off here (the restricted-MW cut derivation does not support it,
# see notes/2026-07-17_restricted_mw_cut_benders_y.md) -- this is a deliberate divergence from
# scripts/check_bss_bendersy_repricing_gap.jl's ss_model, which does allow walk-only.

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
            cut_derivation=cut_derivation,
        ),
    )
end

function selected_ss(data, result)
    vals = JuMP.value.(result.model[:y])
    ids = Int[]
    for j in eachindex(vals)
        vals[j] > 0.5 && push!(ids, data.array_idx_to_station_id[j])
    end
    return sort(ids)
end

function print_ss(label, data, result)
    println(label)
    println("  objective = ", result.objective_value)
    println("  status    = ", result.termination_status)
    println("  selected  = ", selected_ss(data, result))
    println("  benders_iterations = ", get(result.metadata, "benders_iterations", nothing))
end

function main()
    gurobi_available() || return 77
    data = load_station_selection_sample(10)
    println("Loaded Zhuzhou sample09 n=10: n_orders=$(nrow(data.scenarios[1].requests)), n_stations=$(data.n_stations)")

    for max_stops in (3, 4, 5)
        println("\n================ max_stops=$(max_stops) ================")
        model = ss_model(max_stops=max_stops)

        ss_plain = run_ss_benders_y(data, model; reprice=false, cut_derivation=:standard)
        ss_repriced = run_ss_benders_y(data, model; reprice=true, cut_derivation=:standard)
        ss_mw_no_reprice = run_ss_benders_y(data, model; reprice=false, cut_derivation=:restricted_mw_fixed_pi)
        ss_zero_no_reprice = run_ss_benders_y(data, model; reprice=false, cut_derivation=:zero_completion)

        print_ss("BendersY standard, no repricing (known-bad baseline)", data, ss_plain)
        print_ss("BendersY standard, WITH repricing (known-good ground truth)", data, ss_repriced)
        print_ss("BendersY restricted-MW-fixed-pi, NO repricing", data, ss_mw_no_reprice)
        print_ss("BendersY zero-completion, NO repricing", data, ss_zero_no_reprice)

        ref = ss_repriced.objective_value
        println("\nObjective deltas against repriced ground truth ($(ref)):")
        println("  standard, no repricing        = ", abs(ss_plain.objective_value - ref))
        println("  restricted-MW, no repricing   = ", abs(ss_mw_no_reprice.objective_value - ref))
        println("  zero-completion, no repricing = ", abs(ss_zero_no_reprice.objective_value - ref))
    end
    return 0
end

if abspath(PROGRAM_FILE) == @__FILE__
    exit(main())
end
