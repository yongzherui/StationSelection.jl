#!/usr/bin/env julia
#
# Empirical study of the x_{p,j,k}-coverage-row dual values (the raw duals of
# `sum_r a[r,p,j,k] theta[r] >= x[p,j,k]`, per (request, pair)) at one fixed y_hat:
#   (a) the un-repriced nearest-open subproblem LP (a CG-primed but possibly incomplete pool),
#   (b) the repriced (certified-complete) nearest-open subproblem LP,
#   (c) the fixed-assignment route-covering problem R(x_bar)'s own certified per-request duals
#       (Section C of notes/2026-07-17_restricted_mw_cut_benders_y.md), zero-extended.
# Also checks what happens when two distinct requests share the same (scenario, j, k)
# assignment: does the per-request dual split evenly, arbitrarily, or does one request
# "absorb" the whole credit -- and does it matter for cut validity?

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

function load_station_selection_sample(n_stations::Int)
    all_stations = SS.read_candidate_stations(joinpath(SAMPLE_DIR, "station.csv"))
    stations = all_stations[1:n_stations, :]
    keep = Set(Int.(stations.id))
    requests = SS.read_customer_requests(
        joinpath(SAMPLE_DIR, "order.csv");
        start_time="2025-03-03 11:00:00", end_time="2025-03-03 15:00:00",
    )
    requests = requests[
        in.(requests.origin_station_id, Ref(keep)) .&
        in.(requests.destination_station_id, Ref(keep)) .&
        (requests.origin_station_id .!= requests.destination_station_id),
        :,
    ]
    walking_costs = SS.compute_station_pairwise_costs(stations)
    routing_costs = SS.read_routing_costs_from_segments(joinpath(SAMPLE_DIR, "segment.csv"), stations)
    return SS.create_station_selection_data(stations, requests, walking_costs; routing_costs=routing_costs)
end

function ss_model(; l::Int, max_stops::Int)
    return SS.AggregateODRouteModel(
        l;
        assignment_policy=SS.NearestOpenAggregateODAssignmentPolicy(:big_m_nearest),
        max_walking_distance=500.0, route_regularization_weight=0.1, repositioning_time=0.0,
        max_stops=max_stops, max_wait_time=3600.0, detour_factor=2.0,
        max_new_columns=50, n_candidates=50, pricing_time_limit_sec=30.0, allow_walk_only=false,
    )
end

function y_hat_for_station_ids(data, ids::Vector{Int})
    y = zeros(data.n_stations)
    for id in ids
        y[data.station_id_to_array_idx[id]] = 1.0
    end
    return y
end

function main()
    optimizer_env = Gurobi.Env()
    n_stations = 15
    l = 7
    max_stops = 4
    data = load_station_selection_sample(n_stations)
    model = ss_model(l=l, max_stops=max_stops)
    mapping = SS.create_map(model, data)
    requests, demand, feasible_pairs = SS._aggregate_od_route_benders_requests(mapping)

    # The optimum every cut_derivation mode agreed on in check_zhuzhou_restricted_mw_timing.jl.
    y_hat = y_hat_for_station_ids(data, [11, 21, 22, 40, 129, 138, 196])
    open_stations = SS._open_station_values(y_hat)
    assignments, infeasible = SS._fixed_assignments_from_y(
        data, requests, feasible_pairs, y_hat;
        style=:big_m_nearest, max_walking_distance=model.max_walking_distance, allow_walk_only=false,
    )
    @assert isempty(infeasible)
    println("n_requests=", length(requests), "  n_distinct_assigned_pairs=", length(Set(values(assignments))))

    by_pair = Dict{Tuple{Int, Int, Int}, Vector{NTuple{3, Int}}}()
    for (req, pair) in assignments
        s, _o, _d = req
        push!(get!(by_pair, (s, pair[1], pair[2]), NTuple{3, Int}[]), req)
    end
    shared = Dict(k => v for (k, v) in by_pair if length(v) > 1)
    println("shared (s,j,k) assignments (>1 request): ", length(shared))
    for (k, reqs) in shared
        println("  (s,j,k)=", k, " requests=", reqs)
    end

    inner = SS.ColumnGenerationSolver(
        config=SS.SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        max_iterations=500, max_columns_per_iteration=50, n_candidates=50,
        pricing_time_limit_sec=30.0, final_ip_time_limit_sec=300.0,
    )
    solver = SS.BendersSolver(
        config=SS.SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=0.0),
        decomposition=SS.BendersY(), inner_solver=inner, max_iterations=120,
    )

    cg_seed = SS._solve_fixed_route_covering_by_cg(data, model, assignments, solver, nothing, open_stations)
    seed_pool = cg_seed.generated_columns
    println("seed pool size=", length(seed_pool), "  seed CG objective=", cg_seed.final_result.objective_value)

    # (a) Un-repriced LP duals.
    m_plain, _fix_cons_plain, _x_plain, cover_cons_plain = SS._build_nearest_open_y_subproblem_lp(
        data, model, mapping, requests, demand, feasible_pairs, seed_pool, y_hat, optimizer_env, true,
    )
    optimize!(m_plain)
    v_plain = objective_value(m_plain)
    duals_plain = SS._extract_nearest_open_y_subproblem_coverage_duals(cover_cons_plain)

    # (b) Repriced LP duals -- get the certified pool, then rebuild+resolve once more to expose
    # cover_cons duals (the production function doesn't return them directly).
    v_repriced, _rho_repriced, repriced_pool, n_new, rounds, exhausted, _delta =
        SS._solve_nearest_open_y_subproblem_lp_with_repricing(
            data, model, mapping, requests, demand, feasible_pairs, seed_pool, y_hat, optimizer_env, true,
        )
    println("repricing: n_new=", n_new, " rounds=", rounds, " exhausted=", exhausted, " v_plain=", v_plain, " v_repriced=", v_repriced)
    m_repriced, _fix_cons_repriced, _x_repriced, cover_cons_repriced = SS._build_nearest_open_y_subproblem_lp(
        data, model, mapping, requests, demand, feasible_pairs, repriced_pool, y_hat, optimizer_env, true,
    )
    optimize!(m_repriced)
    duals_repriced = SS._extract_nearest_open_y_subproblem_coverage_duals(cover_cons_repriced)

    # (c) R(x_bar) certified per-request duals (Section C), zero-extended.
    certified, qbar = SS._certified_qbar(data, model, solver, requests, assignments, open_stations)
    pi_full = SS._zero_extended_pi(requests, feasible_pairs, assignments, certified.pi_by_request)
    println("R(x_bar): r_value(=Q_bar)=", certified.r_value, " n_cg_iters=", certified.n_cg_iterations, " n_pool=", length(certified.pool))

    max_diff_at_1_plain = 0.0
    max_diff_at_1_repriced = 0.0
    max_diff_at_0_plain = 0.0
    max_diff_at_0_repriced = 0.0
    n_at_1 = 0
    n_at_0 = 0
    println("\nrequest, pair, x_bar, dual_plain, dual_repriced, dual_Rxbar")
    for req in requests, pair in feasible_pairs[req]
        SS.is_walk_only_pair(pair) && continue
        is_active = assignments[req] == pair
        d_plain = get(duals_plain.raw_duals, (req, pair), 0.0)
        d_repriced = get(duals_repriced.raw_duals, (req, pair), 0.0)
        d_rxbar = get(pi_full, (req, pair), 0.0)
        if is_active
            n_at_1 += 1
            max_diff_at_1_plain = max(max_diff_at_1_plain, abs(d_plain - d_rxbar))
            max_diff_at_1_repriced = max(max_diff_at_1_repriced, abs(d_repriced - d_rxbar))
        else
            n_at_0 += 1
            max_diff_at_0_plain = max(max_diff_at_0_plain, abs(d_plain - d_rxbar))
            max_diff_at_0_repriced = max(max_diff_at_0_repriced, abs(d_repriced - d_rxbar))
        end
        (is_active || abs(d_plain) > 1e-9 || abs(d_repriced) > 1e-9 || abs(d_rxbar) > 1e-9) &&
            println(req, " ", pair, " x=", is_active ? 1 : 0, "  plain=", round(d_plain; digits=4),
                "  repriced=", round(d_repriced; digits=4), "  Rxbar=", round(d_rxbar; digits=4))
    end
    println("\nn_at_x=1: ", n_at_1, "  max|plain-Rxbar|=", max_diff_at_1_plain, "  max|repriced-Rxbar|=", max_diff_at_1_repriced)
    println("n_at_x=0: ", n_at_0, "  max|plain-Rxbar|=", max_diff_at_0_plain, "  max|repriced-Rxbar|=", max_diff_at_0_repriced)

    println("\n--- shared-pair dual splits (does it matter?) ---")
    for (k, reqs) in shared
        s, j, kk = k
        pair = (j, kk)
        println("(s,j,k)=", k)
        for req in reqs
            println("  req=", req, " dual_plain=", get(duals_plain.raw_duals, (req, pair), 0.0),
                " dual_repriced=", get(duals_repriced.raw_duals, (req, pair), 0.0),
                " dual_Rxbar=", get(pi_full, (req, pair), 0.0))
        end
        println("  sum_plain=", sum(get(duals_plain.raw_duals, (req, pair), 0.0) for req in reqs),
            "  sum_repriced=", sum(get(duals_repriced.raw_duals, (req, pair), 0.0) for req in reqs),
            "  sum_Rxbar=", sum(get(pi_full, (req, pair), 0.0) for req in reqs),
            "  aggregated_sigma_plain(j,k,s)=", get(duals_plain.sigma, (j, kk, s), 0.0),
            "  aggregated_sigma_repriced(j,k,s)=", get(duals_repriced.sigma, (j, kk, s), 0.0))
    end
    return 0
end

main()
