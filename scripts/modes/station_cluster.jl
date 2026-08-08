"""
    diagnose.jl station_cluster -- adaptive station-cluster certification for
    passenger free-assignment pricing: does clustering stations and refining
    the cluster partition on demand certify the LP bound as reliably (and
    faster) as exhaustive full-network pricing?

Two variants:

  pricing <n> [seed] [scenario]
      One-shot pricing A/B on a seeded-RMP dual snapshot: EXACT full-network
      pricing vs. the INITIAL cluster lower bound vs. iterative REFINE rounds
      vs. the FINAL adaptively-certified bound. No args runs the default grid
      (n in 10,15; seed in 42,314,2718; scenario in 1:3).

  cg <n> <seed> <baseline|cluster> <output.txt>
      Full passenger free-assignment CG loop, with adaptive cluster
      certification either off (baseline) or on (cluster). Budgets are sized
      for n<=~30 by default; pass the PFA_DIAG_* env overrides below to
      reproduce the larger n=50-100 scaling study (TOTAL_TIME=18000
      CERT_TIME=3600 CLUSTER_TIME=3600 IP_TIME=1800 MAX_CG_ITERS=2000
      N_CANDIDATES=20 MAX_NEW_COLUMNS=20).

Usage:
    julia --project=. scripts/diagnose.jl station_cluster pricing [n] [seed] [scenario]
    julia --project=. scripts/diagnose.jl station_cluster cg <n> <seed> <baseline|cluster> <output.txt>

Env overrides for `cg` (all prefixed PFA_DIAG_):
    MAX_STOPS default 4   MAX_VISITS default 3   TOTAL_TIME default 5400
    CERT_TIME default 900   CLUSTER_TIME default 300   IP_TIME default 900
    N_CANDIDATES default 100   MAX_NEW_COLUMNS default 20   MAX_CG_ITERS default 1000
"""

function _station_cluster_pricing_case(n::Int, seed::Int, scenario::Int)
    n_pairs = 16
    data, _ = diag_zz_data(n; n_pairs=n_pairs, n_scenarios=3, seed=seed)
    travel = diag_travel_cost(data, n)
    candidates = diag_scenario_candidates(data, n, scenario)
    println("CASE n=$n seed=$seed scenario=$scenario candidates=$(length(candidates)) passengers=$(length(unique(c.passenger for c in candidates)))")
    pd = create_passenger_free_assignment_pricing_data(scenario, collect(1:n), travel, candidates;
        route_regularization_weight=10.0, max_wait_time=900.0, repositioning_time=20.0,
        max_stops=4, max_visits_per_node=3)

    exact = price_exact_on_stations(pd, BitSet(pd.nodes); time_limit=300.0, use_reduced_cost_pruning=false)
    @printf("EXACT n=%d seed=%d scenario=%d rc=%.6f certified=%s labels=%d time=%.3f route=%s\n",
        n, seed, scenario, exact.reduced_cost, string(exact.certified), exact.labels_generated, exact.runtime_sec, string(exact.route))

    k0 = diag_l_for(n; divisor=3)
    cfg = StationClusteringConfig(initial_num_clusters=k0, max_num_clusters=n,
        max_cluster_size=ceil(Int, n / k0) + 1, pricing_tolerance=1e-6,
        numerical_tolerance=1e-6, time_limit=300.0)
    h = initial_station_clustering(travel, n, cfg)
    cache = build_cluster_pricing_cache(h, pd, candidates)
    rows = NamedTuple[]
    initial = solve_cluster_pricer(h, cache)
    @printf("INITIAL n=%d seed=%d scenario=%d K=%d lb=%.6f gap_to_exact=%.6f labels=%d time=%.3f route=%s\n",
        n, seed, scenario, length(h.clusters), initial.lower_bound_reduced_cost,
        exact.reduced_cost - initial.lower_bound_reduced_cost, initial.labels_generated,
        initial.runtime_seconds, string(initial.cluster_route))
    result, reason = solve_adaptive_cluster_lower_bound(h, cache; logger=row -> push!(rows, row))
    for row in rows
        @printf("REFINE n=%d seed=%d scenario=%d iter=%d K=%d lb=%.6f split=%s score=%.4f time=%.3f stop=%s\n",
            n, seed, scenario, row.iteration, row.num_clusters, row.lower_bound_reduced_cost,
            string(row.selected_cluster_to_split), row.split_score, row.cluster_pricing_time, string(row.stop_reason))
    end
    valid = result.lower_bound_reduced_cost <= exact.reduced_cost + 1e-6
    @printf("FINAL n=%d seed=%d scenario=%d K=%d lb=%.6f exact=%.6f valid=%s certified_nonnegative=%s stop=%s labels=%d time=%.3f\n",
        n, seed, scenario, length(h.clusters), result.lower_bound_reduced_cost, exact.reduced_cost, string(valid),
        string(result.certified_no_negative_column), string(reason), result.labels_generated, result.runtime_seconds)
end

function _run_station_cluster_pricing(args::Vector{String})
    if isempty(args)
        for n in (10, 15), seed in (42, 314, 2718), scenario in 1:3
            _station_cluster_pricing_case(n, seed, scenario)
        end
    else
        length(args) == 3 || error("usage: diagnose.jl station_cluster pricing [n seed scenario]")
        _station_cluster_pricing_case(parse(Int, args[1]), parse(Int, args[2]), parse(Int, args[3]))
    end
end

function _run_station_cluster_cg(args::Vector{String})
    length(args) == 4 || error("usage: diagnose.jl station_cluster cg <n> <seed> <baseline|cluster> <output.txt>")
    n = parse(Int, args[1]); seed = parse(Int, args[2]); mode = args[3]; output = args[4]
    mode in ("baseline", "cluster") || error("mode must be baseline or cluster")

    max_stops = diag_unbounded(env_int("PFA_DIAG_MAX_STOPS", 4))
    max_visits = diag_unbounded(env_int("PFA_DIAG_MAX_VISITS", 3))
    total_time = env_float("PFA_DIAG_TOTAL_TIME", 5400.0)
    cert_time = env_float("PFA_DIAG_CERT_TIME", 900.0)
    cluster_time = env_float("PFA_DIAG_CLUSTER_TIME", 300.0)
    ip_time = env_float("PFA_DIAG_IP_TIME", 900.0)
    n_candidates = env_int("PFA_DIAG_N_CANDIDATES", 100)
    max_new_columns = env_int("PFA_DIAG_MAX_NEW_COLUMNS", 20)
    max_cg_iters = env_int("PFA_DIAG_MAX_CG_ITERS", 1000)

    data, _ = diag_zz_data(n; n_pairs=16, n_scenarios=3, seed=seed)
    model = diag_zz_model(n; max_stops=max_stops, max_visits_per_node=max_visits)
    t0 = time()
    result = run_passenger_free_assignment_column_generation(model, data;
        optimizer_env=diag_grb_env(), max_cg_iters=max_cg_iters, n_candidates=n_candidates,
        max_new_columns=max_new_columns, pricing_time_limit_sec=120.0,
        certification_time_limit_sec=cert_time, ip_time_limit_sec=ip_time,
        total_time_limit_sec=total_time, parallel_scenarios=true, station_simple_warm_start=false,
        use_adaptive_cluster_certification=(mode == "cluster"),
        cluster_initial_num_clusters=diag_l_for(n; divisor=3), cluster_max_num_clusters=n,
        cluster_time_limit_sec=cluster_time, verify_reduced_costs=true, verbose=true)
    wall = time() - t0
    cluster_rows = get(result.final_result.metadata, "cluster_certificate_rows", NamedTuple[])
    n_attempts = length(cluster_rows)
    n_scenario_cert = count(r -> r.certified, cluster_rows)
    cert_rounds = Set((r.round, r.iteration) for r in cluster_rows if r.certified)
    all_rounds = Set((r.round, r.iteration) for r in cluster_rows)
    fully_certified_rounds = count(key -> all(r -> r.certified,
        filter(r -> (r.round, r.iteration) == key, cluster_rows)), all_rounds)
    cluster_seconds = sum((r.seconds for r in cluster_rows); init=0.0)
    max_clusters = maximum((r.num_clusters_after for r in cluster_rows); init=0)

    mkpath(dirname(output))
    open(output, "w") do io
        @printf(io, "SUMMARY n=%d seed=%d scenarios=3 max_stops=%s max_visits=%s n_candidates=%d max_new_columns=%d mode=%s status=%s stop=%s lp_certified=%s cg_iters=%d rounds=%d columns=%d labels=%d pricing_seconds=%.6f certification_seconds=%.6f total_seconds=%.6f wall=%.6f\n",
            n, seed, max_stops == typemax(Int) ? "uncapped" : string(max_stops),
            max_visits == typemax(Int) ? "uncapped" : string(max_visits), n_candidates, max_new_columns, mode,
            string(result.status), string(result.cg_stop_reason), string(result.lp_bound_certified),
            result.n_cg_iters, result.n_rounds, result.n_columns, result.total_labels_generated,
            result.total_pricing_seconds, result.certification_seconds, result.total_seconds, wall)
        @printf(io, "CLUSTER attempts=%d scenario_certificates=%d certification_rounds=%d fully_certified_rounds=%d cluster_seconds=%.6f max_clusters=%d\n",
            n_attempts, n_scenario_cert, length(cert_rounds), fully_certified_rounds, cluster_seconds, max_clusters)
        for r in cluster_rows
            @printf(io, "CLUSTER_ROW round=%d iteration=%d scenario=%d K_before=%d K_after=%d lb=%.9f certified=%s stop=%s labels=%d seconds=%.6f route=%s\n",
                r.round, r.iteration, r.scenario, r.num_clusters_before, r.num_clusters_after,
                r.lower_bound_reduced_cost, string(r.certified), r.stop_reason, r.labels_generated, r.seconds, r.cluster_route)
        end
    end
    println(read(output, String))
end

function run_station_cluster(args::Vector{String})
    length(args) >= 1 || error(
        "usage: diagnose.jl station_cluster <pricing|cg> ...  (see mode docstring)",
    )
    variant, rest = args[1], args[2:end]
    if variant == "pricing"
        _run_station_cluster_pricing(rest)
    elseif variant == "cg"
        _run_station_cluster_cg(rest)
    else
        error("unknown station_cluster variant '$variant' (expected pricing|cg)")
    end
end

register_mode!("station_cluster", run_station_cluster)
