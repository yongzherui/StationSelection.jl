"""
    diagnose.jl station_subset_pricing -- benchmark certified station-subset
    pricing (branch-and-bound over station subsets) against exact full-network
    pricing, on a seeded-RMP dual snapshot.

Usage:
    julia --project=. scripts/diagnose.jl station_subset_pricing <n_stations> <output.csv>

Env overrides (all prefixed PFASS_):
    N_PAIRS default 16          TOTAL_TIME default 3000   ORACLE_TIME default 300
    EARLY_TIME default 300      INTEGRAL_REWARD default 0  ROUTING_BOUND default 1
    TRIPLE default 0            TRIPLE_ALTS default 3       PHASE default "both" (both|direct|bnb)
    EXACT_PRUNE default 1       EXACT_POSTW default 0        NODE_LIMIT default 0 (0 => unbounded)
    MAX_STOPS default 4 (0 => unbounded)
"""

function run_station_subset_pricing(args::Vector{String})
    length(args) == 2 || error("usage: diagnose.jl station_subset_pricing <n_stations> <output.csv>")
    n = parse(Int, args[1]); output = args[2]

    n_pairs = env_int("PFASS_N_PAIRS", 16)
    total_time = env_float("PFASS_TOTAL_TIME", 3000.0)
    oracle_time = env_float("PFASS_ORACLE_TIME", 300.0)
    early_time = env_float("PFASS_EARLY_TIME", 300.0)
    integral_reward = env_bool("PFASS_INTEGRAL_REWARD", false)
    routing_bound = env_bool("PFASS_ROUTING_BOUND", true)
    use_triple = env_bool("PFASS_TRIPLE", false)
    triple_alts = env_int("PFASS_TRIPLE_ALTS", 3)
    phase = get(ENV, "PFASS_PHASE", "both")
    exact_prune = env_bool("PFASS_EXACT_PRUNE", true)
    exact_postw = env_bool("PFASS_EXACT_POSTW", false)
    node_limit = diag_unbounded(env_int("PFASS_NODE_LIMIT", 0))
    max_stops = diag_unbounded(env_int("PFASS_MAX_STOPS", 4))
    grb_env = diag_grb_env()
    optimizer = () -> Gurobi.Optimizer(grb_env)

    l = diag_l_for(n)
    data, _ = diag_zz_data(n; n_pairs=n_pairs, n_scenarios=1, seed=42)
    model = diag_zz_model(n; l=l, max_stops=max_stops)
    _master, _mapping, md, alpha, gamma_o, gamma_d = diag_dual_snapshot(model, data; grb_env=grb_env)
    candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, 1)
    pd = create_passenger_free_assignment_pricing_data(1, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time, repositioning_time=md.repositioning_time,
        max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node)

    row(mode, c) = (; n, L=l, mode, optimal_value=c.optimal_value,
        reduced_cost=c.best_exact_result.reduced_cost,
        improving=c.optimal_value > 1e-6, globally_certified=c.globally_certified,
        global_upper_bound=c.final_global_upper_bound, absolute_gap=c.absolute_gap,
        nodes_created=c.nodes_created, nodes_processed=c.nodes_processed,
        pruned_cheap=c.nodes_pruned_cheap, pruned_lp=c.nodes_pruned_lp,
        fixed_priced=c.fixed_subsets_priced, heuristic_priced=c.heuristic_subsets_priced,
        unique_priced=c.unique_subsets_priced, lp_solves=c.reward_lp_solves,
        exact_seconds=c.total_exact_pricing_time, bound_seconds=c.total_bound_time,
        total_seconds=c.total_runtime_sec, labels=c.best_exact_result.labels_generated,
        station_set=join(collect(c.best_exact_result.station_set), ';'),
        route=join(c.best_exact_result.route, ';'))

    baseline_row(r) = (; n, L=l, mode="full_network_exact",
        optimal_value=r.value, reduced_cost=r.reduced_cost, improving=r.value > 1e-6,
        globally_certified=r.certified, global_upper_bound=r.certified ? r.value : Inf,
        absolute_gap=r.certified ? 0.0 : Inf, nodes_created=0, nodes_processed=0,
        pruned_cheap=0, pruned_lp=0, fixed_priced=1, heuristic_priced=0,
        unique_priced=1, lp_solves=0, exact_seconds=r.runtime_sec, bound_seconds=0.0,
        total_seconds=r.runtime_sec, labels=r.labels_generated,
        station_set=join(collect(r.station_set), ';'), route=join(r.route, ';'))

    rows = NamedTuple[]
    variant = (integral_reward ? "integral" : "lp") *
              (routing_bound ? "_routing" : "_rewardonly") *
              (use_triple ? "_triple" : "")
    phase in ("both", "direct", "bnb") || error("PFASS_PHASE must be both|direct|bnb")

    if phase in ("both", "bnb")
        early = price_by_station_subset_branch_and_bound(pd, l; optimizer=optimizer,
            settings=StationSubsetPricingSettings(stop_on_first_improving_column=true,
                integral_reward_stations=integral_reward,
                use_routing_reward_bound=routing_bound,
                use_triple_routing_bounds=use_triple, triple_alternatives_per_passenger=triple_alts,
                node_limit=node_limit,
                time_limit=early_time, exact_oracle_time_limit=min(oracle_time, early_time), verbose=true))
        push!(rows, row("early_column_$(variant)", early)); diag_safe_csv_write(output, DataFrame(rows)); flush(stdout)
        @printf("EARLY n=%d L=%d profit=%.6f certified=%s ub=%.6f time=%.1f\n",
            n, l, early.optimal_value, string(early.globally_certified), early.final_global_upper_bound, early.total_runtime_sec)
    end
    if phase in ("both", "direct")
        baseline = price_exact_on_stations(pd, BitSet(pd.nodes); time_limit=oracle_time,
            use_reduced_cost_pruning=exact_prune, use_post_w_completion_bound=exact_postw)
        push!(rows, baseline_row(baseline)); diag_safe_csv_write(output, DataFrame(rows)); flush(stdout)
        @printf("BASELINE n=%d profit=%.6f certified=%s labels=%d time=%.1f\n",
            n, baseline.value, string(baseline.certified), baseline.labels_generated, baseline.runtime_sec)
    end
    if phase in ("both", "bnb")
        full = price_by_station_subset_branch_and_bound(pd, l; optimizer=optimizer,
            settings=StationSubsetPricingSettings(integral_reward_stations=integral_reward,
                use_routing_reward_bound=routing_bound,
                use_triple_routing_bounds=use_triple, triple_alternatives_per_passenger=triple_alts,
                node_limit=node_limit,
                time_limit=total_time,
                exact_oracle_time_limit=oracle_time, verbose=true))
        push!(rows, row("certification_$(variant)", full)); diag_safe_csv_write(output, DataFrame(rows)); flush(stdout)
        @printf("CERT n=%d L=%d profit=%.6f certified=%s ub=%.6f gap=%.6f nodes=%d subsets=%d time=%.1f\n",
            n, l, full.optimal_value, string(full.globally_certified), full.final_global_upper_bound,
            full.absolute_gap, full.nodes_processed, full.unique_subsets_priced, full.total_runtime_sec)
    end
end

register_mode!("station_subset_pricing", run_station_subset_pricing)
