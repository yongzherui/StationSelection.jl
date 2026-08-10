"""
    diagnose.jl lagrangian_gap -- measure the passenger-uniqueness Lagrangian
    relaxation (and the post-W completion bound) on the first seeded RMP dual
    snapshot. A cheap gate before integrating either into the full CG
    trajectory. Reports bound validity, repeated-passenger multiplicity, exact
    replayed route quality, labels, and wall time for 1/3/5 multiplier rounds.

Usage:
    julia --project=. scripts/diagnose.jl lagrangian_gap <n_stations>

Env overrides (all prefixed PFALAG_ except PFAPOSTW_SAMPLES):
    N_PAIRS default 16   SEED default 42   N_SCENARIOS default 3
    TIME default 300 (per-search time limit)   ROUNDS default "1,3,5"
    PFAPOSTW_SAMPLES default 10
"""

function run_lagrangian_gap(args::Vector{String})
    length(args) == 1 || error("usage: diagnose.jl lagrangian_gap <n_stations>")
    n = parse(Int, args[1])

    n_pairs = env_int("PFALAG_N_PAIRS", 16)
    seed = env_int("PFALAG_SEED", 42)
    n_scenarios = env_int("PFALAG_N_SCENARIOS", 3)
    time_limit = env_float("PFALAG_TIME", 300.0)
    rounds_list = env_ints("PFALAG_ROUNDS", "1,3,5")
    post_w_samples = env_int("PFAPOSTW_SAMPLES", 10)

    data, _meta = diag_zz_data(n; n_pairs=n_pairs, n_scenarios=n_scenarios, seed=seed)
    model = diag_zz_model(n; route_regularization_weight=1.0, max_stops=5)
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, diag_grb_env(); relax_integrality=true)
    set_silent(master.model)
    next_id = 1
    for column in passenger_free_assignment_two_stop_seed_columns(md; next_column_id=next_id)
        StationSelection.add_passenger_free_assignment_column!(master, column)
        next_id += 1
    end
    optimize!(master.model)
    alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)

    pricing_data(s, candidates) = create_passenger_free_assignment_pricing_data(
        s, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time, repositioning_time=md.repositioning_time,
        max_stops=md.max_stops, max_visits_per_node=md.max_visits_per_node,
    )

    for scenario in sort!(collect(keys(md.passengers_by_scenario)))
        candidates = passenger_free_assignment_pricing_candidates(md, alpha, gamma_o, gamma_d, scenario)
        exact_data = pricing_data(scenario, candidates)
        t0 = time()
        exact_columns, exact_exhausted, exact_stats =
            passenger_free_assignment_pricing_by_label_setting(
                exact_data, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1, reduced_cost_tol=1e-6,
                max_new_columns=typemax(Int) ÷ 2, n_candidates=typemax(Int) ÷ 2, time_limit=time_limit,
            )
        exact_wall = time() - t0
        exact_rc = isempty(exact_columns) ? 0.0 :
            minimum(Float64(column.metadata["reduced_cost"]) for column in exact_columns)

        t_post_search = time()
        post_columns, post_exhausted, post_stats =
            passenger_free_assignment_pricing_by_label_setting(
                exact_data, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1, reduced_cost_tol=1e-6,
                max_new_columns=typemax(Int) ÷ 2, n_candidates=typemax(Int) ÷ 2, time_limit=time_limit,
                use_post_w_completion_bound=true,
            )
        post_wall = time() - t_post_search
        post_rc = isempty(post_columns) ? 0.0 :
            minimum(Float64(column.metadata["reduced_cost"]) for column in post_columns)
        @printf(
            "POSTWSEARCH\tn=%d\ts=%d\texact_rc=%.3f\tpost_rc=%.3f\tvalid=%s\tlabels=%d\texact_labels=%d\twall=%.3f\texact_wall=%.3f\tspeedup=%.3f\tbound_calls=%d\tbound_states=%d\tbound_wall=%.3f\texhausted=%s\n",
            n, scenario, exact_rc, post_rc,
            string(post_exhausted && abs(post_rc - exact_rc) <= 1e-6),
            post_stats.labels_generated, exact_stats.labels_generated,
            post_wall, exact_wall, exact_wall / max(post_wall, 1e-9),
            post_stats.post_w_bound_calls, post_stats.post_w_bound_states,
            post_stats.t_post_w_bound_sec, string(post_exhausted),
        )

        # Sample real post-W labels and compare the current cheap remaining-
        # reward bound with the exact destination-only completion oracle.
        measured_labels, measured_exhausted, _measured_stats =
            StationSelection._enumerate_passenger_free_assignment_pricing_labels(
                exact_data; time_limit=time_limit, reduced_cost_tol=1e-6,
                max_visits_per_node=exact_data.max_visits_per_node, use_reduced_cost_pruning=false,
            )
        post_w = filter(
            label -> label.time + 1e-9 >= exact_data.max_wait_time &&
                label.route_length < exact_data.max_stops,
            measured_labels,
        )
        sort!(post_w; by=label -> label.reduced_cost)
        post_w = post_w[1:min(post_w_samples, length(post_w))]
        if !isempty(post_w)
            index = StationSelection._build_passenger_free_assignment_search_index(exact_data)
            workspace = StationSelection._create_passenger_free_assignment_bound_workspace(length(exact_data.nodes))
            total_states = 0
            total_wall = 0.0
            total_gap = 0.0
            for label in post_w
                bits = StationSelection._make_passenger_free_assignment_label_bitsets(
                    label, index.node_index, length(exact_data.nodes),
                )
                reward_bound = StationSelection._passenger_free_assignment_remaining_reward_bound(
                    label, bits, exact_data, index, workspace,
                )
                current_lower_bound = label.reduced_cost - reward_bound
                completion, completion_exhausted, completion_stats =
                    passenger_free_assignment_post_w_completion(label, exact_data; time_limit=time_limit)
                completion_exhausted || error("post-W sample timed out")
                total_states += completion_stats.states
                total_wall += completion_stats.wall_seconds
                total_gap += completion.reduced_cost - current_lower_bound
            end
            @printf(
                "POSTW\tn=%d\ts=%d\tsamples=%d\tmean_states=%.1f\tmean_wall=%.6f\tmean_tightening=%.3f\tlabel_search_exhausted=%s\n",
                n, scenario, length(post_w), total_states / length(post_w),
                total_wall / length(post_w), total_gap / length(post_w), string(measured_exhausted),
            )
        else
            @printf("POSTW\tn=%d\ts=%d\tsamples=0\n", n, scenario)
        end

        for rounds in rounds_list
            bound, certified, stats = passenger_free_assignment_lagrangian_bound(
                exact_data, candidates; max_iterations=rounds, time_limit=time_limit,
            )
            @printf(
                "LAG\tn=%d\ts=%d\trounds=%d\tbound=%.3f\texact=%.3f\tgap=%.3f\tvalid=%s\tmultiplicity=%d\trepeated=%d\treplay=%.3f\tlabels=%d\texact_labels=%d\twall=%.3f\texact_wall=%.3f\texhausted=%s\n",
                n, scenario, rounds, bound, exact_rc, exact_rc - bound,
                string(certified && bound <= exact_rc + 1e-6),
                stats.max_passenger_multiplicity, stats.repeated_passenger_count,
                stats.best_exact_replay_rc, stats.labels_generated,
                exact_stats.labels_generated, stats.wall_seconds, exact_wall, string(exact_exhausted),
            )
        end
    end
end

register_mode!("lagrangian_gap", run_lagrangian_gap)
