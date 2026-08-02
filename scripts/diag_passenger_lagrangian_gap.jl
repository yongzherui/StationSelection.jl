"""
Measure the passenger-uniqueness Lagrangian relaxation on the first seeded RMP
dual snapshot. This is a cheap gate before integrating it into the full CG
trajectory. Reports bound validity, repeated-passenger multiplicity, exact
replayed route quality, labels, and wall time for 1/3/5 multiplier rounds.

Usage: julia --project=. scripts/diag_passenger_lagrangian_gap.jl <n_stations>
"""

using Gurobi, JuMP, Printf, StationSelection
include(joinpath(@__DIR__, "generate_zhuzhou_instance.jl"))

const DATA_DIR = normpath(joinpath(@__DIR__, "..", "..", "Data", "base_data"))
const N_PAIRS = parse(Int, get(ENV, "PFALAG_N_PAIRS", "16"))
const SEED = parse(Int, get(ENV, "PFALAG_SEED", "42"))
const N_SCENARIOS = parse(Int, get(ENV, "PFALAG_N_SCENARIOS", "3"))
const TIME_LIMIT = parse(Float64, get(ENV, "PFALAG_TIME", "300"))
const ROUNDS = parse.(Int, split(get(ENV, "PFALAG_ROUNDS", "1,3,5"), ","))
const POST_W_SAMPLES = parse(Int, get(ENV, "PFAPOSTW_SAMPLES", "10"))
const GRB_ENV = Gurobi.Env()

function build_model(n)
    AggregateODRouteModel(
        max(2, ceil(Int, n / 2));
        route_regularization_weight=1.0,
        walk_cost_weight=0.1,
        repositioning_time=20.0,
        max_walking_distance=600.0,
        max_wait_time=900.0,
        detour_factor=2.0,
        max_stops=5,
        max_visits_per_node=3,
    )
end

function pricing_data(md, scenario, candidates)
    create_passenger_free_assignment_pricing_data(
        scenario, md.nodes, md.travel_cost, candidates;
        route_regularization_weight=md.route_regularization_weight,
        max_wait_time=md.max_wait_time,
        repositioning_time=md.repositioning_time,
        max_stops=md.max_stops,
        max_visits_per_node=md.max_visits_per_node,
    )
end

function main()
    length(ARGS) == 1 || error("usage: diag_passenger_lagrangian_gap.jl <n_stations>")
    n = parse(Int, ARGS[1])
    data, _meta = generate_zhuzhou_data(
        DATA_DIR, n, N_PAIRS; n_scenarios=N_SCENARIOS, seed=SEED,
    )
    model = build_model(n)
    mapping = create_map(model, data)
    md = StationSelection.create_passenger_free_assignment_master_data(model, data, mapping)
    master = build_passenger_free_assignment_master(md, GRB_ENV; relax_integrality=true)
    set_silent(master.model)
    next_id = 1
    for column in passenger_free_assignment_two_stop_seed_columns(md; next_column_id=next_id)
        StationSelection.add_passenger_free_assignment_column!(master, column)
        next_id += 1
    end
    optimize!(master.model)
    alpha, gamma_o, gamma_d = extract_passenger_free_assignment_duals(master)

    for scenario in sort!(collect(keys(md.passengers_by_scenario)))
        candidates = passenger_free_assignment_pricing_candidates(
            md, alpha, gamma_o, gamma_d, scenario,
        )
        exact_data = pricing_data(md, scenario, candidates)
        t0 = time()
        exact_columns, exact_exhausted, exact_stats =
            passenger_free_assignment_pricing_by_label_setting(
                exact_data, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1, reduced_cost_tol=1e-6,
                max_new_columns=typemax(Int) ÷ 2,
                n_candidates=typemax(Int) ÷ 2,
                time_limit=TIME_LIMIT,
            )
        exact_wall = time() - t0
        exact_rc = isempty(exact_columns) ? 0.0 : minimum(
            Float64(column.metadata["reduced_cost"]) for column in exact_columns
        )
        t_post_search = time()
        post_columns, post_exhausted, post_stats =
            passenger_free_assignment_pricing_by_label_setting(
                exact_data, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1, reduced_cost_tol=1e-6,
                max_new_columns=typemax(Int) ÷ 2,
                n_candidates=typemax(Int) ÷ 2,
                time_limit=TIME_LIMIT,
                use_post_w_completion_bound=true,
            )
        post_wall = time() - t_post_search
        post_rc = isempty(post_columns) ? 0.0 : minimum(
            Float64(column.metadata["reduced_cost"]) for column in post_columns
        )
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
                exact_data; time_limit=TIME_LIMIT, reduced_cost_tol=1e-6,
                max_visits_per_node=exact_data.max_visits_per_node,
                use_reduced_cost_pruning=false,
            )
        post_w = filter(
            label -> label.time + 1e-9 >= exact_data.max_wait_time &&
                label.route_length < exact_data.max_stops,
            measured_labels,
        )
        sort!(post_w; by=label -> label.reduced_cost)
        post_w = post_w[1:min(POST_W_SAMPLES, length(post_w))]
        if !isempty(post_w)
            index = StationSelection._build_passenger_free_assignment_search_index(exact_data)
            workspace = StationSelection._create_passenger_free_assignment_bound_workspace(
                length(exact_data.nodes),
            )
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
                    passenger_free_assignment_post_w_completion(
                        label, exact_data; time_limit=TIME_LIMIT,
                    )
                completion_exhausted || error("post-W sample timed out")
                total_states += completion_stats.states
                total_wall += completion_stats.wall_seconds
                total_gap += completion.reduced_cost - current_lower_bound
            end
            @printf(
                "POSTW\tn=%d\ts=%d\tsamples=%d\tmean_states=%.1f\tmean_wall=%.6f\tmean_tightening=%.3f\tlabel_search_exhausted=%s\n",
                n, scenario, length(post_w), total_states / length(post_w),
                total_wall / length(post_w), total_gap / length(post_w),
                string(measured_exhausted),
            )
        else
            @printf("POSTW\tn=%d\ts=%d\tsamples=0\n", n, scenario)
        end
        for rounds in ROUNDS
            bound, certified, stats = passenger_free_assignment_lagrangian_bound(
                exact_data, candidates;
                max_iterations=rounds, time_limit=TIME_LIMIT,
            )
            @printf(
                "LAG\tn=%d\ts=%d\trounds=%d\tbound=%.3f\texact=%.3f\tgap=%.3f\tvalid=%s\tmultiplicity=%d\trepeated=%d\treplay=%.3f\tlabels=%d\texact_labels=%d\twall=%.3f\texact_wall=%.3f\texhausted=%s\n",
                n, scenario, rounds, bound, exact_rc, exact_rc - bound,
                string(certified && bound <= exact_rc + 1e-6),
                stats.max_passenger_multiplicity, stats.repeated_passenger_count,
                stats.best_exact_replay_rc, stats.labels_generated,
                exact_stats.labels_generated, stats.wall_seconds, exact_wall,
                string(exact_exhausted),
            )
        end
        max_reward = Dict{Int, Float64}()
        for candidate in candidates
            max_reward[candidate.passenger] = max(
                get(max_reward, candidate.passenger, 0.0), candidate.reward,
            )
        end
        ranked = sort!(collect(keys(max_reward)); by=p -> (-max_reward[p], p))
        for initial_k in (0, 4, 8)
            initial = Set(ranked[1:min(initial_k, length(ranked))])
            dssr_bound, dssr_certified, dssr =
                passenger_free_assignment_passenger_dssr_bound(
                    exact_data, candidates; initial_exact_passengers=initial,
                    max_rounds=20, time_limit=TIME_LIMIT,
                )
            @printf(
                "DSSR\tn=%d\ts=%d\tinitial_k=%d\tbound=%.3f\texact=%.3f\tgap=%.3f\tvalid=%s\texact_match=%s\trounds=%d\texact_passengers=%d\tlabels=%d\texact_labels=%d\treplay=%.3f\twall=%.3f\texact_wall=%.3f\n",
                n, scenario, initial_k, dssr_bound, exact_rc, exact_rc - dssr_bound,
                string(dssr_certified && dssr_bound <= exact_rc + 1e-6),
                string(dssr.exact), dssr.rounds, dssr.n_exact_passengers,
                dssr.labels_generated, exact_stats.labels_generated,
                dssr.best_exact_replay_rc, dssr.wall_seconds, exact_wall,
            )
        end
    end
end

main()
