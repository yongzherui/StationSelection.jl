@testset "Passenger free-assignment label-setting pricing" begin
    using Random

    @testset "column over-generation defaults" begin
        model = AggregateODRouteModel(2)
        @test model.max_new_columns == 20
        @test model.n_candidates == 100
        @test ColumnGenerationSolver().max_columns_per_iteration == 20
        @test ColumnGenerationSolver().n_candidates == 100
    end

    function line_travel_cost(n::Int)
        costs = Dict{Tuple{Int, Int}, Float64}()
        for i in 1:n, j in 1:n
            i == j && continue
            costs[(i, j)] = Float64(abs(i - j))
        end
        return costs
    end

    function only_at(labels, current)
        return only(filter(l -> l.current == current, labels))
    end

    @testset "closed-station reduced-cost slack certifies pricing eliminations" begin
        candidates = [
            PassengerAssignmentCandidate(1, 2, 1, 10.0, 2.0),
            PassengerAssignmentCandidate(1, 2, 3, 10.0, 1.0),  # same origin side: max stays 2
            PassengerAssignmentCandidate(1, 1, 2, 10.0, 1.0),
            PassengerAssignmentCandidate(2, 2, 3, 10.0, 1.5),
            PassengerAssignmentCandidate(3, 3, 2, 10.0, -9.0),
        ]
        y_value = [1.0, 0.0, 0.0]
        y_lower_rc = [0.0, 5.0, 1.0]

        eliminated, required =
            passenger_free_assignment_station_reduced_cost_eliminations(
                candidates, y_value, y_lower_rc; slack_tol=1e-7,
            )

        @test 2 in eliminated
        @test !(1 in eliminated)  # open stations are never certified by lower-bound slack
        @test !(3 in eliminated)
        @test required[2] ≈ 4.5   # p1: 2 + 1, p2: 1.5
        @test required[3] ≈ 2.5
    end

    @testset "joint reduced-cost slack filter suppresses individual opportunities" begin
        using Gurobi
        env = Gurobi.Env()
        candidates = [
            PassengerAssignmentCandidate(1, 2, 3, 10.0, 3.0),
            PassengerAssignmentCandidate(1, 2, 4, 10.0, 8.0),
            PassengerAssignmentCandidate(1, 5, 3, 10.0, 8.0),
        ]
        y_value = [1.0, 0.0, 0.0, 1.0, 1.0]
        y_lower_rc = [0.0, 2.0, 1.0, 0.0, 0.0]

        joint = StationSelection._station_reduced_cost_joint_lp_filter(
            candidates, y_value, y_lower_rc, env;
            closed_tol=1e-7, reward_tol=1e-7, slack_tol=1e-7,
        )

        @test (1, 2, 3) in joint.excluded_opportunities
        @test !((1, 2, 4) in joint.excluded_opportunities)
        @test !((1, 5, 3) in joint.excluded_opportunities)
        @test joint.adjusted_rewards[(1, 2, 3)] <= 1e-7
        @test all(joint.adjusted_rewards[(c.passenger, c.origin, c.destination)] <=
                  c.reward + 1e-8 for c in candidates)
    end

    @testset "reward coarsening rounds upward and preserves exact default" begin
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 2.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 7.0),
            PassengerAssignmentCandidate(1, 2, 4, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 1, 2, 100.0, 3.0),
            PassengerAssignmentCandidate(2, 1, 3, 100.0, -1.0),
        ]
        unchanged = coarsen_passenger_assignment_rewards(candidates, 0)
        @test [c.reward for c in unchanged] == [c.reward for c in candidates]

        coarse = coarsen_passenger_assignment_rewards(candidates, 2)
        @test all(coarse[i].reward >= candidates[i].reward - 1e-9 for i in eachindex(candidates))
        @test [c.reward for c in coarse[1:4]] == [7.0, 7.0, 7.0, 10.0]
        @test coarse[5].reward == 3.0
        @test coarse[6].reward == -1.0
        @test_throws ArgumentError coarsen_passenger_assignment_rewards(candidates, -1)

        travel = line_travel_cost(4)
        exact = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        relaxed = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, coarse;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        for route in ([1, 2], [1, 3], [1, 4], [1, 2, 4])
            _ra, _rt, relaxed_rc = StationSelection._passenger_free_assignment_column_from_route(route, relaxed)
            _ea, _et, exact_rc = StationSelection._passenger_free_assignment_column_from_route(route, exact)
            @test relaxed_rc <= exact_rc + 1e-9
        end
    end

    @testset "passenger-Lagrangian bound keeps one route and relaxes uniqueness" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 10.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 10.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 4, 10.0, 7.0),
        ]
        exact = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
            max_stops=4, max_visits_per_node=2,
        )
        bound, certified, stats = passenger_free_assignment_lagrangian_bound(
            exact, candidates; max_iterations=3, time_limit=20.0,
        )
        @test certified
        @test stats.max_passenger_multiplicity == 2
        @test stats.repeated_passenger_count > 0
        @test bound <= stats.best_exact_replay_rc + 1e-9
        @test !stats.proves_no_improving_column
        @test_throws ArgumentError passenger_free_assignment_lagrangian_bound(
            exact, candidates; max_iterations=0,
        )
    end

    @testset "passenger-Lagrangian randomized lower-bound validity" begin
        rng = MersenneTwister(731)
        travel = line_travel_cost(5)
        for trial in 1:12
            candidates = PassengerAssignmentCandidate[]
            for passenger in 1:3, origin in 1:4, destination in (origin + 1):5
                rand(rng) < 0.45 || continue
                push!(candidates, PassengerAssignmentCandidate(
                    passenger, origin, destination, 10.0,
                    1.0 + 9.0 * rand(rng),
                ))
            end
            isempty(candidates) && continue
            exact = create_passenger_free_assignment_pricing_data(
                trial, collect(1:5), travel, candidates;
                route_regularization_weight=1.0, max_wait_time=10.0,
                max_stops=5, max_visits_per_node=2,
            )
            exact_columns, exact_exhausted, _stats =
                passenger_free_assignment_pricing_by_label_setting(
                    exact, PassengerFreeAssignmentRouteColumn[];
                    next_column_id=1, reduced_cost_tol=1e-9,
                    max_new_columns=typemax(Int) ÷ 2,
                    n_candidates=typemax(Int) ÷ 2,
                    time_limit=20.0,
                )
            @test exact_exhausted
            exact_optimum = isempty(exact_columns) ? 0.0 :
                minimum(Float64(c.metadata["reduced_cost"]) for c in exact_columns)
            bound, certified, _lag_stats = passenger_free_assignment_lagrangian_bound(
                exact, candidates; max_iterations=3, time_limit=20.0,
            )
            @test certified
            @test bound <= exact_optimum + 1e-6
        end
    end

    @testset "passenger-DSSR promotes duplicated passengers and becomes exact" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 10.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 10.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 4, 10.0, 7.0),
        ]
        exact = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
            max_stops=4, max_visits_per_node=2,
        )
        bound, certified, stats = passenger_free_assignment_passenger_dssr_bound(
            exact, candidates; max_rounds=5, time_limit=20.0,
        )
        @test certified
        @test stats.exact
        @test stats.rounds == 2
        @test stats.promoted_by_round[1] == [1]
        @test issorted(stats.bound_trajectory)
        @test bound ≈ -12.0
        @test bound ≈ stats.best_exact_replay_rc
        @test_throws ArgumentError passenger_free_assignment_passenger_dssr_bound(
            exact, candidates; max_rounds=0,
        )
    end

    @testset "passenger-DSSR randomized bound validity and monotonicity" begin
        rng = MersenneTwister(913)
        travel = line_travel_cost(5)
        for trial in 1:10
            candidates = PassengerAssignmentCandidate[]
            for passenger in 1:3, origin in 1:4, destination in (origin + 1):5
                rand(rng) < 0.5 || continue
                push!(candidates, PassengerAssignmentCandidate(
                    passenger, origin, destination, 10.0,
                    1.0 + 9.0 * rand(rng),
                ))
            end
            isempty(candidates) && continue
            exact = create_passenger_free_assignment_pricing_data(
                trial, collect(1:5), travel, candidates;
                route_regularization_weight=1.0, max_wait_time=10.0,
                max_stops=5, max_visits_per_node=2,
            )
            exact_columns, exact_exhausted, _stats =
                passenger_free_assignment_pricing_by_label_setting(
                    exact, PassengerFreeAssignmentRouteColumn[];
                    next_column_id=1, reduced_cost_tol=1e-9,
                    max_new_columns=typemax(Int) ÷ 2,
                    n_candidates=typemax(Int) ÷ 2,
                    time_limit=20.0,
                )
            @test exact_exhausted
            exact_optimum = isempty(exact_columns) ? 0.0 : minimum(
                Float64(column.metadata["reduced_cost"]) for column in exact_columns
            )
            bound, certified, dssr_stats = passenger_free_assignment_passenger_dssr_bound(
                exact, candidates; max_rounds=10, time_limit=20.0,
            )
            @test certified
            @test bound <= exact_optimum + 1e-6
            @test all(diff(dssr_stats.bound_trajectory) .>= -1e-6)
            dssr_stats.exact && @test bound ≈ exact_optimum atol = 1e-6
        end
    end

    @testset "post-W destination-only completion" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 10.0, 10.0),
            PassengerAssignmentCandidate(2, 1, 3, 10.0, 8.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=5.0,
            max_stops=5, max_visits_per_node=3,
        )
        post_w = PassengerFreeAssignmentPricingLabel(
            1, [1], 5.0, Dict(1 => 0.0), RewardLayerBitset(),
            0.0, 0.0, 1, pricing_data.station_bit[1],
        )
        best, exhausted, stats = passenger_free_assignment_post_w_completion(
            post_w, pricing_data; time_limit=10.0,
        )
        @test exhausted
        @test best.reduced_cost ≈ -16.0
        @test best.route == [1, 2, 3]
        @test stats.suffix_stops == 2
        @test length(unique(best.route[2:end])) == length(best.route[2:end])

        pre_w = PassengerFreeAssignmentPricingLabel(
            1, [1], 4.0, Dict(1 => 0.0), RewardLayerBitset(),
            0.0, 0.0, 1, pricing_data.station_bit[1],
        )
        @test_throws ArgumentError passenger_free_assignment_post_w_completion(
            pre_w, pricing_data,
        )
    end

    @testset "single passenger: improving destination upgrades reward, doesn't double count" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_passenger_free_assignment_pricing_labels(pricing_data), 1)

        label1 = only(extend_passenger_free_assignment_pricing_label(label0, 2, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label1.activated_reward_layers) ≈ 4.0

        label2 = only(extend_passenger_free_assignment_pricing_label(label1, 3, pricing_data))
        total = StationSelection._sum_layer_weights(pricing_data, label2.activated_reward_layers)
        @test total ≈ 10.0
        @test !isapprox(total, 14.0)

        assignments, tau, reduced_cost = StationSelection._passenger_free_assignment_column_from_route(
            label2.route, pricing_data; label_reduced_cost=label2.reduced_cost,
        )
        @test assignments == [(1, 1, 3)]
        @test reduced_cost ≈ label2.reduced_cost
    end

    @testset "single passenger: worse destination later activates nothing" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 4.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_passenger_free_assignment_pricing_labels(pricing_data), 1)

        label1 = only(extend_passenger_free_assignment_pricing_label(label0, 2, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label1.activated_reward_layers) ≈ 10.0

        label2 = only(extend_passenger_free_assignment_pricing_label(label1, 3, pricing_data))
        @test label2.activated_reward_layers == label1.activated_reward_layers
        @test StationSelection._sum_layer_weights(pricing_data, label2.activated_reward_layers) ≈ 10.0
    end

    @testset "alternative origins: only the larger certified reward is retained" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 2, 3, 100.0, 10.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_passenger_free_assignment_pricing_labels(pricing_data), 1)
        label1 = only(extend_passenger_free_assignment_pricing_label(label0, 2, pricing_data))
        label2 = only(extend_passenger_free_assignment_pricing_label(label1, 3, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label2.activated_reward_layers) ≈ 10.0
    end

    @testset "two passengers on the same OD pair are counted independently" begin
        travel = line_travel_cost(2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(2, 1, 2, 100.0, 7.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_passenger_free_assignment_pricing_labels(pricing_data), 1)
        label1 = only(extend_passenger_free_assignment_pricing_label(label0, 2, pricing_data))
        @test StationSelection._sum_layer_weights(pricing_data, label1.activated_reward_layers) ≈ 11.0

        assignments, _tau, _rc = StationSelection._passenger_free_assignment_column_from_route(
            label1.route, pricing_data; label_reduced_cost=label1.reduced_cost,
        )
        @test Set(assignments) == Set([(1, 1, 2), (2, 1, 2)])
    end

    @testset "multiple assignments for one passenger: only the running maximum counts" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 3.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 5.0),  # worse than the upgrade already seen
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label = only_at(initial_passenger_free_assignment_pricing_labels(pricing_data), 1)
        running = Float64[]
        for next_node in (2, 3, 4)
            label = only(extend_passenger_free_assignment_pricing_label(label, next_node, pricing_data))
            push!(running, StationSelection._sum_layer_weights(pricing_data, label.activated_reward_layers))
        end
        @test running ≈ [3.0, 8.0, 8.0]
    end

    @testset "ride-time infeasible assignment activates nothing" begin
        travel = line_travel_cost(3)
        candidates = [PassengerAssignmentCandidate(1, 1, 3, 1.5, 10.0)]  # ride limit 1.5 < travel(1,3) = 2
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(initial_passenger_free_assignment_pricing_labels(pricing_data), 1)
        label1 = only(extend_passenger_free_assignment_pricing_label(label0, 2, pricing_data))
        label2 = only(extend_passenger_free_assignment_pricing_label(label1, 3, pricing_data))
        @test isempty(label2.activated_reward_layers)
    end

    @testset "visiting an origin after the pickup cutoff does not reset its age" begin
        travel = line_travel_cost(3)
        candidates = [PassengerAssignmentCandidate(1, 2, 3, 100.0, 10.0)]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=0.5,
        )
        label0 = PassengerFreeAssignmentPricingLabel(
            1, [1], 0.0, Dict(1 => 0.0), RewardLayerBitset(), 0.0, 0.0, 1,
            pricing_data.station_bit[1],
        )
        late_visit = only(extend_passenger_free_assignment_pricing_label(label0, 2, pricing_data))
        @test late_visit.time == 1.0
        @test !haskey(late_visit.station_age, 2)

        not_certified = only(extend_passenger_free_assignment_pricing_label(late_visit, 3, pricing_data))
        @test isempty(not_certified.activated_reward_layers)
    end

    @testset "station budget cap (max_distinct_stations)" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 3, 4, 100.0, 10.0),
        ]

        @testset "uncapped: nothing is restricted" begin
            pd = create_passenger_free_assignment_pricing_data(
                1, [1, 2, 3, 4], travel, candidates;
                route_regularization_weight=1.0, max_wait_time=100.0,
            )
            @test !pd.bounded_distinct_stations
            label = only_at(initial_passenger_free_assignment_pricing_labels(pd), 1)
            @test StationSelection._passenger_free_assignment_station_budget_allows(label, 3, pd)
        end

        @testset "capped: a new station is refused once the budget is spent" begin
            pd = create_passenger_free_assignment_pricing_data(
                1, [1, 2, 3, 4], travel, candidates;
                route_regularization_weight=1.0, max_wait_time=100.0,
                max_distinct_stations=2,
            )
            @test pd.bounded_distinct_stations
            label = only_at(initial_passenger_free_assignment_pricing_labels(pd), 1)
            at2 = only(extend_passenger_free_assignment_pricing_label(label, 2, pd))
            @test count_ones(at2.visited_mask) == 2
            # Budget is spent: a third distinct station is refused...
            @test !StationSelection._passenger_free_assignment_station_budget_allows(at2, 3, pd)
            # ...but revisiting one already paid for is always free.
            @test StationSelection._passenger_free_assignment_station_budget_allows(at2, 1, pd)
            @test 3 ∉ StationSelection._passenger_free_assignment_candidate_next_nodes(at2, pd)
        end

        @testset "capped search never exceeds the budget, and matches brute force under it" begin
            pd_capped = create_passenger_free_assignment_pricing_data(
                1, [1, 2, 3, 4], travel, candidates;
                route_regularization_weight=0.1, max_wait_time=100.0,
                max_stops=4, max_visits_per_node=2, max_distinct_stations=2,
            )
            cols, _exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
                pd_capped, PassengerFreeAssignmentRouteColumn[];
                next_column_id=1, max_new_columns=50, n_candidates=1000, time_limit=30.0,
            )
            @test !isempty(cols)
            for c in cols
                @test length(unique(c.route)) <= 2
            end

            # The capped search must attain the best reduced cost over exactly the
            # routes the cap permits -- checked against explicit enumeration, since
            # the cap is a restriction and cannot be validated against the
            # unrestricted optimum.
            best_allowed = Inf
            nodes = [1, 2, 3, 4]
            function visit!(route)
                if length(route) >= 2
                    _a, _t, rc = StationSelection._passenger_free_assignment_column_from_route(route, pd_capped)
                    isempty(_a) || (best_allowed = min(best_allowed, rc))
                end
                length(route) >= 4 && return
                for nd in nodes
                    nd == route[end] && continue
                    count(==(nd), route) < 2 || continue
                    length(unique(vcat(route, nd))) <= 2 || continue
                    visit!(vcat(route, nd))
                end
            end
            for start in nodes
                visit!([start])
            end
            @test minimum(c.metadata["reduced_cost"] for c in cols) ≈ best_allowed
        end
    end

    @testset "dominance" begin
        travel = line_travel_cost(3)
        # Two distinct reward levels for passenger 1 so layer subset checks are non-trivial.
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        layer_low = pricing_data.assignment_layer_mask[(1, 1, 2)]   # {1}
        layer_high = pricing_data.assignment_layer_mask[(1, 1, 3)]  # {1, 2}

        mklabel(current, time, station_age, layers, rc) =
            PassengerFreeAssignmentPricingLabel(
                current, [current], time, station_age, layers, time, rc, 1,
                pricing_data.station_bit[current],
            )

        @testset "dominates when time/rc/layers/station-ages are all no worse" begin
            a = mklabel(2, 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
            b = mklabel(2, 2.0, Dict(1 => 2.0), RewardLayerBitset(), 0.0)
            @test StationSelection._dominates_passenger_free_assignment_label(a, b, pricing_data.layer_weight, false)
            @test !StationSelection._dominates_passenger_free_assignment_label(b, a, pricing_data.layer_weight, false)
        end

        @testset "extra activated layers must be paid for out of the reduced-cost gap" begin
            # a holds {1,2} (worth 10), b holds {1} (worth 4), so a's compensation
            # w(A_a \ A_b) is the incremental layer 2, worth 10 - 4 = 6. At equal
            # reduced cost a cannot cover it: a shared suffix that certifies layer 2
            # pays b 6 and pays a nothing, so b can still overtake a.
            a = mklabel(2, 1.0, Dict{Int, Float64}(), layer_high, -1.0)
            b = mklabel(2, 1.0, Dict{Int, Float64}(), layer_low, -1.0)
            @test !StationSelection._dominates_passenger_free_assignment_label(a, b, pricing_data.layer_weight, false)
            # b's mask (low) IS a subset of a's mask (high), so its compensation is
            # zero and b dominates a once it is no worse on time/rc/ages.
            @test StationSelection._dominates_passenger_free_assignment_label(b, a, pricing_data.layer_weight, false)

            # Give a a reduced-cost advantage that *does* cover the 6, and the same
            # pair now dominates in the direction the plain subset rule could never
            # express. 6.5 > 6 clears it; 5.5 < 6 does not.
            a_cheap = mklabel(2, 1.0, Dict{Int, Float64}(), layer_high, -7.5)
            @test StationSelection._dominates_passenger_free_assignment_label(a_cheap, b, pricing_data.layer_weight, false)
            a_not_cheap_enough = mklabel(2, 1.0, Dict{Int, Float64}(), layer_high, -6.5)
            @test !StationSelection._dominates_passenger_free_assignment_label(a_not_cheap_enough, b, pricing_data.layer_weight, false)
        end

        @testset "a live origin the candidate lacks breaks dominance" begin
            a = mklabel(2, 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
            b = mklabel(2, 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
            # a has no live origins at all, b still has station 1 live -- a must not
            # claim to dominate b, since b retains an option a has already lost.
            @test !StationSelection._dominates_passenger_free_assignment_label(a, b, pricing_data.layer_weight, false)
        end

        @testset "worse station age breaks dominance" begin
            a = mklabel(2, 1.0, Dict(1 => 5.0), RewardLayerBitset(), -1.0)
            b = mklabel(2, 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
            @test !StationSelection._dominates_passenger_free_assignment_label(a, b, pricing_data.layer_weight, false)
        end

        @testset "plain and bitset dominance implementations agree (randomized)" begin
            rng = MersenneTwister(42)
            nodes = [1, 2, 3, 4]
            node_index = Dict(n => i for (i, n) in enumerate(nodes))
            n_nodes = length(nodes)
            # Labels below activate layer ids drawn from 1:5; uneven weights make the
            # compensation term discriminate rather than collapse to a bit count.
            layer_weight = [1.0, 2.5, 0.5, 4.0, 3.0]

            function rand_label(rng, current)
                n_ages = rand(rng, 0:3)
                station_age = Dict{Int, Float64}()
                for s in rand(rng, nodes, n_ages)
                    station_age[s] = rand(rng) * 5
                end
                n_layers_active = rand(rng, 0:3)
                layers = RewardLayerBitset(rand(rng, 1:5, n_layers_active))
                t = rand(rng) * 5
                rc = rand(rng) * 10 - 5
                route_length = rand(rng, 1:4)
                return PassengerFreeAssignmentPricingLabel(
                    current, [current], t, station_age, layers, t, rc, route_length,
                    UInt64(1) << (node_index[current] - 1),
                )
            end

            for _ in 1:200
                current = rand(rng, nodes)
                a = rand_label(rng, current)
                b = rand_label(rng, current)
                a_bs = StationSelection._make_passenger_free_assignment_label_bitsets(a, node_index, n_nodes)
                b_bs = StationSelection._make_passenger_free_assignment_label_bitsets(b, node_index, n_nodes)
                for bounded in (true, false)
                    plain = StationSelection._dominates_passenger_free_assignment_label(a, b, layer_weight, bounded)
                    bitset = StationSelection._dominates_passenger_free_assignment_label(a, b, a_bs, b_bs, layer_weight, bounded)
                    @test plain == bitset
                end
            end
        end
    end

    @testset "remaining-reward bound is admissible (brute force over small random instances)" begin
        rng = MersenneTwister(7)
        n = 4
        nodes = collect(1:n)
        travel = line_travel_cost(n)
        n_nodes = length(nodes)

        checked = 0
        trial = 0
        while checked < 15 && trial < 100
            trial += 1
            candidates = PassengerAssignmentCandidate[]
            for p in 1:2, j in nodes, k in nodes
                j == k && continue
                rand(rng) < 0.6 && continue
                reward = rand(rng) * 10
                ride_limit = rand(rng) * 5 + travel[(j, k)]
                push!(candidates, PassengerAssignmentCandidate(p, j, k, ride_limit, reward))
            end
            isempty(candidates) && continue

            pricing_data = create_passenger_free_assignment_pricing_data(
                1, nodes, travel, candidates;
                route_regularization_weight=0.1, max_wait_time=3.0, max_visits_per_node=2,
            )
            isempty(pricing_data.opportunities) && continue

            search_index = StationSelection._build_passenger_free_assignment_search_index(pricing_data)
            bound_workspace = StationSelection._create_passenger_free_assignment_bound_workspace(n_nodes)

            labels0 = initial_passenger_free_assignment_pricing_labels(pricing_data)
            isempty(labels0) && continue
            label = rand(rng, labels0)
            for _ in 1:rand(rng, 0:2)
                next_nodes = StationSelection._passenger_free_assignment_candidate_next_nodes(label, pricing_data)
                isempty(next_nodes) && break
                label = only(extend_passenger_free_assignment_pricing_label(label, rand(rng, next_nodes), pricing_data))
            end

            label_bs = StationSelection._make_passenger_free_assignment_label_bitsets(
                label, search_index.node_index, n_nodes,
            )
            bound = StationSelection._passenger_free_assignment_remaining_reward_bound(
                label, label_bs, pricing_data, search_index, bound_workspace,
            )

            # The bound is on *net* gain (reward minus the route-regularization cost
            # of the travel that collects it), so the invariant it has to satisfy is
            # exactly the one the search relies on: `label.rc - bound` never exceeds
            # the reduced cost of any descendant. Equivalently, `bound` is at least
            # the best reduction in reduced cost any completion achieves.
            best_gain = Ref(0.0)
            function dfs!(l, depth)
                depth <= 0 && return
                for nd in nodes
                    haskey(travel, (l.current, nd)) || continue
                    child = only(extend_passenger_free_assignment_pricing_label(l, nd, pricing_data))
                    best_gain[] = max(best_gain[], label.reduced_cost - child.reduced_cost)
                    dfs!(child, depth - 1)
                end
            end
            dfs!(label, 3)

            @test bound >= best_gain[] - 1e-9
            checked += 1
        end
        @test checked > 0
    end

    @testset "label search finds the brute-force-optimal reduced cost" begin
        nodes = [1, 2, 3]
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 3, 100.0, 5.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0, max_stops=3, max_visits_per_node=1,
        )

        function brute_force_best_reduced_cost()
            best = Inf
            function visit!(route::Vector{Int})
                if length(route) >= 2
                    assignments, _tau, rc = StationSelection._passenger_free_assignment_column_from_route(route, pricing_data)
                    isempty(assignments) || (best = min(best, rc))
                end
                length(route) >= 3 && return
                for nd in nodes
                    nd in route && continue
                    haskey(travel, (route[end], nd)) || continue
                    visit!(vcat(route, nd))
                end
            end
            for start in nodes
                visit!([start])
            end
            return best
        end

        brute_best = brute_force_best_reduced_cost()

        columns, _exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
            pricing_data, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10, n_candidates=10, time_limit=10.0, max_visits_per_node=1,
        )
        search_best = isempty(columns) ? Inf : minimum(c.metadata["reduced_cost"] for c in columns)
        @test isapprox(search_best, brute_best; atol=1e-6)
    end

    @testset "unbounded max_stops: terminates and matches a bounded-but-generous search" begin
        # The real target is no stop limit at all. This is finite even so: every
        # extension adds travel > 0 to `time` and to every live clock, no new clock
        # is created past `max_wait_time`, and clocks are pruned once they cannot
        # certify -- so route length is implicitly capped by roughly
        # (max_wait + max_ride_limit) / min_travel. This checks that it (a) halts,
        # and (b) finds the same optimum as a search whose explicit cap is set well
        # above that implicit bound.
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 6.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 6.0, 9.0),
            PassengerAssignmentCandidate(3, 1, 4, 8.0, 7.0),
        ]
        mk(max_stops, max_visits) = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=3.0,
            max_stops=max_stops, max_visits_per_node=max_visits,
        )

        unbounded = mk(typemax(Int), typemax(Int))
        @test unbounded.max_stops == typemax(Int)
        @test unbounded.bounded_max_stops == false

        cols_unb, exhausted_unb, _st = passenger_free_assignment_pricing_by_label_setting(
            unbounded, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=60.0,
        )
        @test exhausted_unb            # halted on its own, not by timeout
        @test !isempty(cols_unb)

        # generous explicit cap, far above the implicit one
        generous = mk(30, 30)
        cols_gen, exhausted_gen, _st2 = passenger_free_assignment_pricing_by_label_setting(
            generous, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=60.0,
        )
        @test exhausted_gen
        best_unb = minimum(c.metadata["reduced_cost"] for c in cols_unb)
        best_gen = minimum(c.metadata["reduced_cost"] for c in cols_gen)
        @test isapprox(best_unb, best_gen; atol=1e-6)
    end

    @testset "optimized bound/dominance still match brute force under revisits" begin
        # Guards the bound rewrite (per-origin masks) and the sparse-age dominance
        # rewrite against the exhaustive optimum on an instance whose optimum needs
        # a revisit, which is where age bookkeeping is easiest to get wrong.
        nodes = [1, 2, 3, 4]
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 2, 4, 100.0, 11.0),
            PassengerAssignmentCandidate(2, 3, 1, 100.0, 9.0),
            PassengerAssignmentCandidate(3, 2, 1, 100.0, 6.0),
        ]
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=20.0,
            max_stops=4, max_visits_per_node=2,
        )

        best_bf = Inf
        route = Int[]
        counts = Dict{Int, Int}()
        function dfs!()
            if length(route) >= 2
                a, _t, rc = StationSelection._passenger_free_assignment_column_from_route(
                    copy(route), pricing_data,
                )
                isempty(a) || (best_bf = min(best_bf, rc))
            end
            length(route) >= 4 && return
            for nd in nodes
                !isempty(route) && nd == route[end] && continue
                get(counts, nd, 0) < 2 || continue
                push!(route, nd); counts[nd] = get(counts, nd, 0) + 1
                dfs!()
                counts[nd] -= 1; pop!(route)
            end
        end
        for s in nodes
            push!(route, s); counts[s] = 1; dfs!(); counts[s] = 0; pop!(route)
        end

        cols, exhausted, _st = passenger_free_assignment_pricing_by_label_setting(
            pricing_data, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        @test exhausted
        @test isapprox(minimum(c.metadata["reduced_cost"] for c in cols), best_bf; atol=1e-6)
    end

    @testset "reduced-cost consistency for extracted columns" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
        ]
        reward_lookup = Dict((c.passenger, c.origin, c.destination) => c.reward for c in candidates)
        pricing_data = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        columns, _exhausted, _stats = passenger_free_assignment_pricing_by_label_setting(
            pricing_data, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=5, n_candidates=5, time_limit=10.0,
        )
        @test !isempty(columns)
        for column in columns
            expected_reward = sum(reward_lookup[a] for a in column.assignments)
            expected_rc = pricing_data.route_regularization_weight * (column.tau + pricing_data.repositioning_time) - expected_reward
            @test column.metadata["reduced_cost"] ≈ expected_rc atol = 1e-6
            @test column.metadata["reduced_cost"] < -1e-6
        end
    end
end
