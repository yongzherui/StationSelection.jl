@testset "Passenger free-assignment station-simple (elementary) pricing" begin
    using Random

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

    # Best reduced cost over all ELEMENTARY routes (no repeated station). Since
    # station-simple prices over exactly this universe, its optimum must match this,
    # which may be strictly worse than the revisit-tolerant optimum.
    function brute_force_best_elementary_rc(nodes, travel, pricing_data)
        best = Inf
        function visit!(route)
            if length(route) >= 2
                a, _t, rc = StationSelection._passenger_free_assignment_column_from_route(route, pricing_data)
                isempty(a) || (best = min(best, rc))
            end
            for nd in nodes
                nd in route && continue
                haskey(travel, (route[end], nd)) || continue
                visit!(vcat(route, nd))
            end
        end
        for s in nodes
            visit!([s])
        end
        return best
    end

    @testset "extension forbids revisits and grows the visited set" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(StationSelection._initial_passenger_free_assignment_station_simple_labels(pd), 1)
        @test label0.visited == Set([1])

        label1 = StationSelection._extend_passenger_free_assignment_station_simple_label(label0, 2, pd)
        @test label1.visited == Set([1, 2])
        @test label1.route == [1, 2]
        @test StationSelection._sum_layer_weights(pd, label1.activated_reward_layers) ≈ 4.0

        # Revisiting a station on the route is a hard error.
        @test_throws ArgumentError StationSelection._extend_passenger_free_assignment_station_simple_label(label1, 1, pd)
    end

    @testset "candidate generation excludes already-visited nodes" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 1, 100.0, 8.0),  # would make 1 "useful" again
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        label0 = only_at(StationSelection._initial_passenger_free_assignment_station_simple_labels(pd), 1)
        label1 = StationSelection._extend_passenger_free_assignment_station_simple_label(label0, 2, pd)
        cands = StationSelection._passenger_free_assignment_station_simple_candidate_next_nodes(label1, pd)
        @test 1 ∉ cands      # visited, never a candidate despite p2's (2,1) opportunity
        @test 2 ∉ cands      # current, also visited
        @test 3 ∈ cands
    end

    @testset "every returned column is an elementary route" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
            PassengerAssignmentCandidate(3, 2, 1, 100.0, 5.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        cols, _exhausted, _stats = passenger_free_assignment_pricing_by_station_simple_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=50, n_candidates=1000, time_limit=30.0,
        )
        @test !isempty(cols)
        for c in cols
            @test length(unique(c.route)) == length(c.route)  # no station repeated
        end
    end

    @testset "reduced-cost consistency for extracted columns" begin
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
            PassengerAssignmentCandidate(1, 1, 4, 100.0, 12.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
        ]
        reward_lookup = Dict((c.passenger, c.origin, c.destination) => c.reward for c in candidates)
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3, 4], travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        # The driver's route replay asserts label rc == recomputed rc internally; here
        # we also check it against an independent per-assignment recomputation.
        cols, _exhausted, _stats = passenger_free_assignment_pricing_by_station_simple_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=5, n_candidates=5, time_limit=10.0,
        )
        @test !isempty(cols)
        for c in cols
            expected_reward = sum(reward_lookup[a] for a in c.assignments)
            expected_rc = pd.route_regularization_weight * (c.tau + pd.repositioning_time) - expected_reward
            @test c.metadata["reduced_cost"] ≈ expected_rc atol = 1e-6
            @test c.metadata["reduced_cost"] < -1e-6
        end
    end

    @testset "attains the brute-force elementary optimum" begin
        nodes = [1, 2, 3, 4]
        travel = line_travel_cost(4)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 4, 100.0, 6.0),
            PassengerAssignmentCandidate(3, 1, 4, 100.0, 7.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=4,
        )
        brute_best = brute_force_best_elementary_rc(nodes, travel, pd)
        cols, exhausted, _stats = passenger_free_assignment_pricing_by_station_simple_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        @test exhausted
        @test isapprox(minimum(c.metadata["reduced_cost"] for c in cols), brute_best; atol=1e-6)
    end

    @testset "agrees with the revisit-tolerant pricer when the optimum is elementary" begin
        # Route 1->2->3 collects every reward on a single elementary pass; a revisit
        # can only add travel, so the two pricers must reach the same optimum.
        nodes = [1, 2, 3]
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 8.0),
            PassengerAssignmentCandidate(2, 2, 3, 100.0, 5.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=3, max_visits_per_node=2,
        )
        rev_cols, _e1, _s1 = passenger_free_assignment_pricing_by_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        ss_cols, _e2, _s2 = passenger_free_assignment_pricing_by_station_simple_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        rev_best = minimum(c.metadata["reduced_cost"] for c in rev_cols)
        ss_best = minimum(c.metadata["reduced_cost"] for c in ss_cols)
        @test isapprox(rev_best, ss_best; atol=1e-6)
    end

    @testset "restriction is visible where a revisit strictly helps" begin
        # nodes [1,2]: p1 wants 1->2, p2 wants 2->1. Serving both needs the route
        # 1->2->1, which revisits 1. The revisit-tolerant pricer collects both
        # (reward 20); an elementary route can serve at most one (reward 10).
        nodes = [1, 2]
        travel = line_travel_cost(2)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 10.0),
            PassengerAssignmentCandidate(2, 2, 1, 100.0, 10.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, nodes, travel, candidates;
            route_regularization_weight=0.5, max_wait_time=10.0, max_stops=3, max_visits_per_node=2,
        )

        rev_cols, _e1, _s1 = passenger_free_assignment_pricing_by_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )
        ss_cols, _e2, _s2 = passenger_free_assignment_pricing_by_station_simple_label_setting(
            pd, PassengerFreeAssignmentRouteColumn[];
            next_column_id=1, max_new_columns=10^6, n_candidates=10^6, time_limit=30.0,
        )

        # The revisit-tolerant pricer serves both via 1->2->1.
        @test any(length(c.assignments) == 2 for c in rev_cols)
        rev_best = minimum(c.metadata["reduced_cost"] for c in rev_cols)

        # Station-simple returns only elementary routes, each serving one passenger.
        for c in ss_cols
            @test length(unique(c.route)) == length(c.route)
            @test length(c.assignments) == 1
        end
        ss_best = minimum(c.metadata["reduced_cost"] for c in ss_cols)

        # It attains the best ELEMENTARY reduced cost, which is strictly worse
        # (less negative) than the revisit-tolerant optimum -- the documented
        # column-universe restriction.
        @test isapprox(ss_best, brute_force_best_elementary_rc(nodes, travel, pd); atol=1e-6)
        @test ss_best > rev_best + 1e-6
    end

    @testset "dominance: subset-visited rule with compensated layers" begin
        travel = line_travel_cost(3)
        candidates = [
            PassengerAssignmentCandidate(1, 1, 2, 100.0, 4.0),
            PassengerAssignmentCandidate(1, 1, 3, 100.0, 10.0),
        ]
        pd = create_passenger_free_assignment_pricing_data(
            1, [1, 2, 3], travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
        )
        node_index = Dict(n => i for (i, n) in enumerate(pd.nodes))
        n_nodes = length(pd.nodes)

        mklabel(current, visited, time, station_age, layers, rc) =
            StationSelection.PassengerFreeAssignmentStationSimpleLabel(
                current, collect(visited), Set(visited), time, station_age, layers, time, rc, length(visited),
            )
        bs(label) = StationSelection._make_passenger_free_assignment_station_simple_bitsets(label, node_index, n_nodes)
        dominates(x, y) = StationSelection._dominates_passenger_free_assignment_station_simple_label(
            x, y, bs(x), bs(y), pd.layer_weight,
        )

        # Equal visited, strictly no worse on time/rc/age -> dominates.
        a = mklabel(2, [1, 2], 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
        b = mklabel(2, [1, 2], 2.0, Dict(1 => 2.0), RewardLayerBitset(), 0.0)
        @test dominates(a, b)
        @test !dominates(b, a)

        # Subset visited: a lean route dominates a wandered one that is otherwise
        # no worse -- the cross-domination the exact-visited rule could not express.
        lean = mklabel(2, [2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
        wandered = mklabel(2, [1, 2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
        @test dominates(lean, wandered)     # {2} ⊆ {1,2}
        @test !dominates(wandered, lean)    # {1,2} ⊄ {2}

        # A superset-visited label cannot dominate even with a better reduced cost:
        # it has forbidden itself a station the lean label can still use.
        cheap_wandered = mklabel(2, [1, 2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -5.0)
        @test !dominates(cheap_wandered, lean)

        # Live-clock (age) resource: the dominator must hold every live clock the
        # dominated one has. An extra clock is fine; a missing one breaks dominance.
        has_clock = mklabel(2, [2], 1.0, Dict(1 => 1.0), RewardLayerBitset(), -1.0)
        no_clock = mklabel(2, [2], 1.0, Dict{Int, Float64}(), RewardLayerBitset(), -1.0)
        @test dominates(has_clock, no_clock)
        @test !dominates(no_clock, has_clock)
    end
end
