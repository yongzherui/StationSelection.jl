@testset "Certified passenger station-subset pricing" begin
    using Combinatorics

    station_ids = [10, 20, 30, 40, 50]
    travel = Dict((i, j) => Float64(abs(i-j) / 10)
                  for i in station_ids for j in station_ids if i != j)
    candidates = [
        PassengerAssignmentCandidate(1, 10, 30, 10.0, 8.0),
        PassengerAssignmentCandidate(1, 20, 30, 10.0, 5.0),
        PassengerAssignmentCandidate(2, 40, 50, 10.0, 7.0),
        PassengerAssignmentCandidate(3, 20, 50, 10.0, 9.0),
    ]
    data = create_passenger_free_assignment_pricing_data(
        1, station_ids, travel, candidates;
        route_regularization_weight=1.0, max_wait_time=10.0,
        max_stops=5, max_visits_per_node=2,
    )
    reward_data = PassengerRewardBoundData(data)

    @testset "pairwise routing bounds" begin
        pair = build_pairwise_assignment_routing_bounds(data,reward_data;
            alternatives_per_passenger=10)
        @test !isempty(pair.joint_cost_lower_bounds)
        @test all(c >= 0 for (_i,_j,c) in pair.joint_cost_lower_bounds)

        tight_wait = create_passenger_free_assignment_pricing_data(
            9, station_ids, travel, [
                PassengerAssignmentCandidate(1,10,20,10.0,5.0),
                PassengerAssignmentCandidate(2,40,50,10.0,5.0),
            ]; route_regularization_weight=1.0,max_wait_time=0.0,
            max_stops=4,max_visits_per_node=2)
        tight_reward=PassengerRewardBoundData(tight_wait)
        incompatible=build_pairwise_assignment_routing_bounds(tight_wait,tight_reward)
        @test incompatible.conflicts == [(1,2)]
    end

    @testset "fixed oracle and reward bound use station IDs" begin
        T = BitSet([10, 20, 30])
        result = price_exact_on_stations(data, T; time_limit=20.0)
        @test result.certified
        @test result.station_set == T
        @test result.value <= reward_upper_bound_fixed(reward_data, T) + 1e-7
        @test result.reduced_cost == -result.value
        @test_throws ArgumentError price_exact_on_stations(data, BitSet([10, 99]))
    end

    @testset "forced-cardinality propagation" begin
        state, set = StationSelection._fixed_station_set(BitSet([1, 2]), BitSet(), 5, 2)
        @test state == :fixed
        @test set == BitSet([1, 2])
        state, set = StationSelection._fixed_station_set(BitSet([1]), BitSet([4, 5]), 5, 3)
        @test state == :fixed
        @test set == BitSet([1, 2, 3])
    end

    @testset "brute-force agreement, validity, and cache" begin
        exact = ExactPricingResult[]
        for subset in combinations(station_ids, 3)
            result = price_exact_on_stations(data, BitSet(subset); time_limit=20.0)
            @test result.certified
            @test result.value <= reward_upper_bound_fixed(reward_data, BitSet(subset)) + 1e-7
            push!(exact, result)
        end
        enumeration = maximum(r.value for r in exact)
        settings = StationSubsetPricingSettings(
            use_reward_lp=false, time_limit=60.0, exact_oracle_time_limit=20.0,
        )
        certificate = price_by_station_subset_branch_and_bound(data, 3; settings=settings)
        @test certificate.globally_certified
        @test certificate.optimal_value ≈ enumeration atol=1e-7
        @test certificate.final_global_upper_bound <= certificate.optimal_value + settings.bound_tolerance
        @test certificate.unique_subsets_priced ==
              certificate.fixed_subsets_priced + certificate.heuristic_subsets_priced

        # Availability is monotone, so :at_most is certified by the exact-L tree.
        at_most = price_by_station_subset_branch_and_bound(data, 3; settings=
            StationSubsetPricingSettings(budget_mode=:at_most, use_reward_lp=false,
                time_limit=60.0, exact_oracle_time_limit=20.0))
        @test at_most.globally_certified
        @test at_most.optimal_value ≈ enumeration atol=1e-7

        early = price_by_station_subset_branch_and_bound(data, 3; settings=
            StationSubsetPricingSettings(use_reward_lp=false,
                stop_on_first_improving_column=true, time_limit=60.0,
                exact_oracle_time_limit=20.0))
        @test early.optimal_value > 0
        @test !early.globally_certified
        @test early.final_global_upper_bound >= early.optimal_value

        # L=5 availability search is exactly reducible to K=3 because these
        # routes have at most three stops, hence at most three distinct stations.
        capped_data = create_passenger_free_assignment_pricing_data(
            3, station_ids, travel, candidates;
            route_regularization_weight=1.0, max_wait_time=10.0,
            max_stops=3, max_visits_per_node=2,
        )
        reduced = price_by_station_subset_branch_and_bound(capped_data, 5;
            settings=StationSubsetPricingSettings(use_reward_lp=false,
                time_limit=60.0, exact_oracle_time_limit=20.0))
        enum3 = maximum(price_exact_on_stations(capped_data, BitSet(s);
            time_limit=20.0).value for s in combinations(station_ids,3))
        @test reduced.globally_certified
        @test reduced.optimal_value ≈ enum3 atol=1e-7
        @test reduced.unique_subsets_priced <= binomial(5,3)
    end

    @testset "reduced-cost pruning preserves the exact optimum" begin
        # Feature A: the admissible completion-reward bound may prune non-improving
        # partial labels but must never change the certified optimum. Check OFF vs
        # ON agree, and agree with brute-force enumeration, on every 3-subset.
        for subset in combinations(station_ids, 3)
            T = BitSet(subset)
            off = price_exact_on_stations(data, T; time_limit=20.0, use_reduced_cost_pruning=false)
            on  = price_exact_on_stations(data, T; time_limit=20.0, use_reduced_cost_pruning=true)
            @test on.certified && off.certified
            @test on.value ≈ off.value atol=1e-9
        end
    end

    @testset "triple routing bounds: validity and cut safety" begin
        triple = build_triple_assignment_routing_bounds(data, reward_data;
            alternatives_per_passenger=10)
        @test all(c >= 0 for (_i,_j,_k,c) in triple.joint_cost_lower_bounds)

        # max_wait=0 with three distinct origins makes any 3-passenger route
        # infeasible, so the single triple is a certified conflict (not a cost).
        tight = create_passenger_free_assignment_pricing_data(
            7, station_ids, travel, [
                PassengerAssignmentCandidate(1, 10, 30, 10.0, 5.0),
                PassengerAssignmentCandidate(2, 40, 50, 10.0, 5.0),
                PassengerAssignmentCandidate(3, 20, 50, 10.0, 5.0),
            ]; route_regularization_weight=1.0, max_wait_time=0.0,
            max_stops=6, max_visits_per_node=2)
        conflicted = build_triple_assignment_routing_bounds(tight, PassengerRewardBoundData(tight))
        @test length(conflicted.conflicts) == 1
        @test isempty(conflicted.joint_cost_lower_bounds)

        # Cut safety: triple cuts must never remove the true optimum. LP-guided
        # B&B with triples ON must still certify the brute-force enumeration value.
        enumeration = maximum(price_exact_on_stations(data, BitSet(s);
            time_limit=20.0).value for s in combinations(station_ids, 3))
        with_triples = price_by_station_subset_branch_and_bound(data, 3; settings=
            StationSubsetPricingSettings(use_reward_lp=true,
                use_routing_reward_bound=true, use_triple_routing_bounds=true,
                triple_alternatives_per_passenger=10,
                time_limit=60.0, exact_oracle_time_limit=20.0))
        @test with_triples.globally_certified
        @test with_triples.optimal_value ≈ enumeration atol=1e-7
        @test with_triples.final_global_upper_bound <= with_triples.optimal_value + 1e-7
    end

    @testset "no improving column" begin
        weak = [PassengerAssignmentCandidate(1, 10, 20, 10.0, 0.25)]
        no_improvement = create_passenger_free_assignment_pricing_data(
            2, station_ids, travel, weak;
            route_regularization_weight=1.0, max_wait_time=10.0,
            max_stops=3, max_visits_per_node=1,
        )
        certificate = price_by_station_subset_branch_and_bound(no_improvement, 2;
            settings=StationSubsetPricingSettings(use_reward_lp=false,
                time_limit=30.0, exact_oracle_time_limit=10.0))
        @test certificate.globally_certified
        @test certificate.optimal_value == 0.0
        @test certificate.final_global_upper_bound <= 1e-7
    end
end
