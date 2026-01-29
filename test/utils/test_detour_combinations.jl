@testset "Detour Combinations" begin
    using DataFrames
    using Dates

    # Helper to create minimal StationSelectionData with routing costs
    function create_test_data_with_routing(station_ids::Vector{Int}, routing_costs::Dict{Tuple{Int,Int}, Float64})
        stations = DataFrame(
            id = station_ids,
            lon = zeros(length(station_ids)),
            lat = zeros(length(station_ids))
        )
        requests = DataFrame(
            id = [1],
            start_station_id = [station_ids[1]],
            end_station_id = [station_ids[end]],
            request_time = [DateTime(2024, 1, 1, 8, 0, 0)]
        )
        walking_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in station_ids, j in station_ids
            walking_costs[(i, j)] = 0.0
        end

        return StationSelection.StationSelectionData(
            stations,
            length(station_ids),
            walking_costs,
            routing_costs,
            StationSelection.ScenarioData[]
        )
    end

    @testset "Basic triangle inequality constraint" begin
        # 3 stations where A->C is the longest edge
        # Triangle inequality: t(A->B) + t(B->C) >= t(A->C) must hold
        # Detour constraint: t(A->B) + t(B->C) <= t(A->C) + delay
        station_ids = [1, 2, 3]

        # Costs: A->B = 10, B->C = 12, A->C = 18
        # Triangle inequality: 10 + 12 = 22 >= 18 (satisfied)
        # A->C (18) is longest edge (18 > 10 and 18 > 12)
        # Detour check: 22 <= 18 + 5 = 23 (satisfied with delay=5)
        routing_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 10.0, (2, 1) => 10.0,
            (2, 3) => 12.0, (3, 2) => 12.0,
            (1, 3) => 18.0, (3, 1) => 18.0,
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 5.0)

        combinations = find_detour_combinations(model, data)

        # Should find (1, 2, 3) since it satisfies all conditions
        @test length(combinations) >= 1
        @test (1, 2, 3) in combinations
    end

    @testset "Delay constraint too tight" begin
        # Same setup but delay is too small
        station_ids = [1, 2, 3]

        # Costs: A->B = 10, B->C = 10, A->C = 15
        # 10 + 10 = 20 > 15 + 2 = 17 (should NOT be included with delay=2)
        routing_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 10.0, (2, 1) => 10.0,
            (2, 3) => 10.0, (3, 2) => 10.0,
            (1, 3) => 15.0, (3, 1) => 15.0,
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 2.0)

        combinations = find_detour_combinations(model, data)

        # With tight delay constraint, the triplet should not be included
        @test !((1, 2, 3) in combinations)
    end

    @testset "Longest edge requirement" begin
        # Setup where A->C is NOT the longest edge
        station_ids = [1, 2, 3]

        # Costs: A->B = 30, B->C = 10, A->C = 15
        # A->B (30) is the longest, not A->C, so (1, 2, 3) should not be included
        routing_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 30.0, (2, 1) => 30.0,
            (2, 3) => 10.0, (3, 2) => 10.0,
            (1, 3) => 15.0, (3, 1) => 15.0,
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 100.0)

        combinations = find_detour_combinations(model, data)

        # (1, 2, 3) should NOT be in results because A->C is not the longest edge
        @test !((1, 2, 3) in combinations)
    end

    @testset "Multiple valid combinations" begin
        # 4 stations arranged in a pattern with valid detours
        station_ids = [1, 2, 3, 4]

        # Create a network satisfying triangle inequality
        # All short edges = 10, some long edges = 18
        # Triangle inequality: 10 + 10 = 20 >= 18 (satisfied)
        # Long edge is longest (18 > 10)
        # Detour: 20 <= 18 + 5 = 23 (satisfied)
        routing_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in station_ids, j in station_ids
            if i == j
                routing_costs[(i, j)] = 0.0
            else
                routing_costs[(i, j)] = 10.0
            end
        end

        # Create long edges 1->4 and 2->4 that enable detours
        routing_costs[(1, 4)] = 18.0
        routing_costs[(4, 1)] = 18.0
        routing_costs[(2, 4)] = 18.0
        routing_costs[(4, 2)] = 18.0

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 4, 1.0, 300.0, 5.0)

        combinations = find_detour_combinations(model, data)

        # Expected valid combinations based on the routing costs:
        # (1, 2, 4): t(1->2)=10, t(2->4)=18, t(1->4)=18. Longest=18, sum=28 <= 18+5=23? No
        # (1, 3, 4): t(1->3)=10, t(3->4)=10, t(1->4)=18. Longest=18, sum=20 <= 18+5=23? Yes
        # (2, 1, 4): t(2->1)=10, t(1->4)=18, t(2->4)=18. Longest=18, sum=28 <= 18+5=23? No
        # (2, 3, 4): t(2->3)=10, t(3->4)=10, t(2->4)=18. Longest=18, sum=20 <= 18+5=23? Yes
        # Valid: (1, 3, 4), (2, 3, 4)
        @test length(combinations) >= 1
        @test (1, 3, 4) in combinations
        @test (2, 3, 4) in combinations
    end

    @testset "Empty result for equal distances" begin
        # When all edges are equal, no intermediate point saves time
        station_ids = [1, 2, 3]

        # All edges equal (equilateral triangle)
        routing_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in station_ids, j in station_ids
            if i == j
                routing_costs[(i, j)] = 0.0
            else
                routing_costs[(i, j)] = 10.0
            end
        end

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 5.0)

        combinations = find_detour_combinations(model, data)

        # With equal edges, no edge is the longest, so no valid detours
        @test isempty(combinations)
    end

    @testset "No duplicate combinations" begin
        station_ids = [1, 2, 3, 4]

        routing_costs = Dict{Tuple{Int,Int}, Float64}()
        for i in station_ids, j in station_ids
            if i == j
                routing_costs[(i, j)] = 0.0
            else
                routing_costs[(i, j)] = 10.0
            end
        end
        # Create some long edges
        routing_costs[(1, 4)] = 18.0
        routing_costs[(4, 1)] = 18.0

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 4, 1.0, 300.0, 5.0)

        combinations = find_detour_combinations(model, data)

        # Check no duplicates by sorting tuples and comparing
        sorted_combinations = [Tuple(sort(collect(c))) for c in combinations]
        unique_sorted = unique(sorted_combinations)
        @test length(sorted_combinations) == length(unique_sorted)
    end

    @testset "Same source detour combinations" begin
        station_ids = [1, 2, 3]

        # Same setup as basic test
        routing_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 10.0, (2, 1) => 10.0,
            (2, 3) => 12.0, (3, 2) => 12.0,
            (1, 3) => 18.0, (3, 1) => 18.0,
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 5.0)

        # Same source returns triplets (j, k, l)
        same_source = find_same_source_detour_combinations(model, data)

        @test length(same_source) >= 1
        @test all(length(t) == 3 for t in same_source)
        @test (1, 2, 3) in same_source
    end

    @testset "Same dest detour combinations with time delta" begin
        station_ids = [1, 2, 3]

        # Costs: j->k = 600 seconds, so with time_window=300, t' = 2
        routing_costs = Dict{Tuple{Int,Int}, Float64}(
            (1, 2) => 600.0, (2, 1) => 600.0,  # j->k takes 600s
            (2, 3) => 300.0, (3, 2) => 300.0,  # k->l takes 300s
            (1, 3) => 800.0, (3, 1) => 800.0,  # j->l takes 800s (longest)
            (1, 1) => 0.0, (2, 2) => 0.0, (3, 3) => 0.0
        )
        # Triangle: 600 + 300 = 900 >= 800 (satisfied)
        # Longest: 800 > 600 and 800 > 300 (satisfied)
        # Delay: 900 <= 800 + 150 = 950 (satisfied with delay=150)

        data = create_test_data_with_routing(station_ids, routing_costs)
        model = TwoStageSingleDetourModel(2, 3, 1.0, 300.0, 150.0)  # time_window=300

        # Same dest returns quadruplets (j, k, l, t')
        same_dest = find_same_dest_detour_combinations(model, data)

        @test length(same_dest) >= 1
        @test all(length(t) == 4 for t in same_dest)

        # Find the (1, 2, 3, t') tuple
        matching = filter(t -> t[1:3] == (1, 2, 3), same_dest)
        @test length(matching) == 1

        # t' = floor(600 / 300) = 2
        @test matching[1][4] == 2
    end
end
