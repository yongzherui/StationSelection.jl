using StationSelection
using Test
using DataFrames, Dates, JuMP

@testset "ClusteringTwoStageLOriginDestPair" begin
    using .ClusteringTwoStageLOriginDestPair: clustering_two_stage_l_od_pair

    # -----------------------
    # Simple test case with OD pair routing
    # -----------------------
    @testset "Simple Testcase with OD Pairs" begin
        candidate_stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
        customer_requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[2],
            request_time=DateTime.(["2025-01-15 10:42:33"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        # Origin costs: walking from origin o to pick-up station j
        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0,
            (2, 1) => 10.0, (2, 2) => 0.0
        )

        # Destination costs: walking from drop-off station k to destination d
        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0
        )

        # Routing costs: vehicle routing from station j to station k
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 2) => 3.0, (2, 1) => 3.0, (1, 1) => 0.0, (2, 2) => 0.0
        )

        k = 1
        l = 2
        lambda = 1.0

        result = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l  # l stations pre-selected

        # Check scenario-specific selections (stored in station_df columns)
        scenario_cols = filter(name -> name ∉ ["id", "lon", "lat", "selected"], names(result.station_df))
        for col in scenario_cols
            @test sum(result.station_df[!, col]) == k  # k stations per scenario
        end
    end

    # -----------------------
    # Multi-request test case with multiple OD pairs
    # -----------------------
    @testset "Multi-request Testcase with OD Pairs" begin
        candidate_stations = DataFrame(id=[4, 5, 6], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1, 2, 3, 4],
            start_station_id = [4, 4, 4, 4],
            end_station_id   = [5, 5, 5, 5],
            request_time     = DateTime.(
                ["2025-01-15 10:42:33", "2025-01-15 10:42:33", "2025-01-15 10:42:33", "2025-01-15 10:42:33"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        # Origin costs: walking from origin to pick-up station
        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (4, 4) => 0.0, (4, 5) => 5.0, (4, 6) => 8.0,
            (5, 4) => 10.0, (5, 5) => 0.0, (5, 6) => 3.0,
            (6, 4) => 8.0, (6, 5) => 3.0, (6, 6) => 0.0
        )

        # Destination costs: walking from drop-off station to destination
        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (4, 4) => 0.0, (4, 5) => 10.0, (4, 6) => 8.0,
            (5, 4) => 5.0, (5, 5) => 1000.0, (5, 6) => 3.0,
            (6, 4) => 8.0, (6, 5) => 3.0, (6, 6) => 0.0
        )

        # Routing costs: vehicle routing between stations
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (4, 5) => 2.0, (5, 4) => 2.0, (4, 4) => 0.0, (5, 5) => 0.0,
            (4, 6) => 4.0, (6, 4) => 4.0, (5, 6) => 1.5, (6, 5) => 1.5,
            (6, 6) => 0.0
        )

        k = 2
        l = 3
        lambda = 1.0

        result = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l

        # Check scenario-specific selections
        scenario_cols = filter(name -> name ∉ ["id", "lon", "lat", "selected"], names(result.station_df))
        for col in scenario_cols
            @test sum(result.station_df[!, col]) == k
        end
    end

    # -----------------------
    # Test different OD pairs in different scenarios
    # -----------------------
    @testset "Different OD pairs per scenario" begin
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1, 2, 3],
            start_station_id = [1, 2, 3],
            end_station_id   = [2, 3, 1],
            request_time     = DateTime.(
                ["2025-01-15 08:00:00", "2025-01-15 09:00:00", "2025-01-15 14:00:00"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [
            ("2025-01-15 08:00:00","2025-01-15 11:59:59"),  # Morning: only request 1 and 2
            ("2025-01-15 12:00:00","2025-01-15 23:59:59")   # Afternoon: only request 3
        ]

        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )

        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )

        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 2.0, (1, 3) => 4.0,
            (2, 1) => 2.0, (2, 2) => 0.0, (2, 3) => 2.0,
            (3, 1) => 4.0, (3, 2) => 2.0, (3, 3) => 0.0
        )

        k = 2
        l = 3
        lambda = 1.0

        result = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l

        # Check each scenario has exactly k stations
        scenario_cols = filter(name -> name ∉ ["id", "lon", "lat", "selected"], names(result.station_df))
        @test length(scenario_cols) == 2
        for col in scenario_cols
            @test sum(result.station_df[!, col]) == k
        end
    end

    # -----------------------
    # Test lambda parameter effect on OD pairs
    # -----------------------
    @testset "Lambda parameter effect on OD assignment" begin
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id   = [3],
            request_time     = DateTime.(["2025-01-15 10:00:00"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        # Setup costs where routing distance matters
        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 10.0,
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 10.0, (3, 2) => 1.0, (3, 3) => 0.0
        )

        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 10.0,
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 10.0, (3, 2) => 1.0, (3, 3) => 0.0
        )

        # Routing costs: long distance routing is expensive
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 100.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 100.0, (3, 2) => 5.0, (3, 3) => 0.0
        )

        k = 2
        l = 3

        # Test with low lambda (emphasize walking distance)
        result_low_lambda = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=0.1
        )

        # Test with high lambda (emphasize routing cost)
        result_high_lambda = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=100.0
        )

        @test result_low_lambda.status == true
        @test result_high_lambda.status == true

        # Both should select correct number of stations
        @test sum(values(result_low_lambda.stations)) == l
        @test sum(values(result_high_lambda.stations)) == l

        # Objective values should be different due to different lambda
        obj_low = objective_value(result_low_lambda.model)
        obj_high = objective_value(result_high_lambda.model)
        @test obj_low != obj_high
    end

    # -----------------------
    # Test OD pair variable structure with high lambda
    # -----------------------
    @testset "OD pair variable structure verification" begin
        # Setup: 3 stations, trip from origin 1 to destination 3
        # Expected: model should choose stations to minimize total cost
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id   = [3],
            request_time     = DateTime.(["2025-01-15 10:00:00"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        # Origin costs: cheap to walk from 1 to station 1 or 2, expensive to station 3
        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 100.0,
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 100.0, (3, 2) => 1.0, (3, 3) => 0.0
        )

        # Destination costs: cheap to walk from station 2 or 3 to destination 3, expensive from station 1
        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 100.0,
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 100.0, (3, 2) => 1.0, (3, 3) => 0.0
        )

        # Routing costs: cheap to route from 2 to 3, expensive otherwise
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1000.0, (1, 3) => 1000.0,
            (2, 1) => 1000.0, (2, 2) => 0.0, (2, 3) => 0.1,
            (3, 1) => 1000.0, (3, 2) => 1000.0, (3, 3) => 0.0
        )

        k = 2
        l = 2
        lambda = 100.0

        result = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true

        # Extract the model
        m = result.model

        # Check station selection - should select stations 2 and 3
        y = m[:y]
        @test value(y[1]) < 0.5  # Station 1 not selected
        @test value(y[2]) > 0.5  # Station 2 selected
        @test value(y[3]) > 0.5  # Station 3 selected

        # Check scenario activation
        z = m[:z]
        @test value(z[1, 1]) < 0.5  # Station 1 not active
        @test value(z[2, 1]) > 0.5  # Station 2 active
        @test value(z[3, 1]) > 0.5  # Station 3 active

        # Verify objective: should pick up at station 2, drop off at station 3
        # Walking origin: 1.0 (origin 1 to station 2)
        # Walking dest: 0.0 (station 3 to destination 3)
        # Routing: 100.0 * 0.1 = 10.0 (route from station 2 to 3)
        # Total: 1.0 + 0.0 + 10.0 = 11.0
        @test isapprox(objective_value(m), 11.0, atol=1e-6)
    end

    # -----------------------
    # Test multiple OD pairs with aggregation
    # -----------------------
    @testset "Multiple OD pairs with same origin-destination" begin
        # Two requests with the same origin and destination should create one OD pair with demand 2
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 1],
            end_station_id   = [3, 3],
            request_time     = DateTime.(
                ["2025-01-15 10:00:00", "2025-01-15 10:05:00"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 10.0,
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 10.0, (3, 2) => 1.0, (3, 3) => 0.0
        )

        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 10.0,
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 10.0, (3, 2) => 1.0, (3, 3) => 0.0
        )

        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 2.0, (1, 3) => 4.0,
            (2, 1) => 2.0, (2, 2) => 0.0, (2, 3) => 2.0,
            (3, 1) => 4.0, (3, 2) => 2.0, (3, 3) => 0.0
        )

        k = 2
        l = 3
        lambda = 1.0

        result = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l

        # The objective should account for demand of 2
        # (exact value depends on optimal station selection)
        @test objective_value(result.model) > 0.0
    end

    # -----------------------
    # Test multi-scenario with different OD sets
    # -----------------------
    @testset "Multi-scenario with distinct OD pair sets" begin
        candidate_stations = DataFrame(id=[1, 2, 3, 4], lat=[27.9, 27.91, 27.92, 27.93], lon=[113.1, 113.11, 113.12, 113.13])
        customer_requests = DataFrame(
            id = [1, 2, 3, 4],
            start_station_id = [1, 2, 3, 4],
            end_station_id   = [2, 3, 4, 1],
            request_time     = DateTime.(
                ["2025-01-15 08:00:00", "2025-01-15 09:00:00", "2025-01-15 14:00:00", "2025-01-15 15:00:00"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [
            ("2025-01-15 08:00:00","2025-01-15 11:59:59"),  # Morning: OD pairs (1,2) and (2,3)
            ("2025-01-15 12:00:00","2025-01-15 23:59:59")   # Afternoon: OD pairs (3,4) and (4,1)
        ]

        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0, (1, 4) => 15.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0, (2, 4) => 10.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0, (3, 4) => 5.0,
            (4, 1) => 15.0, (4, 2) => 10.0, (4, 3) => 5.0, (4, 4) => 0.0
        )

        dest_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0, (1, 4) => 15.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0, (2, 4) => 10.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0, (3, 4) => 5.0,
            (4, 1) => 15.0, (4, 2) => 10.0, (4, 3) => 5.0, (4, 4) => 0.0
        )

        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 2.0, (1, 3) => 4.0, (1, 4) => 6.0,
            (2, 1) => 2.0, (2, 2) => 0.0, (2, 3) => 2.0, (2, 4) => 4.0,
            (3, 1) => 4.0, (3, 2) => 2.0, (3, 3) => 0.0, (3, 4) => 2.0,
            (4, 1) => 6.0, (4, 2) => 4.0, (4, 3) => 2.0, (4, 4) => 0.0
        )

        k = 2
        l = 3
        lambda = 1.0

        result = clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l

        # Check each scenario has exactly k stations
        scenario_cols = filter(name -> name ∉ ["id", "lon", "lat", "selected"], names(result.station_df))
        @test length(scenario_cols) == 2
        for col in scenario_cols
            @test sum(result.station_df[!, col]) == k
        end
    end

    # -----------------------
    # Test constraint: k <= l
    # -----------------------
    @testset "Parameter validation: k <= l" begin
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id   = [2],
            request_time     = DateTime.(["2025-01-15 10:00:00"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        origin_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )

        dest_costs = origin_costs
        routing_costs = origin_costs

        k = 3
        l = 2  # l < k, should error
        lambda = 1.0

        @test_throws ErrorException clustering_two_stage_l_od_pair(
            candidate_stations, k, customer_requests, origin_costs, dest_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )
    end
end
