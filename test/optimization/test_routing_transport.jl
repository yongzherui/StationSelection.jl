using StationSelection
using Test
using DataFrames, Dates, JuMP

@testset "ClusteringTwoStageLRoutingTransportation" begin
    using .ClusteringTwoStageLRoutingTransportation: clustering_two_stage_l_routing_transportation, validate_request_flow_mapping
    using .ClusteringTwoStageL: clustering_two_stage_l

    # -----------------------
    # Simple test case with routing
    # -----------------------
    @testset "Simple Testcase with Routing" begin
        candidate_stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
        customer_requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[2],
            request_time=DateTime.(["2025-01-15 10:42:33"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 2) => 5.0, (2, 1) => 10.0, (1, 1) => 0.0, (2, 2) => 0.0
        )
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 2) => 3.0, (2, 1) => 3.0, (1, 1) => 0.0, (2, 2) => 0.0
        )

        k = 1
        l = 2
        lambda = 1.0

        result = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
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
    # Multi-request test case
    # -----------------------
    @testset "Multi-request Testcase" begin
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
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (4, 5) => 5.0,
            (5, 4) => 10.0,
            (4, 4) => 0.0,
            (5, 5) => 1000.0,
            (4, 6) => 8.0,
            (6, 4) => 8.0,
            (5, 6) => 3.0,
            (6, 5) => 3.0,
            (6, 6) => 0.0
        )
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (4, 5) => 2.0,
            (5, 4) => 2.0,
            (4, 4) => 0.0,
            (5, 5) => 0.0,
            (4, 6) => 4.0,
            (6, 4) => 4.0,
            (5, 6) => 1.5,
            (6, 5) => 1.5,
            (6, 6) => 0.0
        )

        k = 2
        l = 3
        lambda = 1.0

        result = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l

        # Check scenario-specific selections (stored in station_df columns)
        scenario_cols = filter(name -> name ∉ ["id", "lon", "lat", "selected"], names(result.station_df))
        for col in scenario_cols
            @test sum(result.station_df[!, col]) == k
        end
    end

    # -----------------------
    # Test lambda = 0 matches ClusteringTwoStageL
    # -----------------------
    @testset "Lambda = 0 matches ClusteringTwoStageL" begin
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 2],
            end_station_id   = [2, 3],
            request_time     = DateTime.(
                ["2025-01-15 10:00:00", "2025-01-15 10:30:00"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 100.0, (1, 3) => 200.0,
            (2, 1) => 100.0, (2, 2) => 0.0, (2, 3) => 100.0,
            (3, 1) => 200.0, (3, 2) => 100.0, (3, 3) => 0.0
        )

        k = 2
        l = 3

        # Run ClusteringTwoStageL
        result_two_stage_l = clustering_two_stage_l(
            candidate_stations, k, customer_requests, walking_costs, scenarios;
            l=l
        )

        # Run ClusteringTwoStageLRoutingTransportation with lambda = 0
        result_routing_lambda0 = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=0.0
        )

        @test result_two_stage_l.status == true
        @test result_routing_lambda0.status == true

        # Both should select the same stations
        @test result_two_stage_l.stations == result_routing_lambda0.stations

        # Both should have the same objective value (walking distance only)
        @test isapprox(objective_value(result_two_stage_l.model),
                      objective_value(result_routing_lambda0.model),
                      atol=1e-6)
    end

    # -----------------------
    # Test lambda parameter effect
    # -----------------------
    @testset "Lambda parameter effect" begin
        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 2],
            end_station_id   = [2, 3],
            request_time     = DateTime.(
                ["2025-01-15 10:00:00", "2025-01-15 10:30:00"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 100.0, (1, 3) => 200.0,
            (2, 1) => 100.0, (2, 2) => 0.0, (2, 3) => 100.0,
            (3, 1) => 200.0, (3, 2) => 100.0, (3, 3) => 0.0
        )

        k = 2
        l = 3

        # Test with low lambda (emphasize walking distance)
        result_low_lambda = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=0.1
        )

        # Test with high lambda (emphasize routing cost)
        result_high_lambda = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=100.0
        )

        @test result_low_lambda.status == true
        @test result_high_lambda.status == true

        # Both should select correct number of stations
        @test sum(values(result_low_lambda.stations)) == l
        @test sum(values(result_high_lambda.stations)) == l
    end

    # -----------------------
    # Test flow variable activation
    # -----------------------
    @testset "Flow variable activation with high lambda" begin
        # Setup: 3 stations, k=2, l=2
        # Trip: station 1 -> station 3
        # Expected: Select stations 2 and 3, with flow from 2->3
        # Customer walks to station 2 for pickup, gets dropped at station 3
        # Vehicle routes from station 2 to station 3

        candidate_stations = DataFrame(id=[1, 2, 3], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        customer_requests = DataFrame(
            id = [1],
            start_station_id = [1],
            end_station_id   = [3],
            request_time     = DateTime.(["2025-01-15 10:00:00"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        # Walking costs:
        # - Make walking from origin 1 -> station 3 expensive (pickup)
        # - Make walking from destination 3 <- stations 1,2 expensive (dropoff)
        # This forces: pickup at station 2, dropoff at station 3
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 1.0, (1, 3) => 100.0,    # Origin 1: walking to station 3 is expensive
            (2, 1) => 1.0, (2, 2) => 0.0, (2, 3) => 1.0,
            (3, 1) => 100.0, (3, 2) => 100.0, (3, 3) => 0.0   # Destination 3: must drop off at station 3
        )

        # Routing costs: route from 2->3 is very cheap (0.1), self-loops are 0.0, others are expensive
        # Self-loops (i->i) should always be 0.0 since no routing is needed
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0,   (1, 2) => 1000.0, (1, 3) => 500.0,
            (2, 1) => 1000.0, (2, 2) => 0.0,   (2, 3) => 0.1,
            (3, 1) => 500.0,  (3, 2) => 1000.0, (3, 3) => 0.0
        )

        k = 2
        l = 2
        lambda = 100.0  # Heavy penalty on routing cost

        result = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true

        # Extract the model to check variable values
        m = result.model

        # Check station selection (y variables) - should select stations 2 and 3
        y = m[:y]
        @test value(y[1]) < 0.5  # Station 1 not selected
        @test value(y[2]) > 0.5  # Station 2 selected
        @test value(y[3]) > 0.5  # Station 3 selected

        # Check scenario activation (z variables)
        z = m[:z]
        @test value(z[1, 1]) < 0.5  # Station 1 not active in scenario 1
        @test value(z[2, 1]) > 0.5  # Station 2 active in scenario 1
        @test value(z[3, 1]) > 0.5  # Station 3 active in scenario 1

        # Check pick-up assignment (x_pick) - customer at station 1 picks up at station 2
        x_pick = m[:x_pick]
        @test value(x_pick[1, 2, 1]) > 0.5  # Pick up at station 2
        @test value(x_pick[1, 1, 1]) < 0.5  # Not at station 1
        @test value(x_pick[1, 3, 1]) < 0.5  # Not at station 3

        # Check drop-off assignment (x_drop) - customer going to station 3 drops off at station 3
        x_drop = m[:x_drop]
        @test value(x_drop[3, 3, 1]) > 0.5  # Drop off at station 3
        @test value(x_drop[3, 1, 1]) < 0.5  # Not at station 1
        @test value(x_drop[3, 2, 1]) < 0.5  # Not at station 2

        # Check supply variables (p) - supply at station 2
        p = m[:p]
        @test isapprox(value(p[2, 1]), 1.0, atol=1e-6)  # 1 passenger picked up at station 2
        @test isapprox(value(p[1, 1]), 0.0, atol=1e-6)  # No pick-ups at station 1
        @test isapprox(value(p[3, 1]), 0.0, atol=1e-6)  # No pick-ups at station 3

        # Check demand variables (d) - demand at station 3
        d = m[:d]
        @test isapprox(value(d[3, 1]), 1.0, atol=1e-6)  # 1 passenger to drop off at station 3
        @test isapprox(value(d[1, 1]), 0.0, atol=1e-6)  # No drop-offs at station 1
        @test isapprox(value(d[2, 1]), 0.0, atol=1e-6)  # No drop-offs at station 2

        # THE KEY CHECK: Flow variables (f) - flow from station 2 to station 3
        f = m[:f]
        @test isapprox(value(f[2, 3, 1]), 1.0, atol=1e-6)  # Flow of 1 passenger from station 2 to 3

        # All other flows should be zero (or very small)
        @test isapprox(value(f[1, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[1, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[1, 3, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[2, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[2, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[3, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[3, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[3, 3, 1]), 0.0, atol=1e-6)

        # Verify objective value matches expected calculation
        # Walking: 1.0 (pickup at station 2) + 0.0 (dropoff at station 3) = 1.0
        # Routing: 100.0 * 0.1 (flow from 2 to 3) = 10.0
        # Total: 1.0 + 10.0 = 11.0
        @test isapprox(objective_value(m), 11.0, atol=1e-6)

        # NEW: Test flow validation function
        scenario_requests = Dict{Int, DataFrame}()
        scenario_requests[1] = customer_requests

        validation_df = validate_request_flow_mapping(
            result,
            scenario_requests,
            candidate_stations
        )

        # Check validation results
        @test nrow(validation_df) == 1  # One request
        @test validation_df[1, :valid] == true  # Should be valid
        @test validation_df[1, :pickup_station_id] == 2  # Picked up at station 2
        @test validation_df[1, :dropoff_station_id] == 3  # Dropped off at station 3
        @test validation_df[1, :flow_value] ≈ 1.0 atol=1e-6  # Flow should be 1.0
        @test validation_df[1, :flow_remaining_after] ≈ 0.0 atol=1e-6  # All flow accounted for
    end

    # -----------------------
    # Test flow aggregation with multiple itineraries
    # -----------------------
    @testset "Flow aggregation with multiple passengers" begin
        # Setup: 4 stations, k=2, l=2
        # Two trips: 1->3 and 2->4
        # Expected: Select stations 2, 3 (NOT 4 due to expensive routing 2->4)
        # Both trips pick up at station 2, creating p[2,1] = 2
        # BOTH trips drop off at station 3, creating d[3,1] = 2
        # Trip 2 passenger walks from station 3 to final destination 4
        # This creates flow f[2,3,1] = 2 (aggregated flow of 2 passengers)

        candidate_stations = DataFrame(id=[1, 2, 3, 4], lat=[27.9, 27.91, 27.92, 27.93], lon=[113.1, 113.11, 113.12, 113.13])
        customer_requests = DataFrame(
            id = [1, 2],
            start_station_id = [1, 2],
            end_station_id   = [3, 4],
            request_time     = DateTime.(
                ["2025-01-15 10:00:00", "2025-01-15 10:05:00"],
                "yyyy-mm-dd HH:MM:SS"
            )
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]

        # Walking costs:
        # - Force both to pick up at station 2
        # - Destination 3 drops off at station 3 (free)
        # - Destination 4 can drop off at station 3 (with penalty of 10.0)
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            # Origin 1: only station 2 is cheap for pickup
            (1, 1) => 100.0, (1, 2) => 1.0, (1, 3) => 100.0, (1, 4) => 100.0,
            # Origin 2: only station 2 is cheap for pickup
            (2, 1) => 100.0, (2, 2) => 1.0, (2, 3) => 100.0, (2, 4) => 100.0,
            # Destination 3: drop off at station 3 is free
            (3, 1) => 100.0, (3, 2) => 100.0, (3, 3) => 0.0, (3, 4) => 100.0,
            # Destination 4: drop off at station 3 with penalty 10.0, others expensive
            (4, 1) => 100.0, (4, 2) => 100.0, (4, 3) => 10.0, (4, 4) => 0.0
        )

        # Routing costs: 2->3 is cheap (0.1), 2->4 is EXPENSIVE (100.0)
        # This forces model to select {2,3} and route everyone to station 3
        routing_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0,   (1, 2) => 1000.0, (1, 3) => 1000.0, (1, 4) => 1000.0,
            (2, 1) => 1000.0, (2, 2) => 0.0,   (2, 3) => 0.1,    (2, 4) => 100.0,
            (3, 1) => 1000.0, (3, 2) => 1000.0, (3, 3) => 0.0,   (3, 4) => 1000.0,
            (4, 1) => 1000.0, (4, 2) => 1000.0, (4, 3) => 1000.0, (4, 4) => 0.0
        )

        k = 2
        l = 2
        lambda = 100.0

        result = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true

        # Extract the model to check variable values
        m = result.model

        # Check station selection (y variables) - should select stations 2 and 3 (NOT 4)
        y = m[:y]
        @test value(y[1]) < 0.5  # Station 1 not selected
        @test value(y[2]) > 0.5  # Station 2 selected
        @test value(y[3]) > 0.5  # Station 3 selected
        @test value(y[4]) < 0.5  # Station 4 NOT selected (routing 2->4 too expensive)

        # Check pick-up assignments - BOTH customers pick up at station 2
        x_pick = m[:x_pick]
        @test value(x_pick[1, 2, 1]) > 0.5  # Trip 1 (origin=1) picks up at station 2
        @test value(x_pick[2, 2, 1]) > 0.5  # Trip 2 (origin=2) picks up at station 2

        # Check drop-off assignments - BOTH customers drop off at station 3
        x_drop = m[:x_drop]
        @test value(x_drop[3, 3, 1]) > 0.5  # Trip 1 (destination=3) drops off at station 3
        @test value(x_drop[4, 3, 1]) > 0.5  # Trip 2 (destination=4) drops off at station 3

        # Verify trip 2 does NOT drop off at other stations
        @test value(x_drop[4, 1, 1]) < 0.5
        @test value(x_drop[4, 2, 1]) < 0.5
        @test value(x_drop[4, 4, 1]) < 0.5

        # KEY CHECK: Supply variable p[2,1] should equal 2 (both passengers picked up at station 2)
        p = m[:p]
        @test isapprox(value(p[2, 1]), 2.0, atol=1e-6)  # 2 passengers picked up at station 2
        @test isapprox(value(p[1, 1]), 0.0, atol=1e-6)  # No pick-ups at station 1
        @test isapprox(value(p[3, 1]), 0.0, atol=1e-6)  # No pick-ups at station 3
        @test isapprox(value(p[4, 1]), 0.0, atol=1e-6)  # No pick-ups at station 4

        # KEY CHECK: Demand variable d[3,1] should equal 2 (BOTH passengers dropped off at station 3)
        d = m[:d]
        @test isapprox(value(d[1, 1]), 0.0, atol=1e-6)  # No drop-offs at station 1
        @test isapprox(value(d[2, 1]), 0.0, atol=1e-6)  # No drop-offs at station 2
        @test isapprox(value(d[3, 1]), 2.0, atol=1e-6)  # 2 passengers dropped off at station 3
        @test isapprox(value(d[4, 1]), 0.0, atol=1e-6)  # No drop-offs at station 4 (not selected)

        # KEY CHECK: Flow variable f[2,3,1] should equal 2 (flow of 2 passengers from station 2 to 3)
        f = m[:f]
        @test isapprox(value(f[2, 3, 1]), 2.0, atol=1e-6)  # Flow of 2 passengers from station 2 to 3

        # All other flows should be zero
        @test isapprox(value(f[1, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[1, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[1, 3, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[1, 4, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[2, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[2, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[2, 4, 1]), 0.0, atol=1e-6)  # No flow to station 4
        @test isapprox(value(f[3, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[3, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[3, 3, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[3, 4, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[4, 1, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[4, 2, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[4, 3, 1]), 0.0, atol=1e-6)
        @test isapprox(value(f[4, 4, 1]), 0.0, atol=1e-6)

        # Verify objective value
        # Walking pickup: 1.0 + 1.0 = 2.0 (both walk to station 2)
        # Walking dropoff: 0.0 + 10.0 = 10.0 (trip 1 arrives at dest, trip 2 walks from station 3 to dest 4)
        # Routing: lambda * f[2,3,1] * cost = 100.0 * 2.0 * 0.1 = 20.0
        # Total: 2.0 + 10.0 + 20.0 = 32.0
        @test isapprox(objective_value(m), 32.0, atol=1e-6)

        # NEW: Test flow validation function with multiple requests
        scenario_requests = Dict{Int, DataFrame}()
        scenario_requests[1] = customer_requests

        validation_df = validate_request_flow_mapping(
            result,
            scenario_requests,
            candidate_stations
        )

        # Check validation results
        @test nrow(validation_df) == 2  # Two requests
        @test all(validation_df.valid)  # Both should be valid

        # Both requests should pick up at station 2
        @test all(validation_df.pickup_station_id .== 2)

        # Both requests should drop off at station 3
        @test all(validation_df.dropoff_station_id .== 3)

        # Both should use the same flow f[2,3,1] which has value 2.0
        @test all(validation_df.flow_value .≈ 2.0)

        # After accounting for both passengers, flow should be 0
        @test validation_df[2, :flow_remaining_after] ≈ 0.0 atol=1e-6
    end

    # -----------------------
    # Test multi-scenario case
    # -----------------------
    @testset "Multi-scenario Testcase" begin
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
            ("2025-01-15 08:00:00","2025-01-15 11:59:59"),  # Morning scenario
            ("2025-01-15 12:00:00","2025-01-15 23:59:59")   # Afternoon scenario
        ]

        walking_costs = Dict{Tuple{Int, Int}, Float64}(
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

        result = clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )

        @test result.status == true
        @test sum(values(result.stations)) == l

        # Check each scenario has exactly k stations (stored in station_df columns)
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
        walking_costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )
        routing_costs = walking_costs  # Same costs for simplicity

        k = 3
        l = 2  # l < k, should error
        lambda = 1.0

        @test_throws ErrorException clustering_two_stage_l_routing_transportation(
            candidate_stations, k, customer_requests, walking_costs, routing_costs, scenarios;
            l=l, lambda=lambda
        )
    end
end
