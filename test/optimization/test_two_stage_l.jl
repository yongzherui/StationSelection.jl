using StationSelection
using Test
using DataFrames, Dates, JuMP

@testset "ClusteringTwoStageL" begin
    using .ClusteringTwoStageL: clustering_two_stage_l
    using .ClusteringBase: clustering_base

    # -----------------------
    # Simple test case
    # -----------------------
    @testset "Simple Testcase" begin
        candidate_stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
        customer_requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[2],
            request_time=DateTime.(["2025-01-15 10:42:33"], "yyyy-mm-dd HH:MM:SS")
        )

        scenarios = [("2025-01-15 00:00:00","2025-01-15 23:59:59")]
        costs = Dict{Tuple{Int, Int}, Float64}((1, 2) => 5.0, (2, 1) => 10.0, (1, 1) => 0.0, (2, 2) => 0.0)

        k = 1
        l = 2
        result = clustering_two_stage_l(candidate_stations, k, customer_requests, costs, scenarios; l=l)

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
        costs = Dict{Tuple{Int, Int}, Float64}(
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

        k = 2
        l = 3
        result = clustering_two_stage_l(candidate_stations, k, customer_requests, costs, scenarios; l=l)

        @test result.status == true
        @test sum(values(result.stations)) == l

        # Check scenario-specific selections (stored in station_df columns)
        scenario_cols = filter(name -> name ∉ ["id", "lon", "lat", "selected"], names(result.station_df))
        for col in scenario_cols
            @test sum(result.station_df[!, col]) == k
        end
    end

    # -----------------------
    # Multi-scenario test case
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

        costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0, (1, 4) => 15.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0, (2, 4) => 10.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0, (3, 4) => 5.0,
            (4, 1) => 15.0, (4, 2) => 10.0, (4, 3) => 5.0, (4, 4) => 0.0
        )

        k = 2
        l = 3
        result = clustering_two_stage_l(candidate_stations, k, customer_requests, costs, scenarios; l=l)

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
        costs = Dict{Tuple{Int, Int}, Float64}(
            (1, 1) => 0.0, (1, 2) => 5.0, (1, 3) => 10.0,
            (2, 1) => 5.0, (2, 2) => 0.0, (2, 3) => 5.0,
            (3, 1) => 10.0, (3, 2) => 5.0, (3, 3) => 0.0
        )

        k = 3
        l = 2  # l < k, should error

        @test_throws ErrorException clustering_two_stage_l(
            candidate_stations, k, customer_requests, costs, scenarios; l=l
        )
    end
end
