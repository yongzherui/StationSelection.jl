using StationSelection
using Test
using DataFrames, Dates, JuMP

@testset "ClusteringTwoStageLambda" begin
    using .ClusteringTwoStageLambda: clustering_two_stage_lambda
    using .ClusteringBase: clustering_base

    @testset "No Scenario defined" begin
        candidate_stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
        customer_requests = DataFrame(
            id=[1], start_station_id=[1], end_station_id=[2],
            request_time=DateTime.(["2025-01-15 10:42:33"], "yyyy-mm-dd HH:MM:SS")
        )

        costs = Dict{Tuple{Int, Int}, Float64}((1, 2) => 5.0, (2, 1) => 10.0, (1, 1) => 0.0, (2, 2) => 0.0)

        k = 1
        result_base = clustering_base(candidate_stations, k, customer_requests, costs)
        result_two_stage = clustering_two_stage_lambda(candidate_stations, k, customer_requests, costs; lambda=0.0)

        @test result_two_stage.status == true
        @test sum(values(result_two_stage.stations)) == 1

        # Compare selected stations
        @test result_two_stage.stations == result_base.stations
    end

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
        result_base = clustering_base(candidate_stations, k, customer_requests, costs)
        result_two_stage = clustering_two_stage_lambda(candidate_stations, k, customer_requests, costs, scenarios; lambda=0.0)

        @test result_two_stage.status == true
        @test sum(values(result_two_stage.stations)) == 1

        # Compare selected stations
        @test result_two_stage.stations == result_base.stations
    end

    # -----------------------
    # Multi-request test case
    # -----------------------
    @testset "Simple Multi-request Testcase" begin
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
        result_base = clustering_base(candidate_stations, k, customer_requests, costs)
        result_two_stage = clustering_two_stage_lambda(candidate_stations, k, customer_requests, costs, scenarios; lambda=0.0)

        @test result_two_stage.status == true
        @test sum(values(result_two_stage.stations)) == 2

        # Compare selected stations
        @test result_two_stage.stations == result_base.stations
        @test result_two_stage.value == result_base.value
    end
end
