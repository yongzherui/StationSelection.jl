using StationSelection
using Test

@testset "ClusteringBase" begin
    using JuMP
    using StationSelection: clustering_base
    
    # The simple test case should have 2 candidates stations and 1 customer request, and we should only choose the station that has the lowest cost.
    # We will have an asymmetric cost matrix so that we know candidate station A is better than candidate station B.
    @testset "Simple Testcase" begin
        candidate_stations = DataFrame(id=[1, 2], lat=[27.9, 27.91], lon=[113.1, 113.11])
        customer_requests = DataFrame(id=[1], start_station_id=[1], end_station_id=[2], request_time=DateTime.(["2025-01-15 10:42:33"], "yyyy-mm-dd HH:MM:SS"))

        costs = Dict{Tuple{Int, Int}, Float64}((1, 2) => 5.0, (2, 1) => 10.0, (1, 1) => 0.0, (2, 2) => 0.0)

        k = 1
        result = clustering_base(candidate_stations, k, customer_requests, costs)

        @test result.status == true
        @test sum(values(result.stations)) == 1
        @test result.stations[1] == false
        @test result.stations[2] == true
    end

    # we need a test case that is able to choose an alternative station based on the fact that the number of request has made the cost
    # at station 2 more expensive than 1 and we open station 1 instead
    # we introduce a 3rd station because by symmetry if we had 2 stations we will open station 2
    @testset "Simple Multi-request Testcase" begin
        # we test the mapping by setting the ids to 4, 5, 6
        candidate_stations = DataFrame(id=[4, 5, 6], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])
        # we added enough requests from 1 to 2 such that 2 to 1 is favourable
        # customer_requests = [
        #     DataFrame(id=[1], start_station_id=[4], end_station_id=[5], request_time=["2025-01-15 10:42:33"]),
        #     DataFrame(id=[2], start_station_id=[4], end_station_id=[5], request_time=["2025-01-15 10:42:33"]),
        #     DataFrame(id=[3], start_station_id=[4], end_station_id=[5], request_time=["2025-01-15 10:42:33"]),
        #     DataFrame(id=[4], start_station_id=[5], end_station_id=[4], request_time=["2025-01-15 10:42:33"])
        # ]
        customer_requests = DataFrame(
                id = [1, 2, 3, 4],
                start_station_id = [4, 4, 4, 4],
                end_station_id   = [5, 5, 5, 5],
                request_time     = DateTime.(
                    ["2025-01-15 10:42:33",
                     "2025-01-15 10:42:33",
                     "2025-01-15 10:42:33",
                     "2025-01-15 10:42:33"],
                    "yyyy-mm-dd HH:MM:SS"
                )
            )

        # Now the cost of opening station 2 should be 15 and thus it is not cheaper to open station 1
        costs = Dict{Tuple{Int, Int}, Float64}(
            (4, 5) => 5.0, 
            (5, 4) => 10.0, 
            (4, 4) => 0.0, 
            (5, 5) => 1000.0, # (unrealistic assumption but it is just to test the model)we make this very expensive so that they will go to station 6 and then walk to station 5
            (4, 6) => 8.0, 
            (6, 4) => 8.0, 
            (5, 6) => 3.0, 
            (6, 5) => 3.0, 
            (6, 6) => 0.0)

        # we allow k to be 2 so we choose 1 and 3
        k = 2
        result = clustering_base(candidate_stations, k, customer_requests, costs)

        @test result.status == true
        @test sum(values(result.stations)) == 2
        @test result.stations == Dict(4 => true, 5 => false, 6 => true)
        @test result.value == [1, 0, 1]
        @test result.stations[4] == true
        @test result.stations[5] == false
        @test result.stations[6] == true
    end
end