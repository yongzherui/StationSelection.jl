using Test
using StationSelection
using DataFrames
using Dates

@testset "StationSelection.jl" begin
    @testset "Module loads" begin
        @test isdefined(StationSelection, :Result)
        @test isdefined(StationSelection, :read_candidate_stations)
        @test isdefined(StationSelection, :read_customer_requests)
        @test isdefined(StationSelection, :bd09_to_wgs84)
        @test isdefined(StationSelection, :compute_station_pairwise_costs)
        @test isdefined(StationSelection, :generate_scenarios)
    end

    @testset "CoordTransform" begin
        # Test BD-09 to WGS84 conversion
        lon, lat = bd09_to_wgs84(113.0, 28.0)
        @test isa(lon, Float64)
        @test isa(lat, Float64)

        # Test with known coordinates
        bd_lon, bd_lat = 113.16900071992336, 27.91697197316557
        wgs_lon, wgs_lat = bd09_to_wgs84(bd_lon, bd_lat)
        @test wgs_lon < bd_lon  # WGS84 should be slightly different
        @test wgs_lat < bd_lat
    end

    @testset "Data Loading" begin
        include("data/test_stations.jl")
        include("data/test_requests.jl")
    end

    @testset "Utilities" begin
        include("utils/test_scenarios.jl")
        include("utils/test_costs.jl")
    end

    @testset "Optimization Methods" begin
        include("optimization/test_base.jl")
        include("optimization/test_two_stage_l.jl")
        include("optimization/test_two_stage_lambda.jl")
        include("optimization/test_routing_transport.jl")
        include("optimization/test_origin_dest_pair.jl")
    end
end
