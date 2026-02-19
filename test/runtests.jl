using Test
using StationSelection
using DataFrames
using Dates
using JuMP
const MOI = JuMP.MOI

@testset "StationSelection.jl" begin
    @testset "Module loads" begin
        @test isdefined(StationSelection, :BuildResult)
        @test isdefined(StationSelection, :OptResult)
        @test isdefined(StationSelection, :read_candidate_stations)
        @test isdefined(StationSelection, :read_customer_requests)
        @test isdefined(StationSelection, :bd09_to_wgs84)
        @test isdefined(StationSelection, :compute_station_pairwise_costs)
        @test isdefined(StationSelection, :generate_scenarios)
        # New exports for TwoStageSingleDetourModel
        @test isdefined(StationSelection, :TwoStageSingleDetourModel)
        @test isdefined(StationSelection, :StationSelectionData)
        @test isdefined(StationSelection, :ScenarioData)
        @test isdefined(StationSelection, :find_detour_combinations)
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

    @testset "Data Structures" begin
        include("data/test_struct.jl")
    end

    @testset "Detour Combinations" begin
        include("utils/test_detour_combinations.jl")
    end

    @testset "Corridor Clustering" begin
        include("utils/test_corridor_clustering.jl")
    end

    @testset "Model Integration" begin
        include("opt/test_integration.jl")
    end

    @testset "Corridor Integration" begin
        include("opt/test_corridor_integration.jl")
    end

    @testset "Solution Analysis" begin
        include("utils/test_solution_analysis.jl")
    end

    @testset "Export Variables" begin
        include("utils/temp_export_variables.jl")
    end
end
