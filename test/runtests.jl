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
        @test isdefined(StationSelection, :StationSelectionData)
        @test isdefined(StationSelection, :ScenarioData)
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
        include("utils/test_exact_darp_route_runner.jl")
        include("utils/test_exact_darp_route_column_generation.jl")
        include("utils/test_iterative_route_generation.jl")
        include("utils/test_alpha_profile_enrichment.jl")
        include("utils/test_generators.jl")
        include("utils/test_case_generators/test_base_middle_zone.jl")
        include("utils/test_case_generators/test_test1_vehicle.jl")
        include("utils/test_case_generators/test_test2_zone_proximity.jl")
        include("utils/test_case_generators/test_test3_north_shift.jl")
        include("utils/test_case_generators/test_test4_mirrored_zone.jl")
        include("utils/test_case_generators/test_test5_triangle.jl")
        include("utils/test_case_generators/test_test6_bidirectional.jl")
        include("utils/test_export_variables.jl")
    end

    @testset "Data Structures" begin
        include("data/test_struct.jl")
    end

    @testset "Model Integration" begin
        include("opt/test_integration.jl")
        include("opt/test_aggregate_od_route_pricing.jl")
        include("opt/test_aggregate_od_route_heuristic_enumeration.jl")
        include("opt/test_aggregate_od_route_nearest_open_alignment.jl")
        include("opt/test_aggregate_od_route_direct_walking.jl")
        include("opt/test_aggregate_od_route_restricted_mw_cut.jl")
    end

end
