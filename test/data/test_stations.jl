# Test the read_candidate_stations function
using Test
using DataFrames
using StationSelection: read_candidate_stations, bd09_to_wgs84

@testset "read_candidate_stations" begin
    @testset "Basic functionality" begin
        # Create a temporary CSV file with test data
        temp_file = tempname() * ".csv"
        open(temp_file, "w") do io
            write(io, "station_id,station_name,station_lon,station_lat\n")
            write(io, "9,学府港湾-侧门,113.16900071992336,27.91697197316557\n")
            write(io, "11,铁路科技职院-侧门,113.16498150676756,27.90703026244468\n")
        end

        # Read the candidate stations
        df = read_candidate_stations(temp_file)

        # Check the structure of the DataFrame
        @test isa(df, DataFrame)
        @test issubset(["id", "lat", "lon"], names(df))
        @test nrow(df) == 2

        # Check if the values are converted correctly (approximate values)
        # The values need to be changed to consider the bd09_to_wgs84 transformation

        lon1, lat1 = 113.16900071992336, 27.91697197316557
        lon2, lat2 = 113.16498150676756, 27.90703026244468
        wgs_lon1, wgs_lat1 = bd09_to_wgs84(lon1, lat1)
        wgs_lon2, wgs_lat2 = bd09_to_wgs84(lon2, lat2)

        @test isapprox(df[1, :lon], wgs_lon1, atol=1e-5)
        @test isapprox(df[1, :lat], wgs_lat1, atol=1e-5)
        @test isapprox(df[2, :lon], wgs_lon2, atol=1e-5)
        @test isapprox(df[2, :lat], wgs_lat2, atol=1e-5)

        # Clean up the temporary file
        rm(temp_file)
    end
end
