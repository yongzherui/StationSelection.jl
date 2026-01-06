using StationSelection
# Test station cost calculations
using Test
using DataFrames

@testset "Test StationCosts" begin

    @testset "Basic functionality" begin
        # Create a simple DataFrame with candidate stations
        df = DataFrame(id=[4,12,16], lat=[27.9, 27.91, 27.92], lon=[113.1, 113.11, 113.12])

        # Compute the pairwise costs
        costs = compute_station_pairwise_costs(df)

        # Check the size of the cost matrix
        @test length(costs) == 9

        # Check that the diagonal is zero (cost from a station to itself)
        @test all(costs[(i, i)] == 0.0 for i in [4, 12, 16])

        # Check that costs are symmetric (cost from i to j equals cost from j to i)
        @test all(costs[(i, j)] == costs[(j, i)] for i in [4, 12, 16] for j in [4, 12, 16] if i != j)

        # Check that costs are positive for different stations
        @test all(costs[(i, j)] >= 0.0 for i in [4, 12, 16] for j in [4, 12, 16] if i != j)
    end
end