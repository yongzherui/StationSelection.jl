using Random

include(joinpath(@__DIR__, "..", "..", "scripts", "generate_zhuzhou_instance.jl"))

@testset "Zhuzhou benchmark weighted sampling contract" begin
    items = [(1, 2), (1, 3), (2, 3), (3, 1)]
    weights = [4.0, 3.0, 2.0, 1.0]

    sample3 = _zz_weighted_sample_without_replacement(
        MersenneTwister(42), items, weights, 3,
    )
    sample4 = _zz_weighted_sample_without_replacement(
        MersenneTwister(42), items, weights, 4,
    )

    @test length(sample3) == 3
    @test length(unique(sample3)) == 3
    @test sample3 == sample4[1:3]
    @test_throws ArgumentError _zz_weighted_sample_without_replacement(
        MersenneTwister(42), items, weights, 5,
    )
    @test_throws ArgumentError _zz_weighted_sample_without_replacement(
        MersenneTwister(42), items, [1.0, 1.0, 0.0, 1.0], 2,
    )
end
