using Pkg
Pkg.activate("/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl")

using StationSelection
using Test
using Combinatorics
using JuMP

const TESTDIR = "/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/test"

@testset "station-subset + triple cuts" begin
    include(joinpath(TESTDIR, "opt/test_passenger_station_subset_pricing.jl"))
end
