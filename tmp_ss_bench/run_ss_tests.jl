using Pkg
Pkg.activate("/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl")

using StationSelection
using Test
using Random

const TESTDIR = "/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/test"

@testset "station-simple verification" begin
    include(joinpath(TESTDIR, "opt/test_passenger_free_assignment_pricing.jl"))
    include(joinpath(TESTDIR, "opt/test_passenger_free_assignment_station_simple_pricing.jl"))
end
