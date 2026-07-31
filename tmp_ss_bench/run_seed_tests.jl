using Pkg
Pkg.activate("/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl")

using StationSelection
using Test
using DataFrames
using Dates
using JuMP
const MOI = JuMP.MOI

const TESTDIR = "/home/yongzr/2025-09-JacqWang-Microtransit/StationSelection.jl/test"

@testset "two-stop seeding" begin
    include(joinpath(TESTDIR, "opt/test_passenger_free_assignment_seeding.jl"))
end
