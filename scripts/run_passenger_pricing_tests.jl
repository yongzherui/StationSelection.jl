"""
    scripts/run_passenger_pricing_tests.jl

Runs only the passenger free-assignment pricing unit tests. The full
`test/runtests.jl` builds and solves Gurobi models for every model family, which
is far more than is needed when iterating on the label search -- this keeps the
correctness feedback loop to seconds.
"""

using Test
using StationSelection
using Random

const TEST_DIR = normpath(joinpath(@__DIR__, "..", "test"))

@testset "passenger pricing (focused)" begin
    include(joinpath(TEST_DIR, "opt", "test_passenger_free_assignment_pricing.jl"))
end
