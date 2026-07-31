"""
    scripts/run_passenger_mcf_relaxation_tests.jl

Runs only the passenger MCF relaxation unit tests. The full `test/runtests.jl`
pulls in every model in the package; this is the focused loop to use while
iterating on `pricing/passenger/mcf_relaxation.jl`.

Run it through `scripts/sbatch_run_passenger_mcf_relaxation_tests.sh` rather than
interactively -- these tests build Gurobi models.
"""

using Test
using StationSelection
using JuMP
const MOI = JuMP.MOI

const TEST_DIR = joinpath(@__DIR__, "..", "test")

@testset "passenger MCF relaxation (focused)" begin
    include(joinpath(TEST_DIR, "opt", "test_passenger_mcf_relaxation.jl"))
end
