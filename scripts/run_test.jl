"""
    scripts/run_test.jl

Run one `test/` file in isolation, skipping the full `test/runtests.jl` suite
(which builds and solves Gurobi models for every model family). This is the
fast correctness loop to use while iterating on one component -- replaces the
one-off run_<feature>_test[s].jl scripts this repo used to grow one of per
focused test file.

Usage:
    julia --project=. scripts/run_test.jl <path-relative-to-test/> [more paths...]

Examples:
    julia --project=. scripts/run_test.jl opt/test_passenger_free_assignment_pricing.jl
    julia --project=. scripts/run_test.jl opt/test_passenger_mcf_relaxation.jl
    julia --project=. scripts/run_test.jl opt/test_aggregate_od_route_lifted_routing_lower_bound.jl
"""

using Test, StationSelection, Random, CSV, DataFrames, Dates, JuMP
const MOI = JuMP.MOI

function main()
    isempty(ARGS) && error("usage: run_test.jl <path-relative-to-test/> [more paths...]")
    test_dir = normpath(joinpath(@__DIR__, "..", "test"))
    for rel in ARGS
        path = joinpath(test_dir, rel)
        isfile(path) || error("no such test file: $path")
        @testset "$rel" begin
            include(path)
        end
    end
end

main()
