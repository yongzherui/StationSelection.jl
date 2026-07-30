"""
    scripts/run_lifted_routing_lower_bound_test.jl

Runs just `test/opt/test_aggregate_od_route_lifted_routing_lower_bound.jl` (the new
`BendersSolver(lifted_routing_lower_bound=true)` correctness test) rather than the full test
suite, for a fast sbatch-submitted verification pass while developing the feature. See
`.claude/plans/ticklish-herding-honey.md` for the feature design.

Usage:
    julia --project=. scripts/run_lifted_routing_lower_bound_test.jl
"""

using Test
using StationSelection
using DataFrames
using Dates
using JuMP
const MOI = JuMP.MOI

@testset "lifted_routing_lower_bound standalone run" begin
    include(joinpath(@__DIR__, "..", "test", "opt", "test_aggregate_od_route_lifted_routing_lower_bound.jl"))
end
