"""
    scripts/diagnose.jl

Single entry point for this repo's ad hoc AggregateODRouteModel / passenger
free-assignment diagnostics.

WHY THIS FILE EXISTS: this repo used to grow one new `scripts/diag_*.jl` (plus
a paired `sbatch_diag_*.sh`) for every new investigation question, which
eventually reached well over 100 such one-off files -- nearly all of them
mostly boilerplate (load a Zhuzhou instance, build the standard
AggregateODRouteModel, parse a dozen env vars, run a search, print/CSV the
result) around a small kernel of actually-novel logic. When an investigation
concluded, its scripts had no reason to be kept around individually.

To add a new diagnostic: put shared boilerplate in
`scripts/lib/diagnostics_common.jl` (instance/model builders, env-var parsing,
the Gurobi env, the seeded-dual-snapshot helper, ...), write the novel kernel
as `scripts/modes/<name>.jl` defining `run_<name>(args::Vector{String})` and
ending with `register_mode!("<name>", run_<name>)`, then add the filename to
the `include` list below. One generic `scripts/sbatch_diagnose.sh` can launch
any registered mode on the cluster.

Usage:
    julia --project=. scripts/diagnose.jl <mode> [args...]
    julia --project=. scripts/diagnose.jl --list

Each mode's own [args...] / env vars are documented in its
scripts/modes/<name>.jl docstring.
"""

using CSV, DataFrames, StationSelection

include(joinpath(@__DIR__, "lib", "diagnostics_common.jl"))

const MODES = Dict{String, Function}()

function register_mode!(name::AbstractString, f::Function)
    haskey(MODES, name) && error("mode '$name' already registered")
    MODES[name] = f
    return nothing
end

for file in (
    "dominance_audit.jl",
    "ncandidates_sensitivity.jl",
    "split_census.jl",
    "station_subset_pricing.jl",
    "cg_iteration_profile.jl",
    "station_cluster.jl",
    "reward_ladder_census.jl",
    "clock_quantization.jl",
    "lagrangian_gap.jl",
)
    include(joinpath(@__DIR__, "modes", file))
end

function main()
    if isempty(ARGS) || ARGS[1] in ("--list", "-l", "--help", "-h")
        println("Available modes:")
        for name in sort(collect(keys(MODES)))
            println("  $name")
        end
        println()
        println("Usage: julia --project=. scripts/diagnose.jl <mode> [args...]")
        println("See scripts/modes/<mode>.jl for that mode's argument/env-var docs.")
        return
    end
    mode = ARGS[1]
    haskey(MODES, mode) || error(
        "unknown mode '$mode'. Run `julia --project=. scripts/diagnose.jl --list` " *
        "to see available modes.",
    )
    MODES[mode](ARGS[2:end])
end

main()
