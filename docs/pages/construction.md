# Model construction

There is one `build_model` method per (problem × formulation × solver) combination. They
compose the [Model building blocks](@ref) — the blocks themselves live outside
`formulations/` and `problems/` precisely so that several formulations can share one row
family without inheriting each other.

The Benders master and subproblem builders are documented with
[BendersSolver](@ref) instead, since they only exist as halves of that algorithm.

## Clustering

{{autodocs opt/optimize.jl opt/optimize/clustering}}

## AggregateODRoute — direct

{{autodocs opt/optimize/aggregate_od_route/direct opt/optimize/aggregate_od_route/base_shared.jl !opt/optimize/aggregate_od_route/direct/build_feasibility.jl}}

## AggregateODRoute — column generation

`integer_recovery_build` lives here too: it is a real `build_model`-shaped rebuild rather
than an in-place mutation, which is what lets it share construction code with
`build_model` instead of duplicating it as post-hoc `set_binary` calls.

{{autodocs opt/optimize/aggregate_od_route/column_generation opt/optimize/aggregate_od_route/joint_shared.jl}}
