# Formulations

An [`AbstractFormulation`](@ref) is the mathematical encoding: which variables exist,
which rows tie them together, and what the objective weighs.



## Two families

The **Clustering** family keeps a two-stage split — `y` (build) and `z` (activate per
scenario) — and is solved by [`DirectMIPSolver`](@ref).

The **AggregateODRoute** family has **no `z` and no `l`**: every built station is usable in
every scenario. Both of its formulations always expose direct walking (`x_walk`,
`WALK_ONLY_PAIR`) as a station-free coverage option, and both validate build-time
feasibility with [`aggregate_od_route_validate_feasible_coverage`](@ref).

## Clustering

{{autodocs opt/formulations/clustering.jl}}

## AggregateODRoute

{{autodocs opt/formulations/aggregate_od_route/base.jl opt/formulations/aggregate_od_route/feasibility.jl}}

### Joint routing/assignment

`θ` columns carry the OD assignment directly, so there is no separate `x`. The pool is
grown by column generation instead of enumerated up front.

The two **derived** types below are the Benders decomposition of the monolith. They are
derived from the parent and never built from loose keywords: a master at one
`detour_factor` against a subproblem at another yields invalid cuts and a confidently
wrong `OPTIMAL`, so the inconsistent combination is made unrepresentable.

{{autodocs opt/formulations/aggregate_od_route/joint_routing_assignment}}

### Cut aggregation

{{autodocs opt/formulations/aggregate_od_route/benders}}
