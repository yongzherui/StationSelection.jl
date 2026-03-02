"""
Constraint creation functions for station selection optimization models.

These functions add constraints to JuMP models. They are designed
to be composable - models can pick and choose which constraint sets they need.

Uses multiple dispatch to provide specialized implementations for different
mapping types (TwoStageSingleDetourMap, ClusteringTwoStageODMap,
ClusteringBaseModelMap).

This file includes:
1. Base constraints (station limit, activation limit, linking) - from constraints/base.jl
2. Assignment constraints (assignment, assignment-to-active) - from constraints/assignment.jl
3. Flow constraints - from constraints/flow.jl
4. Detour constraints - from constraints/detour.jl
5. Corridor constraints (α, f_corridor) - from constraints/corridor.jl
6. Route activation constraints (f_flow ≥ x) - from constraints/route_activation.jl
"""

include("constraints/base.jl")
include("constraints/assignment.jl")
include("constraints/flow.jl")
include("constraints/detour.jl")
include("constraints/corridor.jl")
include("constraints/route_activation.jl")
include("constraints/transportation.jl")
