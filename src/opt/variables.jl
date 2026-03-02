"""
Variable creation functions for station selection optimization models.

These functions add decision variables to JuMP models. They are designed
to be composable - models can pick and choose which variable sets they need.

Uses multiple dispatch to provide specialized implementations for different
mapping types (TwoStageSingleDetourMap, ClusteringTwoStageODMap,
ClusteringBaseModelMap).

This file includes:
1. Base variables (y, z) - from variables/base.jl
2. Assignment variables (x) - from variables/assignment.jl
3. Flow variables (f) - from variables/flow.jl
4. Detour variables (u, v) - from variables/detour.jl
5. Corridor variables (α, f_corridor) - from variables/corridor.jl
6. Route activation variables (w_route) - from variables/route_activation.jl
"""

include("variables/base.jl")
include("variables/assignment.jl")
include("variables/flow.jl")
include("variables/detour.jl")
include("variables/corridor.jl")
include("variables/route_activation.jl")
include("variables/transportation.jl")
