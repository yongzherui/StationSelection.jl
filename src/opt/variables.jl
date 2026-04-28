"""
Variable creation functions for station selection optimization models.

These functions add decision variables to JuMP models. They are designed
to be composable - models can pick and choose which variable sets they need.

Uses multiple dispatch to provide specialized implementations for different
mapping types (ClusteringTwoStageODMap, ClusteringBaseModelMap).

This file includes:
1. Base variables (y, z) - from variables/base.jl
2. Assignment variables (x) - from variables/assignment.jl
3. Flow variables (f_flow) - from variables/flow.jl
"""

include("variables/base.jl")
include("variables/assignment.jl")
include("variables/robust.jl")
include("variables/flow.jl")
include("variables/route_theta.jl")
include("variables/v_jkts.jl")
