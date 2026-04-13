"""
Constraint creation functions for station selection optimization models.

These functions add constraints to JuMP models. They are designed
to be composable - models can pick and choose which constraint sets they need.

Uses multiple dispatch to provide specialized implementations for different
mapping types (ClusteringTwoStageODMap, ClusteringBaseModelMap).

This file includes:
1. Base constraints (station limit, activation limit, linking) - from constraints/base.jl
2. Assignment constraints (assignment, assignment-to-active) - from constraints/assignment.jl
3. Flow activation constraints (f_flow ≥ x) - from constraints/flow_activation.jl
"""

include("constraints/base.jl")
include("constraints/assignment.jl")
include("constraints/flow_activation.jl")
include("constraints/route_capacity.jl")
include("constraints/route_fleet_limit.jl")
