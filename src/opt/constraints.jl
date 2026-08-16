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
include("constraints/aggregate_od_route/core.jl")
include("constraints/aggregate_od_route/joint_routing_assignment/coverage.jl")
include("constraints/aggregate_od_route/joint_routing_assignment/linking.jl")
include("constraints/aggregate_od_route/joint_routing_assignment/routing_and_assignment.jl")
include("constraints/aggregate_od_route/base/coverage.jl")
include("constraints/aggregate_od_route/base/linking.jl")
include("constraints/aggregate_od_route/base/route_activation.jl")
# The old nearest-open-assignment-policy Benders machinery (constraints/aggregate_od_route/
# benders/*.jl, constraints/aggregate_od_route/nearest_open/*.jl) was removed -- see
# opt/formulations/aggregate_od_route/benders/*.jl for the surviving Formulation marker
# structs (kept as a reminder of the decompositions still to build against the
# Problem/Formulation/Solver architecture) and opt/problems/route_covering.jl
# (RouteCoveringProblem, likewise kept).
