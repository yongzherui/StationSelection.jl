"""
Objective functions for station selection optimization models.

Contains objective functions for:
- TwoStageODPolicy: walking + routing costs (no pooling)
- SingleStagePolicy: simple walking cost minimization (k-medoids style)

Uses multiple dispatch for different model/mapping types.
"""

# Model-specific objectives
include("objectives/clustering_od.jl")
include("objectives/clustering_two_stage_station.jl")
include("objectives/clustering_base.jl")
# aggregate_od_route/core.jl (set_aggregate_od_route_objective!, the old combined-model
# objective, pre-Base/Joint split) was removed -- zero callers, fully superseded by
# set_aggregate_od_route_base_objective!/set_joint_routing_assignment_objective! below.
include("objectives/aggregate_od_route/joint_routing_assignment/unserved_penalty.jl")
include("objectives/aggregate_od_route/joint_routing_assignment/assembly.jl")
include("objectives/aggregate_od_route/base/assembly.jl")
# The old nearest-open-assignment-policy Benders objective files (expressions/
# aggregate_od_route/benders.jl, aggregate_od_route/benders/{subproblem,master,dual_lp}.jl)
# were removed -- see opt/formulations/aggregate_od_route/benders/*.jl for the surviving
# Formulation marker structs.
