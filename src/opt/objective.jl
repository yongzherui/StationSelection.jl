"""
Objective functions for station selection optimization models.

Contains objective functions for:
- TwoStageSingleDetourModel: walking + routing costs with pooling savings
- TwoStageSingleDetourNoWalkingLimitModel: same objective, different mapping
- ClusteringTwoStageODModel: walking + routing costs (no pooling)
- ClusteringBaseModel: simple walking cost minimization (k-medoids style)

Uses multiple dispatch for different model/mapping types.

This file includes:
1. Expression components (reusable building blocks):
   - assignment_cost.jl: walking + routing costs per assignment
   - flow_cost.jl: flow routing costs
   - pooling_savings.jl: same-source and same-dest pooling savings
2. Model-specific objectives that compose expressions:
   - single_detour.jl: TwoStageSingleDetourModel objectives
   - clustering_od.jl: ClusteringTwoStageODModel objective
   - clustering_base.jl: ClusteringBaseModel objective
"""

# Expression components (reusable building blocks)
include("objectives/expressions/assignment_cost.jl")
include("objectives/expressions/flow_cost.jl")
include("objectives/expressions/pooling_savings.jl")

# Model-specific objectives
include("objectives/single_detour.jl")
include("objectives/clustering_od.jl")
include("objectives/clustering_base.jl")
