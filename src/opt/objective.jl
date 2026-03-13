"""
Objective functions for station selection optimization models.

Contains objective functions for:
- ClusteringTwoStageODModel: walking + routing costs (no pooling)
- ClusteringBaseModel: simple walking cost minimization (k-medoids style)

Uses multiple dispatch for different model/mapping types.
"""

# Model-specific objectives
include("objectives/clustering_od.jl")
include("objectives/clustering_base.jl")
