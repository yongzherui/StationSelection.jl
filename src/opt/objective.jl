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
include("objectives/route_od.jl")
include("objectives/aggregate_od_route.jl")
