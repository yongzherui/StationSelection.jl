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
include("objectives/aggregate_od_route/core.jl")
include("objectives/aggregate_od_route/column_generation/master.jl")
include("objectives/expressions/aggregate_od_route/benders.jl")
include("objectives/aggregate_od_route/benders/subproblem.jl")
include("objectives/aggregate_od_route/benders/master.jl")
include("objectives/aggregate_od_route/benders/dual_lp.jl")
