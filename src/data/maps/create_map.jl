"""
Model-to-map dispatch helpers.

This file provides a single entry point for creating the appropriate mapping
struct based on the model type.
"""

export create_map

"""
    create_map(problem::StationSelectionProblem, formulation::AbstractClusteringFormulation,
               data::StationSelectionData)

Create the appropriate clustering map for `formulation`, dispatched on its concrete type.
`max_walking_distance` comes from `problem`, not `formulation`.
"""
create_map(
    problem::StationSelectionProblem, formulation::ClusteringBaseFormulation, data::StationSelectionData,
)::ClusteringBaseModelMap = create_clustering_base_model_map(problem, formulation, data)

create_map(
    problem::StationSelectionProblem, formulation::ClusteringTwoStageFormulation, data::StationSelectionData,
)::ClusteringTwoStageStationMap = create_clustering_two_stage_station_map(problem, formulation, data)

create_map(
    problem::StationSelectionProblem, formulation::AbstractClusteringTwoStageODFormulation, data::StationSelectionData,
)::ClusteringTwoStageODMap = create_clustering_two_stage_od_map(problem, formulation, data)

# No create_map(::AggregateODRouteBase|JointRoutingAssignmentFormulation, ...) here:
# their build_model calls create_aggregate_od_route_map(problem, formulation, data)
# directly (it needs both problem and formulation, not this dispatcher's 2-arg shape).
# The AggregateODRouteProblem/RouteCoveringProblem-based methods that used to live here
# were removed along with AggregateODRouteProblem -- see StationSelection.jl's include
# comments.
