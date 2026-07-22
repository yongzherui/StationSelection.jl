"""
Model-to-map dispatch helpers.

This file provides a single entry point for creating the appropriate mapping
struct based on the model type.
"""

export create_map

"""
    create_map(model::ClusteringModel, data::StationSelectionData)

Create the appropriate clustering map for `model`, dispatched on `model.policy`.
"""
function create_map(
        model::ClusteringModel,
        data::StationSelectionData
    )
    return _create_clustering_map(model.policy, data)
end

_create_clustering_map(policy::SingleStagePolicy, data::StationSelectionData)::ClusteringBaseModelMap =
    create_clustering_base_model_map(policy, data)

_create_clustering_map(policy::TwoStagePolicy, data::StationSelectionData)::ClusteringTwoStageStationMap =
    create_clustering_two_stage_station_map(policy, data)

_create_clustering_map(policy::TwoStageODPolicy, data::StationSelectionData)::ClusteringTwoStageODMap =
    create_clustering_two_stage_od_map(policy, data)

"""
    create_map(model::ExactDARPRouteModel, data::StationSelectionData)

Create an ExactDARPRouteODMap for ExactDARPRouteModel.
"""
function create_map(
        model::ExactDARPRouteModel,
        data::StationSelectionData
    )::ExactDARPRouteODMap
    return create_exact_darp_route_od_map(model, data)
end

function create_map(
        model::AggregateODRouteModel,
        data::StationSelectionData
    )::AggregateODRouteMap
    return create_aggregate_od_route_map(model, data)
end

function create_map(
        model::RouteCoveringProblem,
        data::StationSelectionData
    )::AggregateODRouteMap
    return create_aggregate_od_route_map(model, data)
end
