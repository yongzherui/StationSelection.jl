"""
Model-to-map dispatch helpers.

This file provides a single entry point for creating the appropriate mapping
struct based on the model type.
"""

export create_map

"""
    create_map(model::ClusteringTwoStageODModel, data::StationSelectionData)

Create a ClusteringTwoStageODMap for ClusteringTwoStageODModel.
"""
function create_map(
        model::ClusteringTwoStageODModel,
        data::StationSelectionData
    )::ClusteringTwoStageODMap
    return create_clustering_two_stage_od_map(model, data)
end

"""
    create_map(model::ClusteringTwoStageStationModel, data::StationSelectionData)

Create a ClusteringTwoStageStationMap for ClusteringTwoStageStationModel.
"""
function create_map(
        model::ClusteringTwoStageStationModel,
        data::StationSelectionData
    )::ClusteringTwoStageStationMap
    return create_clustering_two_stage_station_map(model, data)
end

"""
    create_map(model::ClusteringBaseModel, data::StationSelectionData)

Create a ClusteringBaseModelMap for ClusteringBaseModel.
"""
function create_map(
        model::ClusteringBaseModel,
        data::StationSelectionData
    )::ClusteringBaseModelMap
    return create_clustering_base_model_map(model, data)
end

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
