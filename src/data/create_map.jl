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
    create_map(model::RouteVehicleCapacityModel, data::StationSelectionData)

Create a VehicleCapacityODMap for RouteVehicleCapacityModel.
"""
function create_map(
        model::RouteVehicleCapacityModel,
        data::StationSelectionData
    )::VehicleCapacityODMap
    return create_vehicle_capacity_od_map(model, data)
end

"""
    create_map(model::AlphaRouteModel, data::StationSelectionData)

Create an AlphaRouteODMap for AlphaRouteModel.
"""
function create_map(
        model::AlphaRouteModel,
        data::StationSelectionData
    )::AlphaRouteODMap
    return create_alpha_route_od_map(model, data)
end
