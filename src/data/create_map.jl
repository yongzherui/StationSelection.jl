"""
Model-to-map dispatch helpers.

This file provides a single entry point for creating the appropriate mapping
struct based on the model type.
"""

export create_map

"""
    create_map(model::TwoStageSingleDetourModel, data::StationSelectionData; Xi_same_source=[], Xi_same_dest=[])

Create a TwoStageSingleDetourMap for TwoStageSingleDetourModel.
"""
function create_map(
        model::TwoStageSingleDetourModel,
        data::StationSelectionData;
        Xi_same_source::Vector{Tuple{Int, Int, Int}}=Tuple{Int, Int, Int}[],
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}=Tuple{Int, Int, Int, Int}[]
    )::TwoStageSingleDetourMap
    return create_two_stage_single_detour_map(
        model,
        data;
        Xi_same_source=Xi_same_source,
        Xi_same_dest=Xi_same_dest
    )
end

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
    create_map(model::AbstractCorridorODModel, data::StationSelectionData; optimizer_env=nothing)

Create a CorridorTwoStageODMap for any corridor model (ZCorridorODModel, XCorridorODModel).
"""
function create_map(
        model::AbstractCorridorODModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::CorridorTwoStageODMap
    return create_corridor_two_stage_od_map(model, data; optimizer_env=optimizer_env)
end

"""
    create_map(model::TransportationModel, data::StationSelectionData; optimizer_env=nothing)

Create a TransportationMap for TransportationModel.
"""
function create_map(
        model::TransportationModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::TransportationMap
    return create_transportation_map(model, data; optimizer_env=optimizer_env)
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
