"""
Corridor OD mapping for corridor models (ZCorridorODModel, XCorridorODModel).

Extends the ClusteringTwoStageODMap pattern with corridor clustering data.
"""

using DataFrames

export CorridorTwoStageODMap
export create_corridor_two_stage_od_map

"""
    CorridorTwoStageODMap <: AbstractClusteringMap

Maps scenarios to OD pairs with corridor clustering data.

# Fields
- Station/scenario mappings (same as ClusteringTwoStageODMap)
- `Omega_s`: scenario → OD pairs
- `Q_s`: scenario → OD pair → demand count
- Walking distance data
- `cluster_labels::Vector{Int}`: station array index → cluster label
- `n_clusters::Int`: number of clusters
- `cluster_medoids::Vector{Int}`: array indices of medoid stations
- `corridor_indices::Vector{Tuple{Int,Int}}`: g → (cluster_a, cluster_b)
- `cluster_station_sets::Vector{Vector{Int}}`: cluster_id → station array indices
- `corridor_costs::Vector{Float64}`: r_g routing distance between medoids
"""
struct CorridorTwoStageODMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    Omega_s::Dict{Int, Vector{Tuple{Int, Int}}}
    Q_s::Dict{Int, Dict{Tuple{Int, Int}, Int}}

    max_walking_distance::Union{Float64, Nothing}
    valid_jk_pairs::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}

    # Corridor clustering data
    cluster_labels::Vector{Int}
    n_clusters::Int
    cluster_medoids::Vector{Int}
    corridor_indices::Vector{Tuple{Int, Int}}
    cluster_station_sets::Vector{Vector{Int}}
    corridor_costs::Vector{Float64}
end


"""
    create_corridor_two_stage_od_map(model::AbstractCorridorODModel,
                                     data::StationSelectionData;
                                     optimizer_env=nothing) -> CorridorTwoStageODMap

Create a corridor scenario OD map with clustering and corridor data.
Works for both ZCorridorODModel and XCorridorODModel.
"""
function create_corridor_two_stage_od_map(
        model::AbstractCorridorODModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::CorridorTwoStageODMap

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    # Compute Omega_s and Q_s for all scenarios
    Omega_s = Dict{Int, Vector{Tuple{Int, Int}}}()
    Q_s = Dict{Int, Dict{Tuple{Int, Int}, Int}}()
    all_od_pairs = Set{Tuple{Int, Int}}()

    for (scenario_id, scenario_data) in enumerate(data.scenarios)
        od_count = compute_scenario_od_count(scenario_data)
        Omega_s[scenario_id] = collect(keys(od_count))
        Q_s[scenario_id] = od_count
        union!(all_od_pairs, Omega_s[scenario_id])
    end

    max_walking_distance = model.use_walking_distance_limit ? model.max_walking_distance : nothing
    valid_jk_pairs = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    if model.use_walking_distance_limit
        valid_jk_pairs = compute_valid_jk_pairs(
            all_od_pairs,
            data,
            station_id_to_array_idx,
            array_idx_to_station_id,
            model.max_walking_distance
        )
    end

    # Compute corridor clustering
    if !isnothing(model.n_clusters)
        cluster_labels, medoids, n_clusters = cluster_stations_by_count(
            data, array_idx_to_station_id, model.n_clusters;
            optimizer_env=optimizer_env)
    else
        cluster_labels, medoids, n_clusters = cluster_stations_by_diameter(
            data, array_idx_to_station_id, model.max_cluster_diameter;
            optimizer_env=optimizer_env)
    end

    corridor_indices, cluster_station_sets, corridor_costs = compute_corridor_data(
        cluster_labels, medoids, n_clusters, data, array_idx_to_station_id
    )

    return CorridorTwoStageODMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        Omega_s,
        Q_s,
        max_walking_distance,
        valid_jk_pairs,
        cluster_labels,
        n_clusters,
        medoids,
        corridor_indices,
        cluster_station_sets,
        corridor_costs
    )
end

"""
    has_walking_distance_limit(mapping::CorridorTwoStageODMap) -> Bool
"""
has_walking_distance_limit(mapping::CorridorTwoStageODMap) = !isnothing(mapping.max_walking_distance)

"""
    get_valid_jk_pairs(mapping::CorridorTwoStageODMap, o::Int, d::Int) -> Vector{Tuple{Int, Int}}
"""
function get_valid_jk_pairs(mapping::CorridorTwoStageODMap, o::Int, d::Int)
    if has_walking_distance_limit(mapping)
        return get(mapping.valid_jk_pairs, (o, d), Tuple{Int, Int}[])
    else
        n = length(mapping.array_idx_to_station_id)
        return [(j, k) for j in 1:n for k in 1:n]
    end
end
