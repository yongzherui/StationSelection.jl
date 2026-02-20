"""
Transportation map for TransportationModel.

Extends corridor clustering with zone-pair anchor data for directional
transportation flow modeling. Each anchor g = (zone_a, zone_b) represents
an ordered pair of zones, capturing movement directionality.
"""

using DataFrames

export TransportationMap
export create_transportation_map

"""
    TransportationMap <: AbstractClusteringMap

Maps scenarios to zone-pair anchors with per-trip pickup/dropoff demand data.

# Fields
## Standard fields (station/scenario mappings)
- `station_id_to_array_idx`: Station ID -> array index
- `array_idx_to_station_id`: Array index -> station ID
- `scenarios`: Reference to scenario data
- `scenario_label_to_array_idx`: Scenario label -> array index
- `array_idx_to_scenario_label`: Array index -> scenario label
- `max_walking_distance`: Walking distance limit (optional)

## Zone clustering data
- `cluster_labels`: Station array index -> cluster label
- `n_clusters`: Number of clusters (zones)
- `cluster_medoids`: Array indices of medoid stations
- `cluster_station_sets`: cluster_id -> station array indices

## Anchor data
- `active_anchors`: List of (zone_a, zone_b) pairs with demand
- `anchor_scenarios`: anchor_idx -> scenarios with demand
- `I_g_pick`: anchor_idx -> scenario -> list of origin station IDs
- `I_g_drop`: anchor_idx -> scenario -> list of destination station IDs
- `m_pick`: anchor_idx -> scenario -> origin_id -> count
- `m_drop`: anchor_idx -> scenario -> dest_id -> count
- `P_g`: anchor_idx -> allowed (j,k) station pairs (j in zone_a, k in zone_b)
- `M_gs`: (anchor_idx, scenario) -> big-M value (total trips in anchor for scenario)
- `w_walk_pick`: (origin_id, j_array_idx) -> walking cost for pickup
- `w_walk_drop`: (k_array_idx, dest_id) -> walking cost for dropoff
"""
struct TransportationMap <: AbstractClusteringMap
    station_id_to_array_idx::Dict{Int, Int}
    array_idx_to_station_id::Vector{Int}

    scenarios::Vector{ScenarioData}
    scenario_label_to_array_idx::Dict{String, Int}
    array_idx_to_scenario_label::Vector{String}

    max_walking_distance::Union{Float64, Nothing}

    # Zone clustering data
    cluster_labels::Vector{Int}
    n_clusters::Int
    cluster_medoids::Vector{Int}
    cluster_station_sets::Vector{Vector{Int}}

    # Anchor data
    active_anchors::Vector{Tuple{Int, Int}}
    anchor_scenarios::Dict{Int, Vector{Int}}
    I_g_pick::Dict{Int, Dict{Int, Vector{Int}}}
    I_g_drop::Dict{Int, Dict{Int, Vector{Int}}}
    m_pick::Dict{Int, Dict{Int, Dict{Int, Int}}}
    m_drop::Dict{Int, Dict{Int, Dict{Int, Int}}}
    P_g::Dict{Int, Vector{Tuple{Int, Int}}}
    M_gs::Dict{Tuple{Int, Int}, Float64}
end


"""
    create_transportation_map(model::TransportationModel,
                              data::StationSelectionData;
                              optimizer_env=nothing) -> TransportationMap

Create a transportation map with zone-pair anchor data.

1. Cluster stations using `cluster_stations_by_diameter`
2. For each scenario, assign each trip to anchor (zone_of_origin, zone_of_dest)
3. Build I_g_pick, I_g_drop, m_pick, m_drop from trip assignments
4. Build P(g) from cluster_station_sets
5. Compute M_gs = total trips in anchor g, scenario s
"""
function create_transportation_map(
        model::TransportationModel,
        data::StationSelectionData;
        optimizer_env=nothing
    )::TransportationMap

    # Create station ID mappings
    station_ids = Vector{Int}(data.stations.id)
    station_id_to_array_idx, array_idx_to_station_id = create_station_id_mappings(station_ids)

    # Create scenario label mappings
    scenario_label_to_array_idx, array_idx_to_scenario_label = create_scenario_label_mappings(data.scenarios)

    max_walking_distance = model.use_walking_distance_limit ? model.max_walking_distance : nothing

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

    # Build cluster station sets
    cluster_station_sets = [Int[] for _ in 1:n_clusters]
    for (i, c) in enumerate(cluster_labels)
        push!(cluster_station_sets[c], i)
    end

    S = length(data.scenarios)

    # For each scenario, assign each trip to an anchor (zone_of_origin, zone_of_dest)
    # and build demand data
    anchor_demand = Dict{Tuple{Int, Int}, Dict{Int, Vector{NamedTuple{(:origin_id, :dest_id), Tuple{Int, Int}}}}}()

    for (s, scenario_data) in enumerate(data.scenarios)
        for row in eachrow(scenario_data.requests)
            o = row.start_station_id
            d = row.end_station_id
            o_idx = station_id_to_array_idx[o]
            d_idx = station_id_to_array_idx[d]
            zone_o = cluster_labels[o_idx]
            zone_d = cluster_labels[d_idx]
            anchor = (zone_o, zone_d)

            if !haskey(anchor_demand, anchor)
                anchor_demand[anchor] = Dict{Int, Vector{NamedTuple{(:origin_id, :dest_id), Tuple{Int, Int}}}}()
            end
            if !haskey(anchor_demand[anchor], s)
                anchor_demand[anchor][s] = NamedTuple{(:origin_id, :dest_id), Tuple{Int, Int}}[]
            end
            push!(anchor_demand[anchor][s], (origin_id=o, dest_id=d))
        end
    end

    # Build active anchors list (sorted for determinism)
    active_anchors = sort(collect(keys(anchor_demand)))
    anchor_idx_lookup = Dict(a => idx for (idx, a) in enumerate(active_anchors))

    # Build anchor data structures
    anchor_scenarios = Dict{Int, Vector{Int}}()
    I_g_pick = Dict{Int, Dict{Int, Vector{Int}}}()
    I_g_drop = Dict{Int, Dict{Int, Vector{Int}}}()
    m_pick = Dict{Int, Dict{Int, Dict{Int, Int}}}()
    m_drop = Dict{Int, Dict{Int, Dict{Int, Int}}}()
    P_g = Dict{Int, Vector{Tuple{Int, Int}}}()
    M_gs = Dict{Tuple{Int, Int}, Float64}()

    for (g_idx, anchor) in enumerate(active_anchors)
        zone_a, zone_b = anchor
        scenarios_with_demand = sort(collect(keys(anchor_demand[anchor])))
        anchor_scenarios[g_idx] = scenarios_with_demand

        I_g_pick[g_idx] = Dict{Int, Vector{Int}}()
        I_g_drop[g_idx] = Dict{Int, Vector{Int}}()
        m_pick[g_idx] = Dict{Int, Dict{Int, Int}}()
        m_drop[g_idx] = Dict{Int, Dict{Int, Int}}()

        for s in scenarios_with_demand
            trips = anchor_demand[anchor][s]

            # Build pickup demand: origin_id -> count
            pick_counts = Dict{Int, Int}()
            drop_counts = Dict{Int, Int}()
            for trip in trips
                pick_counts[trip.origin_id] = get(pick_counts, trip.origin_id, 0) + 1
                drop_counts[trip.dest_id] = get(drop_counts, trip.dest_id, 0) + 1
            end

            I_g_pick[g_idx][s] = sort(collect(keys(pick_counts)))
            I_g_drop[g_idx][s] = sort(collect(keys(drop_counts)))
            m_pick[g_idx][s] = pick_counts
            m_drop[g_idx][s] = drop_counts

            # Big-M: total trips in this anchor/scenario
            M_gs[(g_idx, s)] = Float64(length(trips))
        end

        # P(g): allowed station pairs = {(j,k) : j in C_a, k in C_b}
        stations_a = cluster_station_sets[zone_a]
        stations_b = cluster_station_sets[zone_b]
        P_g[g_idx] = [(j, k) for j in stations_a for k in stations_b]
    end

    return TransportationMap(
        station_id_to_array_idx,
        array_idx_to_station_id,
        data.scenarios,
        scenario_label_to_array_idx,
        array_idx_to_scenario_label,
        max_walking_distance,
        cluster_labels,
        n_clusters,
        medoids,
        cluster_station_sets,
        active_anchors,
        anchor_scenarios,
        I_g_pick,
        I_g_drop,
        m_pick,
        m_drop,
        P_g,
        M_gs
    )
end


"""
    has_walking_distance_limit(mapping::TransportationMap) -> Bool
"""
has_walking_distance_limit(mapping::TransportationMap) = !isnothing(mapping.max_walking_distance)
