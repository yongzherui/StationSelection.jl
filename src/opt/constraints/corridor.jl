"""
Corridor constraint creation functions for corridor models.

Adds cluster activation and corridor activation constraints:
- z-based (ZCorridorODModel): α and f linked through z
- x-based (XCorridorODModel): f linked directly through x
"""

using JuMP

export add_cluster_activation_constraints!, add_corridor_activation_constraints!
export add_corridor_x_activation_constraints!
export add_zone_activation_limit_constraints!

# =============================================================================
# z-based corridor activation (ZCorridorODModel)
# =============================================================================

"""
    add_cluster_activation_constraints!(m::Model, data::StationSelectionData,
                                        mapping::CorridorTwoStageODMap) -> Int

Cluster activation: α[a,s] ≥ z[i,s]  ∀i∈C_a, ∀a, s

One constraint per cluster member per scenario. Forces α[a,s] = 1 as soon as
any station in cluster a is active, ensuring corridors activate correctly.

Used by: ZCorridorODModel
"""
function add_cluster_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    z = m[:z]
    α = m[:α]

    for a in 1:mapping.n_clusters
        members = mapping.cluster_station_sets[a]
        for s in 1:S
            for i in members
                @constraint(m, α[a, s] >= z[i, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end

"""
    add_corridor_activation_constraints!(m::Model, data::StationSelectionData,
                                         mapping::CorridorTwoStageODMap) -> Int

z-based corridor activation constraints:
- f[g,s] ≥ α[a,s] + α[b,s] - 1  for g=(a,b) where a≠b
- f[g,s] ≥ α[a,s]                for g=(a,a) (self-corridor)

Used by: ZCorridorODModel
"""
function add_corridor_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    α = m[:α]
    f_corridor = m[:f_corridor]

    for (g, (a, b)) in enumerate(mapping.corridor_indices)
        for s in 1:S
            if a == b
                @constraint(m, f_corridor[g, s] >= α[a, s])
            else
                @constraint(m, f_corridor[g, s] >= α[a, s] + α[b, s] - 1)
            end
        end
    end

    return _total_num_constraints(m) - before
end


# =============================================================================
# x-based corridor activation (XCorridorODModel)
# =============================================================================

"""
    add_corridor_x_activation_constraints!(m::Model, data::StationSelectionData,
                                           mapping::CorridorTwoStageODMap;
                                           variable_reduction::Bool=true) -> Int

x-based corridor activation constraints:
    f_{gs} ≥ x_{odjks}  ∀(o,d), j∈C_a, k∈C_b, s  for g=(a,b)

A corridor is activated only when an actual OD assignment crosses it.

Used by: XCorridorODModel
"""
function add_corridor_x_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap;
        variable_reduction::Bool=true
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]
    f_corridor = m[:f_corridor]
    use_sparse = variable_reduction && has_walking_distance_limit(mapping)

    # Build corridor lookup: (cluster_a, cluster_b) → corridor index g
    corridor_lookup = Dict{Tuple{Int,Int}, Int}()
    for (g, (a, b)) in enumerate(mapping.corridor_indices)
        corridor_lookup[(a, b)] = g
    end

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            if use_sparse
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (idx, (j, k)) in enumerate(valid_pairs)
                    cluster_j = mapping.cluster_labels[j]
                    cluster_k = mapping.cluster_labels[k]
                    g = corridor_lookup[(cluster_j, cluster_k)]
                    @constraint(m, f_corridor[g, s] >= x[s][od_idx][idx])
                end
            else
                n = data.n_stations
                for j in 1:n, k in 1:n
                    cluster_j = mapping.cluster_labels[j]
                    cluster_k = mapping.cluster_labels[k]
                    g = corridor_lookup[(cluster_j, cluster_k)]
                    @constraint(m, f_corridor[g, s] >= x[s][od_idx][j, k])
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end

# =============================================================================
# Per-zone activation limit (XCorridorODModel)
# =============================================================================

"""
    add_zone_activation_limit_constraints!(m, data, mapping, max_active) -> Int

Upper-bound the number of active stations per cluster per scenario:

    Σ_{j ∈ C_a} z[j,s] ≤ max_active   ∀a ∈ 1:n_clusters, ∀s

Prevents all k active stations from concentrating in a single cluster.
Feasibility requires: max_active * n_clusters >= k.

Used by: XCorridorODModel (when max_active_per_zone is set)
"""
function add_zone_activation_limit_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap,
        max_active::Int
    )::Int
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    z = m[:z]

    @constraint(m, zone_activation_limit[a in 1:mapping.n_clusters, s in 1:S],
        sum(z[j, s] for j in mapping.cluster_station_sets[a]) <= max_active
    )

    return _total_num_constraints(m) - before
end

