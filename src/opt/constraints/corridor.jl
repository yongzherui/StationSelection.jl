"""
Corridor constraint creation functions for corridor models.

Adds cluster activation and corridor activation constraints:
- z-based (ZCorridorODModel): α and f linked through z
- x-based (XCorridorODModel): f linked directly through x
"""

using JuMP

export add_cluster_activation_constraints!, add_corridor_activation_constraints!
export add_corridor_x_activation_constraints!

# =============================================================================
# z-based corridor activation (ZCorridorODModel)
# =============================================================================

"""
    add_cluster_activation_constraints!(m::Model, data::StationSelectionData,
                                        mapping::CorridorTwoStageODMap) -> Int

Cluster activation: |C_a| · α[a,s] ≥ Σ_{i∈C_a} z[i,s]  ∀a, s

This ensures α[a,s] ≥ 1/|C_a| whenever any station in cluster a is active,
which due to z integrality means α[a,s] = 1 when any station is active.

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
        cluster_size = length(members)
        for s in 1:S
            @constraint(m,
                cluster_size * α[a, s] >= sum(z[i, s] for i in members)
            )
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
