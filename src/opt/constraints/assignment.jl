"""
Assignment constraint creation functions for station selection optimization models.

These functions add constraints that:
1. Ensure each request is assigned to exactly one station pair
2. Link assignments to station activation/selection

Uses multiple dispatch for different mapping types.
"""

using JuMP

export add_assignment_constraints!
export add_assignment_to_active_constraints!
export add_assignment_to_selected_constraints!
export add_assignment_walking_limit_constraints!


# ============================================================================
# Assignment Constraints - Each request assigned to exactly one station pair
# ============================================================================

"""
    add_assignment_constraints!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Each OD request must be assigned to exactly one station pair (TwoStageSingleDetourModel).
    Σⱼₖ x[s][t][od][j,k] = 1  ∀(o,d,t) ∈ Ω, s

Used by: TwoStageSingleDetourModel
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                # Works for both sparse (Vector) and dense (Matrix) x
                @constraint(m, sum(x[s][time_id][od]) == 1)
            end
        end
    end

    return _total_num_constraints(m) - before
end




"""
    add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )

Each OD pair must be assigned to exactly one station pair (ClusteringTwoStageODModel).
    Σⱼₖ x[s][od_idx][j,k] = 1  ∀od_idx ∈ Ω_s, s

Used by: ClusteringTwoStageODModel
When `variable_reduction=true` and walking limit is enabled, constraints use sparse x.
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )
    before = _total_num_constraints(m)
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]
    use_sparse = variable_reduction && has_walking_distance_limit(mapping)

    for s in 1:S
        for od_idx in 1:length(mapping.Omega_s[s])
            @constraint(m, sum(x[s][od_idx]) == 1)
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_assignment_constraints!(m::Model, data::StationSelectionData, mapping::ClusteringBaseModelMap)

Each station location must be assigned to exactly one medoid (ClusteringBaseModel).
    Σⱼ x[i,j] = 1  ∀i

Used by: ClusteringBaseModel
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = _total_num_constraints(m)
    n = mapping.n_stations
    x = m[:x]

    @constraint(m, [i=1:n], sum(x[i, j] for j in 1:n) == 1)

    return _total_num_constraints(m) - before
end


# ============================================================================
# Assignment to Active Constraints - Assignments require active stations
# ============================================================================

"""
    add_assignment_to_active_constraints!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Assignment requires both stations to be active (TwoStageSingleDetourModel).
    2 * x[s][t][od][j,k] ≤ z[j,s] + z[k,s]  ∀(o,d,t) ∈ Ω, j, k, s

When walking limits are enabled, only iterates over valid (j,k) pairs from mapping.

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap;
        tight_constraints::Bool=true
    )
    before = _total_num_constraints(m)
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                if use_sparse
                    # Sparse x: iterate over valid (j,k) pairs from mapping
                    valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                    for (idx, (j, k)) in enumerate(valid_pairs)
                        if tight_constraints
                            @constraint(m, x[s][time_id][od][idx] <= z[j, s])
                            @constraint(m, x[s][time_id][od][idx] <= z[k, s])
                        else
                            @constraint(m, 2 * x[s][time_id][od][idx] <= z[j, s] + z[k, s])
                        end
                    end
                else
                    # Dense x: iterate over all (j,k) pairs
                    for j in 1:n, k in 1:n
                        if tight_constraints
                            @constraint(m, x[s][time_id][od][j, k] <= z[j, s])
                            @constraint(m, x[s][time_id][od][j, k] <= z[k, s])
                        else
                            @constraint(m, 2 * x[s][time_id][od][j, k] <= z[j, s] + z[k, s])
                        end
                    end
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end




"""
    add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )

Assignment requires both stations to be active (ClusteringTwoStageODModel).
    2 * x[s][od_idx][j,k] ≤ z[j,s] + z[k,s]  ∀od_idx ∈ Ω_s, j, k, s

Used by: ClusteringTwoStageODModel
When `variable_reduction=true` and walking limit is enabled, constraints use sparse x.
"""
function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true,
        tight_constraints::Bool=true
    )
    before = _total_num_constraints(m)
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]
    use_sparse = variable_reduction && has_walking_distance_limit(mapping)

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            if use_sparse
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (idx, (j, k)) in enumerate(valid_pairs)
                    if tight_constraints
                        @constraint(m, x[s][od_idx][idx] <= z[j, s])
                        @constraint(m, x[s][od_idx][idx] <= z[k, s])
                    else
                        @constraint(m, 2 * x[s][od_idx][idx] <= z[j, s] + z[k, s])
                    end
                end
            else
                for j in 1:n, k in 1:n
                    if tight_constraints
                        @constraint(m, x[s][od_idx][j, k] <= z[j, s])
                        @constraint(m, x[s][od_idx][j, k] <= z[k, s])
                    else
                        @constraint(m, 2 * x[s][od_idx][j, k] <= z[j, s] + z[k, s])
                    end
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


# ============================================================================
# Assignment Walking Limit Constraints - For dense x with walking limit
# ============================================================================

"""
    add_assignment_walking_limit_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap,
        max_walking_distance::Float64
    )

Enforce walking distance limits when dense x variables are used.
For each OD pair (o,d) and station pair (j,k):
    d^origin_{o,j} * x <= max_walking_distance
    d^dest_{k,d} * x <= max_walking_distance

Used by: ClusteringTwoStageODModel when walking limits are enabled and variable reduction is disabled.
"""
function add_assignment_walking_limit_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap,
        max_walking_distance::Float64
    )
    before = _total_num_constraints(m)
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            for j in 1:n, k in 1:n
                j_id = mapping.array_idx_to_station_id[j]
                k_id = mapping.array_idx_to_station_id[k]
                @constraint(m, get_walking_cost(data, o, j_id) * x[s][od_idx][j, k] <= max_walking_distance)
                @constraint(m, get_walking_cost(data, k_id, d) * x[s][od_idx][j, k] <= max_walking_distance)
            end
        end
    end

    return _total_num_constraints(m) - before
end


# ============================================================================
# Assignment to Selected Constraints - For models without scenarios
# ============================================================================

"""
    add_assignment_to_selected_constraints!(m::Model, data::StationSelectionData, mapping::ClusteringBaseModelMap)

Assignment can only be made to selected stations (ClusteringBaseModel).
    x[i,j] ≤ y[j]  ∀i, j

Used by: ClusteringBaseModel
"""
function add_assignment_to_selected_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = _total_num_constraints(m)
    n = mapping.n_stations
    y = m[:y]
    x = m[:x]

    @constraint(m, [i=1:n, j=1:n], x[i, j] <= y[j])

    return _total_num_constraints(m) - before
end
