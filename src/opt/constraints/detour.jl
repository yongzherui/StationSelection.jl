"""
Detour constraint creation functions for station selection optimization models.

These functions add constraints for same-source and same-destination pooling,
linking detour pooling variables to assignment variables.

Used by: TwoStageSingleDetourModel, TwoStageSingleDetourNoWalkingLimitModel
"""

using JuMP

export add_assignment_to_same_source_detour_constraints!
export add_assignment_to_same_dest_detour_constraints!


# ============================================================================
# Helper functions for sparse assignment variables
# ============================================================================

"""
    get_x_var_for_edge(
        x_od::Vector{VariableRef},
        valid_pairs::Vector{Tuple{Int, Int}},
        j::Int,
        k::Int
    ) -> Union{VariableRef, Nothing}

Get the assignment variable for edge (j,k) from sparse x storage.
Returns nothing if the edge doesn't exist for this OD pair.
"""
function get_x_var_for_edge(
    x_od::Vector{VariableRef},
    valid_pairs::Vector{Tuple{Int, Int}},
    j::Int,
    k::Int
)::Union{VariableRef, Nothing}
    for (idx, (jj, kk)) in enumerate(valid_pairs)
        if jj == j && kk == k
            return x_od[idx]
        end
    end
    return nothing
end


# ============================================================================
# Same-Source Detour Constraints
# ============================================================================

"""
    add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}}
    )

Same-source pooling constraints for triplet (j, k, l):
- Pools trip (j→l) with trip (k→l) - both end at l
- Vehicle path: j → k → l

For each feasible (j,k,l) ∈ Ξ, if pooling is enabled (u=1), we need assignments:
    Σ_{od with edge (j,k)} x_{od,t,jk,s} ≥ u_{t,idx,s}
    Σ_{od with edge (j,l)} x_{od,t,jl,s} ≥ u_{t,idx,s}

When walking limits are enabled, the sums are over OD pairs that actually have
the required edges available.

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}}
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    u = m[:u]
    x = m[:x]
    n_same_source = length(Xi_same_source)

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            if length(od_vector) <= 1
                continue
            end

            # Get the feasible detour indices for this (s, t)
            if use_sparse
                feasible_indices = get(mapping.feasible_same_source[s], time_id, Int[])
            else
                feasible_indices = collect(1:n_same_source)
            end

            if isempty(feasible_indices)
                continue
            end

            # For each feasible triplet
            for (local_idx, global_idx) in enumerate(feasible_indices)
                (j_id, k_id, l_id) = Xi_same_source[global_idx]

                # Convert station IDs to array indices
                j = mapping.station_id_to_array_idx[j_id]
                k = mapping.station_id_to_array_idx[k_id]
                l = mapping.station_id_to_array_idx[l_id]

                if use_sparse
                    # Sparse x: sum over ODs that have edge (j,k)
                    # Constraint 1: sum of x[od][j,k] for ODs with (j,k) edge >= u
                    jk_terms = AffExpr(0.0)
                    for od in od_vector
                        valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                        x_var = get_x_var_for_edge(x[s][time_id][od], valid_pairs, j, k)
                        if !isnothing(x_var)
                            add_to_expression!(jk_terms, 1.0, x_var)
                        end
                    end
                    @constraint(m, jk_terms >= u[s][time_id][local_idx])

                    # Constraint 2: sum of x[od][j,l] for ODs with (j,l) edge >= u
                    jl_terms = AffExpr(0.0)
                    for od in od_vector
                        valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                        x_var = get_x_var_for_edge(x[s][time_id][od], valid_pairs, j, l)
                        if !isnothing(x_var)
                            add_to_expression!(jl_terms, 1.0, x_var)
                        end
                    end
                    @constraint(m, jl_terms >= u[s][time_id][local_idx])
                else
                    # Dense x: original behavior
                    @constraint(m,
                        sum(x[s][time_id][od][j, k] for od in od_vector) >= u[s][time_id][local_idx]
                    )
                    @constraint(m,
                        sum(x[s][time_id][od][j, l] for od in od_vector) >= u[s][time_id][local_idx]
                    )
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_source::Vector{Tuple{Int, Int, Int}}
    )

Same-source pooling constraints for TwoStageSingleDetourNoWalkingLimitModel.
"""
function add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_source::Vector{Tuple{Int, Int, Int}}
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    u = m[:u]
    u_idx_map = m[:u_idx_map]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            if length(od_vector) <= 1
                continue
            end

            feasible_indices = get(u_idx_map[s], time_id, Int[])
            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                (j_id, k_id, l_id) = Xi_same_source[global_idx]
                j = mapping.station_id_to_array_idx[j_id]
                k = mapping.station_id_to_array_idx[k_id]
                l = mapping.station_id_to_array_idx[l_id]

                @constraint(m,
                    sum(x[s][time_id][od][j, k] for od in od_vector) >= u[s][time_id][local_idx]
                )

                @constraint(m,
                    sum(x[s][time_id][od][j, l] for od in od_vector) >= u[s][time_id][local_idx]
                )
            end
        end
    end

    return _total_num_constraints(m) - before
end


# ============================================================================
# Same-Destination Detour Constraints
# ============================================================================

"""
    add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )

Same-destination pooling constraints for quadruplet (j, k, l, t'):
- Pools trip (j→l) with trip (j→k) - both start at j
- Vehicle picks up both at j, drops first at k, continues to l
- The second leg (k→l) occurs at time t + t'

For each feasible (j,k,l,t') ∈ Ξ, if pooling is enabled (v=1), we need:
    Σ_{od with edge (j,l) at t} x_{od,t,jl,s} ≥ v_{t,idx,s}
    Σ_{od with edge (k,l) at t+t'} x_{od,t+t',kl,s} ≥ v_{t,idx,s}

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    v = m[:v]
    x = m[:x]

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # Get feasible detour indices
            if use_sparse
                feasible_indices = get(mapping.feasible_same_dest[s], time_id, Int[])
            else
                # Without walking limits, check time validity
                feasible_indices = Int[]
                for (idx, (_, _, _, time_delta)) in enumerate(Xi_same_dest)
                    future_time_id = time_id + time_delta
                    if haskey(mapping.Omega_s_t[s], future_time_id)
                        push!(feasible_indices, idx)
                    end
                end
            end

            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                (j_id, k_id, l_id, time_delta) = Xi_same_dest[global_idx]
                future_time_id = time_id + time_delta

                if !haskey(mapping.Omega_s_t[s], future_time_id)
                    continue
                end

                future_od_vector = mapping.Omega_s_t[s][future_time_id]

                j = mapping.station_id_to_array_idx[j_id]
                k = mapping.station_id_to_array_idx[k_id]
                l = mapping.station_id_to_array_idx[l_id]

                if use_sparse
                    # Constraint 1: sum of x[od][j,l] at time t for ODs with (j,l) edge >= v
                    jl_terms = AffExpr(0.0)
                    for od in od_vector
                        valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                        x_var = get_x_var_for_edge(x[s][time_id][od], valid_pairs, j, l)
                        if !isnothing(x_var)
                            add_to_expression!(jl_terms, 1.0, x_var)
                        end
                    end
                    @constraint(m, jl_terms >= v[s][time_id][local_idx])

                    # Constraint 2: sum of x[od][k,l] at time t+t' for ODs with (k,l) edge >= v
                    kl_terms = AffExpr(0.0)
                    for od in future_od_vector
                        valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                        x_var = get_x_var_for_edge(x[s][future_time_id][od], valid_pairs, k, l)
                        if !isnothing(x_var)
                            add_to_expression!(kl_terms, 1.0, x_var)
                        end
                    end
                    @constraint(m, kl_terms >= v[s][time_id][local_idx])
                else
                    @constraint(m,
                        sum(x[s][time_id][od][j, l] for od in od_vector) >= v[s][time_id][local_idx]
                    )
                    @constraint(m,
                        sum(x[s][future_time_id][od][k, l] for od in future_od_vector) >= v[s][time_id][local_idx]
                    )
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )

Same-destination pooling constraints for TwoStageSingleDetourNoWalkingLimitModel.
"""
function add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    v = m[:v]
    v_idx_map = m[:v_idx_map]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            feasible_indices = get(v_idx_map[s], time_id, Int[])
            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                (j_id, k_id, l_id, time_delta) = Xi_same_dest[global_idx]
                future_time_id = time_id + time_delta

                if !haskey(mapping.Omega_s_t[s], future_time_id)
                    continue
                end

                j = mapping.station_id_to_array_idx[j_id]
                k = mapping.station_id_to_array_idx[k_id]
                l = mapping.station_id_to_array_idx[l_id]

                future_od_vector = mapping.Omega_s_t[s][future_time_id]

                @constraint(m,
                    sum(x[s][time_id][od][j, l] for od in od_vector) >= v[s][time_id][local_idx]
                )

                @constraint(m,
                    sum(x[s][future_time_id][od][k, l] for od in future_od_vector) >= v[s][time_id][local_idx]
                )
            end
        end
    end

    return _total_num_constraints(m) - before
end
