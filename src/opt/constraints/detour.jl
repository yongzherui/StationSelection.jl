"""
Detour constraint creation functions for station selection optimization models.

These functions add constraints for same-source and same-destination pooling,
linking detour pooling variables to assignment variables.

Used by: TwoStageSingleDetourModel (with or without walking limits)
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

"""
    feasible_same_source_indices(
        mapping::TwoStageSingleDetourMap,
        s::Int,
        time_id::Int,
        n_same_source::Int,
        use_sparse::Bool
    ) -> Vector{Int}
"""
function feasible_same_source_indices(
    mapping::TwoStageSingleDetourMap,
    s::Int,
    time_id::Int,
    n_same_source::Int,
    use_sparse::Bool
)::Vector{Int}
    if use_sparse
        return get(mapping.feasible_same_source[s], time_id, Int[])
    end
    return collect(1:n_same_source)
end

"""
    feasible_same_dest_indices(
        mapping::TwoStageSingleDetourMap,
        s::Int,
        time_id::Int,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}},
        use_sparse::Bool
    ) -> Vector{Int}
"""
function feasible_same_dest_indices(
    mapping::TwoStageSingleDetourMap,
    s::Int,
    time_id::Int,
    Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}},
    use_sparse::Bool
)::Vector{Int}
    if use_sparse
        return get(mapping.feasible_same_dest[s], time_id, Int[])
    end

    feasible_indices = Int[]
    for (idx, (_, _, _, time_delta)) in enumerate(Xi_same_dest)
        future_time_id = time_id + time_delta
        if haskey(mapping.Omega_s_t[s], future_time_id)
            push!(feasible_indices, idx)
        end
    end
    return feasible_indices
end

"""
    edge_sum_terms_x(
        mapping::TwoStageSingleDetourMap,
        x,
        s::Int,
        time_id::Int,
        od_vector::Vector{Tuple{Int, Int}},
        j::Int,
        k::Int,
        use_sparse::Bool
    ) -> AffExpr
"""
function edge_sum_terms_x(
    mapping::TwoStageSingleDetourMap,
    x,
    s::Int,
    time_id::Int,
    od_vector::Vector{Tuple{Int, Int}},
    j::Int,
    k::Int,
    use_sparse::Bool
)::AffExpr
    expr = AffExpr(0.0)
    if use_sparse
        for od in od_vector
            valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
            x_var = get_x_var_for_edge(x[s][time_id][od], valid_pairs, j, k)
            if !isnothing(x_var)
                add_to_expression!(expr, 1.0, x_var)
            end
        end
    else
        for od in od_vector
            add_to_expression!(expr, 1.0, x[s][time_id][od][j, k])
        end
    end
    return expr
end

"""
    get_flow_var(
        f,
        s::Int,
        time_id::Int,
        j::Int,
        k::Int,
        use_sparse::Bool
    ) -> Union{VariableRef, Nothing}
"""
function get_flow_var(
    f,
    s::Int,
    time_id::Int,
    j::Int,
    k::Int,
    use_sparse::Bool
)::Union{VariableRef, Nothing}
    if use_sparse
        return get(f[s][time_id], (j, k), nothing)
    end
    return f[s][time_id][j, k]
end


# ============================================================================
# Same-Source Detour Constraints
# ============================================================================

"""
    add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap,
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
        mapping::TwoStageSingleDetourMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}};
        use_flow_bounds::Bool=false
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    u = m[:u]
    x = m[:x]
    f = m[:f]
    n_same_source = length(Xi_same_source)

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            if length(od_vector) <= 1
                continue
            end

            feasible_indices = feasible_same_source_indices(
                mapping, s, time_id, n_same_source, use_sparse
            )

            if isempty(feasible_indices)
                continue
            end

            for (local_idx, global_idx) in enumerate(feasible_indices)
                (j_id, k_id, l_id) = Xi_same_source[global_idx]

                j = mapping.station_id_to_array_idx[j_id]
                k = mapping.station_id_to_array_idx[k_id]
                l = mapping.station_id_to_array_idx[l_id]

                if use_flow_bounds
                    jk_var = get_flow_var(f, s, time_id, j, k, use_sparse)
                    jl_var = get_flow_var(f, s, time_id, j, l, use_sparse)
                    if isnothing(jk_var) || isnothing(jl_var)
                        error("Missing flow variable for detour bound at scenario $(s), time $(time_id).")
                    end
                    @constraint(m, jk_var >= u[s][time_id][local_idx])
                    @constraint(m, jl_var >= u[s][time_id][local_idx])
                else
                    jk_terms = edge_sum_terms_x(
                        mapping, x, s, time_id, od_vector, j, k, use_sparse
                    )
                    jl_terms = edge_sum_terms_x(
                        mapping, x, s, time_id, od_vector, j, l, use_sparse
                    )
                    @constraint(m, jk_terms >= u[s][time_id][local_idx])
                    @constraint(m, jl_terms >= u[s][time_id][local_idx])
                end
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
        mapping::TwoStageSingleDetourMap,
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
        mapping::TwoStageSingleDetourMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        use_flow_bounds::Bool=false
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    v = m[:v]
    x = m[:x]
    f = m[:f]

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            feasible_indices = feasible_same_dest_indices(
                mapping, s, time_id, Xi_same_dest, use_sparse
            )

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

                if use_flow_bounds
                    jl_var = get_flow_var(f, s, time_id, j, l, use_sparse)
                    kl_var = get_flow_var(f, s, future_time_id, k, l, use_sparse)
                    if isnothing(jl_var) || isnothing(kl_var)
                        error("Missing flow variable for detour bound at scenario $(s), time $(time_id).")
                    end
                    @constraint(m, jl_var >= v[s][time_id][local_idx])
                    @constraint(m, kl_var >= v[s][time_id][local_idx])
                else
                    jl_terms = edge_sum_terms_x(
                        mapping, x, s, time_id, od_vector, j, l, use_sparse
                    )
                    kl_terms = edge_sum_terms_x(
                        mapping, x, s, future_time_id, future_od_vector, k, l, use_sparse
                    )
                    @constraint(m, jl_terms >= v[s][time_id][local_idx])
                    @constraint(m, kl_terms >= v[s][time_id][local_idx])
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end
