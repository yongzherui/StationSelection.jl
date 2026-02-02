"""
Flow constraint creation functions for station selection optimization models.

These functions add constraints that link assignments to flow variables,
ensuring that flow is tracked on edges where assignments are made.

Used by: TwoStageSingleDetourModel (with or without walking limits)
"""

using JuMP

export add_assignment_to_flow_constraints!
export add_assignment_to_flow_lb_constraints!
export add_assignment_to_flow_ub_constraints!

"""
    sum_edge_assignments(
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
function sum_edge_assignments(
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
            for (idx, (jj, kk)) in enumerate(valid_pairs)
                if jj == j && kk == k
                    add_to_expression!(expr, 1.0, x[s][time_id][od][idx])
                    break
                end
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
    add_assignment_to_flow_lb_constraints!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Assignment implies flow on that edge.
    x[s][t][od][j,k] ≤ f[s][t][j,k]  ∀(o,d,t) ∈ Ω, j, k, s

When walking limits are enabled, only iterates over valid (j,k) pairs from mapping.

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_flow_lb_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    f = m[:f]
    x = m[:x]

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                if use_sparse
                    # Sparse x: iterate over valid (j,k) pairs from mapping
                    valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                    for (idx, (j, k)) in enumerate(valid_pairs)
                        @constraint(m, x[s][time_id][od][idx] <= f[s][time_id][(j, k)])
                    end
                else
                    # Dense x: iterate over all (j,k) pairs
                    n = data.n_stations
                    @constraint(m, [j in 1:n, k in 1:n], x[s][time_id][od][j, k] <= f[s][time_id][j, k])
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end

"""
    add_assignment_to_flow_ub_constraints!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Tighten flow: flow implies at least one assignment on that edge.
    f[s][t][j,k] ≤ Σ_{od} x[s][t][od][j,k]  ∀(t,j,k,s)

When walking limits are enabled, only iterates over valid (j,k) pairs from mapping.

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_flow_ub_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    f = m[:f]
    x = m[:x]

    use_sparse = has_walking_distance_limit(mapping)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            if use_sparse
                for (j, k) in get_valid_f_pairs(mapping, s, time_id)
                    edge_sum = sum_edge_assignments(
                        mapping, x, s, time_id, od_vector, j, k, true
                    )
                    @constraint(m, f[s][time_id][(j, k)] <= edge_sum)
                end
            else
                n = data.n_stations
                for j in 1:n, k in 1:n
                    edge_sum = sum_edge_assignments(
                        mapping, x, s, time_id, od_vector, j, k, false
                    )
                    @constraint(m, f[s][time_id][j, k] <= edge_sum)
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end

"""
    add_assignment_to_flow_constraints!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Adds both lower- and upper-bound assignment-to-flow constraints and returns
the total number of constraints added.

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_flow_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap
    )
    added_lb = add_assignment_to_flow_lb_constraints!(m, data, mapping)
    added_ub = add_assignment_to_flow_ub_constraints!(m, data, mapping)
    return added_lb + added_ub
end
