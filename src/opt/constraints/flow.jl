"""
Flow constraint creation functions for station selection optimization models.

These functions add constraints that link assignments to flow variables,
ensuring that flow is tracked on edges where assignments are made.

Used by: TwoStageSingleDetourModel (with or without walking limits)
"""

using JuMP

export add_assignment_to_flow_constraints!


"""
    add_assignment_to_flow_constraints!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Assignment implies flow on that edge.
    x[s][t][od][j,k] ≤ f[s][t][j,k]  ∀(o,d,t) ∈ Ω, j, k, s

When walking limits are enabled, only iterates over valid (j,k) pairs from mapping.

Used by: TwoStageSingleDetourModel
"""
function add_assignment_to_flow_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap
    )
    before = _total_num_constraints(m)
    n = data.n_stations
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
                        @constraint(m, x[s][time_id][od][idx] <= f[s][time_id][j, k])
                    end
                else
                    # Dense x: iterate over all (j,k) pairs
                    @constraint(m, [j in 1:n, k in 1:n], x[s][time_id][od][j, k] <= f[s][time_id][j, k])
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end

