"""
Assignment cost expressions for objective functions.

These functions return AffExpr representing the cost of assigning OD pairs
to station pairs, including walking costs and routing costs.
"""

using JuMP

export assignment_cost_expr


"""
    assignment_cost_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    ) -> AffExpr

Compute the assignment cost expression for TwoStageSingleDetourModel.

For each OD assignment x[s][t][od][j,k], the cost is:
    q_{od,s,t} · (d^origin_{o,j} + d^dest_{d,k} + c_{jk})

Where:
- q_{od,s,t} = demand count for OD pair (o,d) at time t in scenario s
- d^origin_{o,j} = walking cost from origin o to pickup station j
- d^dest_{d,k} = walking cost from dropoff station k to destination d
- c_{jk} = routing cost from station j to k

When walking limits are enabled, only iterates over valid (j,k) pairs from mapping.

Returns an AffExpr that can be combined with other objective components.
"""
function assignment_cost_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )::AffExpr

    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]

    use_sparse = has_walking_distance_limit(mapping)

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for (o, d) in od_vector
                q_od_s_t = mapping.Q_s_t[s][time_id][(o, d)]

                if use_sparse
                    # Sparse x: iterate over valid (j,k) pairs from mapping
                    valid_pairs = get_valid_jk_pairs(mapping, o, d)
                    for (idx, (j, k)) in enumerate(valid_pairs)
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]

                        d_origin_oj = get_walking_cost(data, o, j_id)
                        d_dest_dk = get_walking_cost(data, k_id, d)
                        c_jk = get_routing_cost(data, j_id, k_id)

                        cost = q_od_s_t * (d_origin_oj + d_dest_dk + c_jk)
                        add_to_expression!(expr, cost, x[s][time_id][(o, d)][idx])
                    end
                else
                    # Dense x: iterate over all (j,k) pairs
                    for j in 1:n, k in 1:n
                        j_id = mapping.array_idx_to_station_id[j]
                        k_id = mapping.array_idx_to_station_id[k]

                        d_origin_oj = get_walking_cost(data, o, j_id)
                        d_dest_dk = get_walking_cost(data, k_id, d)
                        c_jk = get_routing_cost(data, j_id, k_id)

                        cost = q_od_s_t * (d_origin_oj + d_dest_dk + c_jk)
                        add_to_expression!(expr, cost, x[s][time_id][(o, d)][j, k])
                    end
                end
            end
        end
    end

    return expr
end


"""
    assignment_cost_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit
    ) -> AffExpr

Compute the assignment cost expression for TwoStageSingleDetourModel without walking limits.
Same logic as PoolingScenarioOriginDestTimeMap version.
"""
function assignment_cost_expr(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit
    )::AffExpr

    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for (o, d) in od_vector
                q_od_s_t = mapping.Q_s_t[s][time_id][(o, d)]

                for j in 1:n, k in 1:n
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]

                    d_origin_oj = get_walking_cost(data, o, j_id)
                    d_dest_dk = get_walking_cost(data, k_id, d)
                    c_jk = get_routing_cost(data, j_id, k_id)

                    cost = q_od_s_t * (d_origin_oj + d_dest_dk + c_jk)
                    add_to_expression!(expr, cost, x[s][time_id][(o, d)][j, k])
                end
            end
        end
    end

    return expr
end
