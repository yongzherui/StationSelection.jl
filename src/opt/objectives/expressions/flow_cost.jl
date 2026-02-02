"""
Flow cost expressions for objective functions.

These functions return AffExpr representing the cost of vehicle flow
between station pairs.
"""

using JuMP

export flow_cost_expr


"""
    flow_cost_expr(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap;
        routing_weight::Float64=1.0
    ) -> AffExpr

Compute the flow routing cost expression for TwoStageSingleDetourModel.

For each flow variable f[s][t][j,k], the cost is:
    γ · c_{jk} · f[s][t][j,k]

Where:
- γ (routing_weight) = weight for routing costs
- c_{jk} = routing cost from station j to k

Returns an AffExpr that can be combined with other objective components.
"""
function flow_cost_expr(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap;
        routing_weight::Float64=1.0
    )::AffExpr

    S = n_scenarios(data)
    f = m[:f]
    use_sparse = has_walking_distance_limit(mapping)

    expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, _) in mapping.Omega_s_t[s]
            if use_sparse
                for (j, k) in get_valid_f_pairs(mapping, s, time_id)
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    c_jk = get_routing_cost(data, j_id, k_id)
                    add_to_expression!(expr, routing_weight * c_jk, f[s][time_id][(j, k)])
                end
            else
                n = data.n_stations
                for j in 1:n, k in 1:n
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]
                    c_jk = get_routing_cost(data, j_id, k_id)

                    add_to_expression!(expr, routing_weight * c_jk, f[s][time_id][j, k])
                end
            end
        end
    end

    return expr
end
