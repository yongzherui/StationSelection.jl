"""
Objective function for ClusteringTwoStageODModel.

Contains objective for:
- ClusteringTwoStageODModel: walking + routing costs (no pooling)
"""

using JuMP

export set_clustering_od_objective!


"""
    set_clustering_od_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringScenarioODMap;
        routing_weight::Float64=1.0
    )

Set the minimization objective for ClusteringTwoStageODModel.

Objective:
    min Σ_s Σ_{(o,d)∈Ω_s} Σ_{j,k} q_{od,s} · (d^origin_{oj} + d^dest_{dk} + λ·c_{jk}) · x[s][od_idx][j,k]

Where:
- q_{od,s} = demand count for OD pair (o,d) in scenario s (aggregated across time)
- d^origin_{oj} = walking cost from origin o to pickup station j
- d^dest_{dk} = walking cost from dropoff station k to destination d
- c_{jk} = routing cost from station j to k
- λ (routing_weight) = weight for routing costs

# Arguments
- `m::Model`: JuMP model with variables x, y, z already added
- `data::StationSelectionData`: Problem data with walking_costs and routing_costs
- `mapping::ClusteringScenarioODMap`: Scenario to OD mapping
- `routing_weight::Float64`: Weight λ for routing costs (default 1.0)
"""
function set_clustering_od_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringScenarioODMap;
        routing_weight::Float64=1.0
    )

    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]
    use_sparse = has_walking_distance_limit(mapping)

    if use_sparse
        @objective(m, Min,
            sum(
                mapping.Q_s[s][(o, d)] * (
                    get_walking_cost(data, o, mapping.array_idx_to_station_id[j]) +
                    get_walking_cost(data, mapping.array_idx_to_station_id[k], d) +
                    routing_weight * get_routing_cost(data, mapping.array_idx_to_station_id[j], mapping.array_idx_to_station_id[k])
                ) * x[s][od_idx][idx]
                for s in 1:S
                for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                for (idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
            )
        )
    else
        @objective(m, Min,
            sum(
                mapping.Q_s[s][(o, d)] * (
                    get_walking_cost(data, o, mapping.array_idx_to_station_id[j]) +
                    get_walking_cost(data, mapping.array_idx_to_station_id[k], d) +
                    routing_weight * get_routing_cost(data, mapping.array_idx_to_station_id[j], mapping.array_idx_to_station_id[k])
                ) * x[s][od_idx][j, k]
                for s in 1:S
                for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                for j in 1:n
                for k in 1:n
            )
        )
    end

    return nothing
end
