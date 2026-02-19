"""
Objective function for corridor models (ZCorridorODModel, XCorridorODModel).

Assignment cost (same as clustering_od) + corridor penalty γ Σ_{g,s} r_g f_{gs}.
"""

using JuMP

export set_corridor_od_objective!

"""
    set_corridor_od_objective!(m::Model, data::StationSelectionData,
                               mapping::CorridorTwoStageODMap;
                               in_vehicle_time_weight, corridor_weight,
                               variable_reduction)

Set the minimization objective for corridor models (ZCorridorODModel, XCorridorODModel).

Objective:
    min Σ_s Σ_{(o,d)} Σ_{j,k} q_{ods} (d^origin_{oj} + d^dest_{dk} + λ·c_{jk}) x_{odjks}
        + γ Σ_{g,s} r_g f_{gs}
"""
function set_corridor_od_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap;
        in_vehicle_time_weight::Float64=1.0,
        corridor_weight::Float64=1.0,
        variable_reduction::Bool=true
    )

    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]
    f_corridor = m[:f_corridor]
    use_sparse = variable_reduction && has_walking_distance_limit(mapping)

    # Assignment cost (same as clustering_od)
    if use_sparse
        assignment_cost = @expression(m,
            sum(
                mapping.Q_s[s][(o, d)] * (
                    get_walking_cost(data, o, mapping.array_idx_to_station_id[j]) +
                    get_walking_cost(data, mapping.array_idx_to_station_id[k], d) +
                    in_vehicle_time_weight * get_routing_cost(data, mapping.array_idx_to_station_id[j], mapping.array_idx_to_station_id[k])
                ) * x[s][od_idx][idx]
                for s in 1:S
                for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                for (idx, (j, k)) in enumerate(get_valid_jk_pairs(mapping, o, d))
            )
        )
    else
        assignment_cost = @expression(m,
            sum(
                mapping.Q_s[s][(o, d)] * (
                    get_walking_cost(data, o, mapping.array_idx_to_station_id[j]) +
                    get_walking_cost(data, mapping.array_idx_to_station_id[k], d) +
                    in_vehicle_time_weight * get_routing_cost(data, mapping.array_idx_to_station_id[j], mapping.array_idx_to_station_id[k])
                ) * x[s][od_idx][j, k]
                for s in 1:S
                for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                for j in 1:n
                for k in 1:n
            )
        )
    end

    # Corridor penalty
    n_corridors = length(mapping.corridor_indices)
    corridor_cost = @expression(m,
        corridor_weight * sum(
            mapping.corridor_costs[g] * f_corridor[g, s]
            for g in 1:n_corridors
            for s in 1:S
        )
    )

    @objective(m, Min, assignment_cost + corridor_cost)

    return nothing
end
