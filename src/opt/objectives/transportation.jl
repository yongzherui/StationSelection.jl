"""
Objective function for TransportationModel.

min sum_{g,s,i,j} w_walk_pick[i,j] * x_pick[i,j,g,s]
  + sum_{g,s,i,k} w_walk_drop[i,k] * x_drop[i,k,g,s]
  + sum_{g,s,(j,k) in P(g)} w_route * r[j,k] * f[j,k,g,s]
  + sum_{g,s} w_activation * u[g,s]
"""

using JuMP

export set_transportation_objective!

"""
    set_transportation_objective!(m::Model, data::StationSelectionData,
                                  mapping::TransportationMap;
                                  in_vehicle_time_weight, activation_cost)

Set the minimization objective for TransportationModel.

Components:
1. Walking cost for pickup assignments (origin -> pickup station)
2. Walking cost for dropoff assignments (dropoff station -> destination)
3. Routing cost for transportation flow (weighted by in_vehicle_time_weight)
4. Fixed activation cost per active anchor per scenario
"""
function set_transportation_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap;
        in_vehicle_time_weight::Float64=1.0,
        activation_cost::Float64=0.0
    )

    x_pick = m[:x_pick]
    x_drop = m[:x_drop]
    f_transport = m[:f_transport]
    u_anchor = m[:u_anchor]

    obj = AffExpr(0.0)

    for (g_idx, _) in enumerate(mapping.active_anchors)
        for s in mapping.anchor_scenarios[g_idx]
            # Walking cost for pickup: w_walk(origin_id, station_j_id) * x_pick
            for ((i, j), var) in x_pick[g_idx][s]
                j_id = mapping.array_idx_to_station_id[j]
                w = get_walking_cost(data, i, j_id)
                count = mapping.m_pick[g_idx][s][i]
                add_to_expression!(obj, w * count, var)
            end

            # Walking cost for dropoff: w_walk(station_k_id, dest_id) * x_drop
            for ((i, k), var) in x_drop[g_idx][s]
                k_id = mapping.array_idx_to_station_id[k]
                w = get_walking_cost(data, k_id, i)
                count = mapping.m_drop[g_idx][s][i]
                add_to_expression!(obj, w * count, var)
            end

            # Routing cost for flow: in_vehicle_time_weight * routing_cost(j,k) * f
            for ((j, k), var) in f_transport[g_idx][s]
                j_id = mapping.array_idx_to_station_id[j]
                k_id = mapping.array_idx_to_station_id[k]
                r_jk = get_routing_cost(data, j_id, k_id)
                add_to_expression!(obj, in_vehicle_time_weight * r_jk, var)
            end

            # Activation cost
            if activation_cost > 0
                add_to_expression!(obj, activation_cost, u_anchor[g_idx][s])
            end
        end
    end

    @objective(m, Min, obj)

    return nothing
end
