"""
Transportation constraint creation functions for TransportationModel.

Adds assignment, aggregation, flow conservation, flow activation,
and viability constraints for the transportation model.
"""

using JuMP

export add_transportation_assignment_constraints!
export add_transportation_aggregation_constraints!
export add_transportation_flow_conservation_constraints!
export add_transportation_flow_activation_constraints!
export add_transportation_viability_constraints!

"""
    add_transportation_assignment_constraints!(m::Model, data::StationSelectionData,
                                               mapping::TransportationMap) -> Int

Each origin must be assigned to exactly one pickup station per anchor/scenario,
and each destination to exactly one dropoff station.

    sum_j x_pick[i,j,g,s] = 1  for all i in I_g_pick, g, s
    sum_k x_drop[i,k,g,s] = 1  for all i in I_g_drop, g, s
"""
function add_transportation_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = _total_num_constraints(m)
    x_pick = m[:x_pick]
    x_drop = m[:x_drop]

    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        stations_a = mapping.cluster_station_sets[zone_a]
        stations_b = mapping.cluster_station_sets[zone_b]

        for s in mapping.anchor_scenarios[g_idx]
            # Pickup assignment: each origin assigned to exactly one station
            for i in mapping.I_g_pick[g_idx][s]
                pick_vars = [x_pick[g_idx][s][(i, j)] for j in stations_a
                             if haskey(x_pick[g_idx][s], (i, j))]
                if !isempty(pick_vars)
                    @constraint(m, sum(pick_vars) == 1)
                end
            end

            # Dropoff assignment: each dest assigned to exactly one station
            for i in mapping.I_g_drop[g_idx][s]
                drop_vars = [x_drop[g_idx][s][(i, k)] for k in stations_b
                             if haskey(x_drop[g_idx][s], (i, k))]
                if !isempty(drop_vars)
                    @constraint(m, sum(drop_vars) == 1)
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_transportation_aggregation_constraints!(m::Model, data::StationSelectionData,
                                                mapping::TransportationMap) -> Int

Aggregation constraints linking individual assignments to station totals:

    p[j,g,s] = sum_i m_pick[i,g,s] * x_pick[i,j,g,s]
    d[k,g,s] = sum_i m_drop[i,g,s] * x_drop[i,k,g,s]
"""
function add_transportation_aggregation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = _total_num_constraints(m)
    x_pick = m[:x_pick]
    x_drop = m[:x_drop]
    p_agg = m[:p_agg]
    d_agg = m[:d_agg]

    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        stations_a = mapping.cluster_station_sets[zone_a]
        stations_b = mapping.cluster_station_sets[zone_b]

        for s in mapping.anchor_scenarios[g_idx]
            # p[j,g,s] = sum_i m_pick[i,g,s] * x_pick[i,j,g,s]
            for j in stations_a
                pick_expr = AffExpr(0.0)
                for i in mapping.I_g_pick[g_idx][s]
                    if haskey(x_pick[g_idx][s], (i, j))
                        count = mapping.m_pick[g_idx][s][i]
                        add_to_expression!(pick_expr, count, x_pick[g_idx][s][(i, j)])
                    end
                end
                @constraint(m, p_agg[g_idx][s][j] == pick_expr)
            end

            # d[k,g,s] = sum_i m_drop[i,g,s] * x_drop[i,k,g,s]
            for k in stations_b
                drop_expr = AffExpr(0.0)
                for i in mapping.I_g_drop[g_idx][s]
                    if haskey(x_drop[g_idx][s], (i, k))
                        count = mapping.m_drop[g_idx][s][i]
                        add_to_expression!(drop_expr, count, x_drop[g_idx][s][(i, k)])
                    end
                end
                @constraint(m, d_agg[g_idx][s][k] == drop_expr)
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_transportation_flow_conservation_constraints!(m::Model, data::StationSelectionData,
                                                      mapping::TransportationMap) -> Int

Flow conservation constraints:

    sum_k f[j,k,g,s] = p[j,g,s]  for all j in C_a, g, s
    sum_j f[j,k,g,s] = d[k,g,s]  for all k in C_b, g, s
"""
function add_transportation_flow_conservation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = _total_num_constraints(m)
    p_agg = m[:p_agg]
    d_agg = m[:d_agg]
    f_transport = m[:f_transport]

    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        stations_a = mapping.cluster_station_sets[zone_a]
        stations_b = mapping.cluster_station_sets[zone_b]

        for s in mapping.anchor_scenarios[g_idx]
            # Outflow: sum_k f[j,k,g,s] = p[j,g,s]
            for j in stations_a
                flow_out = [f_transport[g_idx][s][(j, k)] for k in stations_b
                            if haskey(f_transport[g_idx][s], (j, k))]
                @constraint(m, sum(flow_out) == p_agg[g_idx][s][j])
            end

            # Inflow: sum_j f[j,k,g,s] = d[k,g,s]
            for k in stations_b
                flow_in = [f_transport[g_idx][s][(j, k)] for j in stations_a
                           if haskey(f_transport[g_idx][s], (j, k))]
                @constraint(m, sum(flow_in) == d_agg[g_idx][s][k])
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_transportation_flow_activation_constraints!(m::Model, data::StationSelectionData,
                                                     mapping::TransportationMap) -> Int

Flow activation constraints:

    f[j,k,g,s] <= M_gs * u[g,s]  for all (j,k) in P(g), g, s

where M_gs is the total number of trips in anchor g, scenario s.
"""
function add_transportation_flow_activation_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = _total_num_constraints(m)
    f_transport = m[:f_transport]
    u_anchor = m[:u_anchor]

    for (g_idx, _) in enumerate(mapping.active_anchors)
        for s in mapping.anchor_scenarios[g_idx]
            big_m = mapping.M_gs[(g_idx, s)]
            for (j, k) in mapping.P_g[g_idx]
                if haskey(f_transport[g_idx][s], (j, k))
                    @constraint(m,
                        f_transport[g_idx][s][(j, k)] <= big_m * u_anchor[g_idx][s]
                    )
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_transportation_viability_constraints!(m::Model, data::StationSelectionData,
                                              mapping::TransportationMap) -> Int

Viability constraints linking assignments to station activation:

    x_pick[i,j,g,s] <= z[j,s]  for all i, j, g, s
    x_drop[i,k,g,s] <= z[k,s]  for all i, k, g, s
"""
function add_transportation_viability_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = _total_num_constraints(m)
    x_pick = m[:x_pick]
    x_drop = m[:x_drop]
    z = m[:z]

    for (g_idx, _) in enumerate(mapping.active_anchors)
        for s in mapping.anchor_scenarios[g_idx]
            # x_pick <= z
            for ((i, j), var) in x_pick[g_idx][s]
                @constraint(m, var <= z[j, s])
            end

            # x_drop <= z
            for ((i, k), var) in x_drop[g_idx][s]
                @constraint(m, var <= z[k, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end
