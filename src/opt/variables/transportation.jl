"""
Transportation variable creation functions for TransportationModel.

Adds per-anchor pickup/dropoff assignment, aggregation, flow, and activation variables.
"""

using JuMP

export add_transportation_assignment_variables!
export add_transportation_aggregation_variables!
export add_transportation_flow_variables!
export add_transportation_activation_variables!

"""
    add_transportation_assignment_variables!(m::Model, data::StationSelectionData,
                                             mapping::TransportationMap) -> Int

Add binary pickup/dropoff assignment variables:
- x_pick[i,j,g,s] in {0,1}: origin i picks up at station j in anchor g, scenario s
- x_drop[i,k,g,s] in {0,1}: destination i drops off at station k in anchor g, scenario s

Stored as nested dicts: x_pick[g_idx][s][(i, j)] and x_drop[g_idx][s][(i, k)]
where i is station ID and j,k are array indices.
"""
function add_transportation_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = JuMP.num_variables(m)
    n = data.n_stations
    has_walk_limit = has_walking_distance_limit(mapping)

    # x_pick[g_idx][s] = Dict of (origin_id, j_array_idx) => variable
    x_pick = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, VariableRef}}}()
    # x_drop[g_idx][s] = Dict of (dest_id, k_array_idx) => variable
    x_drop = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, VariableRef}}}()

    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        x_pick[g_idx] = Dict{Int, Dict{Tuple{Int, Int}, VariableRef}}()
        x_drop[g_idx] = Dict{Int, Dict{Tuple{Int, Int}, VariableRef}}()

        stations_a = mapping.cluster_station_sets[zone_a]
        stations_b = mapping.cluster_station_sets[zone_b]

        for s in mapping.anchor_scenarios[g_idx]
            x_pick[g_idx][s] = Dict{Tuple{Int, Int}, VariableRef}()
            x_drop[g_idx][s] = Dict{Tuple{Int, Int}, VariableRef}()

            # Pickup variables: for each origin i, for each station j in zone_a
            for i in mapping.I_g_pick[g_idx][s]
                for j in stations_a
                    if has_walk_limit
                        j_id = mapping.array_idx_to_station_id[j]
                        walk_dist = get_walking_cost(data, i, j_id)
                        if walk_dist > mapping.max_walking_distance
                            continue
                        end
                    end
                    var = @variable(m, binary=true)
                    x_pick[g_idx][s][(i, j)] = var
                end
            end

            # Dropoff variables: for each dest i, for each station k in zone_b
            for i in mapping.I_g_drop[g_idx][s]
                for k in stations_b
                    if has_walk_limit
                        k_id = mapping.array_idx_to_station_id[k]
                        walk_dist = get_walking_cost(data, k_id, i)
                        if walk_dist > mapping.max_walking_distance
                            continue
                        end
                    end
                    var = @variable(m, binary=true)
                    x_drop[g_idx][s][(i, k)] = var
                end
            end
        end
    end

    m[:x_pick] = x_pick
    m[:x_drop] = x_drop

    return JuMP.num_variables(m) - before
end


"""
    add_transportation_aggregation_variables!(m::Model, data::StationSelectionData,
                                              mapping::TransportationMap) -> Int

Add continuous aggregation variables:
- p[j,g,s] >= 0: total pickups at station j for anchor g, scenario s
- d[k,g,s] >= 0: total dropoffs at station k for anchor g, scenario s

Stored as: p_agg[g_idx][s][j] and d_agg[g_idx][s][k]
"""
function add_transportation_aggregation_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = JuMP.num_variables(m)

    p_agg = Dict{Int, Dict{Int, Dict{Int, VariableRef}}}()
    d_agg = Dict{Int, Dict{Int, Dict{Int, VariableRef}}}()

    for (g_idx, anchor) in enumerate(mapping.active_anchors)
        zone_a, zone_b = anchor
        stations_a = mapping.cluster_station_sets[zone_a]
        stations_b = mapping.cluster_station_sets[zone_b]

        p_agg[g_idx] = Dict{Int, Dict{Int, VariableRef}}()
        d_agg[g_idx] = Dict{Int, Dict{Int, VariableRef}}()

        for s in mapping.anchor_scenarios[g_idx]
            p_agg[g_idx][s] = Dict{Int, VariableRef}()
            d_agg[g_idx][s] = Dict{Int, VariableRef}()

            for j in stations_a
                p_agg[g_idx][s][j] = @variable(m, lower_bound=0)
            end
            for k in stations_b
                d_agg[g_idx][s][k] = @variable(m, lower_bound=0)
            end
        end
    end

    m[:p_agg] = p_agg
    m[:d_agg] = d_agg

    return JuMP.num_variables(m) - before
end


"""
    add_transportation_flow_variables!(m::Model, data::StationSelectionData,
                                       mapping::TransportationMap) -> Int

Add continuous flow variables:
- f_transport[j,k,g,s] >= 0: flow from station j to station k for anchor g, scenario s

Only for (j,k) in P(g) = {(j,k) : j in C_a, k in C_b}.
Stored as: f_transport[g_idx][s][(j,k)]
"""
function add_transportation_flow_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = JuMP.num_variables(m)

    f_transport = Dict{Int, Dict{Int, Dict{Tuple{Int, Int}, VariableRef}}}()

    for (g_idx, _) in enumerate(mapping.active_anchors)
        f_transport[g_idx] = Dict{Int, Dict{Tuple{Int, Int}, VariableRef}}()

        for s in mapping.anchor_scenarios[g_idx]
            f_transport[g_idx][s] = Dict{Tuple{Int, Int}, VariableRef}()

            for (j, k) in mapping.P_g[g_idx]
                f_transport[g_idx][s][(j, k)] = @variable(m, lower_bound=0)
            end
        end
    end

    m[:f_transport] = f_transport

    return JuMP.num_variables(m) - before
end


"""
    add_transportation_activation_variables!(m::Model, data::StationSelectionData,
                                              mapping::TransportationMap) -> Int

Add binary anchor activation variables:
- u_anchor[g,s] in {0,1}: anchor g is used in scenario s

Stored as: u_anchor[g_idx][s]
"""
function add_transportation_activation_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TransportationMap
    )
    before = JuMP.num_variables(m)

    u_anchor = Dict{Int, Dict{Int, VariableRef}}()

    for (g_idx, _) in enumerate(mapping.active_anchors)
        u_anchor[g_idx] = Dict{Int, VariableRef}()

        for s in mapping.anchor_scenarios[g_idx]
            u_anchor[g_idx][s] = @variable(m, binary=true)
        end
    end

    m[:u_anchor] = u_anchor

    return JuMP.num_variables(m) - before
end
