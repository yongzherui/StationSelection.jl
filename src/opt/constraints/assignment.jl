"""
Assignment constraint creation functions for station selection optimization models.

These functions add constraints that:
1. Ensure each request is assigned to exactly one station pair
2. Link assignments to station activation/selection

Uses multiple dispatch for different mapping types.
"""

using JuMP

export add_assignment_constraints!
export add_assignment_to_active_constraints!
export add_assignment_to_selected_constraints!


# ============================================================================
# Assignment Constraints - Each request assigned to exactly one station pair
# ============================================================================

"""
    add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
    )

All demand for each OD pair must be assigned across valid station pairs.
    Σⱼₖ x[s][od_idx][j,k] = Q_s[s][(o,d)]  ∀od_idx ∈ Ω_s, s

Used by: ClusteringTwoStageODModel
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            @constraint(m, sum(x[s][od_idx]) == demand)
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_assignment_constraints!(m::Model, data::StationSelectionData, mapping::ClusteringBaseModelMap)

Each station location must be assigned to exactly one medoid (ClusteringBaseModel).
    Σⱼ x[i,j] = 1  ∀i

Used by: ClusteringBaseModel
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = _total_num_constraints(m)
    n = mapping.n_stations
    x = m[:x]

    @constraint(m, [i=1:n], sum(x[i, j] for j in 1:n) == 1)

    return _total_num_constraints(m) - before
end


# ============================================================================
# Assignment to Active Constraints - Assignments require active stations
# ============================================================================

"""
    add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
    )

Assignment requires both stations to be active (ClusteringTwoStageODModel).
    x[s][od_idx][pair_idx] ≤ Q_s[s][(o,d)] * z[j,s]
    x[s][od_idx][pair_idx] ≤ Q_s[s][(o,d)] * z[k,s]

Used by: ClusteringTwoStageODModel
"""
function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = get(mapping.Q_s[s], (o, d), 0)
            demand == 0 && continue
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            for (idx, (j, k)) in enumerate(valid_pairs)
                @constraint(m, x[s][od_idx][idx] <= demand * z[j, s])
                @constraint(m, x[s][od_idx][idx] <= demand * z[k, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end


# ============================================================================
# VehicleCapacityODMap (RouteVehicleCapacityModel — new formulation)
# ============================================================================

"""
    add_assignment_constraints!(m, data, mapping::Union{VehicleCapacityODMap, AlphaRouteODMap})

All demand for each (OD, time bucket) must be assigned across valid (j,k) pairs.
    Σ_{(j,k)} x[s][t_id][od_idx] == Q_s_t[s][t_id][(o,d)]  ∀(s, t_id, od_idx)

x is integer-valued; the RHS equals the passenger count for that OD/time/scenario.
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::Union{VehicleCapacityODMap, AlphaRouteODMap}
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            for (od_idx, (o, d)) in enumerate(od_pairs)
                x_od = get(x[s][t_id], od_idx, VariableRef[])
                isempty(x_od) && continue
                demand = get(mapping.Q_s_t[s][t_id], (o, d), 0)
                @constraint(m, sum(x_od) == demand)
            end
        end
    end

    return _total_num_constraints(m) - before
end


"""
    add_assignment_to_active_constraints!(m, data, mapping::Union{VehicleCapacityODMap, AlphaRouteODMap})

Assignments require both pickup and dropoff stations to be active (big-M formulation).
    x[s][t_id][od_idx][pair_idx] ≤ Q_s_t[s][t_id][(o,d)] · z[j,s]
    x[s][t_id][od_idx][pair_idx] ≤ Q_s_t[s][t_id][(o,d)] · z[k,s]

The big-M coefficient equals the per-(OD, time bucket, scenario) demand count.
"""
function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::Union{VehicleCapacityODMap, AlphaRouteODMap}
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            for (od_idx, (o, d)) in enumerate(od_pairs)
                demand = get(mapping.Q_s_t[s][t_id], (o, d), 0)
                demand == 0 && continue
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                x_od = get(x[s][t_id], od_idx, VariableRef[])
                isempty(x_od) && continue
                for (pair_idx, (j, k)) in enumerate(valid_pairs)
                    @constraint(m, x_od[pair_idx] <= demand * z[j, s])
                    @constraint(m, x_od[pair_idx] <= demand * z[k, s])
                end
            end
        end
    end

    return _total_num_constraints(m) - before
end


# ============================================================================
# Assignment to Selected Constraints - For models without scenarios
# ============================================================================

"""
    add_assignment_to_selected_constraints!(m::Model, data::StationSelectionData, mapping::ClusteringBaseModelMap)

Assignment can only be made to selected stations (ClusteringBaseModel).
    x[i,j] ≤ y[j]  ∀i, j

Used by: ClusteringBaseModel
"""
function add_assignment_to_selected_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = _total_num_constraints(m)
    n = mapping.n_stations
    y = m[:y]
    x = m[:x]

    @constraint(m, [i=1:n, j=1:n], x[i, j] <= y[j])

    return _total_num_constraints(m) - before
end
