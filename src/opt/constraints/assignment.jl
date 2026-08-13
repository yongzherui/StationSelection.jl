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

Used by: TwoStageODPolicy
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

Each station location must be assigned to exactly one medoid (SingleStagePolicy).
    Σ_{j ∈ Aᵢ} x[i,j] = 1  ∀i

Used by: SingleStagePolicy
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = _total_num_constraints(m)
    x = m[:x]

    for i in 1:mapping.n_stations
        @constraint(m, sum(x[i]) == 1)
    end

    return _total_num_constraints(m) - before
end

"""
    add_assignment_constraints!(m::Model, data::StationSelectionData, mapping::ClusteringTwoStageStationMap)

Each demanded station i must be assigned to exactly one active cluster center j
in each scenario where it has positive endpoint count.
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageStationMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (i_idx, _) in enumerate(mapping.I_s[s])
            @constraint(m, sum(x[s][i_idx]) == 1)
        end
    end

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

Assignment requires both stations to be active (TwoStageODPolicy).
    x[s][od_idx][pair_idx] ≤ Q_s[s][(o,d)] * z[j,s]
    x[s][od_idx][pair_idx] ≤ Q_s[s][(o,d)] * z[k,s]

Used by: TwoStageODPolicy
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
            for (idx, pair) in enumerate(valid_pairs)
                is_walk_only_pair(pair) && continue
                j, k = pair
                @constraint(m, x[s][od_idx][idx] <= demand * z[j, s])
                @constraint(m, x[s][od_idx][idx] <= demand * z[k, s])
            end
        end
    end

    return _total_num_constraints(m) - before
end

"""
    add_assignment_to_active_constraints!(m::Model, data::StationSelectionData, mapping::ClusteringTwoStageStationMap)

Assignments require the chosen cluster center to be active in the scenario.
    x_{ijs} ≤ z_{js}
"""
function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageStationMap
    )
    before = _total_num_constraints(m)
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    for s in 1:S
        for (i_idx, i) in enumerate(mapping.I_s[s])
            valid_js = get_valid_j_assignments(mapping, i)
            for (j_idx, j) in enumerate(valid_js)
                @constraint(m, x[s][i_idx][j_idx] <= z[j, s])
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

Assignment can only be made to selected stations (SingleStagePolicy).
    x[i,j] ≤ y[j]  ∀i, j ∈ Aᵢ

Used by: SingleStagePolicy
"""
function add_assignment_to_selected_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = _total_num_constraints(m)
    y = m[:y]
    x = m[:x]

    for i in 1:mapping.n_stations
        valid_js = get_valid_j_assignments(mapping, i)
        for (j_idx, j) in enumerate(valid_js)
            @constraint(m, x[i][j_idx] <= y[j])
        end
    end

    return _total_num_constraints(m) - before
end
