"""
Base constraint creation functions for station selection optimization models.

These functions add fundamental constraints shared across multiple model types:
1. Station Limit Constraints
2. Scenario Activation Limit Constraints
3. Linking Constraints (activation → selection)
"""

using JuMP

export add_station_limit_constraint!
export add_scenario_activation_limit_constraints!
export add_activation_linking_constraints!


function _total_num_constraints(m::Model)
    total = 0
    for (F, S) in JuMP.list_of_constraint_types(m)
        total += JuMP.num_constraints(m, F, S)
    end
    return total
end


# ============================================================================
# 1. Station Limit Constraints
# ============================================================================

"""
    add_station_limit_constraint!(m::Model, data::StationSelectionData, limit::Int;
                                  equality::Bool=true)

Limit total number of stations selected.
    Σⱼ y[j] = limit  (if equality)
    Σⱼ y[j] ≤ limit  (otherwise)

Used by: All models
"""
function add_station_limit_constraint!(
    m::Model,
    data::StationSelectionData,
    limit::Int;
    equality::Bool=true
)
    before = _total_num_constraints(m)
    y = m[:y]
    if equality
        @constraint(m, station_limit, sum(y) == limit)
    else
        @constraint(m, station_limit, sum(y) <= limit)
    end
    return _total_num_constraints(m) - before
end


# ============================================================================
# 2. Scenario Activation Limit Constraints
# ============================================================================

"""
    add_scenario_activation_limit_constraints!(m::Model, data::StationSelectionData, k::Int)

Limit active stations per scenario.
    Σⱼ z[j,s] = k  ∀s

Used by: TwoStageSingleDetourModel (with or without walking limits), ClusteringTwoStageODModel
"""
function add_scenario_activation_limit_constraints!(
    m::Model,
    data::StationSelectionData,
    k::Int
)
    before = _total_num_constraints(m)
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    @constraint(m, activation_limit[s=1:S], sum(z[j,s] for j in 1:n) == k)
    return _total_num_constraints(m) - before
end


# ============================================================================
# 3. Linking Constraints
# ============================================================================

"""
    add_activation_linking_constraints!(m::Model, data::StationSelectionData)

Active stations must be built.
    z[j,s] ≤ y[j]  ∀j,s

Used by: TwoStageSingleDetourModel (with or without walking limits), ClusteringTwoStageODModel
"""
function add_activation_linking_constraints!(m::Model, data::StationSelectionData)
    before = _total_num_constraints(m)
    n = data.n_stations
    S = n_scenarios(data)
    y = m[:y]
    z = m[:z]
    @constraint(m, link_zy[j=1:n, s=1:S], z[j,s] <= y[j])
    return _total_num_constraints(m) - before
end
