
"""
Shared constraint creation functions for station selection models.

These functions add constraints to JuMP models. They are designed
to be composable - models can pick and choose which constraint sets they need.
"""

using JuMP
using ..DataStructs: StationSelectionData, n_scenarios, get_station_id

export add_single_assignment_constraints!
export add_assignment_constraints!
export add_pickup_assignment_constraints!
export add_dropoff_assignment_constraints!
export add_station_limit_constraint!
export add_scenario_activation_limit_constraints!
export add_assignment_to_selected_constraints!
export add_assignment_to_active_constraints!
export add_pickup_to_active_constraints!
export add_dropoff_to_active_constraints!
export add_activation_linking_constraints!
export add_flow_supply_constraints!
export add_flow_demand_constraints!

# ============================================================================
# Assignment Constraints
# ============================================================================

function add_assignment_constraints!(
        m::Model, 
        model::AbstractPoolingModel, 
        data::,
        mapping::PoolingScenarioOriginDestTimeMap)
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                @constraint(m, sum(x[s][time_id][od]) == 1)
            end
        end
    end

end


# ============================================================================
# Station Selection/Activation Limits
# ============================================================================

"""
    add_station_limit_constraint!(m::Model, data::StationSelectionData, limit::Int;
                                  equality::Bool=true)

Limit total number of stations selected.
    Σⱼ y[j] = limit  (if equality)
    Σⱼ y[j] ≤ limit  (otherwise)
"""
function add_station_limit_constraint!(
    m::Model,
    data::StationSelectionData,
    limit::Int;
    equality::Bool=true
)
    y = m[:y]
    if equality
        @constraint(m, station_limit, sum(y) == limit)
    else
        @constraint(m, station_limit, sum(y) <= limit)
    end
    return nothing
end

"""
    add_scenario_activation_limit_constraints!(m::Model, data::StationSelectionData, k::Int)

Limit active stations per scenario.
    Σⱼ z[j,s] = k  ∀s
"""
function add_scenario_activation_limit_constraints!(
    m::Model,
    data::StationSelectionData,
    k::Int
)
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    @constraint(m, activation_limit[s=1:S], sum(z[j,s] for j in 1:n) == k)
    return nothing
end

# ============================================================================
# Linking Constraints
# ============================================================================
#
"""
    add_activation_linking_constraints!(m::Model, data::StationSelectionData)

Active stations must be built.
    z[j,s] ≤ y[j]  ∀j,s
"""
function add_activation_linking_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    y = m[:y]
    z = m[:z]
    @constraint(m, link_zy[j=1:n, s=1:S], z[j,s] <= y[j])
    return nothing
end

function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )

    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    # for each scenario
    for s in 1:S
        # at each time
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # for each od
            for od in od_vector
                # we need to make sure the linking is correct
                for j in 1:n, k in 1:n
                    @constraint(m, 2 * x[s][time_id][od][j, k] <= z[j, s] + z[k, s])
                end
            end
        end
    end

    return nothing
end

# FLOW CONSTRAINTS
#

function add_assignment_to_flow_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )

    n = data.n_stations
    S = n_scenarios(data)
    f = m[:f]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector

                @constraint(m, [j in 1:n, k in 1:n], x[s][time_id][od][j, k] <= f[s][time_id][j, k])
            end
        end
    end

    return nothing
end

####
# DETOUR CONSTRAINTS
#####

function add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    n = data.n_stations
    S = n_scenarios(data)
    u = m[:u]
    x = m[:x]

    for s in 1:S
        for (time_id, jkl_combinations) in mapping.Xi_same_source_s_t[s]
            for (j, k, l) in jkl_combinations

                @constraint(m, [j in 1:n, k in 1:n], u[s][time_id][od][j, k] <= x[s][time_id][od])
            end
        end
    end
    return nothing
end

function add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )

    n = data.n_stations
    S = n_scenarios(data)
    v = m[:v]
    x = m[:x]

    for s in 1:S
        for (time_id, jklt_combinations) in mapping.Xi_same_dest_s_t[s]
            for (j, k, l, t) in jkl_combinations

                @constraint(m, [j in 1:n, k in 1:n], x[s][time_id][od][j, k] <= f[s][time_id][j, k])
            end
        end
    end

    return nothing
end
