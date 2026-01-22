"""
Shared constraint creation functions for station selection models.

These functions add constraints to JuMP models. They are designed
to be composable - models can pick and choose which constraint sets they need.
"""
module Constraints

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

"""
    add_single_assignment_constraints!(m::Model, data::StationSelectionData)

For single-scenario models: each location must be assigned to exactly one station.
    Σⱼ x[i,j] = 1  ∀i
"""
function add_single_assignment_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    x = m[:x]
    @constraint(m, assign[i=1:n], sum(x[i,j] for j in 1:n) == 1)
    return nothing
end

"""
    add_assignment_constraints!(m::Model, data::StationSelectionData)

For multi-scenario models: each location assigned to exactly one station per scenario.
    Σⱼ x[i,j,s] = 1  ∀i,s
"""
function add_assignment_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]
    @constraint(m, assign[i=1:n, s=1:S], sum(x[i,j,s] for j in 1:n) == 1)
    return nothing
end

"""
    add_pickup_assignment_constraints!(m::Model, data::StationSelectionData)

Each pickup location assigned to exactly one station per scenario.
    Σⱼ x_pick[i,j,s] = 1  ∀i,s
"""
function add_pickup_assignment_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_pick = m[:x_pick]
    @constraint(m, pickup_assign[i=1:n, s=1:S], sum(x_pick[i,j,s] for j in 1:n) == 1)
    return nothing
end

"""
    add_dropoff_assignment_constraints!(m::Model, data::StationSelectionData)

Each dropoff location assigned to exactly one station per scenario.
    Σⱼ x_drop[i,j,s] = 1  ∀i,s
"""
function add_dropoff_assignment_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_drop = m[:x_drop]
    @constraint(m, dropoff_assign[i=1:n, s=1:S], sum(x_drop[i,j,s] for j in 1:n) == 1)
    return nothing
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

"""
    add_assignment_to_selected_constraints!(m::Model, data::StationSelectionData)

For single-scenario: can only assign to selected stations.
    x[i,j] ≤ y[j]  ∀i,j
"""
function add_assignment_to_selected_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    x = m[:x]
    y = m[:y]
    @constraint(m, link_xy[i=1:n, j=1:n], x[i,j] <= y[j])
    return nothing
end

"""
    add_assignment_to_active_constraints!(m::Model, data::StationSelectionData)

For multi-scenario: can only assign to active stations.
    x[i,j,s] ≤ z[j,s]  ∀i,j,s
"""
function add_assignment_to_active_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]
    z = m[:z]
    @constraint(m, link_xz[i=1:n, j=1:n, s=1:S], x[i,j,s] <= z[j,s])
    return nothing
end

"""
    add_pickup_to_active_constraints!(m::Model, data::StationSelectionData)

Pickup assignments only to active stations.
    x_pick[i,j,s] ≤ z[j,s]  ∀i,j,s
"""
function add_pickup_to_active_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_pick = m[:x_pick]
    z = m[:z]
    @constraint(m, link_pick_z[i=1:n, j=1:n, s=1:S], x_pick[i,j,s] <= z[j,s])
    return nothing
end

"""
    add_dropoff_to_active_constraints!(m::Model, data::StationSelectionData)

Dropoff assignments only to active stations.
    x_drop[i,j,s] ≤ z[j,s]  ∀i,j,s
"""
function add_dropoff_to_active_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_drop = m[:x_drop]
    z = m[:z]
    @constraint(m, link_drop_z[i=1:n, j=1:n, s=1:S], x_drop[i,j,s] <= z[j,s])
    return nothing
end

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

# ============================================================================
# Flow Conservation Constraints (Transportation Problem)
# ============================================================================

"""
    add_flow_supply_constraints!(m::Model, data::StationSelectionData)

Add supply expressions and flow conservation: outflow = supply at each station.
Supply (p[j,s]) = total pickups assigned to station j in scenario s.

Requires: x_pick variables and pickup_counts in scenario data.
"""
function add_flow_supply_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_pick = m[:x_pick]
    f = m[:f]

    # Create supply expression: sum of pickups assigned to each station
    @expression(m, p[j=1:n, s=1:S],
        sum(data.scenarios[s].pickup_counts[get_station_id(data, i)] * x_pick[i,j,s]
            for i in 1:n))

    # Outflow from j equals supply at j
    @constraint(m, flow_supply[j=1:n, s=1:S],
        sum(f[j,k,s] for k in 1:n) == p[j,s])

    return nothing
end

"""
    add_flow_demand_constraints!(m::Model, data::StationSelectionData)

Add demand expressions and flow conservation: inflow = demand at each station.
Demand (d[k,s]) = total dropoffs assigned to station k in scenario s.

Requires: x_drop variables and dropoff_counts in scenario data.
"""
function add_flow_demand_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_drop = m[:x_drop]
    f = m[:f]

    # Create demand expression: sum of dropoffs assigned to each station
    @expression(m, d[k=1:n, s=1:S],
        sum(data.scenarios[s].dropoff_counts[get_station_id(data, i)] * x_drop[i,k,s]
            for i in 1:n))

    # Inflow to k equals demand at k
    @constraint(m, flow_demand[k=1:n, s=1:S],
        sum(f[j,k,s] for j in 1:n) == d[k,s])

    return nothing
end

end # module
