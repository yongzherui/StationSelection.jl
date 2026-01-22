"""
Shared objective function components for station selection models.

These functions create objective expressions that can be combined.
Models typically call multiple functions and sum the expressions.
"""
module Objectives

using JuMP
using ..DataStructs: StationSelectionData, n_scenarios, get_station_id

export create_walking_cost_expression_single!
export create_walking_cost_expression!
export create_pickup_walking_cost_expression!
export create_dropoff_walking_cost_expression!
export create_routing_cost_expression!
export create_activation_penalty_expression!
export set_minimize_objective!

# ============================================================================
# Single Scenario Objectives
# ============================================================================

"""
    create_walking_cost_expression_single!(m::Model, data::StationSelectionData)

Create walking cost expression for single-scenario models.
Uses total_counts from the first (only) scenario.

Expression: Σᵢⱼ rᵢ × cᵢⱼ × x[i,j]

Returns the expression and also registers it as m[:walking_cost].
"""
function create_walking_cost_expression_single!(m::Model, data::StationSelectionData)
    n = data.n_stations
    x = m[:x]
    scenario = data.scenarios[1]

    @expression(m, walking_cost,
        sum(scenario.total_counts[get_station_id(data, i)] *
            data.walking_costs[(get_station_id(data, i), get_station_id(data, j))] *
            x[i,j]
            for i in 1:n, j in 1:n))

    return m[:walking_cost]
end

# ============================================================================
# Multi-Scenario Objectives
# ============================================================================

"""
    create_walking_cost_expression!(m::Model, data::StationSelectionData)

Create walking cost expression for multi-scenario models.
Uses total_counts (combined pickup + dropoff) from each scenario.

Expression: Σᵢⱼₛ rᵢₛ × cᵢⱼ × x[i,j,s]

Returns the expression and also registers it as m[:walking_cost].
"""
function create_walking_cost_expression!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]

    @expression(m, walking_cost,
        sum(data.scenarios[s].total_counts[get_station_id(data, i)] *
            data.walking_costs[(get_station_id(data, i), get_station_id(data, j))] *
            x[i,j,s]
            for i in 1:n, j in 1:n, s in 1:S))

    return m[:walking_cost]
end

"""
    create_pickup_walking_cost_expression!(m::Model, data::StationSelectionData)

Create walking cost expression for pickup assignments only.
Uses pickup_counts from each scenario.

Expression: Σᵢⱼₛ pᵢₛ × wᵢⱼ × x_pick[i,j,s]

Returns the expression and also registers it as m[:pickup_walking_cost].
"""
function create_pickup_walking_cost_expression!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_pick = m[:x_pick]

    @expression(m, pickup_walking_cost,
        sum(data.scenarios[s].pickup_counts[get_station_id(data, i)] *
            data.walking_costs[(get_station_id(data, i), get_station_id(data, j))] *
            x_pick[i,j,s]
            for i in 1:n, j in 1:n, s in 1:S))

    return m[:pickup_walking_cost]
end

"""
    create_dropoff_walking_cost_expression!(m::Model, data::StationSelectionData)

Create walking cost expression for dropoff assignments only.
Uses dropoff_counts from each scenario.

Expression: Σᵢⱼₛ dᵢₛ × wᵢⱼ × x_drop[i,j,s]

Returns the expression and also registers it as m[:dropoff_walking_cost].
"""
function create_dropoff_walking_cost_expression!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    x_drop = m[:x_drop]

    @expression(m, dropoff_walking_cost,
        sum(data.scenarios[s].dropoff_counts[get_station_id(data, i)] *
            data.walking_costs[(get_station_id(data, i), get_station_id(data, j))] *
            x_drop[i,j,s]
            for i in 1:n, j in 1:n, s in 1:S))

    return m[:dropoff_walking_cost]
end

"""
    create_routing_cost_expression!(m::Model, data::StationSelectionData;
                                     weight::Float64=1.0)

Create routing cost expression based on flow variables.
Requires routing_costs in data and flow variables f[j,k,s].

Expression: λ × Σⱼₖₛ rⱼₖ × f[j,k,s]

Returns the expression and also registers it as m[:routing_cost].
"""
function create_routing_cost_expression!(
    m::Model,
    data::StationSelectionData;
    weight::Float64=1.0
)
    isnothing(data.routing_costs) && error("Routing costs not available in data")

    n = data.n_stations
    S = n_scenarios(data)
    f = m[:f]

    @expression(m, routing_cost,
        weight * sum(data.routing_costs[(get_station_id(data, j), get_station_id(data, k))] *
                     f[j,k,s]
                     for j in 1:n, k in 1:n, s in 1:S))

    return m[:routing_cost]
end

"""
    create_activation_penalty_expression!(m::Model, data::StationSelectionData;
                                          lambda::Float64)

Create penalty expression for scenario activations.
Encourages using fewer total activations across all scenarios.

Expression: λ × Σⱼₛ z[j,s]

Returns the expression and also registers it as m[:activation_penalty].
"""
function create_activation_penalty_expression!(
    m::Model,
    data::StationSelectionData;
    lambda::Float64
)
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]

    @expression(m, activation_penalty,
        lambda * sum(z[j,s] for j in 1:n, s in 1:S))

    return m[:activation_penalty]
end

# ============================================================================
# Objective Setting
# ============================================================================

"""
    set_minimize_objective!(m::Model, expressions...)

Set the model objective to minimize the sum of given expressions.
"""
function set_minimize_objective!(m::Model, expressions...)
    @objective(m, Min, sum(expressions))
    return nothing
end

end # module
