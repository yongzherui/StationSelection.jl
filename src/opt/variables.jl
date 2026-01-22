"""
Shared variable creation functions for station selection models.

These functions add decision variables to JuMP models. They are designed
to be composable - models can pick and choose which variable sets they need.
"""
module Variables

using JuMP
using ..DataStructs: StationSelectionData, n_scenarios

export add_station_selection_variables!
export add_assignment_variables!
export add_scenario_activation_variables!
export add_pickup_assignment_variables!
export add_dropoff_assignment_variables!
export add_flow_variables!

"""
    add_station_selection_variables!(m::Model, data::StationSelectionData)

Add binary station selection (build) variables y[j] for j ∈ 1:n.

y[j] = 1 if station j is selected/built (permanent decision).
"""
function add_station_selection_variables!(m::Model, data::StationSelectionData)
    n = data.n_stations
    @variable(m, y[1:n], Bin)
    return nothing
end

"""
    add_assignment_variables!(m::Model, data::StationSelectionData)

Add binary assignment variables x[i,j,s] for all locations, stations, and scenarios.

x[i,j,s] = 1 if location i is assigned to station j in scenario s.
Used for combined pickup/dropoff assignment (when they're treated identically).
"""
function add_assignment_variables!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, x[1:n, 1:n, 1:S], Bin)
    return nothing
end

"""
    add_assignment_variables_single_scenario!(m::Model, data::StationSelectionData)

Add binary assignment variables x[i,j] for single-scenario models.

x[i,j] = 1 if location i is assigned to station j.
"""
function add_assignment_variables_single_scenario!(m::Model, data::StationSelectionData)
    n = data.n_stations
    @variable(m, x[1:n, 1:n], Bin)
    return nothing
end

"""
    add_scenario_activation_variables!(m::Model, data::StationSelectionData)

Add binary scenario activation variables z[j,s] for stations and scenarios.

z[j,s] = 1 if station j is activated in scenario s.
Allows different subsets of built stations to be active in each scenario.
"""
function add_scenario_activation_variables!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, z[1:n, 1:S], Bin)
    return nothing
end

"""
    add_pickup_assignment_variables!(m::Model, data::StationSelectionData)

Add binary pickup assignment variables x_pick[i,j,s].

x_pick[i,j,s] = 1 if pickup location i is assigned to station j in scenario s.
Used when pickup and dropoff assignments are handled separately.
"""
function add_pickup_assignment_variables!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, x_pick[1:n, 1:n, 1:S], Bin)
    return nothing
end

"""
    add_dropoff_assignment_variables!(m::Model, data::StationSelectionData)

Add binary dropoff assignment variables x_drop[i,j,s].

x_drop[i,j,s] = 1 if dropoff location i is assigned to station j in scenario s.
Used when pickup and dropoff assignments are handled separately.
"""
function add_dropoff_assignment_variables!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, x_drop[1:n, 1:n, 1:S], Bin)
    return nothing
end

"""
    add_flow_variables!(m::Model, data::StationSelectionData)

Add continuous flow variables f[j,k,s] for vehicle routing.

f[j,k,s] = number of passengers/vehicles flowing from station j to station k
in scenario s. Used for transportation problem formulation.
"""
function add_flow_variables!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, f[1:n, 1:n, 1:S] >= 0)
    return nothing
end

export add_assignment_variables_single_scenario!

end # module
