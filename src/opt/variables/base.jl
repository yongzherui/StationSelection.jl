"""
Base variable creation functions for station selection optimization models.

These functions add the fundamental decision variables that are shared across
most model types.

Variables:
1. Station Selection Variables (y) - build decisions
2. Scenario Activation Variables (z) - per-scenario activation
"""

using JuMP

export add_station_selection_variables!
export add_scenario_activation_variables!


"""
    add_station_selection_variables!(m::Model, data::StationSelectionData)

Add binary station selection (build) variables y[j] for j ∈ 1:n.

y[j] = 1 if station j is selected/built (permanent decision).

Used by: All models
"""
function add_station_selection_variables!(m::Model, data::StationSelectionData)
    before = JuMP.num_variables(m)
    n = data.n_stations
    @variable(m, y[1:n], Bin)
    return JuMP.num_variables(m) - before
end


"""
    add_scenario_activation_variables!(m::Model, data::StationSelectionData)

Add binary scenario activation variables z[j,s] for stations and scenarios.

z[j,s] = 1 if station j is activated in scenario s.
Allows different subsets of built stations to be active in each scenario.

Used by: TwoStageSingleDetourModel (with or without walking limits), ClusteringTwoStageODModel
"""
function add_scenario_activation_variables!(m::Model, data::StationSelectionData)
    before = JuMP.num_variables(m)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, z[1:n, 1:S], Bin)
    return JuMP.num_variables(m) - before
end
