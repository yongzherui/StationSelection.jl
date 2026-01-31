"""
Flow variable creation functions for station selection optimization models.

These functions add flow decision variables that track vehicle movements
between station pairs.

Used by: TwoStageSingleDetourModel (with or without walking limits)
"""

using JuMP

export add_flow_variables!


"""
    add_flow_variables!(m::Model, data::StationSelectionData, mapping::TwoStageSingleDetourMap)

Add flow variables f[s][t][j,k] for each scenario, time, and station pair.

f[s][t][j,k] = 1 if there is vehicle flow from station j to k at time t in scenario s.

Used by: TwoStageSingleDetourModel
"""
function add_flow_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap
    )
    before = JuMP.num_variables(m)
    n = data.n_stations
    S = n_scenarios(data)
    f = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for time_id in keys(mapping.Omega_s_t[s])
            f[s][time_id] = @variable(m, [1:n, 1:n], Bin)
        end
    end

    m[:f] = f
    return JuMP.num_variables(m) - before
end

