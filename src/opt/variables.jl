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

function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )

    S = n_scenarios(data)
    x = [Dict{Int, Dict{Tuple{Int, Int}, Matrix{VariableRef}}}() for _ in 1:S]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            x[s][time_id] = Dict{Tuple{Int, Int}, Matrix{VariableRef}}()

            for od in od_vector
                x[s][time_id][od] = @variable(m, [1:n, 1:n], Bin)
            end
        end
    end


    m[:x] = x
    return nothing
end

function add_flow_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )

    n = data.n_stations
    S = n_scenarios(data)
    f = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for time_id in keys(mapping.Omega_s_t[s])
            # we want the length of the Omega_s_t[s][time_id]
            od_count = length(Omega_s_t[s][time_id])

            f[s][time_id] = @variable(m, [1:n, 1:n], Bin)
        end
    end

    m[:f] = f
    return nothing
end

function add_detour_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    # we need to do calculations to know the potential route combinations
    
    S = n_scenarios(data)


    # for the single source detour
    # we run through each time id and check if the combination exists
    u = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]
    v = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for time_id in keys(mapping.Omega_s_t[s])
            # we want to identify if there are combinations we can form here
            same_source_combinations = mapping.Xi_same_source[s][time_id]
            # add the same source variables
            u[s][time_id] = @variable(m, [1:length(same_source_combinations)], Bin)

            same_dest_combinations = mapping.Xi_same_dest[s][time_id]
            # add the ame dest variables
            v[s][time_id] = @variable(m, [1:length(same_dest_combinations)], Bin)
        end
    end



    m[:v] = v
    return nothing
end
