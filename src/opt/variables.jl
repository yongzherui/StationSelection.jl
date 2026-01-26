"""
Variable creation functions for station selection optimization models.

These functions add decision variables to JuMP models. They are designed
to be composable - models can pick and choose which variable sets they need.
"""

using JuMP

export add_station_selection_variables!
export add_scenario_activation_variables!
export add_assignment_variables!
export add_flow_variables!
export add_detour_variables!

"""
    add_station_selection_variables!(m::Model, data::StationSelectionData)

Add binary station selection (build) variables y[j] for j ∈ 1:n.

y[j] = 1 if station j is selected/built (permanent decision).
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
"""
function add_scenario_activation_variables!(m::Model, data::StationSelectionData)
    before = JuMP.num_variables(m)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, z[1:n, 1:S], Bin)
    return JuMP.num_variables(m) - before
end

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Add assignment variables x[s][t][od][j,k] for each scenario, time, OD pair, and station pair.

x[s][t][od][j,k] = 1 if OD request (o,d) at time t in scenario s is assigned to use
stations j (pickup) and k (dropoff).
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    before = JuMP.num_variables(m)
    n = data.n_stations
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
    return JuMP.num_variables(m) - before
end

"""
    add_flow_variables!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Add flow variables f[s][t][j,k] for each scenario, time, and station pair.

f[s][t][j,k] = 1 if there is vehicle flow from station j to k at time t in scenario s.
"""
function add_flow_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
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

"""
    add_detour_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )

Add detour pooling variables for same-source and same-destination pooling.

Variables:
- u[s][t][idx] = 1 if same-source pooling for triplet Xi_same_source[idx] is used
  at time t in scenario s. Triplet (j,k,l) means pooling trips (j→l) and (k→l).

- v[s][t][idx] = 1 if same-dest pooling for a valid quadruplet is used
  at time t in scenario s. Quadruplet (j,k,l,t') means pooling trips (j→l) and (j→k),
  where the second leg (k→l) occurs at time t+t'.

Note: Xi_same_source and Xi_same_dest are computed externally via:
- find_same_source_detour_combinations(model, data)
- find_same_dest_detour_combinations(model, data)
"""
function add_detour_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    n_same_source = length(Xi_same_source)

    # Same-source pooling variables: u[s][t][idx]
    # Indexed by scenario, time, and triplet index
    u = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    # Same-dest pooling variables: v[s][t][idx]
    v = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]
    v_idx_map = [Dict{Int, Vector{Int}}() for _ in 1:S]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # Create variables for each triplet in Xi_same_source
            if length(od_vector) > 1 && n_same_source > 0
                u[s][time_id] = @variable(m, [1:n_same_source], Bin)
            else
                u[s][time_id] = VariableRef[]
            end

            valid_indices = Int[]
            for (idx, (_, _, _, time_delta)) in enumerate(Xi_same_dest)
                future_time_id = time_id + time_delta
                if haskey(mapping.Omega_s_t[s], future_time_id)
                    push!(valid_indices, idx)
                end
            end

            v_idx_map[s][time_id] = valid_indices
            if !isempty(valid_indices)
                v[s][time_id] = @variable(m, [1:length(valid_indices)], Bin)
            else
                v[s][time_id] = VariableRef[]
            end
        end
    end

    m[:u] = u
    m[:v] = v
    m[:v_idx_map] = v_idx_map

    return JuMP.num_variables(m) - before
end
