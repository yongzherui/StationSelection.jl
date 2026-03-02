"""
Flow variable creation functions for station selection optimization models.

These functions add flow decision variables that track vehicle movements
between station pairs.

Used by: TwoStageSingleDetourModel (with or without walking limits),
         XCorridorWithFlowRegularizerModel, ClusteringTwoStageODModel (with FR)
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
    S = n_scenarios(data)
    f = [Dict{Int, Any}() for _ in 1:S]

    use_sparse = has_walking_distance_limit(mapping)
    for s in 1:S
        for time_id in keys(mapping.Omega_s_t[s])
            if use_sparse
                valid_f_pairs = get_valid_f_pairs(mapping, s, time_id)
                f[s][time_id] = Dict{Tuple{Int, Int}, VariableRef}()
                for (j, k) in valid_f_pairs
                    f[s][time_id][(j, k)] = @variable(m, binary=true)
                end
            else
                n = data.n_stations
                f[s][time_id] = @variable(m, [1:n, 1:n], Bin)
            end
        end
    end

    m[:f] = f
    return JuMP.num_variables(m) - before
end


"""
    add_flow_variables!(m::Model, data::StationSelectionData, mapping::CorridorTwoStageODMap) -> Int

Add sparse per-scenario route-activation variables:
    f_flow[s][(j,k)] ∈ [0,1]
for each (j,k) in the union of valid_pairs across all OD pairs in scenario s.

Continuous relaxation is exact: minimisation + f_flow ≥ x forces values to {0,1}.

Used by: XCorridorWithFlowRegularizerModel
"""
function add_flow_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap
    )::Int
    S = n_scenarios(data)
    total = 0

    f_flow = [Dict{Tuple{Int,Int}, VariableRef}() for _ in 1:S]
    for s in 1:S
        active_pairs = Set{Tuple{Int,Int}}()
        for (o, d) in mapping.Omega_s[s]
            for (j, k) in get_valid_jk_pairs(mapping, o, d)
                push!(active_pairs, (j, k))
            end
        end
        for (j, k) in active_pairs
            f_flow[s][(j, k)] = @variable(m, lower_bound=0, upper_bound=1)
            total += 1
        end
    end

    m[:f_flow] = f_flow
    return total
end


"""
    add_flow_variables!(m::Model, data::StationSelectionData, mapping::ClusteringTwoStageODMap) -> Int

Add sparse per-scenario route-activation variables:
    f_flow[s][(j,k)] ∈ [0,1]
for each (j,k) in the union of valid_pairs across all OD pairs in scenario s.

Continuous relaxation is exact: minimisation + f_flow ≥ x forces values to {0,1}.

Used by: ClusteringTwoStageODModel (when flow_regularization_weight is set)
"""
function add_flow_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap
    )::Int
    S = n_scenarios(data)
    total = 0

    f_flow = [Dict{Tuple{Int,Int}, VariableRef}() for _ in 1:S]
    for s in 1:S
        active_pairs = Set{Tuple{Int,Int}}()
        for (o, d) in mapping.Omega_s[s]
            for (j, k) in get_valid_jk_pairs(mapping, o, d)
                push!(active_pairs, (j, k))
            end
        end
        for (j, k) in active_pairs
            f_flow[s][(j, k)] = @variable(m, lower_bound=0, upper_bound=1)
            total += 1
        end
    end

    m[:f_flow] = f_flow
    return total
end
