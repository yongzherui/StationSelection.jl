"""
Flow variable creation functions for station selection optimization models.

These functions add flow decision variables that track vehicle movements
between station pairs.

Used by: TwoStageODPolicy (with flow_regularization_weight)
"""

using JuMP

export add_flow_variables!


"""
    add_flow_variables!(m::Model, data::StationSelectionData, mapping::ClusteringTwoStageODMap) -> Int

Add sparse per-scenario route-activation variables:
    f_flow[s][(j,k)] ∈ {0,1}
for each (j,k) in the union of valid_pairs across all OD pairs in scenario s.

Used by: TwoStageODPolicy (when flow_regularization_weight is set)
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
            for pair in get_valid_jk_pairs(mapping, o, d)
                # Walk-only trips use no vehicle route, so they get no flow variable.
                is_walk_only_pair(pair) && continue
                push!(active_pairs, pair)
            end
        end
        for (j, k) in active_pairs
            f_flow[s][(j, k)] = @variable(m, binary = true)
            total += 1
        end
    end

    m[:f_flow] = f_flow
    return total
end
