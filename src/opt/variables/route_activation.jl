"""
Route activation variable creation functions.

Adds sparse per-scenario route-activation variables w_route[s][(j,k)] ∈ [0,1]
used by the flow regularization penalty. These are distinct from corridor
variables and apply to both corridor-based and clustering-based models.

Supported mappings:
- CorridorTwoStageODMap  (XCorridorWithFlowRegularizerModel)
- ClusteringTwoStageODMap (ClusteringTwoStageODModel with flow_regularization_weight)
"""

using JuMP

export add_route_activation_variables!

"""
    add_route_activation_variables!(m, data, mapping) -> Int

Sparse per-scenario route-activation variables:
    w_route[s][(j,k)] ∈ [0,1]
for each (j,k) in the union of valid_pairs across all OD pairs in scenario s.

Storage: Vector{Dict{Tuple{Int,Int}, VariableRef}} of length S, stored as m[:w_route].
Continuous relaxation is exact: minimisation + w_route ≥ x forces values to {0,1}.

Used by: XCorridorWithFlowRegularizerModel (CorridorTwoStageODMap),
         ClusteringTwoStageODModel with flow_regularization_weight (ClusteringTwoStageODMap).
"""
function add_route_activation_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap
    )::Int
    S = n_scenarios(data)
    total = 0

    w_route = [Dict{Tuple{Int,Int}, VariableRef}() for _ in 1:S]
    for s in 1:S
        # Union of valid (j,k) pairs across all OD pairs in this scenario
        active_pairs = Set{Tuple{Int,Int}}()
        for (o, d) in mapping.Omega_s[s]
            for (j, k) in get_valid_jk_pairs(mapping, o, d)
                push!(active_pairs, (j, k))
            end
        end
        for (j, k) in active_pairs
            w_route[s][(j, k)] = @variable(m, lower_bound=0, upper_bound=1)
            total += 1
        end
    end

    m[:w_route] = w_route
    return total
end

function add_route_activation_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap
    )::Int
    S = n_scenarios(data)
    total = 0

    w_route = [Dict{Tuple{Int,Int}, VariableRef}() for _ in 1:S]
    for s in 1:S
        active_pairs = Set{Tuple{Int,Int}}()
        for (o, d) in mapping.Omega_s[s]
            for (j, k) in get_valid_jk_pairs(mapping, o, d)
                push!(active_pairs, (j, k))
            end
        end
        for (j, k) in active_pairs
            w_route[s][(j, k)] = @variable(m, lower_bound=0, upper_bound=1)
            total += 1
        end
    end

    m[:w_route] = w_route
    return total
end
