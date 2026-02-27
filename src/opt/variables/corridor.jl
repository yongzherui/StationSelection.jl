"""
Corridor variable creation functions for corridor models (ZCorridorODModel, XCorridorODModel).

Adds cluster activation (α) and corridor usage (f) variables.
"""

using JuMP

export add_cluster_activation_variables!, add_corridor_variables!
export add_route_activation_variables!

"""
    add_cluster_activation_variables!(m::Model, data::StationSelectionData,
                                      mapping::CorridorTwoStageODMap) -> Int

Add cluster activation variables α[a, s] ∈ [0,1] (continuous).

α[a,s] = 1 if any station in cluster a is active in scenario s.
Continuous relaxation suffices due to integrality of z.
"""
function add_cluster_activation_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    n_clusters = mapping.n_clusters

    @variable(m, 0 <= α[1:n_clusters, 1:S] <= 1)
    m[:α] = α

    return JuMP.num_variables(m) - before
end

"""
    add_corridor_variables!(m::Model, data::StationSelectionData,
                            mapping::CorridorTwoStageODMap) -> Int

Add corridor usage variables f_corridor[g, s] ∈ {0,1} (binary).

f_corridor[g,s] = 1 if corridor g is used in scenario s.
"""
function add_corridor_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::CorridorTwoStageODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    n_corridors = length(mapping.corridor_indices)

    @variable(m, f_corridor[1:n_corridors, 1:S], Bin)
    m[:f_corridor] = f_corridor

    return JuMP.num_variables(m) - before
end

"""
    add_route_activation_variables!(m, data, mapping) -> Int

Sparse per-scenario route-activation variables:
    w_route[s][(j,k)] ∈ [0,1]
for each (j,k) in the union of valid_pairs across all OD pairs in scenario s.

Storage: Vector{Dict{Tuple{Int,Int}, VariableRef}} of length S, stored as m[:w_route].
Continuous relaxation is exact: minimisation + w_route ≥ x forces values to {0,1}.
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
