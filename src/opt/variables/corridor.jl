"""
Corridor variable creation functions for corridor models (ZCorridorODModel, XCorridorODModel).

Adds cluster activation (α) and corridor usage (f) variables.
"""

using JuMP

export add_cluster_activation_variables!, add_corridor_variables!

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
