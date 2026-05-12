"""
Objective function for ClusteringTwoStageStationModel.
"""

using JuMP

export set_clustering_two_stage_station_objective!

function set_clustering_two_stage_station_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageStationMap
    )
    S = n_scenarios(data)
    x = m[:x]

    @objective(m, Min,
        sum(
            mapping.q_s[s][i] * get_walking_cost(data, i, j) * x[s][i_idx][j_idx]
            for s in 1:S
            for (i_idx, i) in enumerate(mapping.I_s[s])
            for (j_idx, j) in enumerate(get_valid_j_assignments(mapping, i))
        )
    )

    return nothing
end
