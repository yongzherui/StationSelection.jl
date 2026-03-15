"""
Assignment variable creation functions for station selection optimization models.

These functions add assignment decision variables that map requests/OD pairs
to station pairs.

Uses multiple dispatch to provide specialized implementations for different
mapping types.
"""

using JuMP

export add_assignment_variables!


# ============================================================================
# ClusteringTwoStageODMap (ClusteringTwoStageODModel)
# ============================================================================

"""
    add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )

Add assignment variables x[s][od_idx][j,k] for ClusteringTwoStageODModel.

x[s][od_idx][j,k] = 1 if OD pair od_idx in scenario s is assigned to use
stations j (pickup) and k (dropoff).

Structure: scenario → OD index → (pickup, dropoff) matrix (dense) or vector (sparse)
No time dimension - OD pairs are aggregated across time within each scenario.
When `variable_reduction=true` and a walking limit is enabled, sparse variables are used.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
        variable_reduction::Bool=true
    )
    before = JuMP.num_variables(m)
    n = data.n_stations
    S = n_scenarios(data)
    x = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]

    use_sparse = variable_reduction && has_walking_distance_limit(mapping)
    if use_sparse
        x = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]
        for s in 1:S
            for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                n_pairs = length(valid_pairs)
                if n_pairs > 0
                    x[s][od_idx] = @variable(m, [1:n_pairs], Bin)
                else
                    x[s][od_idx] = VariableRef[]
                end
            end
        end
    else
        for s in 1:S
            num_od_pairs = length(mapping.Omega_s[s])
            for od_idx in 1:num_od_pairs
                x[s][od_idx] = @variable(m, [1:n, 1:n], Bin)
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end


# ============================================================================
# ClusteringBaseModelMap (ClusteringBaseModel)
# ============================================================================

# ============================================================================
# TwoStageRouteODMap (TwoStageRouteModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::TwoStageRouteODMap)

Add sparse binary assignment variables x[s][t_id][od_idx] for TwoStageRouteModel.

For each OD pair (o,d) in time window t of scenario s, one binary variable is created
per valid (j,k) pair in `mapping.valid_jk_pairs[(o,d)]`.

Structure: `m[:x]` is a `Vector{Dict{Int, Dict{Int, Vector{VariableRef}}}}` indexed
by scenario → time_id → od_idx → pair variables.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageRouteODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Dict{Int, Vector{VariableRef}}}() for _ in 1:S]

    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            x[s][t_id] = Dict{Int, Vector{VariableRef}}()
            for (od_idx, (o, d)) in enumerate(od_pairs)
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                n_pairs = length(valid_pairs)
                if n_pairs > 0
                    x[s][t_id][od_idx] = @variable(m, [1:n_pairs], Bin)
                else
                    x[s][t_id][od_idx] = VariableRef[]
                end
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end


# ============================================================================
# ClusteringBaseModelMap (ClusteringBaseModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::ClusteringBaseModelMap)

Add assignment variables x[i,j] for ClusteringBaseModel.

x[i,j] = 1 if station location i is assigned to medoid station j.

Structure: Simple n×n matrix (station-to-station assignment)
No scenario, time, or OD dimensions - all aggregated.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = JuMP.num_variables(m)
    n = mapping.n_stations
    @variable(m, x[1:n, 1:n], Bin)
    m[:x] = x
    return JuMP.num_variables(m) - before
end
