"""
Assignment variable creation functions for station selection optimization models.

These functions add assignment decision variables that map requests/OD pairs
to station pairs.

Uses multiple dispatch to provide specialized implementations for different
mapping types.
"""

using JuMP

export add_assignment_variables!
export add_assignment_variables_with_walking_distance_limit!


# ============================================================================
# PoolingScenarioOriginDestTimeMap (TwoStageSingleDetourModel with walking limit)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Add assignment variables x[s][t][od][j,k] for TwoStageSingleDetourModel.

x[s][t][od][j,k] = 1 if OD request (o,d) at time t in scenario s is assigned
to use stations j (pickup) and k (dropoff).

Structure: scenario → time → OD pair → (pickup, dropoff) matrix
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
    add_assignment_variables_with_walking_distance_limit!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )

Add assignment variables x[s][t][od] only for valid (j,k) pairs based on walking distance.

This is a sparse version of add_assignment_variables! that only creates variables for
station pairs where:
- The walking distance from origin o to pickup station j ≤ max_walking_distance
- The walking distance from dropoff station k to destination d ≤ max_walking_distance

x[s][t][od] is a Vector{VariableRef} of length |valid_jk_pairs|.
Use mapping.valid_jk_pairs[(o,d)] to get the index mapping: idx → (j, k).

Requires that mapping.valid_jk_pairs is populated (i.e., max_walking_distance was set
when creating the mapping).
"""
function add_assignment_variables_with_walking_distance_limit!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    # x[s][t][od] = Vector{VariableRef} of length |valid_jk_pairs|
    x = [Dict{Int, Dict{Tuple{Int, Int}, Vector{VariableRef}}}() for _ in 1:S]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            x[s][time_id] = Dict{Tuple{Int, Int}, Vector{VariableRef}}()

            for od in od_vector
                # Get valid (j, k) pairs for this OD from mapping
                valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                n_pairs = length(valid_pairs)

                if n_pairs > 0
                    x[s][time_id][od] = @variable(m, [1:n_pairs], Bin)
                else
                    x[s][time_id][od] = VariableRef[]
                end
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end


# ============================================================================
# PoolingScenarioOriginDestTimeMapNoWalkingLimit (TwoStageSingleDetourNoWalkingLimitModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit)

Add assignment variables x[s][t][od][j,k] for TwoStageSingleDetourNoWalkingLimitModel.

x[s][t][od][j,k] = 1 if OD request (o,d) at time t in scenario s is assigned
to use stations j (pickup) and k (dropoff).

Structure: scenario → time → OD pair → (pickup, dropoff) matrix
All (j,k) pairs are valid (no walking distance limit).
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMapNoWalkingLimit
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


# ============================================================================
# ClusteringScenarioODMap (ClusteringTwoStageODModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::ClusteringScenarioODMap)

Add assignment variables x[s][od_idx][j,k] for ClusteringTwoStageODModel.

x[s][od_idx][j,k] = 1 if OD pair od_idx in scenario s is assigned to use
stations j (pickup) and k (dropoff).

Structure: scenario → OD index → (pickup, dropoff) matrix
No time dimension - OD pairs are aggregated across time within each scenario.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringScenarioODMap
    )
    before = JuMP.num_variables(m)
    n = data.n_stations
    S = n_scenarios(data)
    x = [Dict{Int, Matrix{VariableRef}}() for _ in 1:S]

    use_sparse = has_walking_distance_limit(mapping)
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


"""
    add_assignment_variables_with_walking_distance_limit!(m::Model, data::StationSelectionData, mapping::ClusteringScenarioODMap)

Add assignment variables x[s][od_idx] only for valid (j,k) pairs based on walking distance.

x[s][od_idx] is a Vector{VariableRef} of length |valid_jk_pairs|.
Use mapping.valid_jk_pairs[(o,d)] to get the index mapping: idx → (j, k).
"""
function add_assignment_variables_with_walking_distance_limit!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringScenarioODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
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

    m[:x] = x
    return JuMP.num_variables(m) - before
end


# ============================================================================
# ClusteringBaseMap (ClusteringBaseModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::ClusteringBaseMap)

Add assignment variables x[i,j] for ClusteringBaseModel.

x[i,j] = 1 if station location i is assigned to medoid station j.

Structure: Simple n×n matrix (station-to-station assignment)
No scenario, time, or OD dimensions - all aggregated.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseMap
    )
    before = JuMP.num_variables(m)
    n = mapping.n_stations
    @variable(m, x[1:n, 1:n], Bin)
    m[:x] = x
    return JuMP.num_variables(m) - before
end
