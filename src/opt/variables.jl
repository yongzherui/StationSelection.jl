"""
Variable creation functions for station selection optimization models.

These functions add decision variables to JuMP models. They are designed
to be composable - models can pick and choose which variable sets they need.

Uses multiple dispatch to provide specialized implementations for different
mapping types (PoolingScenarioOriginDestTimeMap, ClusteringScenarioODMap,
ClusteringBaseMap).

Organization:
1. Station Selection Variables (y)
2. Scenario Activation Variables (z)
3. Assignment Variables (x) - with dispatch by mapping type
4. Flow Variables (f)
5. Detour Variables (u, v)
"""

using JuMP

export add_station_selection_variables!
export add_scenario_activation_variables!
export add_assignment_variables!
export add_flow_variables!
export add_detour_variables!


# ============================================================================
# 1. Station Selection Variables (y)
# ============================================================================

"""
    add_station_selection_variables!(m::Model, data::StationSelectionData)

Add binary station selection (build) variables y[j] for j ∈ 1:n.

y[j] = 1 if station j is selected/built (permanent decision).

Used by: All models
"""
function add_station_selection_variables!(m::Model, data::StationSelectionData)
    before = JuMP.num_variables(m)
    n = data.n_stations
    @variable(m, y[1:n], Bin)
    return JuMP.num_variables(m) - before
end


# ============================================================================
# 2. Scenario Activation Variables (z)
# ============================================================================

"""
    add_scenario_activation_variables!(m::Model, data::StationSelectionData)

Add binary scenario activation variables z[j,s] for stations and scenarios.

z[j,s] = 1 if station j is activated in scenario s.
Allows different subsets of built stations to be active in each scenario.

Used by: TwoStageSingleDetourModel, ClusteringTwoStageODModel
"""
function add_scenario_activation_variables!(m::Model, data::StationSelectionData)
    before = JuMP.num_variables(m)
    n = data.n_stations
    S = n_scenarios(data)
    @variable(m, z[1:n, 1:S], Bin)
    return JuMP.num_variables(m) - before
end


# ============================================================================
# 3. Assignment Variables (x) - Multiple Dispatch by Mapping Type
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
Use x_jk_idx[s][t][od] to get the index mapping: idx → (j, k).

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

    # x_jk_idx[s][t][od] = Vector{Tuple{Int,Int}} mapping index → (j, k)
    x_jk_idx = [Dict{Int, Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}}() for _ in 1:S]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            x[s][time_id] = Dict{Tuple{Int, Int}, Vector{VariableRef}}()
            x_jk_idx[s][time_id] = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()

            for od in od_vector
                # Get valid (j, k) pairs for this OD
                valid_pairs = get_valid_jk_pairs(mapping, od[1], od[2])
                n_pairs = length(valid_pairs)

                if n_pairs > 0
                    x[s][time_id][od] = @variable(m, [1:n_pairs], Bin)
                    x_jk_idx[s][time_id][od] = valid_pairs
                else
                    x[s][time_id][od] = VariableRef[]
                    x_jk_idx[s][time_id][od] = Tuple{Int, Int}[]
                end
            end
        end
    end

    m[:x] = x
    m[:x_jk_idx] = x_jk_idx  # Index mapping for looking up (j, k) from variable index
    m[:x_is_sparse] = true   # Flag to indicate sparse structure
    return JuMP.num_variables(m) - before
end

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

    for s in 1:S
        num_od_pairs = length(mapping.Omega_s[s])
        for od_idx in 1:num_od_pairs
            x[s][od_idx] = @variable(m, [1:n, 1:n], Bin)
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end

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


# ============================================================================
# 4. Flow Variables (f)
# ============================================================================

"""
    add_flow_variables!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Add flow variables f[s][t][j,k] for each scenario, time, and station pair.

f[s][t][j,k] = 1 if there is vehicle flow from station j to k at time t in scenario s.

Used by: TwoStageSingleDetourModel
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


# ============================================================================
# 5. Detour Variables (u, v)
# ============================================================================

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

Used by: TwoStageSingleDetourModel
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

    m[:u] = u  # Store even if empty (no valid detours)
    m[:v] = v
    m[:v_idx_map] = v_idx_map

    return JuMP.num_variables(m) - before
end
