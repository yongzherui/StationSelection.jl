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
    )

Add assignment variables x[s][od_idx][pair_idx] for ClusteringTwoStageODModel.

x[s][od_idx][pair_idx] is the integer passenger count from OD pair od_idx in
scenario s assigned to the corresponding valid pickup/dropoff pair.

Structure: scenario → OD index → sparse vector over valid (pickup, dropoff) pairs.
No time dimension - OD pairs are aggregated across time within each scenario.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            n_pairs = length(valid_pairs)
            demand = get(mapping.Q_s[s], (o, d), 0)
            if n_pairs > 0 && demand > 0
                x[s][od_idx] = @variable(m, [1:n_pairs],
                    integer = true, lower_bound = 0, upper_bound = demand)
            else
                x[s][od_idx] = VariableRef[]
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end

function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::AggregateODRouteMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            demand = get(mapping.Q_s[s], (o, d), 0)
            if !isempty(valid_pairs) && demand > 0
                x[s][od_idx] = @variable(m, [1:length(valid_pairs)],
                    integer = true, lower_bound = 0, upper_bound = 1)
            else
                x[s][od_idx] = VariableRef[]
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
# ClusteringTwoStageStationMap (ClusteringTwoStageStationModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::ClusteringTwoStageStationMap)

Add binary assignment variables x[s][i_idx][j_idx] for ClusteringTwoStageStationModel.

For each scenario s and each demanded station i in that scenario, one binary
variable is created per admissible cluster center j.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageStationMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Vector{VariableRef}}() for _ in 1:S]

    for s in 1:S
        for (i_idx, i) in enumerate(mapping.I_s[s])
            valid_js = get_valid_j_assignments(mapping, i)
            if !isempty(valid_js)
                x[s][i_idx] = @variable(m, [1:length(valid_js)], Bin)
            else
                x[s][i_idx] = VariableRef[]
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end

# ============================================================================
# ExactDARPRouteODMap (ExactDARPRouteModel)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData,
                              mapping::ExactDARPRouteODMap)

Add sparse integer assignment variables x[s][t_id][od_idx] for time-bucketed route models
(ExactDARPRouteModel).

For each OD pair (o,d) in time bucket t of scenario s, one integer variable is created
per valid (j,k) pair, with upper bound equal to Q_s_t[s][t][(o,d)] (the demand count).
This allows demand splitting: passengers with the same OD can use different station pairs.

Structure: `m[:x]` is a `Vector{Dict{Int, Dict{Int, Vector{VariableRef}}}}` indexed by
scenario → t_id → od_idx → pair variables.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ExactDARPRouteODMap
    )
    before = JuMP.num_variables(m)
    S = n_scenarios(data)
    x = [Dict{Int, Dict{Int, Vector{VariableRef}}}() for _ in 1:S]

    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            x[s][t_id] = Dict{Int, Vector{VariableRef}}()
            for (od_idx, (o, d)) in enumerate(od_pairs)
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                n_pairs = length(valid_pairs)
                demand = get(mapping.Q_s_t[s][t_id], (o, d), 0)
                if n_pairs > 0 && demand > 0
                    x[s][t_id][od_idx] = @variable(m, [1:n_pairs],
                        integer = true, lower_bound = 0, upper_bound = demand)
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

Add sparse assignment variables x[i][j_idx] for ClusteringBaseModel.

For each station location i, one binary variable is created per admissible
cluster center j.
"""
function add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringBaseModelMap
    )
    before = JuMP.num_variables(m)
    x = Dict{Int, Vector{VariableRef}}()

    for i in 1:mapping.n_stations
        valid_js = get_valid_j_assignments(mapping, i)
        if !isempty(valid_js)
            x[i] = @variable(m, [1:length(valid_js)], Bin)
        else
            x[i] = VariableRef[]
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end
