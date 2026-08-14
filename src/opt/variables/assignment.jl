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
# ClusteringTwoStageODMap (TwoStageODPolicy)
# ============================================================================

"""
    add_assignment_variables!(
        m::Model,
        data::StationSelectionData,
        mapping::ClusteringTwoStageODMap;
    )

Add assignment variables x[s][p][pair_idx] for TwoStageODPolicy.

x[s][p][pair_idx] is the integer passenger count from demand group p (the OD
pair Omega_s[s][p]) in scenario s assigned to the corresponding valid
pickup/dropoff pair.

Structure: scenario → demand-group index p → sparse vector over valid (pickup, dropoff) pairs.
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
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            valid_pairs = get_valid_jk_pairs(mapping, o, d)
            n_pairs = length(valid_pairs)
            demand = mapping.Q_s[s][p]
            if n_pairs > 0 && demand > 0
                x[s][p] = @variable(m, [1:n_pairs],
                    integer = true, lower_bound = 0, upper_bound = demand)
            else
                x[s][p] = VariableRef[]
            end
        end
    end

    m[:x] = x
    return JuMP.num_variables(m) - before
end

# ============================================================================
# ClusteringBaseModelMap (SingleStagePolicy)
# ============================================================================

# ============================================================================
# ClusteringTwoStageStationMap (TwoStagePolicy)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::ClusteringTwoStageStationMap)

Add binary assignment variables x[s][i_idx][j_idx] for TwoStagePolicy.

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
# ClusteringBaseModelMap (SingleStagePolicy)
# ============================================================================

"""
    add_assignment_variables!(m::Model, data::StationSelectionData, mapping::ClusteringBaseModelMap)

Add sparse assignment variables x[i][j_idx] for SingleStagePolicy.

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
