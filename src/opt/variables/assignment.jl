"""
Assignment variable creation functions for station selection optimization models.

These functions add assignment decision variables that map requests/OD pairs
to station pairs.

Uses multiple dispatch to provide specialized implementations for different
mapping types, including `AggregateODRouteMap`'s own `x[s,p,j,k]` (kept here
rather than under `variables/aggregate_od_route/` so every mapping type's assignment
variable has exactly one home).
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

# ============================================================================
# AggregateODRouteMap (AggregateODRouteBaseFormulation)
# ============================================================================

"""
    add_assignment_variables!(m, data, mapping::AggregateODRouteMap; scenarios=1:n_scenarios(data), relax_integrality=false)
        -> Dict{NTuple{4,Int}, VariableRef}

OD-to-station-pair assignment `x[s,p,j,k]` for `AggregateODRouteBaseFormulation`'s
`y`/`x`/`θ` master -- decoupled from route activation `θ`, linked via
`add_aggregate_od_route_base_route_linking_constraints!`
(`constraints/aggregate_od_route/base/linking.jl`) -- unlike
`AggregateODRouteJointRoutingAssignmentFormulation`, where route columns carry
assignment directly and there is no separate `x`.

`x[s,p,j,k]` for every demand group `p` (position within `mapping.Omega_s[s]`, i.e. the
`(o,d) = mapping.Omega_s[s][p]` pair) with `s in scenarios` and every feasible
`(j,k) in get_valid_jk_pairs(mapping,o,d)` that requires a real vehicle route
(`WALK_ONLY_PAIR` excluded, matching `θ`'s own domain and handled instead by
`add_walk_variables!` -- this formulation doesn't yet support the
station-free option through `x` itself).

`scenarios` restricts variable creation to a subset of scenarios -- used by
`AggregateODRouteBendersYXFormulation`'s subproblem, which is solved one scenario at a
time (default keeps every existing caller's behavior unchanged). `relax_integrality`
declares `x` continuous on `[0,1]` instead of binary, mirroring
`add_route_variables!`'s own kwarg -- the Benders subproblem needs
this for valid LP duals off its `y`-fixing constraints.
"""
function add_assignment_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    scenarios::AbstractVector{Int}=1:n_scenarios(data),
    relax_integrality::Bool=false,
)::Dict{NTuple{4, Int}, VariableRef}
    x = Dict{NTuple{4, Int}, VariableRef}()
    for s in scenarios
        for (p, (o, d)) in enumerate(mapping.Omega_s[s])
            demand = mapping.Q_s[s][p]
            demand > 0 || continue
            for pair in get_valid_jk_pairs(mapping, o, d)
                is_walk_only_pair(pair) && continue
                j, k = pair
                if relax_integrality
                    x[(s, p, j, k)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0, base_name = "x[$s,$p,$j,$k]")
                else
                    x[(s, p, j, k)] = @variable(m, binary = true, base_name = "x[$s,$p,$j,$k]")
                end
            end
        end
    end
    m[:x] = x
    return x
end
