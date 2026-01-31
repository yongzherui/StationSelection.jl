"""
Objective functions for single detour models.

Composes expressions from:
- assignment_cost.jl: walking + routing costs per assignment
- flow_cost.jl: flow routing costs
- pooling_savings.jl: same-source and same-dest pooling savings

Contains objectives for:
- TwoStageSingleDetourModel
"""

using JuMP

export set_two_stage_single_detour_objective!


"""
    set_two_stage_single_detour_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    )

Set the minimization objective for TwoStageSingleDetourModel.

Objective = Assignment Costs + Flow Costs - Pooling Savings

Components:
1. Assignment costs: walking + routing for each OD assignment
2. Flow costs: γ · Σ c_{jk} · f[s][t][j,k]
3. Same-source pooling savings: γ · Σ r_{jl,kl} · u[s][t][idx]
4. Same-dest pooling savings: γ · Σ r_{jl,jk} · v[s][t][idx]

# Arguments
- `m::Model`: JuMP model with variables x, f, u, v already added
- `data::StationSelectionData`: Problem data with walking_costs and routing_costs
- `mapping::TwoStageSingleDetourMap`: Scenario/time to OD mapping
- `Xi_same_source::Vector{Tuple{Int,Int,Int}}`: Same-source detour triplets (j,k,l)
- `Xi_same_dest::Vector{Tuple{Int,Int,Int,Int}}`: Same-dest detour quadruplets (j,k,l,t')
- `routing_weight::Float64`: Weight γ for routing/pooling terms (default 1.0)
"""
function set_two_stage_single_detour_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::TwoStageSingleDetourMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    )

    # Build objective from component expressions
    obj = assignment_cost_expr(m, data, mapping) +
          flow_cost_expr(m, data, mapping; routing_weight=routing_weight) -
          same_source_pooling_savings_expr(m, data, mapping, Xi_same_source; routing_weight=routing_weight) -
          same_dest_pooling_savings_expr(m, data, mapping, Xi_same_dest; routing_weight=routing_weight)

    @objective(m, Min, obj)

    return nothing
end

