"""
Objective for AggregateODRouteProblem.
"""

export set_aggregate_od_route_objective!

function set_aggregate_od_route_objective!(
    m::Model,
    data::StationSelectionData,
    mapping::AggregateODRouteMap;
    route_regularization_weight::Float64=1.0,
    walk_cost_weight::Float64=1.0,
    repositioning_time::Float64=20.0,
    extra_walking_cost_expr::Union{Nothing, AffExpr}=nothing,
)
    obj = AffExpr(0.0)
    x = m[:x]
    for s in 1:n_scenarios(data)
        for (od_idx, (o, d)) in enumerate(mapping.Omega_s[s])
            x_od = get(x[s], od_idx, VariableRef[])
            isempty(x_od) && continue
            for (pair_idx, pair) in enumerate(get_valid_jk_pairs(mapping, o, d))
                cost = walk_cost_weight * od_pair_walking_cost(data, o, d, pair)
                add_to_expression!(obj, cost, x_od[pair_idx])
            end
        end
    end
    # NearestOpenAggregateODAssignmentPolicy(:direct_ly): walking cost has no x to attach to,
    # so add_gamma_chain_nearest_open_coverage! hands back its own (γ-difference-based)
    # walking-cost expression here instead -- same weight, different carrier variable.
    isnothing(extra_walking_cost_expr) || add_to_expression!(obj, walk_cost_weight, extra_walking_cost_expr)

    theta = m[:theta_compat]
    column_by_id = Dict(column.id => column for column in mapping.columns)
    for ((column_id, _s), theta_var) in theta
        column = column_by_id[column_id]
        coef = aggregate_od_route_column_objective_coefficient(
            route_regularization_weight,
            repositioning_time,
            column,
        )
        add_to_expression!(obj, coef, theta_var)
    end

    @objective(m, Min, obj)
    return nothing
end
