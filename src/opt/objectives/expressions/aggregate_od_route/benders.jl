"""
Reusable objective-expression pieces for Benders subproblem LPs, composed by
`objectives/aggregate_od_route/benders/subproblem.jl`'s `set_benders_subproblem_objective!`.
"""

"""
    assignment_walking_cost_expr(data, requests, feasible_pairs, assigned_vars; weight) -> AffExpr

Sum of `_assignment_pair_cost(data, request, pair; weight)` weighted by each
`(request, pair)`'s assignment variable. `BendersY`/`BendersYZ`/`BendersYZH`'s subproblems carry
this; `BendersXY`'s does not (its master already prices assignment walking cost, since `x` is
fully fixed there).
"""
function assignment_walking_cost_expr(
    data::StationSelectionData,
    requests,
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    assigned_vars::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef};
    weight::Float64,
)
    expr = AffExpr(0.0)
    for request in requests
        for pair in feasible_pairs[request]
            add_to_expression!(
                expr, _assignment_pair_cost(data, request, pair; weight = weight), assigned_vars[(request, pair)],
            )
        end
    end
    return expr
end

"""
    physical_pair_walking_cost_expr(data, physical_pairs, feasible_pairs_by_p, occurrence_count, h; weight) -> AffExpr

`BendersYZH`'s master walking cost: one term per `(physical_pair, pair)`, weighted by how many
scenarios that physical pair occurs in (`occurrence_count`) since `h` is scenario-compressed.
"""
function physical_pair_walking_cost_expr(
    data::StationSelectionData,
    physical_pairs::Vector{Tuple{Int, Int}},
    feasible_pairs_by_p::Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}},
    occurrence_count::Dict{Tuple{Int, Int}, Int},
    h::Dict{Tuple{Tuple{Int, Int}, Tuple{Int, Int}}, VariableRef};
    weight::Float64,
)
    expr = AffExpr(0.0)
    for p in physical_pairs, pair in feasible_pairs_by_p[p]
        o, d = p
        add_to_expression!(expr, occurrence_count[p] * weight * od_pair_walking_cost(data, o, d, pair), h[(p, pair)])
    end
    return expr
end

"""
    benders_route_regularization_cost_expr(model, columns, lambda, n_scenarios) -> AffExpr

Sum of `aggregate_od_route_column_objective_coefficient` weighted by each column's `lambda`
activation, across every scenario -- shared by all four subproblem builders.
"""
function benders_route_regularization_cost_expr(
    model::AnyAggregateODRouteProblem,
    columns::Vector{AggregateODRouteColumn},
    lambda,
    n_scenarios::Int,
)
    expr = AffExpr(0.0)
    for (idx, column) in enumerate(columns), s in 1:n_scenarios
        add_to_expression!(
            expr,
            aggregate_od_route_column_objective_coefficient(
                model.route_regularization_weight, model.repositioning_time, column,
            ),
            lambda[idx, s],
        )
    end
    return expr
end
