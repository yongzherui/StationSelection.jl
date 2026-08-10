"""
    set_benders_subproblem_objective!(m, walking_expr, route_expr)

Sets a Benders subproblem LP's objective from its already-built walking-cost and
route-regularization-cost expressions (`objectives/expressions/aggregate_od_route/benders.jl`).
`BendersXY`'s subproblem has no walking term (already fully priced in its master), so it passes
`walking_expr = AffExpr(0.0)`.
"""
function set_benders_subproblem_objective!(m::JuMP.Model, walking_expr::AffExpr, route_expr::AffExpr)
    @objective(m, Min, walking_expr + route_expr)
    return nothing
end
