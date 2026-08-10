"""
Master-objective composers for `BendersMasterModel{D}` (`src/opt/optimize/aggregate_od_route/benders/problems.jl`).
"""

"""
    set_benders_lifted_master_objective!(m, theta, cut_ids, walking_cost_expr, direct_cost_expr, current_beta; lifted_walking_objective)

Shared objective composition for `BendersY`/`BendersYZ`'s master. Under
`lifted_walking_objective=true`: `current_beta * (sum(theta) + direct_cost_expr) +
walking_cost_expr`. Otherwise: `sum(theta)`.
"""
function set_benders_lifted_master_objective!(
    m::JuMP.Model,
    theta,
    cut_ids,
    walking_cost_expr::AffExpr,
    direct_cost_expr::AffExpr,
    current_beta::Float64;
    lifted_walking_objective::Bool,
)
    if lifted_walking_objective
        @objective(
            m, Min,
            current_beta * (sum(theta[cid] for cid in cut_ids) + direct_cost_expr) + walking_cost_expr,
        )
    else
        @objective(m, Min, sum(theta[cid] for cid in cut_ids))
    end
    return nothing
end

"""
    set_benders_xy_master_objective!(m, data, x, theta, cut_ids, requests, feasible_pairs, weight)

`BendersXY`'s master objective: assignment walking cost (reusing
`assignment_walking_cost_expr`, `objectives/expressions/aggregate_od_route/benders.jl`) plus
`sum(theta)` -- no route-lower-bound/direct-enumeration terms (both forbidden for `BendersXY` at
`BendersSolver` construction).
"""
function set_benders_xy_master_objective!(
    m::JuMP.Model,
    data::StationSelectionData,
    x::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    theta,
    cut_ids,
    requests,
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    weight::Float64,
)
    walking_expr = assignment_walking_cost_expr(data, requests, feasible_pairs, x; weight = weight)
    @objective(m, Min, walking_expr + sum(theta[cid] for cid in cut_ids))
    return nothing
end

"""
    set_benders_yzh_master_objective!(m, walking_expr, theta, cut_ids)

`BendersYZH`'s master objective: `physical_pair_walking_cost_expr` (pre-built by the caller,
`objectives/expressions/aggregate_od_route/benders.jl`) plus `sum(theta)`.
"""
function set_benders_yzh_master_objective!(m::JuMP.Model, walking_expr::AffExpr, theta, cut_ids)
    @objective(m, Min, walking_expr + sum(theta[cid] for cid in cut_ids))
    return nothing
end
