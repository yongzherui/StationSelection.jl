"""
Benders-XY decomposition for AggregateODRouteProblem: master = y,x (full nearest-open
resolution + assignment); subproblem = theta (route covering) only (see
`iterative_strategy_types.jl`'s `BendersXY` docstring). Covers both the
NearestOpenAggregateODAssignmentPolicy path and the free-assignment path.
"""

"""
    _add_nearest_open_endpoint_master_x!(master, data, y, requests, feasible_pairs, max_walking_distance, allow_walk_only, selector_style)

BendersXY's nearest-open `x` (and `zp`/`zd`) are declared continuous `[0,1]`,
not `Bin` -- `y` is the only genuinely binary master variable. Given `y`
integer, the chain/big-M constraints already force `zp`/`zd` (and, through the
linking rows, `x`) to resolve to exactly 0/1 at any LP optimum -- the same
reasoning already used and runtime-verified
(`assert_endpoint_chain_near_binary`, called after every master solve) for
BendersY's fixed-`y` subproblem LP. Removing the explicit binary declaration
takes `x`/`zp`/`zd` out of the master's own branch-and-bound entirely, which
should shrink the search tree considerably relative to declaring one binary
per `(request,pair)`. `:big_m_nearest`'s tie-break perturbation
(`_endpoint_big_m_variable!`) makes this provably safe even when two open
candidates are tied at exactly equal walking cost.
"""
function _add_nearest_open_endpoint_master_x!(
    master::Model,
    data::StationSelectionData,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    max_walking_distance::Float64,
    allow_walk_only::Bool,
    selector_style::Symbol,
)
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        pairs = feasible_pairs[request]
        x_by_pair, _sum_con = _add_nearest_open_pair_assignment!(
            master, data, y, request, pairs, max_walking_distance;
            allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
        for (pair, var) in x_by_pair
            x[(request, pair)] = var
        end
    end
    return x
end

function _add_unrestricted_master_x!(
    master::Model,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        x_by_pair, _sum_con = add_free_pair_assignment_constraints!(master, y, request, feasible_pairs[request])
        for (pair, var) in x_by_pair
            x[(request, pair)] = var
        end
    end
    return x
end

function _add_nearest_open_master_x!(
    master::Model,
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        return _add_nearest_open_endpoint_master_x!(
            master, data, y, requests, feasible_pairs, model.max_walking_distance, model.allow_walk_only,
            model.assignment_policy.feasibility_cut_style,
        )
    end
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        x_by_pair, _sum_con = add_ranked_pair_assignment_constraints!(
            master, data, y, request, feasible_pairs[request]; binary=true,
        )
        for (pair, var) in x_by_pair
            x[(request, pair)] = var
        end
    end
    return x
end

function _build_xy_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    x_hat::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    optimizer_env,
    silent::Bool,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    x, fix_cons = add_fixed_pair_assignment_variables!(m, requests, feasible_pairs, x_hat)
    lambda = add_benders_lambda_variables!(m, columns, n_scenarios(data))
    # Walk-only and same-station assignments use no vehicle route, so no route column can (or
    # needs to) cover them — a coverage row here would force x[(request, pair)] to 0 even when
    # the master fixed it to 1. add_benders_route_coverage_constraints! already skips these.
    add_benders_route_coverage_constraints!(m, lambda, requests, feasible_pairs, columns, x)

    route_expr = benders_route_regularization_cost_expr(model, columns, lambda, n_scenarios(data))
    set_benders_subproblem_objective!(m, AffExpr(0.0), route_expr)
    return m, fix_cons
end

function _solve_xy_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteProblem,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    x_hat::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
    optimizer_env,
    silent::Bool,
)
    m, fix_cons = _build_xy_route_subproblem_lp(
        data, model, requests, feasible_pairs, columns, x_hat, optimizer_env, silent
    )
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersXY route LP subproblem failed with status $(termination_status(m))"))
    return objective_value(m), Dict(key => dual(con) for (key, con) in fix_cons)
end

function _selected_assignments_from_x(
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    x_hat::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64},
)
    assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
    for request in requests
        pairs = feasible_pairs[request]
        selected_pair = pairs[argmax([get(x_hat, (request, pair), 0.0) for pair in pairs])]
        get(x_hat, (request, selected_pair), 0.0) < 0.5 &&
            throw(ArgumentError("BendersXY master produced no selected assignment for $(request)"))
        assignments[request] = selected_pair
    end
    return assignments
end

"""
    _run_aggregate_od_route_nearest_open_benders_xy(data, model, solver) -> OptResult
    _run_aggregate_od_route_free_benders_xy(data, model, solver) -> OptResult

Both are thin wrappers around the generic `_run_benders_decomposition`
(`benders/generic_runner.jl`) -- `BendersMasterModel{BendersXY}`'s `build_model` already
picks the nearest-open vs. free-assignment master variant internally based on
`model.assignment_policy`, so both entry points now do the same thing; kept as two
separate names only because `dispatch.jl` still calls them by name.
"""
function _run_aggregate_od_route_nearest_open_benders_xy(
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    solver::BendersSolver,
)
    return _run_benders_decomposition(data, model, solver)
end

function _run_aggregate_od_route_free_benders_xy(
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    solver::BendersSolver,
)
    return _run_benders_decomposition(data, model, solver)
end
