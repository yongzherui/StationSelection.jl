"""
Benders-XY decomposition for AggregateODRouteModel: master = y,x (full nearest-open
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
        pairs = feasible_pairs[request]
        isempty(pairs) && throw(ArgumentError("BendersXY master has no feasible station pair for $(request)"))
        for pair in pairs
            var = @variable(master, binary = true)
            x[(request, pair)] = var
            if !is_walk_only_pair(pair)
                j, k = pair
                @constraint(master, var <= y[j])
                @constraint(master, var <= y[k])
            end
        end
        @constraint(master, sum(x[(request, pair)] for pair in pairs) == 1.0)
    end
    return x
end

function _add_nearest_open_master_x!(
    master::Model,
    data::StationSelectionData,
    model::AggregateODRouteModel,
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
        ranked = _ranked_request_pairs(data, request, feasible_pairs[request])
        for pair in ranked
            x[(request, pair)] = @variable(master, binary = true)
        end
        @constraint(master, sum(x[(request, pair)] for pair in ranked) == 1.0)
        for (rank_idx, pair) in enumerate(ranked)
            j, k = pair
            @constraint(master, x[(request, pair)] <= y[j])
            @constraint(master, x[(request, pair)] <= y[k])
            for prior in ranked[1:max(rank_idx - 1, 0)]
                pj, pk = prior
                @constraint(master, x[(request, pair)] <= 2.0 - y[pj] - y[pk])
            end
        end
    end
    return x
end

function _build_xy_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
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

    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    fix_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for request in requests, pair in feasible_pairs[request]
        key = (request, pair)
        x[key] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        fix_cons[key] = @constraint(m, x[key] == get(x_hat, key, 0.0))
    end

    @variable(m, lambda[1:length(columns), 1:n_scenarios(data)] >= 0)
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            # Walk-only and same-station assignments use no vehicle route, so
            # no route column can (or needs to) cover them — a coverage row
            # here would force x[(request, pair)] to 0 even when the master
            # fixed it to 1.
            requires_no_vehicle_route(pair) && continue
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            @constraint(m, sum(lambda[idx, s] for idx in covering; init=0.0) >= x[(request, pair)])
        end
    end

    obj = AffExpr(0.0)
    for (idx, column) in enumerate(columns), s in 1:n_scenarios(data)
        add_to_expression!(
            obj,
            aggregate_od_route_column_objective_coefficient(
                model.route_regularization_weight,
                model.repositioning_time,
                column,
            ),
            lambda[idx, s],
        )
    end
    @objective(m, Min, obj)
    return m, fix_cons
end

function _solve_xy_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
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
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    return _run_benders_decomposition(data, model, solver)
end

function _run_aggregate_od_route_free_benders_xy(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    return _run_benders_decomposition(data, model, solver)
end
