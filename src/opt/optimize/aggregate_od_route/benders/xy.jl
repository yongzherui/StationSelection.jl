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
    unmet_demand_active = _aggregate_od_route_unmet_demand_active(master)
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    u = unmet_demand_active ? Dict{NTuple{3, Int}, VariableRef}() : nothing
    for request in requests
        _s, o, d = request
        pairs = feasible_pairs[request]
        for pair in pairs
            x[(request, pair)] = @variable(master, lower_bound = 0.0, upper_bound = 1.0)
        end
        if unmet_demand_active
            u[request] = @variable(master, lower_bound = 0.0, upper_bound = 1.0)
            @constraint(master, sum(x[(request, pair)] for pair in pairs; init=0.0) == u[request])
        else
            @constraint(master, sum(x[(request, pair)] for pair in pairs; init=0.0) == 1.0)
        end
        x_by_pair = Dict(pair => x[(request, pair)] for pair in pairs)
        _add_nearest_open_endpoint_linked_x!(
            master, data, y, o, d, pairs, x_by_pair, max_walking_distance;
            binary=false, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
    end
    master[:u] = u
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

    @variable(m, 0 <= lambda[1:length(columns), 1:n_scenarios(data)] <= 1)
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
    x_hat::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, Float64};
    unmet_demand_active::Bool=false,
)
    assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
    for request in requests
        pairs = feasible_pairs[request]
        selected_pair = pairs[argmax([get(x_hat, (request, pair), 0.0) for pair in pairs])]
        if get(x_hat, (request, selected_pair), 0.0) < 0.5
            unmet_demand_active && continue
            throw(ArgumentError("BendersXY master produced no selected assignment for $(request)"))
        end
        assignments[request] = selected_pair
    end
    return assignments
end

function _run_aggregate_od_route_nearest_open_benders_xy(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    else
        assert_no_walk_only_pairs(mapping, "AggregateODRouteModel Benders (BendersXY, NearestOpen, :pair_chain)")
        !isnothing(model.unmet_demand_penalty) && throw(ArgumentError(
            "BendersXY does not support unmet_demand_penalty with feasibility_cut_style=:pair_chain -- " *
            "\"always feasible\" mode relies on the endpoint-nearest z chain's own relaxation, which " *
            ":pair_chain has no equivalent of"
        ))
    end
    unmet_demand_active = !isnothing(model.unmet_demand_penalty)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    _check_aggregate_od_route_endpoint_feasibility!(data, model, requests, optimizer_env, cfg.silent)
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    master[:aggregate_od_route_unmet_demand_penalty] = model.unmet_demand_penalty
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests)
    x = _add_nearest_open_master_x!(master, data, model, y, requests, feasible_pairs)
    u = unmet_demand_active ? master[:u] : nothing

    obj = AffExpr(0.0)
    for request in requests, pair in feasible_pairs[request]
        add_to_expression!(obj, _assignment_pair_cost(data, request, pair; weight=model.walk_cost_weight), x[(request, pair)])
    end
    if unmet_demand_active
        for request in requests
            add_to_expression!(obj, model.unmet_demand_penalty)
            add_to_expression!(obj, -model.unmet_demand_penalty, u[request])
        end
    end
    for cut_id in cut_ids
        add_to_expression!(obj, 1.0, theta[cut_id])
    end
    @objective(master, Min, obj)

    best_result = nothing
    best_ub = Inf
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]

    for iteration in 1:solver.max_iterations
        master_start = time()
        optimize!(master)
        master_solve_seconds = time() - master_start
        primal_status(master) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersXY master failed with status $(termination_status(master))"))
        lower_bound = objective_value(master)
        # No-op unless an endpoint nearest-open style built zp/zd indicators
        # on this master (via _add_nearest_open_endpoint_master_x!).
        assert_endpoint_chain_near_binary(master)
        assert_service_near_binary(master)

        x_hat = Dict(key => round(value(var)) for (key, var) in x)
        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)
        assignments = _selected_assignments_from_x(
            requests, feasible_pairs, x_hat; unmet_demand_active=unmet_demand_active,
        )

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, model, assignments, solver, iteration, _open_station_values(y_hat)
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        if !isnothing(final_result.objective_value) && final_result.objective_value < best_ub
            best_ub = final_result.objective_value
            best_result = final_result
        end

        iteration_lp_value = 0.0
        cuts_added_this_iteration = 0
        subproblem_lp_seconds = 0.0
        for cut_id in cut_ids
            group_requests = cut_groups[cut_id]
            lp_start = time()
            v_hat, rho = _solve_xy_route_subproblem_lp(
                data,
                model,
                group_requests,
                feasible_pairs,
                cg_result.generated_columns,
                x_hat,
                optimizer_env,
                cfg.silent,
            )
            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if theta_hat[cut_id] < v_hat - solver.optimality_tol
                @constraint(master, theta[cut_id] >= v_hat + sum(rho[key] * (x[key] - get(x_hat, key, 0.0)) for key in keys(rho)))
                optimality_cuts += 1
                cuts_added_this_iteration += 1
            end
        end
        push!(benders_rows, (
            iteration=iteration,
            master_status=string(termination_status(master)),
            lower_bound=lower_bound,
            incumbent_objective=isfinite(best_ub) ? best_ub : nothing,
            outer_gap=_outer_gap(lower_bound, best_ub),
            outer_gap_absolute=_outer_gap_absolute(lower_bound, best_ub),
            outer_gap_relative=_outer_gap_relative(lower_bound, best_ub),
            master_solve_seconds=master_solve_seconds,
            priming_cg_seconds=priming_cg_seconds,
            subproblem_lp_seconds=subproblem_lp_seconds,
            cuts_added=cuts_added_this_iteration,
            feasibility_cuts_added=0,
            optimality_cuts_added=optimality_cuts,
            selected_assignment_count=length(assignments),
            generated_column_pool_size=length(cg_result.generated_columns),
            inner_cg_iterations=inner_cg_iters,
        ))
        _flush_benders_iteration_log!(solver, benders_rows)

        if cuts_added_this_iteration == 0
            return _opt_result_from_benders(best_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersXY",
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
                "benders_outer_gap_absolute" => _outer_gap_absolute(lower_bound, best_ub),
                "benders_outer_gap_relative" => _outer_gap_relative(lower_bound, best_ub),
                "benders_master_solve_time_sec" => master_solve_seconds,
                "benders_priming_cg_time_sec" => priming_cg_seconds,
                "benders_subproblem_lp_time_sec" => subproblem_lp_seconds,
                "feasibility_cuts_added" => 0,
                "optimality_cuts_added" => optimality_cuts,
                "inner_cg_iterations" => inner_cg_iters,
                "benders_lp_value" => iteration_lp_value,
                "best_upper_bound" => best_ub,
                "selected_assignment_count" => length(assignments),
                "generated_column_pool_size" => length(cg_result.generated_columns),
                "feasibility_cut_style" => string(model.assignment_policy.feasibility_cut_style),
            ))
        end
    end
    isnothing(best_result) && throw(ArgumentError("BendersXY did not find a feasible incumbent"))
    throw(ArgumentError("BendersXY did not converge within max_iterations=$(solver.max_iterations)"))
end

function _run_aggregate_od_route_free_benders_xy(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel Benders requires positive demand"))
    for request in requests
        isempty(feasible_pairs[request]) &&
            throw(ArgumentError("BendersXY master has no open feasible pair candidate for $(request)"))
    end
    _check_aggregate_od_route_endpoint_feasibility!(data, model, requests, optimizer_env, cfg.silent)
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests)
    x = _add_unrestricted_master_x!(master, y, requests, feasible_pairs)

    obj = AffExpr(0.0)
    for request in requests, pair in feasible_pairs[request]
        add_to_expression!(obj, _assignment_pair_cost(data, request, pair; weight=model.walk_cost_weight), x[(request, pair)])
    end
    for cut_id in cut_ids
        add_to_expression!(obj, 1.0, theta[cut_id])
    end
    @objective(master, Min, obj)

    best_result = nothing
    best_ub = Inf
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]

    for iteration in 1:solver.max_iterations
        master_start = time()
        optimize!(master)
        master_solve_seconds = time() - master_start
        primal_status(master) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersXY master failed with status $(termination_status(master))"))
        lower_bound = objective_value(master)

        x_hat = Dict(key => round(value(var)) for (key, var) in x)
        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)
        assignments = _selected_assignments_from_x(requests, feasible_pairs, x_hat)

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, model, assignments, solver, iteration, _open_station_values(y_hat)
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        if !isnothing(final_result.objective_value) && final_result.objective_value < best_ub
            best_ub = final_result.objective_value
            best_result = final_result
        end

        iteration_lp_value = 0.0
        cuts_added_this_iteration = 0
        subproblem_lp_seconds = 0.0
        for cut_id in cut_ids
            group_requests = cut_groups[cut_id]
            lp_start = time()
            v_hat, rho = _solve_xy_route_subproblem_lp(
                data,
                model,
                group_requests,
                feasible_pairs,
                cg_result.generated_columns,
                x_hat,
                optimizer_env,
                cfg.silent,
            )
            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if theta_hat[cut_id] < v_hat - solver.optimality_tol
                @constraint(master, theta[cut_id] >= v_hat + sum(rho[key] * (x[key] - get(x_hat, key, 0.0)) for key in keys(rho)))
                optimality_cuts += 1
                cuts_added_this_iteration += 1
            end
        end

        push!(benders_rows, (
            iteration=iteration,
            master_status=string(termination_status(master)),
            lower_bound=lower_bound,
            incumbent_objective=isfinite(best_ub) ? best_ub : nothing,
            outer_gap=_outer_gap(lower_bound, best_ub),
            outer_gap_absolute=_outer_gap_absolute(lower_bound, best_ub),
            outer_gap_relative=_outer_gap_relative(lower_bound, best_ub),
            master_solve_seconds=master_solve_seconds,
            priming_cg_seconds=priming_cg_seconds,
            subproblem_lp_seconds=subproblem_lp_seconds,
            cuts_added=cuts_added_this_iteration,
            feasibility_cuts_added=0,
            optimality_cuts_added=optimality_cuts,
            selected_assignment_count=length(assignments),
            generated_column_pool_size=length(cg_result.generated_columns),
            inner_cg_iterations=inner_cg_iters,
        ))
        _flush_benders_iteration_log!(solver, benders_rows)

        if cuts_added_this_iteration == 0
            return _opt_result_from_benders(best_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersXY",
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
                "benders_outer_gap_absolute" => _outer_gap_absolute(lower_bound, best_ub),
                "benders_outer_gap_relative" => _outer_gap_relative(lower_bound, best_ub),
                "benders_master_solve_time_sec" => master_solve_seconds,
                "benders_priming_cg_time_sec" => priming_cg_seconds,
                "benders_subproblem_lp_time_sec" => subproblem_lp_seconds,
                "feasibility_cuts_added" => 0,
                "optimality_cuts_added" => optimality_cuts,
                "inner_cg_iterations" => inner_cg_iters,
                "benders_lp_value" => iteration_lp_value,
                "best_upper_bound" => best_ub,
                "selected_assignment_count" => length(assignments),
                "generated_column_pool_size" => length(cg_result.generated_columns),
            ))
        end
    end
    isnothing(best_result) && throw(ArgumentError("BendersXY did not find a feasible incumbent"))
    throw(ArgumentError("BendersXY did not converge within max_iterations=$(solver.max_iterations)"))
end
