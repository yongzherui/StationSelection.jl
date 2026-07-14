"""
Route-covering solve paths for aggregate OD route models.

These helpers adapt the exploration route-covering ideas to this package's
aggregate scenario-OD representation. A positive-demand `(scenario, o, d)` OD
bucket plays the role of a request; station-pair route coverage remains binary.
"""

export enumerate_aggregate_od_route_columns

function _base_aggregate_od_route_model(model::AnyAggregateODRouteModel)::AggregateODRouteModel
    return model isa AggregateODRouteModel ? model : model.base
end

function _copy_with_initial_columns(
    model::RouteCoveringProblem,
    columns::Vector{AggregateODRouteColumn};
    relax_integrality::Bool=false,
)
    return RouteCoveringProblem(
        model.l,
        model.open_stations,
        model.fixed_assignments;
        route_regularization_weight=model.route_regularization_weight,
        repositioning_time=model.repositioning_time,
        max_walking_distance=model.max_walking_distance,
        max_wait_time=model.max_wait_time,
        detour_factor=model.detour_factor,
        max_stops=model.max_stops,
        max_visits_per_node=model.max_visits_per_node,
        max_new_columns=model.max_new_columns,
        n_candidates=model.n_candidates,
        pricing_time_limit_sec=model.pricing_time_limit_sec,
        reduced_cost_tol=model.reduced_cost_tol,
        initial_columns=columns,
        relax_integrality=relax_integrality,
        assignment_policy=model.assignment_policy,
    )
end

function _copy_with_initial_columns(
    model::AggregateODRouteModel,
    columns::Vector{AggregateODRouteColumn};
    relax_integrality::Bool=false,
)
    return AggregateODRouteModel(
        model.l;
        route_regularization_weight=model.route_regularization_weight,
        repositioning_time=model.repositioning_time,
        max_walking_distance=model.max_walking_distance,
        max_wait_time=model.max_wait_time,
        detour_factor=model.detour_factor,
        max_stops=model.max_stops,
        max_visits_per_node=model.max_visits_per_node,
        max_new_columns=model.max_new_columns,
        n_candidates=model.n_candidates,
        pricing_time_limit_sec=model.pricing_time_limit_sec,
        reduced_cost_tol=model.reduced_cost_tol,
        initial_columns=columns,
        relax_integrality=relax_integrality,
        assignment_policy=model.assignment_policy,
    )
end

function _all_active_aggregate_od_route_pairs(mapping::AggregateODRouteMap)::Vector{Tuple{Int, Int}}
    pairs = Set{Tuple{Int, Int}}()
    for scenario_pairs in values(mapping.active_jk_s)
        union!(pairs, scenario_pairs)
    end
    return sort!(collect(pairs))
end

function _deduplicate_aggregate_od_route_columns(
    columns::Vector{AggregateODRouteColumn},
)::Vector{AggregateODRouteColumn}
    best = Dict{Any, AggregateODRouteColumn}()
    for column in columns
        signature = _aggregate_od_route_column_signature(column)
        incumbent = get(best, signature, nothing)
        if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
            best[signature] = column
        end
    end
    out = AggregateODRouteColumn[]
    next_id = 1
    for column in sort!(collect(values(best)); by=c -> (length(c.od_pairs), c.tau, string(c.od_pairs)))
        push!(out, AggregateODRouteColumn(
            next_id,
            column.od_pairs,
            column.tau;
            metadata=copy(column.metadata),
        ))
        next_id += 1
    end
    return out
end

"""
    enumerate_aggregate_od_route_columns(model, data; kwargs...)

Enumerate route-covering columns for aggregate OD route models using the same
label extension/certification logic as aggregate OD route pricing, with unit rewards
so every certifiable route prefix is retained.
"""
function enumerate_aggregate_od_route_columns(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData;
    max_routes::Int=10_000,
    time_limit_sec::Float64=30.0,
)::Vector{AggregateODRouteColumn}
    max_routes > 0 || throw(ArgumentError("max_routes must be positive"))
    time_limit_sec > 0 || throw(ArgumentError("time_limit_sec must be positive"))

    mapping = create_map(model, data)
    base_model = _base_aggregate_od_route_model(model)
    active_pairs = _all_active_aggregate_od_route_pairs(mapping)
    isempty(active_pairs) && return AggregateODRouteColumn[]

    pricing_data = AggregateODRoutePricingData(
        1,
        collect(1:data.n_stations),
        Dict((i, j) => get_routing_cost(data, i, j) for i in 1:data.n_stations for j in 1:data.n_stations if i != j),
        active_pairs,
        base_model.route_regularization_weight,
        base_model.repositioning_time,
        base_model.max_wait_time,
        base_model.detour_factor,
        base_model.max_stops == typemax(Int) ? data.n_stations : base_model.max_stops,
        base_model.max_visits_per_node,
    )
    duals = AggregateODRoutePricingDuals(Dict(pair => 1.0 for pair in active_pairs))
    labels, exhausted, _stats = _enumerate_aggregate_od_route_pricing_labels(
        pricing_data,
        duals;
        time_limit=time_limit_sec,
        reduced_cost_tol=0.0,
        max_visits_per_node=base_model.max_visits_per_node,
        use_reduced_cost_pruning=false,
    )
    exhausted || throw(ArgumentError("route enumeration did not complete within time_limit_sec=$(time_limit_sec)"))

    columns = AggregateODRouteColumn[]
    next_id = 1
    for label in sort!(labels; by=l -> (length(l.route), l.tau, string(l.route)))
        isempty(label.served_pairs) && continue
        push!(columns, AggregateODRouteColumn(
            next_id,
            collect(label.served_pairs),
            label.tau;
            metadata=Dict{String, Any}(
                "initialization" => "enumeration",
                "route" => Tuple(label.route),
            ),
        ))
        next_id += 1
        length(columns) <= max_routes ||
            throw(ArgumentError("route enumeration exceeded max_routes=$(max_routes)"))
    end
    append!(columns, mapping.columns)
    return _deduplicate_aggregate_od_route_columns(columns)
end

function _run_direct_enumerated_aggregate_od_route(
    instance::StationSelectionData,
    formulation::AnyAggregateODRouteModel,
    solver::DirectSolver,
)
    cfg = solver.config
    columns = enumerate_aggregate_od_route_columns(
        formulation,
        instance;
        max_routes=solver.max_enumerated_routes,
        time_limit_sec=solver.max_enumeration_time_sec,
    )
    enumerated = _copy_with_initial_columns(formulation, columns; relax_integrality=false)
    result = _run_opt_impl(
        enumerated,
        instance;
        optimizer_env=cfg.optimizer_env,
        silent=cfg.silent,
        show_counts=cfg.show_counts,
        do_optimize=cfg.do_optimize,
        warm_start=cfg.warm_start,
        check_feasibility=cfg.check_feasibility,
        mip_gap=cfg.mip_gap,
    )
    result.metadata["solve_method"] = "route_enumeration"
    result.metadata["enumerated_routes"] = length(columns)
    return result
end

function run_opt(
    instance::StationSelectionData,
    formulation::AggregateODRouteModel,
    solver::DirectSolver,
)
    formulation.assignment_policy isa NearestOpenAggregateODAssignmentPolicy ||
        return _run_opt_impl(
            formulation,
            instance;
            optimizer_env=solver.config.optimizer_env,
            silent=solver.config.silent,
            show_counts=solver.config.show_counts,
            do_optimize=solver.config.do_optimize,
            warm_start=solver.config.warm_start,
            check_feasibility=solver.config.check_feasibility,
            mip_gap=solver.config.mip_gap,
        )
    return _run_direct_enumerated_aggregate_od_route(instance, formulation, solver)
end

function run_opt(
    instance::StationSelectionData,
    formulation::RouteCoveringProblem,
    solver::DirectSolver,
)
    return _run_direct_enumerated_aggregate_od_route(instance, formulation, solver)
end

function _benders_decomposition_name(solver::BendersSolver)
    solver.decomposition isa BendersY && return "BendersY"
    solver.decomposition isa BendersXY && return "BendersXY"
    return string(typeof(solver.decomposition))
end

function _benders_cut_mode_name(solver::BendersSolver)
    solver.cut_mode isa SingleCut && return "SingleCut"
    solver.cut_mode isa MultiCut && return "MultiCut($(solver.cut_mode.dimension))"
    return string(typeof(solver.cut_mode))
end

function _aggregate_od_route_benders_requests(mapping::AggregateODRouteMap)
    requests = NTuple{3, Int}[]
    demand = Dict{NTuple{3, Int}, Int}()
    feasible_pairs = Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}}()
    for s in sort!(collect(keys(mapping.Q_s)))
        for (o, d) in mapping.Omega_s[s]
            q = get(mapping.Q_s[s], (o, d), 0)
            q > 0 || continue
            key = (s, o, d)
            push!(requests, key)
            demand[key] = q
            feasible_pairs[key] = get_valid_jk_pairs(mapping, o, d)
        end
    end
    return requests, demand, feasible_pairs
end

function _benders_cut_groups(
    requests::Vector{NTuple{3, Int}},
    cut_mode::AbstractBendersCutMode,
)::Dict{Int, Vector{NTuple{3, Int}}}
    if cut_mode isa SingleCut
        return Dict(0 => requests)
    elseif cut_mode isa MultiCut
        groups = Dict{Int, Vector{NTuple{3, Int}}}()
        for request in requests
            s, _o, _d = request
            push!(get!(groups, s, NTuple{3, Int}[]), request)
        end
        return Dict(k => groups[k] for k in sort!(collect(keys(groups))))
    end
    throw(ArgumentError("unsupported Benders cut mode $(typeof(cut_mode))"))
end

function _assignment_pair_cost(data::StationSelectionData, request::NTuple{3, Int}, pair::Tuple{Int, Int})
    _s, o, d = request
    j, k = pair
    return get_walking_cost(data, o, j) + get_walking_cost(data, k, d)
end

function _ranked_request_pairs(
    data::StationSelectionData,
    request::NTuple{3, Int},
    pairs::Vector{Tuple{Int, Int}},
)
    ranked = copy(pairs)
    sort!(ranked, by=pair -> (_assignment_pair_cost(data, request, pair), pair[1], pair[2]))
    return ranked
end

function _open_station_values(y_values)::Vector{Int}
    return sort!([j for j in eachindex(y_values) if y_values[j] > 0.5])
end

function _fixed_assignments_from_y(
    data::StationSelectionData,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
)
    open_set = Set(_open_station_values(y_hat))
    assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
    infeasible = NTuple{3, Int}[]
    for request in requests
        ranked = _ranked_request_pairs(data, request, feasible_pairs[request])
        idx = findfirst(pair -> pair[1] in open_set && pair[2] in open_set, ranked)
        if isnothing(idx)
            push!(infeasible, request)
        else
            assignments[request] = ranked[idx]
        end
    end
    return assignments, infeasible
end

function _add_pair_open_feasibility_cut!(
    master::Model,
    y,
    pairs::Vector{Tuple{Int, Int}},
)::ConstraintRef
    w = @variable(master, [1:length(pairs)], lower_bound = 0.0, upper_bound = 1.0)
    for (idx, (j, k)) in enumerate(pairs)
        @constraint(master, w[idx] <= y[j])
        @constraint(master, w[idx] <= y[k])
        @constraint(master, w[idx] >= y[j] + y[k] - 1.0)
    end
    return @constraint(master, sum(w) >= 1.0)
end

function _master_endpoint_chain_variable!(
    master::Model,
    data::StationSelectionData,
    y,
    side::Symbol,
    request::NTuple{3, Int},
    endpoints::Vector{Int},
)
    _s, o, d = request
    costs = side == :pickup ?
        [get_walking_cost(data, o, j) for j in endpoints] :
        [get_walking_cost(data, k, d) for k in endpoints]
    order = sortperm(collect(eachindex(endpoints)); by=i -> (costs[i], endpoints[i]))
    sorted_endpoints = endpoints[order]
    sorted_costs = costs[order]
    cache = if haskey(master, :nearest_endpoint_chain_cache)
        master[:nearest_endpoint_chain_cache]
    else
        master[:nearest_endpoint_chain_cache] = Dict{Any, Vector{VariableRef}}()
    end
    key = (side, Tuple(sorted_endpoints), Tuple(round.(sorted_costs; digits=9)))
    z = get!(cache, key) do
        vars = @variable(master, [1:length(sorted_endpoints)], binary = true)
        @constraint(master, sum(vars) == 1.0)
        for (rank, station) in enumerate(sorted_endpoints)
            @constraint(master, vars[rank] <= y[station])
            for prior in 1:(rank - 1)
                @constraint(master, vars[rank] <= 1.0 - y[sorted_endpoints[prior]])
            end
        end
        vars
    end
    return z, sorted_endpoints
end

function _add_nearest_open_endpoint_chain_master_x!(
    master::Model,
    data::StationSelectionData,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
)
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        _s, o, d = request
        pairs = feasible_pairs[request]
        pickups, dropoffs = _validate_endpoint_cartesian!(data, o, d, pairs)
        zp, sorted_pickups = _master_endpoint_chain_variable!(master, data, y, :pickup, request, pickups)
        zd, sorted_dropoffs = _master_endpoint_chain_variable!(master, data, y, :dropoff, request, dropoffs)
        pickup_rank = Dict(station => idx for (idx, station) in enumerate(sorted_pickups))
        dropoff_rank = Dict(station => idx for (idx, station) in enumerate(sorted_dropoffs))
        for pair in pairs
            x[(request, pair)] = @variable(master, binary = true)
        end
        @constraint(master, sum(x[(request, pair)] for pair in pairs) == 1.0)
        for (j, k) in pairs
            @constraint(master, x[(request, (j, k))] <= zp[pickup_rank[j]])
            @constraint(master, x[(request, (j, k))] <= zd[dropoff_rank[k]])
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
        for (j, k) in pairs
            var = @variable(master, binary = true)
            x[(request, (j, k))] = var
            @constraint(master, var <= y[j])
            @constraint(master, var <= y[k])
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
    if model.assignment_policy.feasibility_cut_style == :big_m_nearest
        return _add_nearest_open_endpoint_chain_master_x!(master, data, y, requests, feasible_pairs)
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

function _route_covering_problem_from_assignments(
    model::AnyAggregateODRouteModel,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    open_stations::Union{Nothing, Vector{Int}}=nothing,
)
    base = _base_aggregate_od_route_model(model)
    open = isnothing(open_stations) ?
        sort!(unique!(Int[v for pair in values(assignments) for v in pair])) :
        sort!(unique!(copy(open_stations)))
    return RouteCoveringProblem(
        base.l,
        open,
        assignments;
        route_regularization_weight=base.route_regularization_weight,
        repositioning_time=base.repositioning_time,
        max_walking_distance=base.max_walking_distance,
        max_wait_time=base.max_wait_time,
        detour_factor=base.detour_factor,
        max_stops=base.max_stops,
        max_visits_per_node=base.max_visits_per_node,
        max_new_columns=base.max_new_columns,
        n_candidates=base.n_candidates,
        pricing_time_limit_sec=base.pricing_time_limit_sec,
        reduced_cost_tol=base.reduced_cost_tol,
    )
end

function _solve_fixed_route_covering_by_cg(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    solver::BendersSolver,
    iteration::Union{Nothing, Int}=nothing,
    open_stations::Union{Nothing, Vector{Int}}=nothing,
)
    inner = solver.inner_solver
    cfg = inner.config
    optimizer_env = isnothing(cfg.optimizer_env) ? solver.config.optimizer_env : cfg.optimizer_env
    silent = cfg.silent || solver.config.silent
    mip_gap = isnothing(cfg.mip_gap) ? solver.config.mip_gap : cfg.mip_gap
    route_problem = _route_covering_problem_from_assignments(model, assignments, open_stations)
    cg_result = run_aggregate_od_route_column_generation(
        route_problem,
        data;
        optimizer_env=optimizer_env,
        verbose=!silent,
        max_cg_iters=inner.max_iterations,
        max_new_columns=inner.max_columns_per_iteration,
        n_candidates=inner.n_candidates,
        reduced_cost_tol=inner.reduced_cost_tol,
        pricing_time_limit_sec=inner.pricing_time_limit_sec,
        ip_time_limit_sec=inner.final_ip_time_limit_sec,
        cg_log_path=isnothing(iteration) ? nothing : _aggregate_od_route_cg_log_path(
            solver,
            "aggregate_od_route_benders_subiter$(iteration)_cg_iterations.csv",
        ),
        column_log_path=isnothing(iteration) ? nothing : _aggregate_od_route_cg_log_path(
            solver,
            "aggregate_od_route_benders_subiter$(iteration)_cg_columns.csv",
        ),
        dual_log_path=isnothing(iteration) ? nothing : _aggregate_od_route_cg_log_path(
            solver,
            "aggregate_od_route_benders_subiter$(iteration)_cg_duals.csv",
        ),
        mip_gap=mip_gap,
        silent=silent,
    )
    cg_result.cg_stop_reason == :optimality_proven ||
        throw(ArgumentError("RouteCoveringProblem CG did not prove pricing exhaustion; stop_reason=$(cg_result.cg_stop_reason)"))
    return cg_result
end

function _build_nearest_open_y_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    requests,
    demand,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    y_hat::Vector{Float64},
    optimizer_env,
    silent::Bool,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    @variable(m, 0 <= y[1:data.n_stations] <= 1)
    fix_cons = Dict(j => @constraint(m, y[j] == y_hat[j]) for j in 1:data.n_stations)

    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        ranked = _ranked_request_pairs(data, request, feasible_pairs[request])
        for pair in ranked
            x[(request, pair)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        end
        @constraint(m, sum(x[(request, pair)] for pair in ranked) == 1.0)
        for (rank_idx, pair) in enumerate(ranked)
            j, k = pair
            @constraint(m, x[(request, pair)] <= y[j])
            @constraint(m, x[(request, pair)] <= y[k])
            for prior in ranked[1:max(rank_idx - 1, 0)]
                pj, pk = prior
                @constraint(m, x[(request, pair)] <= 2.0 - y[pj] - y[pk])
            end
        end
    end

    @variable(m, 0 <= lambda[1:length(columns), 1:n_scenarios(data)] <= 1)
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            @constraint(m, sum(lambda[idx, s] for idx in covering; init=0.0) >= x[(request, pair)])
        end
    end

    obj = AffExpr(0.0)
    for request in requests
        for pair in feasible_pairs[request]
            add_to_expression!(obj, _assignment_pair_cost(data, request, pair), x[(request, pair)])
        end
    end
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

function _solve_nearest_open_y_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    requests,
    demand,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    y_hat::Vector{Float64},
    optimizer_env,
    silent::Bool,
)
    m, fix_cons = _build_nearest_open_y_subproblem_lp(
        data, model, mapping, requests, demand, feasible_pairs, columns, y_hat, optimizer_env, silent
    )
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersY full LP subproblem failed with status $(termination_status(m))"))
    return objective_value(m), Dict(j => dual(con) for (j, con) in fix_cons)
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

function _opt_result_from_benders(
    final_result::OptResult,
    metadata::Dict{String, Any},
)
    merged = copy(final_result.metadata)
    merge!(merged, metadata)
    return OptResult(
        final_result.termination_status,
        final_result.objective_value,
        final_result.solution,
        final_result.runtime_sec,
        final_result.model,
        final_result.mapping,
        final_result.detour_combos,
        final_result.counts,
        final_result.warm_start_solution,
        merged,
    )
end

function _benders_log_path(solver::BendersSolver)
    isnothing(solver.log_dir) && return nothing
    return joinpath(solver.log_dir, "aggregate_od_route_benders_iterations.csv")
end

function _flush_benders_iteration_log!(solver::BendersSolver, rows::Vector{NamedTuple})
    path = _benders_log_path(solver)
    isnothing(path) && return nothing
    _write_aggregate_od_route_cg_log_csv(
        path,
        rows;
        headers=[
            :iteration,
            :master_status,
            :lower_bound,
            :incumbent_objective,
            :outer_gap,
            :master_solve_seconds,
            :priming_cg_seconds,
            :subproblem_lp_seconds,
            :cuts_added,
            :feasibility_cuts_added,
            :optimality_cuts_added,
            :selected_assignment_count,
            :generated_column_pool_size,
            :inner_cg_iterations,
        ],
    )
    return nothing
end

function _outer_gap(lb::Float64, ub::Float64)
    isfinite(lb) && isfinite(ub) || return nothing
    abs(ub) <= 1e-9 && return abs(ub - lb)
    return abs(ub - lb) / max(1.0, abs(ub))
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
        get(x_hat, (request, selected_pair), 0.0) >= 0.5 ||
            throw(ArgumentError("BendersXY master produced no selected assignment for $(request)"))
        assignments[request] = selected_pair
    end
    return assignments
end

function _run_aggregate_od_route_nearest_open_benders_y(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    @objective(master, Min, sum(theta[cut_id] for cut_id in cut_ids))

    best_result = nothing
    best_ub = Inf
    feasibility_cuts = 0
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]

    for iteration in 1:solver.max_iterations
        master_start = time()
        optimize!(master)
        master_solve_seconds = time() - master_start
        primal_status(master) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersY master failed with status $(termination_status(master))"))
        lower_bound = objective_value(master)
        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)

        assignments, infeasible = _fixed_assignments_from_y(data, requests, feasible_pairs, y_hat)
        if !isempty(infeasible)
            feasibility_before = feasibility_cuts
            for request in infeasible
                _add_pair_open_feasibility_cut!(master, y, feasible_pairs[request])
                feasibility_cuts += 1
            end
            push!(benders_rows, (
                iteration=iteration,
                master_status=string(termination_status(master)),
                lower_bound=lower_bound,
                incumbent_objective=isfinite(best_ub) ? best_ub : nothing,
                outer_gap=_outer_gap(lower_bound, best_ub),
                master_solve_seconds=master_solve_seconds,
                priming_cg_seconds=0.0,
                subproblem_lp_seconds=0.0,
                cuts_added=feasibility_cuts - feasibility_before,
                feasibility_cuts_added=feasibility_cuts,
                optimality_cuts_added=optimality_cuts,
                selected_assignment_count=length(assignments),
                generated_column_pool_size=0,
                inner_cg_iterations=inner_cg_iters,
            ))
            _flush_benders_iteration_log!(solver, benders_rows)
            continue
        end

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
            v_hat, rho = _solve_nearest_open_y_subproblem_lp(
                data,
                model,
                mapping,
                group_requests,
                demand,
                feasible_pairs,
                cg_result.generated_columns,
                y_hat,
                optimizer_env,
                cfg.silent,
            )
            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if theta_hat[cut_id] < v_hat - solver.optimality_tol
                alpha = v_hat - sum(rho[j] * y_hat[j] for j in 1:data.n_stations)
                @constraint(master, theta[cut_id] >= alpha + sum(rho[j] * y[j] for j in 1:data.n_stations))
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
            master_solve_seconds=master_solve_seconds,
            priming_cg_seconds=priming_cg_seconds,
            subproblem_lp_seconds=subproblem_lp_seconds,
            cuts_added=cuts_added_this_iteration,
            feasibility_cuts_added=feasibility_cuts,
            optimality_cuts_added=optimality_cuts,
            selected_assignment_count=length(assignments),
            generated_column_pool_size=length(cg_result.generated_columns),
            inner_cg_iterations=inner_cg_iters,
        ))
        _flush_benders_iteration_log!(solver, benders_rows)

        if cuts_added_this_iteration == 0
            return _opt_result_from_benders(final_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersY",
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
                "benders_master_solve_time_sec" => master_solve_seconds,
                "benders_priming_cg_time_sec" => priming_cg_seconds,
                "benders_subproblem_lp_time_sec" => subproblem_lp_seconds,
                "feasibility_cuts_added" => feasibility_cuts,
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
    isnothing(best_result) && throw(ArgumentError("BendersY did not find a feasible incumbent"))
    throw(ArgumentError("BendersY did not converge within max_iterations=$(solver.max_iterations)"))
end

function _run_aggregate_od_route_nearest_open_benders_xy(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    if model.assignment_policy.feasibility_cut_style == :big_m_nearest
        validate_big_m_nearest_aggregate_od_route!(data, mapping)
    end
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    x = _add_nearest_open_master_x!(master, data, model, y, requests, feasible_pairs)

    obj = AffExpr(0.0)
    for request in requests, pair in feasible_pairs[request]
        add_to_expression!(obj, _assignment_pair_cost(data, request, pair), x[(request, pair)])
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
            return _opt_result_from_benders(final_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersXY",
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
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
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    x = _add_unrestricted_master_x!(master, y, requests, feasible_pairs)

    obj = AffExpr(0.0)
    for request in requests, pair in feasible_pairs[request]
        add_to_expression!(obj, _assignment_pair_cost(data, request, pair), x[(request, pair)])
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
            return _opt_result_from_benders(final_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersXY",
                "benders_cut_mode" => _benders_cut_mode_name(solver),
                "benders_iterations" => iteration,
                "benders_lower_bound" => lower_bound,
                "benders_incumbent_objective" => best_ub,
                "benders_outer_gap" => _outer_gap(lower_bound, best_ub),
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

function run_opt(
    instance::StationSelectionData,
    formulation::AggregateODRouteModel,
    solver::BendersSolver,
)
    if formulation.assignment_policy isa NearestOpenAggregateODAssignmentPolicy
        solver.decomposition isa BendersY &&
            return _run_aggregate_od_route_nearest_open_benders_y(instance, formulation, solver)
        solver.decomposition isa BendersXY &&
            return _run_aggregate_od_route_nearest_open_benders_xy(instance, formulation, solver)
        throw(ArgumentError("unsupported Benders decomposition $(typeof(solver.decomposition))"))
    end
    solver.decomposition isa BendersY &&
        throw(ArgumentError("AggregateODRouteModel free assignment Benders supports BendersXY only; BendersY is unsupported"))
    solver.decomposition isa BendersXY &&
        return _run_aggregate_od_route_free_benders_xy(instance, formulation, solver)
    throw(ArgumentError("unsupported Benders decomposition $(typeof(solver.decomposition))"))
end
