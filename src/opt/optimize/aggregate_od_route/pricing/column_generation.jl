"""
The AggregateODRouteModel column-generation main loop: single-pass column
generation, the full CG driver (`run_aggregate_od_route_column_generation`),
and the `run_opt` entry points that wire it into `ColumnGenerationSolver`.
"""

export AggregateODRouteColumnGenerationResult
export generate_aggregate_od_route_columns
export run_aggregate_od_route_column_generation

struct AggregateODRouteColumnGenerationResult
    status::Symbol
    final_result::OptResult
    lp_bound::Float64
    n_cg_iters::Int
    cg_stop_reason::Symbol
    generated_columns::Vector{AggregateODRouteColumn}
    selected_column_ids::Vector{Int}
    coverage::Dict{NTuple{3, Int}, Int}
    iteration_rows::Vector{NamedTuple}
    column_log_rows::Vector{NamedTuple}
    dual_log_rows::Vector{NamedTuple}
end

function _aggregate_od_route_coverage_summary(result::OptResult)::Dict{NTuple{3, Int}, Int}
    result.termination_status == MOI.OPTIMAL || return Dict{NTuple{3, Int}, Int}()
    mapping = result.mapping
    theta = result.model[:theta_compat]
    coverage = Dict{NTuple{3, Int}, Int}()
    for s in 1:length(mapping.scenarios)
        for (j, k) in get(mapping.active_jk_s, s, Tuple{Int, Int}[])
            count = 0
            for column_id in get(mapping.columns_by_pair, (j, k), Int[])
                theta_var = get(theta, (column_id, s), nothing)
                theta_var === nothing && continue
                value(theta_var) > 0.5 && (count += 1)
            end
            coverage[(j, k, s)] = count
        end
    end
    return coverage
end

function _selected_aggregate_od_route_column_ids(result::OptResult)::Vector{Int}
    result.termination_status == MOI.OPTIMAL || return Int[]
    theta = result.model[:theta_compat]
    ids = Set{Int}()
    for ((column_id, _s), theta_var) in theta
        value(theta_var) > 0.5 && push!(ids, column_id)
    end
    return sort!(collect(ids))
end

function generate_aggregate_od_route_columns(
    master_state::BuildResult,
    duals::AggregateODRouteCoverageDuals,
    data::StationSelectionData,
)
    m = master_state.model
    mapping = master_state.mapping
    model = AggregateODRouteModel(
        m[:aggregate_od_route_station_budget];
        route_regularization_weight=Float64(m[:aggregate_od_route_route_regularization_weight]),
        repositioning_time=Float64(m[:aggregate_od_route_repositioning_time]),
        max_walking_distance=mapping.max_walking_distance,
        max_wait_time=Float64(m[:aggregate_od_route_max_wait_time]),
        detour_factor=Float64(m[:aggregate_od_route_detour_factor]),
        max_stops=Int(m[:aggregate_od_route_max_stops]),
        max_visits_per_node=Int(m[:aggregate_od_route_max_visits_per_node]),
        max_new_columns=Int(m[:aggregate_od_route_max_new_columns]),
        n_candidates=Int(m[:aggregate_od_route_n_candidates]),
        pricing_time_limit_sec=Float64(m[:aggregate_od_route_pricing_time_limit_sec]),
        reduced_cost_tol=Float64(m[:aggregate_od_route_reduced_cost_tol]),
        relax_integrality=Bool(m[:aggregate_od_route_relax_integrality]),
    )

    next_column_id = isempty(mapping.column_ids) ? 1 : maximum(mapping.column_ids) + 1
    all_columns = AggregateODRouteColumn[]
    for s in 1:n_scenarios(data)
        pricing_duals = _scenario_pricing_duals(duals, s)
        pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
        new_columns, _exhausted, _stats = aggregate_od_route_pricing_by_label_setting(
            pricing_data,
            mapping.columns,
            pricing_duals;
            next_column_id=next_column_id,
            reduced_cost_tol=model.reduced_cost_tol,
            max_new_columns=model.max_new_columns,
            n_candidates=model.n_candidates,
            time_limit=model.pricing_time_limit_sec,
        )
        append!(all_columns, new_columns)
        next_column_id += length(new_columns)
    end

    dedup = Dict{Any, AggregateODRouteColumn}()
    for column in all_columns
        signature = _aggregate_od_route_column_signature(column)
        incumbent = get(dedup, signature, nothing)
        if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
            dedup[signature] = column
        end
    end
    columns = collect(values(dedup))
    sort!(columns, by=column -> (column.tau, string(column.od_pairs)))
    return columns
end

function _clone_for_final_mip(model::AggregateODRouteModel, columns::Vector{AggregateODRouteColumn})
    return AggregateODRouteModel(
        model.l;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        max_walking_distance        = model.max_walking_distance,
        max_wait_time               = model.max_wait_time,
        detour_factor               = model.detour_factor,
        max_stops                   = model.max_stops,
        max_visits_per_node         = model.max_visits_per_node,
        max_new_columns             = model.max_new_columns,
        n_candidates                = model.n_candidates,
        pricing_time_limit_sec      = model.pricing_time_limit_sec,
        reduced_cost_tol            = model.reduced_cost_tol,
        initial_columns             = columns,
        relax_integrality           = false,
        assignment_policy           = model.assignment_policy,
        allow_walk_only             = model.allow_walk_only,
        unmet_demand_penalty        = model.unmet_demand_penalty,
    )
end

function _clone_for_final_mip(model::RouteCoveringProblem, columns::Vector{AggregateODRouteColumn})
    return RouteCoveringProblem(
        model.l,
        model.open_stations,
        model.fixed_assignments;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        max_walking_distance        = model.max_walking_distance,
        max_wait_time               = model.max_wait_time,
        detour_factor               = model.detour_factor,
        max_stops                   = model.max_stops,
        max_visits_per_node         = model.max_visits_per_node,
        max_new_columns             = model.max_new_columns,
        n_candidates                = model.n_candidates,
        pricing_time_limit_sec      = model.pricing_time_limit_sec,
        reduced_cost_tol            = model.reduced_cost_tol,
        initial_columns             = columns,
        relax_integrality           = false,
        assignment_policy           = model.assignment_policy,
        allow_walk_only             = model.allow_walk_only,
        unmet_demand_penalty        = model.unmet_demand_penalty,
    )
end

function run_aggregate_od_route_column_generation(
    model::AnyAggregateODRouteModel,
    data::StationSelectionData;
    optimizer_env=nothing,
    verbose::Bool=true,
    cg_log_path::Union{Nothing, AbstractString}=nothing,
    column_log_path::Union{Nothing, AbstractString}=nothing,
    dual_log_path::Union{Nothing, AbstractString}=nothing,
    max_cg_iters::Int=10_000,
    max_iterations::Union{Nothing, Int}=nothing,
    max_new_columns::Int=model.max_new_columns,
    n_candidates::Int=max(model.n_candidates, max_new_columns),
    max_visits_per_node::Int=model.max_visits_per_node,
    reduced_cost_tol::Float64=model.reduced_cost_tol,
    pricing_time_limit_sec::Float64=model.pricing_time_limit_sec,
    pricing_initial_sec::Float64=pricing_time_limit_sec,
    pricing_ramp_factor::Float64=1.0,
    profile_pricing::Bool=false,
    ip_time_limit_sec::Float64=3600.0,
    mip_gap::Union{Float64, Nothing}=nothing,
    silent::Bool=!verbose,
)::AggregateODRouteColumnGenerationResult
    isnothing(max_iterations) || (max_cg_iters = max_iterations)
    max_cg_iters > 0 || throw(ArgumentError("max_cg_iters must be positive"))
    max_new_columns > 0 || throw(ArgumentError("max_new_columns must be positive"))
    n_candidates >= max_new_columns || throw(ArgumentError("n_candidates must be >= max_new_columns"))
    pricing_time_limit_sec > 0 || throw(ArgumentError("pricing_time_limit_sec must be positive"))
    pricing_initial_sec > 0 || throw(ArgumentError("pricing_initial_sec must be positive"))
    pricing_ramp_factor > 0 || throw(ArgumentError("pricing_ramp_factor must be positive"))
    ip_time_limit_sec > 0 || throw(ArgumentError("ip_time_limit_sec must be positive"))

    start_time = time()
    if isnothing(optimizer_env)
        optimizer_env = Gurobi.Env()
    end

    build_result = build_model(
        model,
        data;
        optimizer_env=optimizer_env,
        relax_integrality=true,
    )
    m = build_result.model
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    mapping = build_result.mapping
    logger = _create_aggregate_od_route_cg_logger(verbose=verbose, cg_log_path=cg_log_path)
    column_log_rows = NamedTuple[]
    dual_log_rows = NamedTuple[]
    generated_columns = AggregateODRouteColumn[]
    initial_pool_size = length(mapping.columns)
    n_active_pairs = sum(length(mapping.active_jk_s[s]) for s in 1:n_scenarios(data); init=0)
    _aggregate_od_route_log_header!(logger, n_active_pairs, initial_pool_size, max_cg_iters, pricing_time_limit_sec, max_new_columns)

    lp_bound = NaN
    cg_stop_reason = :max_cg_iters
    cg_iterations = 0
    last_status = :error

    for iteration in 1:max_cg_iters
        cg_iterations = iteration
        columns_before = length(mapping.columns)
        lp_start = time()
        optimize!(m)
        lp_solve_seconds = time() - lp_start
        term_status = termination_status(m)
        last_status = term_status == MOI.OPTIMAL ? :optimal :
            term_status == MOI.INFEASIBLE ? :infeasible :
            term_status == MOI.TIME_LIMIT ? :timeout : :error

        if primal_status(m) != MOI.FEASIBLE_POINT
            cg_stop_reason = :no_primal_solution
            _record_aggregate_od_route_cg_iteration!(logger, AggregateODRouteCGIterationLog(
                iteration, columns_before, length(mapping.columns), last_status,
                nothing, lp_solve_seconds, nothing, lp_solve_seconds,
                0, 0, 0, nothing, false, cg_stop_reason,
                nothing, nothing, nothing, nothing,
                nothing, nothing, nothing, nothing, nothing, nothing,
                nothing, nothing, nothing, nothing,
            ))
            break
        end

        lp_bound = objective_value(m)
        duals = extract_aggregate_od_route_coverage_duals(m)
        if !isnothing(dual_log_path)
            for ((j, k, s), val) in duals.sigma
                push!(dual_log_rows, (iteration=iteration, scenario=s, pickup=j, dropoff=k, sigma=val))
            end
        end
        dual_min, dual_max, dual_mean, dual_std = _aggregate_od_route_dual_stats(duals)

        pricing_started = time()
        iter_pricing_sec = min(
            pricing_time_limit_sec,
            pricing_initial_sec * (pricing_ramp_factor ^ (iteration - 1)),
        )

        next_column_id = isempty(mapping.column_ids) ? 1 : maximum(mapping.column_ids) + 1
        all_new_columns = AggregateODRouteColumn[]
        pricing_exhausted = true
        pricing_stats_by_scenario = []
        for s in 1:n_scenarios(data)
            pricing_duals = _scenario_pricing_duals(duals, s)
            pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
            pricing_data = AggregateODRoutePricingData(
                pricing_data.scenario,
                pricing_data.nodes,
                pricing_data.travel_cost,
                pricing_data.active_pairs,
                pricing_data.route_regularization_weight,
                pricing_data.repositioning_time,
                pricing_data.max_wait_time,
                pricing_data.detour_factor,
                pricing_data.max_stops,
                max_visits_per_node,
                pricing_data.bounded_max_stops,
            )
            new_columns_s, exhausted_s, stats_s = aggregate_od_route_pricing_by_label_setting(
                pricing_data,
                mapping.columns,
                pricing_duals;
                next_column_id=next_column_id,
                reduced_cost_tol=reduced_cost_tol,
                max_new_columns=max_new_columns,
                n_candidates=n_candidates,
                time_limit=iter_pricing_sec,
                max_visits_per_node=max_visits_per_node,
                profile=profile_pricing,
            )
            pricing_exhausted &= exhausted_s
            push!(pricing_stats_by_scenario, stats_s)
            append!(all_new_columns, new_columns_s)
            next_column_id += length(new_columns_s)
        end

        dedup = Dict{Any, AggregateODRouteColumn}()
        for column in all_new_columns
            signature = _aggregate_od_route_column_signature(column)
            incumbent = get(dedup, signature, nothing)
            if isnothing(incumbent) || column.tau < incumbent.tau - 1e-9
                dedup[signature] = column
            end
        end
        new_columns = collect(values(dedup))
        sort!(new_columns, by=column -> (
            get(column.metadata, "reduced_cost", Inf),
            column.tau,
            string(get(column.metadata, "route", ())),
        ))
        new_columns = new_columns[1:min(length(new_columns), max_new_columns)]
        pricing_seconds = time() - pricing_started
        iteration_seconds = lp_solve_seconds + pricing_seconds
        best_reduced_cost = isempty(new_columns) ? nothing :
            minimum(Float64(get(column.metadata, "reduced_cost", Inf)) for column in new_columns)

        if isempty(new_columns)
            cg_stop_reason = pricing_exhausted ? :optimality_proven : :no_columns_not_exhausted
            stats = _merge_pricing_stats(pricing_stats_by_scenario)
            _record_aggregate_od_route_cg_iteration!(logger, AggregateODRouteCGIterationLog(
                iteration, columns_before, length(mapping.columns), last_status,
                lp_bound, lp_solve_seconds, pricing_seconds, iteration_seconds,
                0, 0, 0, best_reduced_cost, pricing_exhausted, cg_stop_reason,
                dual_min, dual_max, dual_mean, dual_std,
                stats.labels_generated, stats.labels_rejected_by_dominance,
                stats.labels_removed_by_dominance, stats.stale_pops,
                stats.max_frontier_size, stats.max_live_labels,
                profile_pricing ? stats.t_queue_sec : nothing,
                profile_pricing ? stats.t_candidates_sec : nothing,
                profile_pricing ? stats.t_extension_sec : nothing,
                profile_pricing ? stats.t_dominance_sec : nothing,
            ))
            break
        end

        columns_added = 0
        columns_replaced = 0
        for column in new_columns
            _theta, action = add_or_update_aggregate_od_route_column!(build_result, column)
            action == :added && (columns_added += 1)
            action == :replaced && (columns_replaced += 1)
            action in (:added, :replaced) && push!(generated_columns, column)
            if !isnothing(column_log_path)
                route = get(column.metadata, "route", ())
                push!(column_log_rows, (
                    iteration=iteration,
                    action=string(action),
                    scenario=get(column.metadata, "scenario", missing),
                    column_id=column.id,
                    n_pairs=length(column.od_pairs),
                    tau=column.tau,
                    reduced_cost=get(column.metadata, "reduced_cost", missing),
                    route_length=length(route),
                    route=string(route),
                    pairs=string(Tuple(column.od_pairs)),
                ))
            end
        end

        stats = _merge_pricing_stats(pricing_stats_by_scenario)
        _record_aggregate_od_route_cg_iteration!(logger, AggregateODRouteCGIterationLog(
            iteration, columns_before, length(mapping.columns), last_status,
            lp_bound, lp_solve_seconds, pricing_seconds, iteration_seconds,
            length(new_columns), columns_added, columns_replaced,
            best_reduced_cost, pricing_exhausted, :continue,
            dual_min, dual_max, dual_mean, dual_std,
            stats.labels_generated, stats.labels_rejected_by_dominance,
            stats.labels_removed_by_dominance, stats.stale_pops,
            stats.max_frontier_size, stats.max_live_labels,
            profile_pricing ? stats.t_queue_sec : nothing,
            profile_pricing ? stats.t_candidates_sec : nothing,
            profile_pricing ? stats.t_extension_sec : nothing,
            profile_pricing ? stats.t_dominance_sec : nothing,
        ))
    end

    _flush_aggregate_od_route_cg_log!(logger)
    !isnothing(column_log_path) && _write_aggregate_od_route_cg_log_csv(
        String(column_log_path),
        column_log_rows;
        headers=[:iteration, :action, :scenario, :column_id, :n_pairs, :tau, :reduced_cost, :route_length, :route, :pairs],
    )
    !isnothing(dual_log_path) && _write_aggregate_od_route_cg_log_csv(
        String(dual_log_path),
        dual_log_rows;
        headers=[:iteration, :scenario, :pickup, :dropoff, :sigma],
    )
    _record_aggregate_od_route_cg_termination!(
        logger,
        AggregateODRouteCGTerminationLog(cg_stop_reason, cg_iterations, length(mapping.columns)),
    )

    final_model = _clone_for_final_mip(model, copy(mapping.columns))
    final_build_start = time()
    final_build = build_model(final_model, data; optimizer_env=optimizer_env)
    final_m = final_build.model
    silent && set_silent(final_m)
    set_optimizer_attribute(final_m, "TimeLimit", ip_time_limit_sec)
    isnothing(mip_gap) || set_optimizer_attribute(final_m, "MIPGap", mip_gap)
    final_build_time_sec = time() - final_build_start

    final_solve_start = time()
    optimize!(final_m)
    final_solve_time_sec = time() - final_solve_start
    final_term = termination_status(final_m)
    final_obj = final_term == MOI.OPTIMAL ? objective_value(final_m) : nothing
    final_solution = final_term == MOI.OPTIMAL ?
        (_value_recursive(final_m[:x]), _value_recursive(final_m[:y])) :
        nothing
    # No-op unless an endpoint nearest-open style built zp/zd indicators.
    final_term == MOI.OPTIMAL && assert_endpoint_chain_near_binary(final_m)
    final_result = OptResult(
        final_term,
        final_obj,
        final_solution,
        time() - start_time,
        final_m,
        final_build.mapping,
        final_build.detour_combos,
        final_build.counts,
        nothing,
        Dict{String, Any}(
            "build_time_sec" => final_build_time_sec,
            "solve_time_sec" => final_solve_time_sec,
            "cg_time_sec" => final_build_time_sec + final_solve_time_sec,
        ),
    )

    status = final_result.termination_status == MOI.OPTIMAL ? :optimal :
        final_result.termination_status == MOI.INFEASIBLE ? :infeasible :
        final_result.termination_status == MOI.TIME_LIMIT ? :timeout : :error
    cg_stop_reason == :optimality_proven || status != :optimal || (status = :feasible)

    return AggregateODRouteColumnGenerationResult(
        status,
        final_result,
        lp_bound,
        cg_iterations,
        cg_stop_reason,
        copy(mapping.columns),
        _selected_aggregate_od_route_column_ids(final_result),
        _aggregate_od_route_coverage_summary(final_result),
        copy(logger.iteration_rows),
        column_log_rows,
        dual_log_rows,
    )
end

function run_opt(
    instance::StationSelectionData,
    formulation::AnyAggregateODRouteModel,
    solver::ColumnGenerationSolver,
)
    cfg = solver.config
    result = run_aggregate_od_route_column_generation(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        verbose=!cfg.silent,
        cg_log_path=_aggregate_od_route_cg_log_path(solver, "aggregate_od_route_cg_iterations.csv"),
        column_log_path=_aggregate_od_route_cg_log_path(solver, "aggregate_od_route_cg_columns.csv"),
        dual_log_path=_aggregate_od_route_cg_log_path(solver, "aggregate_od_route_cg_duals.csv"),
        max_cg_iters=solver.max_iterations,
        max_new_columns=solver.max_columns_per_iteration,
        n_candidates=solver.n_candidates,
        reduced_cost_tol=solver.reduced_cost_tol,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        ip_time_limit_sec=solver.final_ip_time_limit_sec,
        mip_gap=cfg.mip_gap,
        silent=cfg.silent,
    )
    return result.final_result
end

function run_opt(
    instance::StationSelectionData,
    formulation::RouteCoveringProblem,
    solver::ColumnGenerationSolver,
)
    cfg = solver.config
    result = run_aggregate_od_route_column_generation(
        formulation,
        instance;
        optimizer_env=cfg.optimizer_env,
        verbose=!cfg.silent,
        cg_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_iterations.csv"),
        column_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_columns.csv"),
        dual_log_path=_aggregate_od_route_cg_log_path(solver, "route_covering_cg_duals.csv"),
        max_cg_iters=solver.max_iterations,
        max_new_columns=solver.max_columns_per_iteration,
        n_candidates=solver.n_candidates,
        reduced_cost_tol=solver.reduced_cost_tol,
        pricing_time_limit_sec=solver.pricing_time_limit_sec,
        ip_time_limit_sec=solver.final_ip_time_limit_sec,
        mip_gap=cfg.mip_gap,
        silent=cfg.silent,
    )
    return result.final_result
end
