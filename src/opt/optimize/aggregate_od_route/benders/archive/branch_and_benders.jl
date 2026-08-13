"""
Single-tree branch-and-Benders for nearest-open AggregateODRouteProblem.

The master contains exact walking cost, binary station decisions, and one full-routing recourse
variable per scenario. Certified standard Benders cuts are separated only at integer callback
solutions. BendersY cuts are affine in `y`; BendersYZ cuts are affine in the endpoint-chain
selectors `z`.
"""

mutable struct _BranchBendersStats
    unique_y::Int
    cache_hits::Int
    cuts_submitted::Int
    repeated_submissions::Int
    oracle_seconds::Float64
    priming_cg_seconds::Float64
    repricing_seconds::Float64
    reprice_rounds::Int
    reprice_columns::Int
    mw_completion_seconds::Float64
end

_BranchBendersStats() = _BranchBendersStats(0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0.0)

function _branch_benders_cache_get!(cache, key, stats::_BranchBendersStats, oracle::Function)
    if haskey(cache, key)
        stats.cache_hits += 1
        return cache[key], true
    end
    result = oracle()
    cache[key] = result
    stats.unique_y += 1
    return result, false
end

struct _BranchBendersOracleResult
    y_key::Tuple{Vararg{Int}}
    y::Vector{Float64}
    z::Union{Nothing, Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}}}
    recourse::Dict{Int, Float64}
    cuts::Dict{Int, BranchBendersCut}
    walking_cost::Float64
end

_branch_benders_y_key(y::AbstractVector{<:Real}) = Tuple(findall(v -> v > 0.5, y))

function _branch_benders_z_key(z_hat)
    isnothing(z_hat) && return ()
    entries = [
        (key, Tuple(Int(round(value)) for value in values))
        for (key, values) in z_hat
    ]
    sort!(entries; by=entry -> string(entry[1]))
    return Tuple(entries)
end

function _branch_benders_cache_key(solver::BranchAndBendersSolver, y_hat, z_hat)
    y_key = _branch_benders_y_key(y_hat)
    return solver.decomposition isa BendersY ?
        _BranchBendersYCacheKey(y_key) :
        _BranchBendersYZCacheKey(y_key, _branch_benders_z_key(z_hat))
end

function _branch_benders_proxy_solver(solver::BranchAndBendersSolver, optimizer_env)
    inner = solver.inner_solver
    inner_cfg = SolverConfig(
        optimizer_env=optimizer_env,
        silent=true,
        mip_gap=inner.config.mip_gap,
    )
    return BendersSolver(
        config=SolverConfig(optimizer_env=optimizer_env, silent=true, mip_gap=solver.config.mip_gap),
        decomposition=solver.decomposition,
        cut_mode=MultiCut(),
        inner_solver=ColumnGenerationSolver(
            config=inner_cfg,
            max_iterations=inner.max_iterations,
            max_columns_per_iteration=inner.max_columns_per_iteration,
            n_candidates=inner.n_candidates,
            reduced_cost_tol=solver.pricing_tolerance,
            pricing_time_limit_sec=inner.pricing_time_limit_sec,
            final_ip_time_limit_sec=inner.final_ip_time_limit_sec,
        ),
        reprice_subproblem=true,
        max_reprice_rounds=solver.max_reprice_rounds,
        cut_derivation=solver.cut_derivation,
        lifted_walking_objective=true,
        log_subiteration_details=false,
    )
end

function _branch_benders_z_hat(chain_cache, value_of, tol::Float64)
    z_hat = Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}}()
    for (key, vars) in chain_cache
        vals = Float64[value_of(var) for var in vars]
        all(v -> v <= tol || v >= 1.0 - tol, vals) || throw(ArgumentError(
            "BranchAndBendersSolver BendersYZ candidate has nonbinary endpoint selectors " *
            "for chain $(key): $(vals)"
        ))
        rounded = round.(vals)
        isapprox(sum(rounded), 1.0; atol=tol) || throw(ArgumentError(
            "BranchAndBendersSolver BendersYZ candidate does not select exactly one endpoint " *
            "for chain $(key): $(rounded)"
        ))
        z_hat[key] = rounded
    end
    return z_hat
end

function _branch_benders_cut_rhs(cut::BranchBendersCut, y_values, chain_values)
    if cut.decomposition == :y
        return cut.alpha + sum(coef * y_values[key] for (key, coef) in cut.beta; init=0.0)
    elseif cut.decomposition == :yz
        return cut.alpha + sum(coef * chain_values[key[1]][key[2]] for (key, coef) in cut.beta; init=0.0)
    end
    throw(ArgumentError("unknown branch-and-Benders cut decomposition $(cut.decomposition)"))
end

function _branch_benders_cut_constraint(cut::BranchBendersCut, theta, y, chain_cache)
    if cut.decomposition == :y
        return @build_constraint(
            theta[cut.block_id] >= cut.alpha +
                sum(coef * y[key] for (key, coef) in cut.beta; init=0.0)
        )
    elseif cut.decomposition == :yz
        return @build_constraint(
            theta[cut.block_id] >= cut.alpha +
                sum(coef * chain_cache[key[1]][key[2]] for (key, coef) in cut.beta; init=0.0)
        )
    end
    throw(ArgumentError("unknown branch-and-Benders cut decomposition $(cut.decomposition)"))
end

function _add_branch_benders_cut!(master, cut::BranchBendersCut, theta, y, chain_cache)
    con = _branch_benders_cut_constraint(cut, theta, y, chain_cache)
    return add_constraint(master, con)
end

function _validate_branch_benders_initial_cut!(
    cut::BranchBendersCut, solver::BranchAndBendersSolver, cut_ids, n_stations, chain_cache,
)
    expected = solver.decomposition isa BendersY ? :y : :yz
    cut.decomposition == expected || throw(ArgumentError(
        "initial cut decomposition=$(cut.decomposition) does not match solver decomposition=$(expected)"
    ))
    cut.block_id in cut_ids || throw(ArgumentError("initial cut has unknown block_id=$(cut.block_id)"))
    isfinite(cut.alpha) && isfinite(cut.recourse_value) && all(isfinite, values(cut.beta)) ||
        throw(ArgumentError("initial cut contains non-finite values"))
    if expected == :y
        all(k -> k isa Int && 1 <= k <= n_stations, keys(cut.beta)) ||
            throw(ArgumentError("BendersY initial cut has an invalid station coefficient key"))
    else
        all(k -> k isa Tuple && length(k) == 2 && haskey(chain_cache, k[1]) &&
                 k[2] isa Int && 1 <= k[2] <= length(chain_cache[k[1]]), keys(cut.beta)) ||
            throw(ArgumentError("BendersYZ initial cut has an invalid endpoint-chain coefficient key"))
    end
    return nothing
end

function _solve_branch_benders_oracle!(
    data,
    model,
    subproblem_model,
    solver::BranchAndBendersSolver,
    proxy_solver::BendersSolver,
    mapping,
    requests,
    demand,
    feasible_pairs,
    cut_groups,
    y_hat::Vector{Float64},
    z_hat,
    shared_pool::Vector{AggregateODRouteColumn},
    optimizer_env,
    stats::_BranchBendersStats,
    z_core,
)
    oracle_start = time()
    assignments, infeasible = _fixed_assignments_from_y(
        data, requests, feasible_pairs, y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only,
        allow_same_station=true,
    )
    isempty(infeasible) || throw(ArgumentError(
        "BranchAndBendersSolver candidate y=$(y_hat) has infeasible nearest-open requests $(infeasible)"
    ))

    cg_start = time()
    cg_result = _solve_fixed_route_covering_by_cg(
        data, subproblem_model, assignments, proxy_solver, nothing, _open_station_values(y_hat);
        seed_columns=shared_pool,
    )
    stats.priming_cg_seconds += time() - cg_start
    cg_result.cg_stop_reason == :optimality_proven || throw(ArgumentError(
        "BranchAndBendersSolver priming CG was not certified for y=$(y_hat): " *
        "stop_reason=$(cg_result.cg_stop_reason)"
    ))
    append!(shared_pool, cg_result.generated_columns)
    unique_pool = _deduplicate_aggregate_od_route_columns(shared_pool)
    empty!(shared_pool)
    append!(shared_pool, unique_pool)

    recourse = Dict{Int, Float64}()
    cuts = Dict{Int, BranchBendersCut}()
    for cut_id in sort!(collect(keys(cut_groups)))
        group_requests = cut_groups[cut_id]
        reprice_start = time()
        if solver.decomposition isa BendersY
            v_hat, rho, repriced_pool, n_new, rounds, exhausted, delta =
                _solve_nearest_open_y_subproblem_lp_with_repricing(
                    data, subproblem_model, mapping, group_requests, demand, feasible_pairs,
                    shared_pool, y_hat, optimizer_env, true;
                    max_reprice_rounds=solver.max_reprice_rounds,
                )
            exhausted || throw(ArgumentError("BendersY pricing did not exhaust for block $(cut_id)"))
            delta <= solver.cut_tightness_tolerance * max(1.0, abs(v_hat)) || throw(ArgumentError(
                "BendersY repricing changed the certified objective for block $(cut_id): delta=$(delta)"
            ))
            alpha = v_hat - sum(rho[j] * y_hat[j] for j in keys(rho); init=0.0)
            beta = Dict{Any, Float64}(j => coef for (j, coef) in rho)
            cut = BranchBendersCut(:y, cut_id, alpha, beta, v_hat)
            tight_value = _branch_benders_cut_rhs(cut, y_hat, nothing)
        elseif solver.cut_derivation == :restricted_mw_fixed_pi
            assignments_for_group = Dict(request => assignments[request] for request in group_requests)
            certified, q_bar = _certified_qbar(
                data, subproblem_model, cg_result, group_requests, assignments_for_group,
            )
            mw_result = _restricted_yz_optimality_cut(
                data, subproblem_model, proxy_solver, group_requests, feasible_pairs,
                z_hat, assignments_for_group, _open_station_values(y_hat), z_core,
                optimizer_env, :maximize_core; certified=certified, Q_bar=q_bar,
            )
            mw_result.status == :ok || throw(ArgumentError(
                "BranchAndBendersSolver restricted YZ completion failed for block $(cut_id): " *
                "status=$(mw_result.status)"
            ))
            stats.mw_completion_seconds += mw_result.completion_runtime_sec
            v_hat = q_bar
            cut = BranchBendersCut(
                :yz, cut_id, mw_result.cut_constant, mw_result.beta, v_hat,
            )
            repriced_pool = certified.pool
            n_new = 0
            rounds = 0
            tight_value = _branch_benders_cut_rhs(cut, y_hat, z_hat)
        else
            v_hat, rho, repriced_pool, n_new, rounds, exhausted, delta =
                _solve_yz_route_subproblem_lp_with_repricing(
                    data, subproblem_model, mapping, group_requests, feasible_pairs,
                    shared_pool, z_hat, optimizer_env, true;
                    max_reprice_rounds=solver.max_reprice_rounds,
                )
            exhausted || throw(ArgumentError("BendersYZ pricing did not exhaust for block $(cut_id)"))
            delta <= solver.cut_tightness_tolerance * max(1.0, abs(v_hat)) || throw(ArgumentError(
                "BendersYZ repricing changed the certified objective for block $(cut_id): delta=$(delta)"
            ))
            alpha = v_hat - sum(rho[key] * z_hat[key[1]][key[2]] for key in keys(rho); init=0.0)
            beta = Dict{Any, Float64}(key => coef for (key, coef) in rho)
            cut = BranchBendersCut(:yz, cut_id, alpha, beta, v_hat)
            tight_value = _branch_benders_cut_rhs(cut, y_hat, z_hat)
        end
        solver.cut_derivation == :standard &&
            (stats.repricing_seconds += time() - reprice_start)
        stats.reprice_rounds += rounds
        stats.reprice_columns += n_new
        isapprox(tight_value, v_hat; atol=solver.cut_tightness_tolerance,
                 rtol=solver.cut_tightness_tolerance) || throw(ArgumentError(
            "BranchAndBendersSolver cut is not tight for y=$(y_hat), block=$(cut_id): " *
            "cut=$(tight_value), recourse=$(v_hat)"
        ))
        recourse[cut_id] = v_hat
        cuts[cut_id] = cut
        append!(shared_pool, repriced_pool)
        unique_pool = _deduplicate_aggregate_od_route_columns(shared_pool)
        empty!(shared_pool)
        append!(shared_pool, unique_pool)
    end
    stats.oracle_seconds += time() - oracle_start
    return _BranchBendersOracleResult(
        _branch_benders_y_key(y_hat), y_hat,
        solver.decomposition isa BendersYZ ? deepcopy(z_hat) : nothing,
        recourse, cuts,
        _lifted_walking_cost(data, model, assignments),
    )
end

include("branch_and_benders/state.jl")
include("branch_and_benders/master.jl")

function run_opt(
    data::StationSelectionData,
    model::AggregateODRouteProblem,
    solver::BranchAndBendersSolver,
)
    run_opt_start = time()
    model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy || throw(ArgumentError(
        "BranchAndBendersSolver supports NearestOpenAggregateODAssignmentPolicy only"
    ))
    _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style) || throw(ArgumentError(
        "BranchAndBendersSolver requires :big_m_nearest or :endpoint_chain"
    ))
    cfg = solver.config
    master_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    oracle_env = Gurobi.Env()
    mapping = create_map(model, data)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("BranchAndBendersSolver requires positive demand"))
    cut_groups = _benders_cut_groups(requests, MultiCut())
    cut_ids = sort!(collect(keys(cut_groups)))
    _check_aggregate_od_route_endpoint_feasibility!(data, model, requests, master_env, cfg.silent)
    validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    subproblem_model = _unit_weighted_routing_model(model)
    proxy_solver = _branch_benders_proxy_solver(solver, oracle_env)
    z_core = if solver.cut_derivation == :restricted_mw_fixed_pi
        model.assignment_policy.feasibility_cut_style == :big_m_nearest || throw(ArgumentError(
            "BranchAndBendersSolver restricted MW cuts require :big_m_nearest"
        ))
        _yz_joint_core_point(
            data, subproblem_model, requests, oracle_env, true,
        ).z
    else
        nothing
    end

    artifacts, master_mip_gap = _build_branch_benders_master(
        data, model, solver, requests, feasible_pairs, cut_ids, master_env,
    )
    master, y, theta, chain_cache =
        artifacts.model, artifacts.y, artifacts.theta, artifacts.chain_cache
    state = _BranchBendersRuntimeState(model.initial_columns)

    function evaluate_y(y_hat::Vector{Float64}, z_hat)
        key = _branch_benders_cache_key(solver, y_hat, z_hat)
        result, cached = _branch_benders_cache_get!(state.cache, key, state.stats, () -> begin
            fresh = _solve_branch_benders_oracle!(
                data, model, subproblem_model, solver, proxy_solver, mapping, requests, demand,
                feasible_pairs, cut_groups, y_hat, z_hat, state.shared_pool, oracle_env,
                state.stats, z_core,
            )
            candidate_ub = fresh.walking_cost +
                model.route_regularization_weight * sum(values(fresh.recourse))
            if candidate_ub < state.best_ub
                state.best_ub = candidate_ub
                state.best_result = fresh
            end
            fresh
        end)
        if cached
            candidate_ub = result.walking_cost +
                model.route_regularization_weight * sum(values(result.recourse))
            if candidate_ub < state.best_ub
                state.best_ub = candidate_ub
                state.best_result = result
            end
        end
        return result, cached, key
    end

    # Optional ordinary-cut warm rounds. They are deliberately absent by default.
    for _round in 1:solver.initial_benders_cut_rounds
        optimize!(master)
        primal_status(master) == MOI.FEASIBLE_POINT || throw(ArgumentError(
            "BranchAndBendersSolver initial-cut master failed with status $(termination_status(master))"
        ))
        y_hat = round.([value(y[j]) for j in 1:data.n_stations])
        z_hat = _branch_benders_z_hat(chain_cache, value, max(1e-3, solver.integrality_tolerance))
        result, _, _ = evaluate_y(y_hat, z_hat)
        added = 0
        for cut_id in cut_ids
            cut = result.cuts[cut_id]
            rhs = _branch_benders_cut_rhs(cut, y_hat, z_hat)
            if value(theta[cut_id]) < rhs - solver.lazy_cut_tolerance
                _add_branch_benders_cut!(master, cut, theta, y, chain_cache)
                added += 1
            end
        end
        added == 0 && break
    end


    function lazy_callback(cb_data)
        callback_node_status(cb_data, master) == MOI.CALLBACK_NODE_STATUS_INTEGER || return
        state.callback_count += 1
        y_values = Float64[callback_value(cb_data, y[j]) for j in 1:data.n_stations]
        all(v -> v <= solver.integrality_tolerance || v >= 1.0 - solver.integrality_tolerance, y_values) ||
            throw(ArgumentError(
                "Gurobi delivered a MIPSOL candidate outside the configured IntFeasTol=" *
                "$(solver.integrality_tolerance); refusing to accept it without separation: y=$(y_values)"
            ))
        y_hat = round.(y_values)
        z_values = _branch_benders_z_hat(
            chain_cache, var -> callback_value(cb_data, var), max(1e-3, solver.integrality_tolerance),
        )
        result, was_cached, cache_key = evaluate_y(y_hat, z_values)
        candidate_ub = result.walking_cost +
            model.route_regularization_weight * sum(values(result.recourse))
        function gurobi_mipsol_value(attribute)
            output = Ref{Cdouble}(NaN)
            ret = Gurobi.GRBcbget(cb_data, cb_data.cb_where, attribute, output)
            return ret == 0 ? Float64(output[]) : NaN
        end
        gurobi_incumbent = gurobi_mipsol_value(Gurobi.GRB_CB_MIPSOL_OBJBST)
        gurobi_lower_bound = gurobi_mipsol_value(Gurobi.GRB_CB_MIPSOL_OBJBND)
        gurobi_node_count = gurobi_mipsol_value(Gurobi.GRB_CB_MIPSOL_NODCNT)
        certified_gap = isfinite(state.best_ub) && isfinite(gurobi_lower_bound) ?
            (state.best_ub - gurobi_lower_bound) / max(1.0, abs(state.best_ub)) : NaN
        violated_blocks = Int[]
        for cut_id in cut_ids
            cut = result.cuts[cut_id]
            theta_value = callback_value(cb_data, theta[cut_id])
            rhs = _branch_benders_cut_rhs(cut, y_values, z_values)
            theta_value < rhs - solver.lazy_cut_tolerance || continue
            push!(violated_blocks, cut_id)
            signature = (cache_key, cut_id)
            signature in state.submitted && (state.stats.repeated_submissions += 1)
            push!(state.submitted, signature)
            MOI.submit(
                master, MOI.LazyConstraint(cb_data),
                _branch_benders_cut_constraint(cut, theta, y, chain_cache),
            )
            state.stats.cuts_submitted += 1
        end
        event = (
            callback=state.callback_count,
            elapsed_sec=time() - solve_start,
            open_station_indices=join(result.y_key, ";"),
            cache_hit=was_cached,
            candidate_exact_ub=candidate_ub,
            best_certified_ub=state.best_ub,
            gurobi_master_incumbent=gurobi_incumbent,
            gurobi_lower_bound=gurobi_lower_bound,
            certified_relative_gap=certified_gap,
            gurobi_node_count=gurobi_node_count,
            violated_blocks=join(violated_blocks, ";"),
            n_violated=length(violated_blocks),
            unique_y=state.stats.unique_y,
            cache_hits=state.stats.cache_hits,
            cuts_submitted=state.stats.cuts_submitted,
            repeated_submissions=state.stats.repeated_submissions,
            oracle_seconds=state.stats.oracle_seconds,
            priming_cg_seconds=state.stats.priming_cg_seconds,
            repricing_seconds=state.stats.repricing_seconds,
            mw_completion_seconds=state.stats.mw_completion_seconds,
            shared_pool_size=length(state.shared_pool),
        )
        push!(state.callback_events, event)
        @info "BranchAndBenders integer candidate" callback=event.callback elapsed_sec=event.elapsed_sec open_station_indices=event.open_station_indices cache_hit=event.cache_hit candidate_exact_ub=event.candidate_exact_ub best_certified_ub=event.best_certified_ub gurobi_lower_bound=event.gurobi_lower_bound certified_relative_gap=event.certified_relative_gap gurobi_node_count=event.gurobi_node_count violated_blocks=event.violated_blocks cuts_submitted=event.cuts_submitted unique_y=event.unique_y oracle_seconds=event.oracle_seconds
        return
    end
    MOI.set(master, MOI.LazyConstraintCallback(), lazy_callback)

    solve_start = time()
    pre_optimize_seconds = solve_start - run_opt_start
    optimize!(master)
    master_optimize_seconds = time() - solve_start
    term = termination_status(master)

    # Persist the callback trace before any final correctness assertion can fail.
    if !isnothing(solver.log_dir) && !isempty(state.callback_events)
        mkpath(solver.log_dir)
        CSV.write(
            joinpath(solver.log_dir, "aggregate_od_route_branch_benders_callbacks.csv"),
            DataFrame(state.callback_events),
        )
    end
    isnothing(state.best_result) && throw(ArgumentError(
        "BranchAndBendersSolver ended with status $(term) without a certified integer candidate"
    ))
    best_result = state.best_result::_BranchBendersOracleResult
    lower_bound = try objective_bound(master) catch; NaN end
    exact_objective = best_result.walking_cost +
        model.route_regularization_weight * sum(values(best_result.recourse))
    if term == MOI.OPTIMAL
        allowed_gap = max(
            solver.cut_tightness_tolerance,
            master_mip_gap,
        )
        isapprox(lower_bound, exact_objective;
                 atol=allowed_gap * max(1.0, abs(exact_objective)),
                 rtol=allowed_gap) || throw(ArgumentError(
            "BranchAndBendersSolver reported OPTIMAL but certified objective=$(exact_objective) " *
            "and Gurobi bound=$(lower_bound) disagree"
        ))
    end
    gap = isfinite(lower_bound) ? abs(exact_objective - lower_bound) / max(1.0, abs(exact_objective)) : NaN
    run_opt_seconds = time() - run_opt_start
    metadata = Dict{String, Any}(
        "branch_benders_decomposition" => solver.decomposition isa BendersY ? "BendersY" : "BendersYZ",
        "branch_benders_cut_derivation" => string(solver.cut_derivation),
        "branch_benders_open_stations" => collect(best_result.y_key),
        "branch_benders_recourse_by_scenario" => best_result.recourse,
        "branch_benders_walking_cost" => best_result.walking_cost,
        "branch_benders_certified_ub" => exact_objective,
        "branch_benders_global_lb" => lower_bound,
        "branch_benders_gap_relative" => gap,
        "branch_benders_unique_y" => state.stats.unique_y,
        "branch_benders_unique_first_stage_states" => state.stats.unique_y,
        "branch_benders_cache_hits" => state.stats.cache_hits,
        "branch_benders_callback_count" => state.callback_count,
        "branch_benders_cuts_submitted" => state.stats.cuts_submitted,
        "branch_benders_repeated_submissions" => state.stats.repeated_submissions,
        "branch_benders_oracle_seconds" => state.stats.oracle_seconds,
        "branch_benders_priming_cg_seconds" => state.stats.priming_cg_seconds,
        "branch_benders_repricing_seconds" => state.stats.repricing_seconds,
        "branch_benders_mw_completion_seconds" => state.stats.mw_completion_seconds,
        "branch_benders_reprice_rounds" => state.stats.reprice_rounds,
        "branch_benders_reprice_columns" => state.stats.reprice_columns,
        "branch_benders_shared_pool_size" => length(state.shared_pool),
        "branch_benders_node_count" => try MOI.get(backend(master), MOI.NodeCount()) catch; missing end,
        "branch_benders_threads" => 1,
        # `master_optimize_seconds` includes all synchronous callback/oracle work.
        # The oracle component timings below are therefore nested, not additive.
        "branch_benders_pre_optimize_seconds" => pre_optimize_seconds,
        "branch_benders_master_optimize_seconds" => master_optimize_seconds,
        "branch_benders_run_opt_seconds" => run_opt_seconds,
        "branch_benders_generated_cuts" => BranchBendersCut[
            result.cuts[cut_id] for result in values(state.cache) for cut_id in sort!(collect(keys(result.cuts)))
        ],
        "branch_benders_oracle_results" => collect(values(state.cache)),
    )
    if !isnothing(solver.log_dir)
        mkpath(solver.log_dir)
        CSV.write(
            joinpath(solver.log_dir, "aggregate_od_route_branch_benders_summary.csv"),
            DataFrame([(
                termination_status=string(term), decomposition=metadata["branch_benders_decomposition"],
                cut_derivation=metadata["branch_benders_cut_derivation"],
                objective=exact_objective, lower_bound=lower_bound, relative_gap=gap,
                nodes=metadata["branch_benders_node_count"], unique_y=state.stats.unique_y,
                callback_count=state.callback_count, cache_hits=state.stats.cache_hits,
                cuts_submitted=state.stats.cuts_submitted,
                repeated_submissions=state.stats.repeated_submissions,
                oracle_seconds=state.stats.oracle_seconds,
                priming_cg_seconds=state.stats.priming_cg_seconds,
                repricing_seconds=state.stats.repricing_seconds,
                mw_completion_seconds=state.stats.mw_completion_seconds,
                pre_optimize_seconds=pre_optimize_seconds,
                master_optimize_seconds=master_optimize_seconds,
                run_opt_seconds=run_opt_seconds,
                shared_pool_size=length(state.shared_pool),
            )]),
        )
    end
    solution = (Dict{Any, Float64}(), copy(best_result.y))
    return OptResult(
        term, exact_objective, solution, master_optimize_seconds,
        master, mapping, nothing, nothing, nothing, metadata,
    )
end
