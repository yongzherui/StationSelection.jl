"""
Benders-YZ decomposition for AggregateODRouteModel (NearestOpen policy only): master =
y,z; subproblem = x,theta (see `iterative_strategy_types.jl`'s `BendersYZ` docstring).
Requires `reprice_subproblem=true` for a provably optimal result -- see
`_solve_yz_route_subproblem_lp_with_repricing`'s docstring.
"""

"""
    _build_yz_route_subproblem_lp(data, model, requests, feasible_pairs, columns, z_hat, optimizer_env, silent)

BendersYZ's per-cut-group subproblem LP: unlike `_build_xy_route_subproblem_lp`
(which fixes `x` and leaves the assignment cost entirely to the master), this
fixes `z` -- `x` and `θ` (via `lambda`) are both free here, and the walking
cost lives in this LP's objective since the master carries none (mirrors
`_build_nearest_open_y_subproblem_lp`'s objective shape, not
`_build_xy_route_subproblem_lp`'s). There is no `y` in this LP at all: `z` is
built bare and fixed directly to `z_hat`, using `_sorted_endpoint_chain`
(`aggregate_od_route_benders_y_mw_cut.jl`) to get the same
`(key, sorted_stations)` a request's physical endpoints resolve to in the
master's `nearest_endpoint_chain_cache` -- guaranteed to line up positionally
since both use the identical `sortperm`+`_endpoint_chain_key` logic. A local
(not model-attached) cache dedupes `z` construction within this one LP build
when a physical endpoint recurs across the group's requests.
"""
function _build_yz_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    optimizer_env,
    silent::Bool,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    z_cache = Dict{_AggregateODRouteEndpointChainKey, Vector{VariableRef}}()
    fix_cons = Dict{Tuple{_AggregateODRouteEndpointChainKey, Int}, ConstraintRef}()
    fixed_z! = key -> get!(z_cache, key) do
        haskey(z_hat, key) || throw(ArgumentError(
            "BendersYZ subproblem: no master z_hat entry for chain key $(key)"
        ))
        n = length(key[2])
        zvar = @variable(m, [1:n], lower_bound = 0.0, upper_bound = 1.0)
        for i in 1:n
            fix_cons[(key, i)] = @constraint(m, zvar[i] == z_hat[key][i])
        end
        zvar
    end

    unmet_demand_active = !isnothing(model.unmet_demand_penalty)
    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    u = unmet_demand_active ? Dict{NTuple{3, Int}, VariableRef}() : nothing
    for request in requests
        _s, o, d = request
        pairs = feasible_pairs[request]
        for pair in pairs
            x[(request, pair)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        end
        if unmet_demand_active
            u[request] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
            @constraint(m, sum(x[(request, pair)] for pair in pairs; init=0.0) == u[request])
        else
            @constraint(m, sum(x[(request, pair)] for pair in pairs; init=0.0) == 1.0)
        end
        x_by_pair = Dict(pair => x[(request, pair)] for pair in pairs)

        pickup_key, sorted_pickups, _pickup_costs = _sorted_endpoint_chain(data, o, model.max_walking_distance, :pickup)
        dropoff_key, sorted_dropoffs, _dropoff_costs = _sorted_endpoint_chain(data, d, model.max_walking_distance, :dropoff)
        zp = fixed_z!(pickup_key)
        zd = fixed_z!(dropoff_key)
        pickup_rank = Dict(station => idx for (idx, station) in enumerate(sorted_pickups))
        dropoff_rank = Dict(station => idx for (idx, station) in enumerate(sorted_dropoffs))
        real_pairs = filter(!is_walk_only_pair, pairs)
        _add_endpoint_x_linking!(
            m, real_pairs, pairs, x_by_pair, zp, zd, pickup_rank, dropoff_rank, sorted_pickups, sorted_dropoffs,
        )
    end

    @variable(m, 0 <= lambda[1:length(columns), 1:n_scenarios(data)] <= 1)
    cover_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            # See _build_nearest_open_y_subproblem_lp: walk-only and same-station
            # assignments use no vehicle route, so no coverage row for them.
            requires_no_vehicle_route(pair) && continue
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            cover_cons[(request, pair)] =
                @constraint(m, sum(lambda[idx, s] for idx in covering; init=0.0) >= x[(request, pair)])
        end
    end

    obj = AffExpr(0.0)
    for request in requests
        for pair in feasible_pairs[request]
            add_to_expression!(obj, _assignment_pair_cost(data, request, pair; weight=model.walk_cost_weight), x[(request, pair)])
        end
        if unmet_demand_active
            add_to_expression!(obj, model.unmet_demand_penalty)
            add_to_expression!(obj, -model.unmet_demand_penalty, u[request])
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
    m[:u] = u
    m[:x] = x
    return m, fix_cons, cover_cons
end

function _solve_yz_route_subproblem_lp(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    optimizer_env,
    silent::Bool,
)
    m, fix_cons, _cover_cons = _build_yz_route_subproblem_lp(
        data, model, requests, feasible_pairs, columns, z_hat, optimizer_env, silent
    )
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersYZ route LP subproblem failed with status $(termination_status(m))"))
    assert_service_near_binary(m)
    return objective_value(m), Dict(key => dual(con) for (key, con) in fix_cons)
end

"""
    _solve_yz_route_subproblem_lp_with_repricing(...)

BendersYZ analogue of [`_solve_nearest_open_y_subproblem_lp_with_repricing`](@ref):
`_build_yz_route_subproblem_lp` also lets `x` vary freely (only `z` is fixed),
so a column pool proven exhaustive by `_solve_fixed_route_covering_by_cg` for
just the one nearest-open assignment at `y_hat` is not necessarily complete
for *this* LP's own, more general dual structure -- the same completeness gap
`_solve_nearest_open_y_subproblem_lp_with_repricing`'s docstring describes for
BendersY (confirmed empirically: the plain, non-repricing
`_solve_yz_route_subproblem_lp` converges BendersYZ to a genuinely
suboptimal-but-correctly-costed `y` on the real-data alignment fixture).
Reuses `_extract_nearest_open_y_subproblem_coverage_duals` unchanged since
`cover_cons` has the identical `(request, pair) => ConstraintRef` shape. Like
`_solve_nearest_open_y_subproblem_lp_with_repricing`, this is a certification
check for dual-basis degeneracy: newly priced columns may appear under the LP's
chosen duals, but after adding them the LP objective must remain unchanged. If
the objective improves, the restricted subproblem value used for the cut was not
certified and this routine throws.
"""
function _solve_yz_route_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    requests,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    z_hat::Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}},
    optimizer_env,
    silent::Bool;
    max_reprice_rounds::Int=20,
)
    pool = copy(columns)
    v_hat = NaN
    baseline_v_hat = nothing
    max_objective_delta = 0.0
    rho = Dict{Tuple{_AggregateODRouteEndpointChainKey, Int}, Float64}()
    n_new_columns_total = 0
    rounds = 0
    fully_exhausted = true
    for round in 1:max_reprice_rounds
        rounds = round
        m, fix_cons, cover_cons = _build_yz_route_subproblem_lp(
            data, model, requests, feasible_pairs, pool, z_hat, optimizer_env, silent
        )
        optimize!(m)
        primal_status(m) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersYZ repricing subproblem LP failed with status $(termination_status(m))"))
        assert_endpoint_chain_near_binary(m)
        assert_service_near_binary(m)
        v_hat = objective_value(m)
        if isnothing(baseline_v_hat)
            baseline_v_hat = v_hat
        else
            objective_delta = abs(v_hat - baseline_v_hat)
            max_objective_delta = max(max_objective_delta, objective_delta)
            objective_delta <= 1e-6 * max(1.0, abs(baseline_v_hat)) || throw(ArgumentError(
                "BendersYZ repricing changed subproblem objective: before=$(baseline_v_hat), " *
                "after=$(v_hat), delta=$(objective_delta). Repricing is expected to certify the " *
                "same LP value, not improve it."
            ))
        end
        rho = Dict(key => dual(con) for (key, con) in fix_cons)

        duals = _extract_nearest_open_y_subproblem_coverage_duals(cover_cons)
        next_column_id = isempty(pool) ? 1 : maximum(column.id for column in pool) + 1
        all_new_columns = AggregateODRouteColumn[]
        pricing_exhausted = true
        for s in 1:n_scenarios(data)
            pricing_duals = _scenario_pricing_duals(duals, s)
            pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
            new_columns_s, exhausted_s, _stats = aggregate_od_route_pricing_by_label_setting(
                pricing_data,
                pool,
                pricing_duals;
                next_column_id=next_column_id,
                reduced_cost_tol=model.reduced_cost_tol,
                max_new_columns=model.max_new_columns,
                n_candidates=model.n_candidates,
                time_limit=model.pricing_time_limit_sec,
                max_visits_per_node=model.max_visits_per_node,
            )
            pricing_exhausted &= exhausted_s
            append!(all_new_columns, new_columns_s)
            next_column_id += length(new_columns_s)
        end
        fully_exhausted = pricing_exhausted
        isempty(all_new_columns) && break
        pricing_exhausted ||
            @warn "BendersYZ subproblem repricing: pricing hit its time limit before exhausting the search " *
                "while new columns were still being found -- completeness not fully proven this round" round
        @warn "BendersYZ subproblem repricing found columns beyond the seeded pool -- pool was not complete " *
            "for this subproblem's own dual structure (dual degeneracy or genuine pool gap)" round n_new=length(all_new_columns)
        n_new_columns_total += length(all_new_columns)
        pool = _deduplicate_aggregate_od_route_columns(vcat(pool, all_new_columns))
    end
    return v_hat, rho, pool, n_new_columns_total, rounds, fully_exhausted, max_objective_delta
end

"""
    _run_aggregate_od_route_nearest_open_benders_yz(data, model, solver)

Benders-YZ (Variant 2): master = `y,z`; subproblem = `x,θ`. Only
`feasibility_cut_style in (:big_m_nearest, :endpoint_chain)` is supported --
`:pair_chain` has no addressable `z` separate from `x`. Structurally a hybrid
of `_run_aggregate_od_route_nearest_open_benders_y` (master has no `x`, so a
rounded `y_hat` can still fail to admit a valid nearest-open assignment via an
endpoint collision -- reuses that function's feasibility-cut branch verbatim)
and `_run_aggregate_od_route_nearest_open_benders_xy` (CG-priming and the
per-cut-group optimality-cut loop, both reused as-is; safe to derive
CG-priming `assignments` from `y_hat` alone via `_fixed_assignments_from_y`,
ignoring `z_hat`, since the chain constraints make that a deterministic
bijection whenever the master is feasible).

Unlike `BendersXY`, whose subproblem fixes `x` fully (so its CG priming is
always exhaustive for exactly the LP the cut is drawn from), BendersYZ's
subproblem fixes only `z` and lets `x` vary freely -- the same structural gap
`BendersY`'s subproblem has (see `_solve_nearest_open_y_subproblem_lp_with_repricing`'s
docstring), confirmed empirically to cause premature convergence to a
correctly-costed but suboptimal `y` without repricing. `solver.reprice_subproblem=true`
routes each cut through `_solve_yz_route_subproblem_lp_with_repricing` instead
of the plain `_solve_yz_route_subproblem_lp` and should be passed whenever
BendersYZ's result needs to be provably optimal, exactly as with `BendersY`.
"""
function _run_aggregate_od_route_nearest_open_benders_yz(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver,
)
    _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style) ||
        throw(ArgumentError(
            "BendersYZ requires NearestOpenAggregateODAssignmentPolicy(:big_m_nearest) or " *
            "(:endpoint_chain); got :$(model.assignment_policy.feasibility_cut_style) -- :pair_chain has no " *
            "addressable z separate from x, so there is nothing for BendersYZ's master to lift."
        ))
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
    _check_aggregate_od_route_endpoint_feasibility!(data, model, requests, optimizer_env, cfg.silent)
    cut_groups = _benders_cut_groups(requests, solver.cut_mode)
    cut_ids = sort!(collect(keys(cut_groups)))

    if solver.cut_derivation != :standard
        (model.assignment_policy isa NearestOpenAggregateODAssignmentPolicy &&
            model.assignment_policy.feasibility_cut_style == :big_m_nearest) ||
            throw(ArgumentError(
                "BendersSolver(cut_derivation=$(solver.cut_derivation)) requires " *
                "NearestOpenAggregateODAssignmentPolicy(:big_m_nearest)"
            ))
        model.allow_walk_only && throw(ArgumentError(
            "BendersSolver(cut_derivation=$(solver.cut_derivation)) does not support allow_walk_only=true"
        ))
        !isnothing(model.unmet_demand_penalty) && throw(ArgumentError(
            "BendersSolver(cut_derivation=$(solver.cut_derivation)) does not support unmet_demand_penalty -- " *
            "the restricted dual-completion LP in yz_mw_cut.jl does not yet account for the relaxed z/x/u " *
            "constraints \"always feasible\" mode uses; use cut_derivation=:standard with unmet_demand_penalty for now"
        ))
    end
    z_core_point = solver.cut_derivation == :standard ? nothing :
        _yz_joint_core_point(data, model, requests, optimizer_env, cfg.silent)
    z_core = isnothing(z_core_point) ? nothing : z_core_point.z

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    master[:aggregate_od_route_unmet_demand_penalty] = model.unmet_demand_penalty
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests)
    _add_nearest_open_master_z!(
        master, data, y, requests, feasible_pairs, model.max_walking_distance, model.allow_walk_only,
        model.assignment_policy.feasibility_cut_style,
    )
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
            throw(ArgumentError("BendersYZ master failed with status $(termination_status(master))"))
        lower_bound = objective_value(master)
        assert_endpoint_chain_near_binary(master)

        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)

        assignments, infeasible = _fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
            allow_same_station=true,
        )
        # Under "always feasible" mode, `infeasible` requests are genuinely
        # unserved (u=0), not grounds for a feasibility cut -- see BendersY's
        # outer loop for the identical reasoning.
        if !isempty(infeasible) && isnothing(model.unmet_demand_penalty)
            feasibility_before = feasibility_cuts
            open_set = Set(_open_station_values(y_hat))
            for request in infeasible
                endpoint_cuts_added = _add_endpoint_nearest_feasibility_cuts!(
                    master, y, data, request, model.max_walking_distance, open_set,
                )
                if endpoint_cuts_added > 0
                    feasibility_cuts += endpoint_cuts_added
                else
                    cut_pairs = _feasibility_cut_candidate_pairs(
                        data, request, feasible_pairs[request],
                        model.assignment_policy.feasibility_cut_style, model.max_walking_distance,
                    )
                    if _pair_open_cut_satisfied_by_y(cut_pairs, open_set)
                        # Both endpoint sides have an open candidate, but they
                        # independently resolved to the same station (a
                        # collision) with allow_walk_only=false -- see the
                        # BendersYZ struct docstring. Cut that collision
                        # structurally rather than excluding the whole station set.
                        _add_endpoint_collision_feasibility_cut!(
                            master, y, data, request, model.max_walking_distance, open_set,
                        )
                    else
                        _add_pair_open_feasibility_cut!(master, y, cut_pairs)
                    end
                    feasibility_cuts += 1
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
                priming_cg_seconds=0.0,
                subproblem_lp_seconds=0.0,
                cuts_added=feasibility_cuts - feasibility_before,
                feasibility_cuts_added=feasibility_cuts,
                optimality_cuts_added=optimality_cuts,
                selected_assignment_count=length(assignments),
                generated_column_pool_size=0,
                inner_cg_iterations=inner_cg_iters,
                cut_derivation=string(solver.cut_derivation),
                mw_fallback_count=0,
                mw_completion_seconds=0.0,
                mw_phi_core=nothing,
            ))
            _flush_benders_iteration_log!(
                solver, benders_rows;
                extra_headers=[:cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core],
            )
            continue
        end

        z_hat = Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}}(
            key => round.(value.(vars)) for (key, vars) in master[:nearest_endpoint_chain_cache]
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
        mw_fallback_count = 0
        mw_completion_seconds = 0.0
        mw_last_phi_core = nothing
        for cut_id in cut_ids
            group_requests = cut_groups[cut_id]
            lp_start = time()
            if solver.reprice_subproblem
                v_hat, rho, _pool, _n_new, _rounds, exhausted, _delta = _solve_yz_route_subproblem_lp_with_repricing(
                    data,
                    model,
                    mapping,
                    group_requests,
                    feasible_pairs,
                    cg_result.generated_columns,
                    z_hat,
                    optimizer_env,
                    cfg.silent;
                    max_reprice_rounds=solver.max_reprice_rounds,
                )
                exhausted ||
                    @warn "BendersYZ subproblem repricing hit max_reprice_rounds without pricing exhaustion" iteration cut_id
            else
                v_hat, rho = _solve_yz_route_subproblem_lp(
                    data,
                    model,
                    group_requests,
                    feasible_pairs,
                    cg_result.generated_columns,
                    z_hat,
                    optimizer_env,
                    cfg.silent,
                )
            end

            # For the restricted-completion cut modes, `v_hat` above is only as good as
            # `cg_result.generated_columns`'s completeness at this `z_hat` when
            # `reprice_subproblem=false` -- tighten it with `_certified_qbar`'s independently
            # certified value before the gating decision, exactly mirroring BendersY. See
            # notes/2026-07-17_restricted_mw_cut_benders_y.md.
            # Only built when actually needed: under `unmet_demand_penalty` (where `:standard`
            # is the only allowed mode, enforced above), `assignments` legitimately omits
            # genuinely-unserved requests, so an unconditional per-group restriction here would
            # `KeyError` even though the `:standard` cut branch never touches `assignments`.
            certified_for_cut = nothing
            qbar_for_cut = nothing
            certification_already_failed = false
            assignments_for_group = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
            if solver.cut_derivation != :standard
                assignments_for_group = Dict(request => assignments[request] for request in group_requests)
                try
                    certified_for_cut, qbar_for_cut = _certified_qbar(
                        data, model, solver, group_requests, assignments_for_group, _open_station_values(y_hat),
                    )
                    v_hat = min(v_hat, qbar_for_cut)
                catch err
                    certification_already_failed = true
                    @warn "BendersYZ restricted cut_derivation: certified Q_bar computation failed; " *
                        "falling back to the plain (possibly stale) v_hat for this (iteration, cut_id)" iteration cut_id error = err
                end
            end

            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if theta_hat[cut_id] < v_hat - solver.optimality_tol
                cut_diag = _add_aggregate_od_route_benders_yz_optimality_cut!(
                    master, theta, cut_id, data, model, solver,
                    group_requests, feasible_pairs, z_hat, assignments_for_group, _open_station_values(y_hat),
                    z_core, optimizer_env, v_hat, rho;
                    certified=certified_for_cut, Q_bar=qbar_for_cut,
                    certification_already_failed=certification_already_failed,
                )
                optimality_cuts += 1
                cuts_added_this_iteration += 1
                cut_diag.fallback && (mw_fallback_count += 1)
                mw_completion_seconds += cut_diag.completion_runtime_sec
                isnan(cut_diag.phi_core) || (mw_last_phi_core = cut_diag.phi_core)
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
            feasibility_cuts_added=feasibility_cuts,
            optimality_cuts_added=optimality_cuts,
            selected_assignment_count=length(assignments),
            generated_column_pool_size=length(cg_result.generated_columns),
            inner_cg_iterations=inner_cg_iters,
            cut_derivation=string(solver.cut_derivation),
            mw_fallback_count=mw_fallback_count,
            mw_completion_seconds=mw_completion_seconds,
            mw_phi_core=mw_last_phi_core,
        ))
        _flush_benders_iteration_log!(
            solver, benders_rows;
            extra_headers=[:cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core],
        )

        if cuts_added_this_iteration == 0
            return _opt_result_from_benders(best_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersYZ",
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
    isnothing(best_result) && throw(ArgumentError("BendersYZ did not find a feasible incumbent"))
    throw(ArgumentError("BendersYZ did not converge within max_iterations=$(solver.max_iterations)"))
end
