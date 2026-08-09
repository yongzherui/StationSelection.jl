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

    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    for request in requests
        _s, o, d = request
        pairs = feasible_pairs[request]
        for pair in pairs
            x[(request, pair)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
        end
        @constraint(m, sum(x[(request, pair)] for pair in pairs; init=0.0) == 1.0)
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

    @variable(m, lambda[1:length(columns), 1:n_scenarios(data)] >= 0)
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
check for dual-basis degeneracy. The priming solve already established the activated assignment's
objective, so newly priced columns may change the dual basis but must not improve that objective.
An improvement indicates a pricing or formulation-alignment defect, and this routine throws.
Successful return also requires an exhaustive pricing pass with no negative columns.
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
    max_reprice_rounds::Int=10_000,
)
    pool = copy(columns)
    v_hat = NaN
    baseline_v_hat = nothing
    max_objective_delta = 0.0
    rho = Dict{Tuple{_AggregateODRouteEndpointChainKey, Int}, Float64}()
    n_new_columns_total = 0
    rounds = 0
    fully_exhausted = false
    for round in 1:max_reprice_rounds
        rounds = round
        m, fix_cons, cover_cons = _build_yz_route_subproblem_lp(
            data, model, requests, feasible_pairs, pool, z_hat, optimizer_env, silent
        )
        optimize!(m)
        primal_status(m) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersYZ repricing subproblem LP failed with status $(termination_status(m))"))
        assert_endpoint_chain_near_binary(m)
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
        all_new_columns, pricing_exhausted = _price_aggregate_od_route_subproblem_columns(
            data, model, mapping, pool, duals,
        )
        if isempty(all_new_columns)
            if pricing_exhausted
                fully_exhausted = true
                break
            end
            round == max_reprice_rounds && throw(ArgumentError(
                "BendersYZ repricing did not exhaust pricing within max_reprice_rounds=$(max_reprice_rounds); " *
                "no cut can be certified from the current duals."
            ))
            continue
        end
        pricing_exhausted ||
            @warn "BendersYZ subproblem repricing: pricing hit its time limit before exhausting the search " *
                "while new columns were still being found -- completeness not fully proven this round" round
        @warn "BendersYZ subproblem repricing found columns beyond the seeded pool -- pool was not complete " *
            "for this subproblem's own dual structure (dual degeneracy or genuine pool gap)" round n_new=length(all_new_columns)
        n_new_columns_total += length(all_new_columns)
        pool = _deduplicate_aggregate_od_route_columns(vcat(pool, all_new_columns))
        round == max_reprice_rounds && throw(ArgumentError(
            "BendersYZ repricing found negative route columns in the final allowed round " *
            "max_reprice_rounds=$(max_reprice_rounds); the expanded LP must be re-solved and " *
            "re-priced to exhaustion before its cut is valid."
        ))
    end
    fully_exhausted || throw(ArgumentError("BendersYZ repricing terminated without pricing exhaustion"))
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
    solver::BendersSolver;
    direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
    seed_cuts::Vector{<:NamedTuple}=NamedTuple[],
    harvested_cuts::Union{Nothing, Vector{<:NamedTuple}}=nothing,
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
    end
    # Under lifted_walking_objective, every subproblem/pricing/cut-derivation call below uses
    # `subproblem_model` (walk_cost_weight=0, route_regularization_weight=1) instead of `model` --
    # see benders/lifted_walking.jl. `model` itself is kept for everything master-side (station
    # count, feasibility/candidate structure, and the master's own walking-cost/theta terms).
    subproblem_model = solver.lifted_walking_objective ? _unit_weighted_routing_model(model) : model
    z_core_point = solver.cut_derivation == :standard ? nothing :
        _yz_joint_core_point(data, subproblem_model, requests, optimizer_env, cfg.silent)
    z_core = isnothing(z_core_point) ? nothing : z_core_point.z

    # See BendersSolver's `route_regularization_weight_schedule` docstring: a single implicit
    # stage at model.route_regularization_weight reproduces today's behavior exactly. The
    # constructor already requires lifted_walking_objective=true whenever a schedule is given.
    beta_schedule = solver.lifted_walking_objective && !isnothing(solver.route_regularization_weight_schedule) ?
        solver.route_regularization_weight_schedule : [model.route_regularization_weight]
    isapprox(beta_schedule[end], model.route_regularization_weight; atol=1e-9) || throw(ArgumentError(
        "route_regularization_weight_schedule must end at model.route_regularization_weight " *
        "($(model.route_regularization_weight)); got $(beta_schedule[end])"
    ))
    stage_idx = 1
    current_beta = beta_schedule[stage_idx]

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests)
    direct_cost_expr = AffExpr(0.0)
    if solver.lifted_walking_objective
        walking_cost_expr, x_by_pair_full = _add_nearest_open_master_walking_cost!(master, data, model, y, requests, feasible_pairs)
        isempty(seed_cuts) || _seed_yz_cuts!(master, theta, seed_cuts)
        if !isnothing(direct_enumeration_pool)
            direct_cost_expr = _add_direct_enumeration_guide!(
                master, data, model, requests, feasible_pairs, x_by_pair_full, direct_enumeration_pool;
                relax_integrality=solver.direct_enumeration_relax_integrality,
            )
        end
        # route_lb_exprs (below) reuses the zp/zd chains _add_nearest_open_master_walking_cost!
        # just built, so it must be constructed after this call, not before.
        route_lb_exprs = solver.lifted_routing_lower_bound ?
            _build_lifted_routing_lower_bound_exprs!(master, data, subproblem_model, y, cut_ids, requests, feasible_pairs) :
            solver.common_od_mcf_lower_bound ?
                _build_common_od_mcf_lower_bound_exprs!(master, data, subproblem_model, y, cut_ids, requests, feasible_pairs) :
                nothing
        route_lb_term = isnothing(route_lb_exprs) ? AffExpr(0.0) : sum(route_lb_exprs[cut_id] for cut_id in cut_ids; init=AffExpr(0.0))
        @objective(master, Min, current_beta * (sum(theta[cut_id] for cut_id in cut_ids) + direct_cost_expr + route_lb_term) + walking_cost_expr)
    else
        _add_nearest_open_master_z!(
            master, data, y, requests, feasible_pairs, model.max_walking_distance, model.allow_walk_only,
            model.assignment_policy.feasibility_cut_style,
        )
        route_lb_exprs = solver.lifted_routing_lower_bound ?
            _build_lifted_routing_lower_bound_exprs!(master, data, subproblem_model, y, cut_ids, requests, feasible_pairs) :
            solver.common_od_mcf_lower_bound ?
                _build_common_od_mcf_lower_bound_exprs!(master, data, subproblem_model, y, cut_ids, requests, feasible_pairs) :
                nothing
        route_lb_term = isnothing(route_lb_exprs) ? AffExpr(0.0) : sum(route_lb_exprs[cut_id] for cut_id in cut_ids; init=AffExpr(0.0))
        @objective(master, Min, sum(theta[cut_id] for cut_id in cut_ids) + route_lb_term)
    end

    best_result = nothing
    best_open_stations = nothing
    best_ub = Inf
    feasibility_cuts = 0
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]
    stage_log = NamedTuple[]
    # Grows across the whole outer loop, mirroring BendersY's `shared_pool` (`y.jl`) -- without
    # this, `_solve_fixed_route_covering_by_cg` below re-derives every route from scratch each
    # iteration via a single from-scratch CG pass. That pass's own `cg_stop_reason==
    # :optimality_proven` check only certifies no improving column exists against whichever dual
    # vertex *that* pass's restricted LP happened to settle at -- a weaker guarantee than global
    # exhaustion under dual degeneracy (see `_solve_nearest_open_y_subproblem_lp_with_repricing`'s
    # docstring for the same phenomenon elsewhere). Confirmed empirically: without a seeded pool,
    # BendersYZ can converge to a real, positive-valued incumbent gap on the SAME y_hat that
    # BendersY's seeded pool prices exactly (off by a fixed, reproducible amount, not noise).
    shared_pool = isnothing(model.initial_columns) ?
        AggregateODRouteColumn[] :
        copy(model.initial_columns)
    previous_y_hat_signature = nothing
    y_hat_repeat_streak = 0

    for iteration in 1:solver.max_iterations
        master_termination_status, lower_bound, master_solve_seconds = _benders_solve_master!(master, "BendersYZ")
        assert_endpoint_chain_near_binary(master)

        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)
        # theta is eta when lifted_routing_lower_bound is enabled.  Its cuts subtract the live
        # route_lb_expr rather than this incumbent value; route_lb_hat is used only to test the
        # current residual violation.
        route_lb_hat = isnothing(route_lb_exprs) ?
            nothing : Dict(cut_id => value(route_lb_exprs[cut_id]) for cut_id in cut_ids)

        y_hat_signature, y_hat_changed, y_hat_repeat_streak = _benders_y_hat_bookkeeping(
            mapping, y_hat, iteration, "BendersYZ", lower_bound,
            previous_y_hat_signature, y_hat_repeat_streak,
        )
        previous_y_hat_signature = y_hat_signature

        assignments, infeasible = _fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
            allow_same_station=true,
        )
        # The master's own eager `_add_default_endpoint_coverage_constraints!` makes every
        # request resolve to a real pair by construction -- see BendersY's outer loop for the
        # identical reasoning; this is a correctness assertion, not reactive cut-derivation
        # machinery.
        isempty(infeasible) || throw(ArgumentError(
            "BendersYZ: y_hat=$(y_hat) left requests infeasible ($(infeasible)); this should be " *
            "structurally impossible given the master's eager endpoint-coverage constraints -- " *
            "check max_walking_distance, l, and _add_default_endpoint_coverage_constraints!"
        ))

        z_hat = Dict{_AggregateODRouteEndpointChainKey, Vector{Float64}}(
            key => round.(value.(vars)) for (key, vars) in master[:nearest_endpoint_chain_cache]
        )

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, subproblem_model, assignments, solver, iteration, _open_station_values(y_hat);
            seed_columns=shared_pool,
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        # See the identical comment in benders/y.jl: under lifted_walking_objective,
        # `final_result.objective_value` is the unweighted routing value only; reconstruct the
        # true combined objective exactly. `current_beta` is the active schedule stage's weight.
        walking_cost_hat = solver.lifted_walking_objective ? _lifted_walking_cost(data, model, assignments) : 0.0
        incumbent_objective_value = if isnothing(final_result.objective_value)
            nothing
        elseif solver.lifted_walking_objective
            walking_cost_hat + current_beta * final_result.objective_value
        else
            final_result.objective_value
        end
        if !isnothing(incumbent_objective_value) && incumbent_objective_value < best_ub
            best_ub = incumbent_objective_value
            best_result = solver.lifted_walking_objective ?
                _with_objective_value(final_result, incumbent_objective_value) :
                final_result
            best_open_stations = _open_station_values(y_hat)
            println(
                "  [BendersYZ iteration $iteration] new best incumbent: obj=$(round(best_ub, digits=2))  ",
                "stations=$(sort([mapping.array_idx_to_station_id[i] for i in best_open_stations]))",
            )
            flush(stdout)
        end
        # Absorb this iteration's complete restricted pool (seed columns + everything CG
        # discovered on top of them) back into the shared pool -- see BendersY's identical
        # comment in y.jl for why this must grow across the whole outer loop, never reset.
        shared_pool = _deduplicate_aggregate_od_route_columns(
            vcat(shared_pool, final_result.mapping.columns)
        )

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
                v_hat, rho, repriced_pool, n_new, _rounds, exhausted, _delta = _solve_yz_route_subproblem_lp_with_repricing(
                    data,
                    subproblem_model,
                    mapping,
                    group_requests,
                    feasible_pairs,
                    shared_pool,
                    z_hat,
                    optimizer_env,
                    cfg.silent;
                    max_reprice_rounds=solver.max_reprice_rounds,
                )
                n_new > 0 && (shared_pool = _deduplicate_aggregate_od_route_columns(vcat(shared_pool, repriced_pool)))
                exhausted ||
                    @warn "BendersYZ subproblem repricing hit max_reprice_rounds without pricing exhaustion" iteration cut_id
            else
                v_hat, rho = _solve_yz_route_subproblem_lp(
                    data,
                    subproblem_model,
                    group_requests,
                    feasible_pairs,
                    shared_pool,
                    z_hat,
                    optimizer_env,
                    cfg.silent,
                )
            end

            # For the restricted-completion cut modes, `v_hat` above is only as good as
            # `cg_result.generated_columns`'s completeness at this `z_hat` when
            # `reprice_subproblem=false` -- tighten it with `_certified_qbar(cg_result, ...)`'s
            # certified value before the gating decision, exactly mirroring BendersY. `cg_result`
            # (this iteration's own priming CG solve) already ran pricing to
            # `cg_stop_reason == :optimality_proven` regardless of how it was seeded, so its own
            # per-request duals are the certification -- no separate re-derivation needed. See
            # notes/2026-07-17_restricted_mw_cut_benders_y.md.
            # `assignments_for_group` is only built when actually needed, since the `:standard`
            # cut branch never touches `assignments`.
            certified_for_cut = nothing
            qbar_for_cut = nothing
            certification_already_failed = false
            assignments_for_group = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
            _debug_qbar_raw = nothing
            if solver.cut_derivation != :standard
                assignments_for_group = Dict(request => assignments[request] for request in group_requests)
                try
                    certified_for_cut, qbar_for_cut = _certified_qbar(data, subproblem_model, cg_result, group_requests, assignments_for_group)
                    _debug_qbar_raw = qbar_for_cut
                    v_hat = min(v_hat, qbar_for_cut)
                catch err
                    throw(ErrorException(
                        "BendersYZ restricted cut certification failed at iteration=$(iteration), " *
                        "cut_id=$(cut_id); refusing to fall back to an uncertified standard cut: " *
                        sprint(showerror, err)
                    ))
                end
            end

            if get(ENV, "CS_DEBUG_LIFTED_LB", "0") == "1"
                println(
                    "    [debug] iter=$iteration cut_id=$cut_id theta_hat=$(round(theta_hat[cut_id], digits=3)) ",
                    "v_hat(full)=$(round(v_hat, digits=3)) qbar_raw=$(isnothing(_debug_qbar_raw) ? "n/a" : round(_debug_qbar_raw, digits=3)) ",
                    "route_lb_hat=$(isnothing(route_lb_hat) ? "n/a" : round(route_lb_hat[cut_id], digits=3)) ",
                    "will_add_cut=$(theta_hat[cut_id] + (isnothing(route_lb_hat) ? 0.0 : route_lb_hat[cut_id]) < v_hat - solver.optimality_tol)",
                )
                flush(stdout)
            end

            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            current_full_lb = theta_hat[cut_id] + (isnothing(route_lb_hat) ? 0.0 : route_lb_hat[cut_id])
            if current_full_lb < v_hat - solver.optimality_tol
                cut_diag = _add_aggregate_od_route_benders_yz_optimality_cut!(
                    master, theta, cut_id, data, subproblem_model, solver,
                    group_requests, feasible_pairs, z_hat, assignments_for_group, _open_station_values(y_hat),
                    z_core, optimizer_env, v_hat, rho;
                    route_lb_expr=isnothing(route_lb_exprs) ? nothing : route_lb_exprs[cut_id],
                    certified=certified_for_cut, Q_bar=qbar_for_cut,
                    certification_already_failed=certification_already_failed,
                )
                optimality_cuts += 1
                cuts_added_this_iteration += 1
                cut_diag.fallback && (mw_fallback_count += 1)
                mw_completion_seconds += cut_diag.completion_runtime_sec
                isnan(cut_diag.phi_core) || (mw_last_phi_core = cut_diag.phi_core)
                isnothing(harvested_cuts) || push!(
                    harvested_cuts,
                    (cut_id=cut_id, cut_constant=cut_diag.cut_constant, coeffs=cut_diag.coeffs),
                )
            end
        end
        push!(benders_rows, (
            iteration=iteration,
            master_status=string(master_termination_status),
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
            generated_column_pool_size=length(shared_pool),
            inner_cg_iterations=inner_cg_iters,
            cut_derivation=string(solver.cut_derivation),
            mw_fallback_count=mw_fallback_count,
            mw_completion_seconds=mw_completion_seconds,
            mw_phi_core=mw_last_phi_core,
            route_regularization_weight=current_beta,
            y_hat_signature=y_hat_signature,
            y_hat_changed=y_hat_changed,
            y_hat_repeat_streak=y_hat_repeat_streak,
        ))
        _flush_benders_iteration_log!(
            solver, benders_rows;
            extra_headers=[
                :cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core, :route_regularization_weight,
            ],
        )

        if cuts_added_this_iteration == 0 && stage_idx < length(beta_schedule)
            stage_idx += 1
            current_beta = beta_schedule[stage_idx]
            @objective(master, Min, current_beta * (sum(theta[cut_id] for cut_id in cut_ids) + direct_cost_expr + route_lb_term) + walking_cost_expr)
            # An incumbent optimal for the previous stage's beta is not comparable once beta
            # changes (same y_hat, different weighted total) -- only the master (with its
            # accumulated cuts) and the CG-priming pool carry forward into the next stage.
            best_ub, best_result, best_open_stations = Inf, nothing, nothing
            push!(stage_log, (stage=stage_idx, route_regularization_weight=current_beta, iterations_to_reach=iteration))
            println(
                "  [BendersYZ] route_regularization_weight_schedule: advancing to stage ",
                "$stage_idx/$(length(beta_schedule)) (β=$current_beta) at iteration $iteration",
            )
            flush(stdout)
            continue
        end

        if cuts_added_this_iteration == 0
            return _finalize_benders_result(best_result, Dict{String, Any}(
                "route_regularization_weight_schedule" => beta_schedule,
                "route_regularization_weight_stage_log" => stage_log,
                "solve_method" => "benders",
                "benders_decomposition" => "BendersYZ",
                "benders_open_stations" => best_open_stations,
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
                "generated_column_pool_size" => length(shared_pool),
                "feasibility_cut_style" => string(model.assignment_policy.feasibility_cut_style),
            ), solver; phase1_guided=!isnothing(direct_enumeration_pool))
        end
    end
    _benders_not_converged!("BendersYZ", solver, best_result)
end
