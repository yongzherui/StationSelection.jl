"""
Benders-Y decomposition for AggregateODRouteModel (NearestOpen policy): master = y
only; subproblem = z,x,theta together (see `iterative_strategy_types.jl`'s
`BendersY` docstring). Companion cut-derivation logic (`:standard`/`:zero_completion`/
`:restricted_mw_fixed_pi`) lives in `aggregate_od_route_benders_y_mw_cut.jl`.
"""

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
    silent::Bool;
    lambda_binary::Bool=false,
)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    if !lambda_binary
        set_optimizer_attribute(m, "Method", 1)
        set_optimizer_attribute(m, "Presolve", 0)
    end
    @variable(m, 0 <= y[1:data.n_stations] <= 1)
    fix_cons = Dict(j => @constraint(m, y[j] == y_hat[j]) for j in 1:data.n_stations)

    x = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef}()
    m[:debug_sum_x_cons] = Dict{NTuple{3, Int}, ConstraintRef}()  # DEBUG (temporary instrumentation)
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        for request in requests
            _s, o, d = request
            pairs = feasible_pairs[request]
            for pair in pairs
                x[(request, pair)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
            end
            m[:debug_sum_x_cons][request] = @constraint(m, sum(x[(request, pair)] for pair in pairs; init=0.0) == 1.0)
            x_by_pair = Dict(pair => x[(request, pair)] for pair in pairs)
            _add_nearest_open_endpoint_linked_x!(
                m, data, y, o, d, pairs, x_by_pair, model.max_walking_distance;
                binary=false, allow_walk_only=model.allow_walk_only,
                selector_style=model.assignment_policy.feasibility_cut_style,
                debug_key_prefix=request,
            )
        end
    else
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
    end

    lambda = lambda_binary ?
        @variable(m, [1:length(columns), 1:n_scenarios(data)], Bin) :
        @variable(m, lambda[1:length(columns), 1:n_scenarios(data)] >= 0)
    cover_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            # Walk-only and same-station assignments use no vehicle route, so no
            # route column can (or needs to) cover them -- a coverage row here
            # would wrongly force x[(request, pair)] to 0 even when the
            # endpoint-collision constraint (_add_nearest_open_endpoint_linked_x!)
            # forces it to 1, making the LP infeasible.
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
    return m, fix_cons, x, cover_cons
end

"""
    _assert_x_matches_nearest_open(x, data, requests, feasible_pairs, y_hat; atol=1e-6)

Runtime check (not just a constraint-design argument) that a solved
`_build_nearest_open_y_subproblem_lp` LP's `x` values, for `y` fixed to
`y_hat`, actually reproduce nearest-open assignment: exactly one `x[request,
pair]` at (near-)1 per request, and that pair must equal the pair
independently computed by `_fixed_assignments_from_y` (the same routine
`_run_aggregate_od_route_nearest_open_benders_y` uses to fix assignments for
priming CG). Throws `ArgumentError` naming the first mismatch found, rather
than silently trusting the chain-constraint encoding.
"""
function _assert_x_matches_nearest_open(
    x::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, VariableRef},
    data::StationSelectionData,
    requests,
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64},
    model::AnyAggregateODRouteModel;
    atol::Float64=1e-6,
)::Nothing
    expected, infeasible = _fixed_assignments_from_y(
        data, collect(requests), feasible_pairs, y_hat;
        style=model.assignment_policy.feasibility_cut_style,
        max_walking_distance=model.max_walking_distance,
        allow_walk_only=model.allow_walk_only,
        allow_same_station=true,
    )
    isempty(infeasible) || throw(ArgumentError(
        "nearest-open subproblem LP check: y_hat=$(y_hat) leaves requests infeasible: $(infeasible)"
    ))
    for request in requests
        ranked = _ranked_request_pairs(data, request, feasible_pairs[request])
        positive = [(pair, value(x[(request, pair)])) for pair in ranked if value(x[(request, pair)]) > atol]
        length(positive) == 1 || throw(ArgumentError(
            "nearest-open subproblem LP check failed for request $(request): expected exactly one " *
            "positive x at y_hat=$(y_hat), got $(positive)"
        ))
        selected_pair, val = positive[1]
        isapprox(val, 1.0; atol=atol) || throw(ArgumentError(
            "nearest-open subproblem LP check failed for request $(request): x[$(selected_pair)]=$(val) " *
            "is not binary (not within atol=$(atol) of 1.0) at y_hat=$(y_hat)"
        ))
        selected_pair == expected[request] || throw(ArgumentError(
            "nearest-open subproblem LP check failed for request $(request): LP selected pair " *
            "$(selected_pair) but independently-computed nearest-open assignment is $(expected[request]) " *
            "at y_hat=$(y_hat)"
        ))
    end
    return nothing
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
    m, fix_cons, x, _cover_cons = _build_nearest_open_y_subproblem_lp(
        data, model, mapping, requests, demand, feasible_pairs, columns, y_hat, optimizer_env, silent
    )
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersY full LP subproblem failed with status $(termination_status(m))"))
    _assert_x_matches_nearest_open(x, data, requests, feasible_pairs, y_hat, model)
    # No-op unless an endpoint nearest-open style built zp/zd indicators above.
    assert_endpoint_chain_near_binary(m)
    return objective_value(m), Dict(j => dual(con) for (j, con) in fix_cons)
end

"""
    _solve_nearest_open_y_subproblem_ip(...)

Diagnostic-only companion to [`_solve_nearest_open_y_subproblem_lp`](@ref): solves the
*same* nearest-open subproblem (`y` fixed to `y_hat`, same column pool) but with `lambda`
(route/column selection) restricted to `Bin` instead of relaxed to `[0,1]`, to directly
measure whether the LP relaxation used for BendersY's optimality cuts has an integrality
gap at the point it's derived from. `x`/`zp`/`zd` are left as in the LP build (already
forced near-binary by the nearest-open cost structure and chain constraints, per
`_assert_x_matches_nearest_open`/`assert_endpoint_chain_near_binary`), so only the
covering-type `lambda` variables -- the ones with no such forcing structure -- are
tightened. Gated behind `BendersSolver.check_lp_ip_gap` since it's an extra MIP solve
on top of the LP every iteration; see notes/2026-07-15_bendersy_stale_cut_soundness.md.
"""
function _solve_nearest_open_y_subproblem_ip(
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
)::Float64
    m, _fix_cons, _x, _cover_cons = _build_nearest_open_y_subproblem_lp(
        data, model, mapping, requests, demand, feasible_pairs, columns, y_hat, optimizer_env, silent;
        lambda_binary=true,
    )
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT ||
        throw(ArgumentError("BendersY LP/IP gap check: IP subproblem failed with status $(termination_status(m))"))
    return objective_value(m)
end

"""
    _solve_nearest_open_y_subproblem_lp_with_repricing(...)

Diagnostic-only companion to [`_solve_nearest_open_y_subproblem_lp`](@ref) that guarantees
`v_hat`/`rho` are valid against the *full* route universe, not just whatever `columns`
(the shared pool) happens to contain. The plain LP solve trusts `columns` outright --
sound only if the pool is already complete for *this* subproblem's own dual structure,
which is a different, more general LP (free `x` over every globally feasible pair, all
`data.n_stations` as potential route nodes) than the restricted, fixed-assignment problem
`_solve_fixed_route_covering_by_cg`'s priming CG actually proved complete for. This
function closes that gap directly: after each LP solve, it extracts the covering-constraint
duals (see `_extract_nearest_open_y_subproblem_coverage_duals`) and runs genuine
label-setting pricing against them, over every scenario, exactly mirroring
`generate_aggregate_od_route_columns`'s own pricing round. If pricing finds any column with
negative reduced cost, that pool is *not* actually complete for this subproblem -- a
real completeness gap regardless of cause, though dual degeneracy (an alternate optimal
dual vertex under which a column looks non-improving) is one plausible source, since the
duals used are whichever vertex of the LP's optimal face the solver happened to return.
Either way the newly found columns are folded in and the LP is re-solved, repeating until
pricing finds nothing more (mirroring standard CG's own convergence, `cg_stop_reason ==
:optimality_proven`). Hitting `max_reprice_rounds` before that pass is an error. This is a
certification check, not a corrective CG loop for an underpriced LP: the fixed-assignment priming
solve has already established the objective that the activated-assignment subproblem must attain.
Alternate columns exposed by a different dual basis must preserve that objective. An improvement
therefore indicates invalid pricing output or a mismatch between the priming and activated-
assignment formulations, and the routine throws rather than hiding that upstream defect.
Returns `(v_hat, rho, pool, n_new_columns_total, n_rounds, fully_exhausted,
max_objective_delta)`; `n_new_columns_total > 0` is itself the signal worth
surfacing -- see notes/2026-07-15_bendersy_stale_cut_soundness.md.
"""
function _solve_nearest_open_y_subproblem_lp_with_repricing(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    requests,
    demand,
    feasible_pairs,
    columns::Vector{AggregateODRouteColumn},
    y_hat::Vector{Float64},
    optimizer_env,
    silent::Bool;
    max_reprice_rounds::Int=10_000,
)
    pool = copy(columns)
    v_hat = NaN
    baseline_v_hat = nothing
    max_objective_delta = 0.0
    rho = Dict{Int, Float64}()
    n_new_columns_total = 0
    rounds = 0
    fully_exhausted = false
    for round in 1:max_reprice_rounds
        rounds = round
        m, fix_cons, x, cover_cons = _build_nearest_open_y_subproblem_lp(
            data, model, mapping, requests, demand, feasible_pairs, pool, y_hat, optimizer_env, silent
        )
        optimize!(m)
        primal_status(m) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersY repricing subproblem LP failed with status $(termination_status(m))"))
        _assert_x_matches_nearest_open(x, data, requests, feasible_pairs, y_hat, model)
        assert_endpoint_chain_near_binary(m)
        v_hat = objective_value(m)
        if isnothing(baseline_v_hat)
            baseline_v_hat = v_hat
        else
            objective_delta = abs(v_hat - baseline_v_hat)
            max_objective_delta = max(max_objective_delta, objective_delta)
            objective_delta <= 1e-6 * max(1.0, abs(baseline_v_hat)) || throw(ArgumentError(
                "BendersY repricing changed subproblem objective at y_hat=$(y_hat): " *
                "before=$(baseline_v_hat), after=$(v_hat), delta=$(objective_delta). " *
                "Repricing is expected to certify the same LP value, not improve it."
            ))
        end
        rho = Dict(j => dual(con) for (j, con) in fix_cons)

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
                "BendersY repricing did not exhaust pricing within max_reprice_rounds=$(max_reprice_rounds); " *
                "no cut can be certified from the current duals."
            ))
            continue
        end
        pricing_exhausted ||
            @warn "BendersY subproblem repricing: pricing hit its time limit before exhausting the search " *
                "while new columns were still being found -- completeness not fully proven this round" round
        @warn "BendersY subproblem repricing found columns beyond the seeded pool -- pool was not complete " *
            "for this subproblem's own dual structure (dual degeneracy or genuine pool gap)" round n_new=length(all_new_columns)
        n_new_columns_total += length(all_new_columns)
        pool = _deduplicate_aggregate_od_route_columns(vcat(pool, all_new_columns))
        round == max_reprice_rounds && throw(ArgumentError(
            "BendersY repricing found negative route columns in the final allowed round " *
            "max_reprice_rounds=$(max_reprice_rounds); the expanded LP must be re-solved and " *
            "re-priced to exhaustion before its cut is valid."
        ))
    end
    fully_exhausted || throw(ArgumentError("BendersY repricing terminated without pricing exhaustion"))
    return v_hat, rho, pool, n_new_columns_total, rounds, fully_exhausted, max_objective_delta
end

function _run_aggregate_od_route_nearest_open_benders_y(
    data::StationSelectionData,
    model::AggregateODRouteModel,
    solver::BendersSolver;
    direct_enumeration_pool::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
    seed_cuts::Vector{<:NamedTuple}=NamedTuple[],
    harvested_cuts::Union{Nothing, Vector{<:NamedTuple}}=nothing,
)
    cfg = solver.config
    optimizer_env = isnothing(cfg.optimizer_env) ? Gurobi.Env() : cfg.optimizer_env
    mapping = create_map(model, data)
    model.assignment_policy.feasibility_cut_style == :pair_chain &&
        assert_no_walk_only_pairs(mapping, "AggregateODRouteModel Benders (BendersY, NearestOpen, :pair_chain)")
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
    y_core_point = solver.cut_derivation == :standard ? nothing :
        _y_master_core_point(data, subproblem_model, requests, optimizer_env, cfg.silent)

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
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    end
    isempty(seed_cuts) || _seed_y_cuts!(master, y, theta, seed_cuts)
    direct_cost_expr = AffExpr(0.0)
    if solver.lifted_walking_objective
        walking_cost_expr, x_by_pair_full = _add_nearest_open_master_walking_cost!(master, data, model, y, requests, feasible_pairs)
        if !isnothing(direct_enumeration_pool)
            direct_cost_expr = _add_direct_enumeration_guide!(
                master, data, model, requests, feasible_pairs, x_by_pair_full, direct_enumeration_pool;
                relax_integrality=solver.direct_enumeration_relax_integrality,
            )
        end
        @objective(master, Min, current_beta * (sum(theta[cut_id] for cut_id in cut_ids) + direct_cost_expr) + walking_cost_expr)
    else
        @objective(master, Min, sum(theta[cut_id] for cut_id in cut_ids))
    end

    best_result = nothing
    best_open_stations = nothing
    best_ub = Inf
    feasibility_cuts = 0
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]
    stage_log = NamedTuple[]
    # Grows across the whole outer loop (never reset per-y_hat), mirroring
    # ../../exploration/BendersStationSelection.jl's shared CompatibilitySetPool:
    # optimality cuts are only valid supporting hyperplanes of the true value
    # function everywhere once the column pool they're derived from is rich
    # enough to be simultaneously complete for every y_hat visited so far, not
    # just the one iteration's y_hat that happened to prime it.
    shared_pool = isnothing(model.initial_columns) ?
        AggregateODRouteColumn[] :
        copy(model.initial_columns)
    total_reprice_columns_found = 0
    total_reprice_rounds = 0
    previous_y_hat_signature = nothing
    y_hat_repeat_streak = 0

    for iteration in 1:solver.max_iterations
        master_start = time()
        optimize!(master)
        master_solve_seconds = time() - master_start
        primal_status(master) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersY master failed with status $(termination_status(master))"))
        # Captured here, not at the log push below: this iteration's cuts get added to `master`
        # further down (before the log row is built), and JuMP resets a solved model's cached
        # termination status to OPTIMIZE_NOT_CALLED as soon as it's mutated. Reading
        # `termination_status(master)` at the log push would therefore always report
        # OPTIMIZE_NOT_CALLED regardless of how the solve actually went -- capture the real status
        # right after `optimize!` instead.
        master_termination_status = termination_status(master)
        lower_bound = objective_bound(master)
        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)

        y_hat_signature = _benders_y_hat_signature(mapping, _open_station_values(y_hat))
        y_hat_changed = isnothing(previous_y_hat_signature) || y_hat_signature != previous_y_hat_signature
        y_hat_repeat_streak = y_hat_changed ? 1 : y_hat_repeat_streak + 1
        if y_hat_changed && !isnothing(previous_y_hat_signature)
            println(
                "  [BendersY iteration $iteration] master lower-bound y changed (lower_bound=",
                "$(round(lower_bound, digits=2))): stations=$(y_hat_signature)",
            )
            flush(stdout)
        end
        previous_y_hat_signature = y_hat_signature

        assignments, infeasible = _fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
            allow_same_station=true,
        )
        # The master's own eager `_add_default_endpoint_coverage_constraints!` (every physical
        # endpoint has some open candidate, combined with `allow_same_station=true`) makes every
        # request resolve to a real pair by construction -- `infeasible` can never be non-empty
        # here, so this is a correctness assertion, not reactive cut-derivation machinery.
        isempty(infeasible) || throw(ArgumentError(
            "BendersY: y_hat=$(y_hat) left requests infeasible ($(infeasible)); this should be " *
            "structurally impossible given the master's eager endpoint-coverage constraints -- " *
            "check max_walking_distance, l, and _add_default_endpoint_coverage_constraints!"
        ))

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, subproblem_model, assignments, solver, iteration, _open_station_values(y_hat);
            seed_columns=shared_pool,
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        # Under lifted_walking_objective, `cg_result`/`final_result` were solved against
        # `subproblem_model` (zero walking cost, unit route weight), so `final_result.objective_value`
        # is the unweighted routing value Q^route(y_hat) only. Reconstruct the true combined
        # objective walk(y_hat) + β*Q^route(y_hat) here -- exact, not an approximation, since the
        # optimal route selection is invariant to a uniform positive rescaling of its own cost.
        # `current_beta` (not model.route_regularization_weight) is the active schedule stage's
        # weight; the two coincide once/if the final stage is reached.
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
                "  [BendersY iteration $iteration] new best incumbent: obj=$(round(best_ub, digits=2))  ",
                "stations=$(sort([mapping.array_idx_to_station_id[i] for i in best_open_stations]))",
            )
            flush(stdout)
        end
        # Absorb this iteration's complete restricted pool (seed columns +
        # everything CG discovered on top of them) back into the shared pool,
        # so the next iteration's priming CG and this iteration's own cut
        # derivation below both see the union of every column found for any
        # y_hat tried so far.
        shared_pool = _deduplicate_aggregate_od_route_columns(
            vcat(shared_pool, final_result.mapping.columns)
        )

        iteration_lp_value = 0.0
        cuts_added_this_iteration = 0
        subproblem_lp_seconds = 0.0
        subproblem_ip_seconds = 0.0
        worst_lp_ip_gap = nothing
        reprice_columns_found = 0
        reprice_rounds_total = 0
        max_reprice_objective_delta = 0.0
        mw_fallback_count = 0
        mw_completion_seconds = 0.0
        mw_last_phi_core = nothing
        for cut_id in cut_ids
            group_requests = cut_groups[cut_id]
            lp_start = time()
            if solver.reprice_subproblem
                v_hat, rho, repriced_pool, n_new, reprice_rounds, reprice_exhausted, reprice_objective_delta =
                    _solve_nearest_open_y_subproblem_lp_with_repricing(
                        data,
                        subproblem_model,
                        mapping,
                        group_requests,
                        demand,
                        feasible_pairs,
                        shared_pool,
                        y_hat,
                        optimizer_env,
                        cfg.silent;
                        max_reprice_rounds=solver.max_reprice_rounds,
                    )
                reprice_columns_found += n_new
                reprice_rounds_total += reprice_rounds
                max_reprice_objective_delta = max(max_reprice_objective_delta, reprice_objective_delta)
                if n_new > 0
                    shared_pool = _deduplicate_aggregate_od_route_columns(vcat(shared_pool, repriced_pool))
                end
                reprice_exhausted ||
                    @warn "BendersY subproblem repricing hit max_reprice_rounds without pricing exhaustion" iteration cut_id rounds=reprice_rounds
                pool_for_ip_check = repriced_pool
            else
                v_hat, rho = _solve_nearest_open_y_subproblem_lp(
                    data,
                    subproblem_model,
                    mapping,
                    group_requests,
                    demand,
                    feasible_pairs,
                    shared_pool,
                    y_hat,
                    optimizer_env,
                    cfg.silent,
                )
                pool_for_ip_check = shared_pool
            end

            # For the restricted-completion cut modes, `v_hat` above (from
            # `_solve_nearest_open_y_subproblem_lp[_with_repricing]`) is only as good as
            # `shared_pool`'s completeness at this `y_hat` when `reprice_subproblem=false` -- an
            # incomplete pool can only ever inflate `v_hat` (fewer columns can't reduce covering
            # cost), so an inflated `v_hat` can make the `theta_hat < v_hat - tol` gate below
            # believe convergence has already happened, before the cut-derivation code ever runs.
            # `cg_result` (this iteration's own priming CG solve above) already ran its own
            # pricing to `cg_stop_reason == :optimality_proven` regardless of how it was seeded,
            # so `_certified_qbar(cg_result, ...)`'s `Q_bar` is a genuinely certified bound for
            # this `y_hat` -- tightening `v_hat` with it here closes the gap for these modes
            # without requiring `reprice_subproblem=true`. See
            # notes/2026-07-17_restricted_mw_cut_benders_y.md.
            certified_for_cut = nothing
            qbar_for_cut = nothing
            certification_already_failed = false
            if solver.cut_derivation != :standard
                try
                    certified_for_cut, qbar_for_cut = _certified_qbar(data, subproblem_model, cg_result, group_requests, assignments)
                    println(stderr, "DEBUG cut_id=$(cut_id): v_hat(broad LP, pre-min)=$(v_hat)  qbar_for_cut(R(x_bar) LP)=$(qbar_for_cut)  diff=$(qbar_for_cut - v_hat)")
                    flush(stderr)

                    v_hat = min(v_hat, qbar_for_cut)
                catch err
                    throw(ErrorException(
                        "BendersY restricted cut certification failed at iteration=$(iteration), " *
                        "cut_id=$(cut_id); refusing to fall back to an uncertified standard cut: " *
                        sprint(showerror, err)
                    ))
                end
            end

            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if solver.check_lp_ip_gap
                ip_start = time()
                v_hat_ip = _solve_nearest_open_y_subproblem_ip(
                    data,
                    subproblem_model,
                    mapping,
                    group_requests,
                    demand,
                    feasible_pairs,
                    pool_for_ip_check,
                    y_hat,
                    optimizer_env,
                    cfg.silent,
                )
                subproblem_ip_seconds += time() - ip_start
                cut_gap = _outer_gap(v_hat, v_hat_ip)
                if !isnothing(cut_gap)
                    worst_lp_ip_gap = isnothing(worst_lp_ip_gap) ? cut_gap : max(worst_lp_ip_gap, cut_gap)
                    cut_gap > 0.03 && @warn "BendersY subproblem LP/IP gap exceeds 3%" iteration cut_id v_hat v_hat_ip gap=cut_gap
                end
            end
            if theta_hat[cut_id] < v_hat - solver.optimality_tol
                cut_diag = _add_aggregate_od_route_benders_y_optimality_cut!(
                    master, y, theta, cut_id, data, subproblem_model, solver,
                    group_requests, feasible_pairs, y_hat, assignments, _open_station_values(y_hat),
                    y_core_point, optimizer_env, v_hat, rho;
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
        total_reprice_columns_found += reprice_columns_found
        total_reprice_rounds += reprice_rounds_total
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
            subproblem_ip_seconds=subproblem_ip_seconds,
            lp_ip_gap=worst_lp_ip_gap,
            reprice_objective_delta=max_reprice_objective_delta,
            reprice_columns_found=reprice_columns_found,
            reprice_rounds=reprice_rounds_total,
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
                :subproblem_ip_seconds, :lp_ip_gap, :reprice_objective_delta, :reprice_columns_found, :reprice_rounds,
                :cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core, :route_regularization_weight,
            ],
        )

        if cuts_added_this_iteration == 0 && stage_idx < length(beta_schedule)
            stage_idx += 1
            current_beta = beta_schedule[stage_idx]
            @objective(master, Min, current_beta * (sum(theta[cut_id] for cut_id in cut_ids) + direct_cost_expr) + walking_cost_expr)
            # An incumbent optimal for the previous stage's beta is not comparable once beta
            # changes (same y_hat, different weighted total) -- only the master (with its
            # accumulated cuts) and shared_pool carry forward into the next stage.
            best_ub, best_result, best_open_stations = Inf, nothing, nothing
            push!(stage_log, (stage=stage_idx, route_regularization_weight=current_beta, iterations_to_reach=iteration))
            println(
                "  [BendersY] route_regularization_weight_schedule: advancing to stage ",
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
                "benders_decomposition" => "BendersY",
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
                "benders_subproblem_ip_time_sec" => subproblem_ip_seconds,
                "benders_subproblem_lp_ip_gap" => worst_lp_ip_gap,
                "reprice_columns_found" => reprice_columns_found,
                "reprice_rounds" => reprice_rounds_total,
                "total_reprice_columns_found" => total_reprice_columns_found,
                "total_reprice_rounds" => total_reprice_rounds,
                "feasibility_cuts_added" => feasibility_cuts,
                "optimality_cuts_added" => optimality_cuts,
                "inner_cg_iterations" => inner_cg_iters,
                "benders_lp_value" => iteration_lp_value,
                "best_upper_bound" => best_ub,
                "selected_assignment_count" => length(assignments),
                "generated_column_pool_size" => length(shared_pool),
                "feasibility_cut_style" => string(model.assignment_policy.feasibility_cut_style),
                "cut_derivation" => string(solver.cut_derivation),
            ), solver; phase1_guided=!isnothing(direct_enumeration_pool))
        end
    end
    isnothing(best_result) && throw(ArgumentError("BendersY did not find a feasible incumbent"))
    throw(ArgumentError("BendersY did not converge within max_iterations=$(solver.max_iterations)"))
end
