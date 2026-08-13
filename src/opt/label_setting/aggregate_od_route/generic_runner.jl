"""
Generic column-generation outer loop, shared by every `AbstractColumnGenerationAlgorithm` --
mirrors `benders/generic_runner.jl`'s shape exactly: one thin `for iteration in
1:max_outer_iterations` loop (`_run_column_generation`), with everything that varies across
algorithms factored into small functions dispatched on the algorithm type (`_cg_build_master`,
`_cg_solve_master!`, `_cg_extract_duals`, `_cg_price_and_add_columns!`, `_cg_finalize_result`,
below). Two algorithms plug in by adding their own methods for these hooks, not by copying
`_run_column_generation` itself: `AggregateODRouteCG` (this file, the aggregate station-pair
formulation) and `JointRoutingAssignmentCG` (`label_setting/aggregate_od_route/joint_routing_assignment/column_generation.jl`, the
passenger-level free-assignment formulation with a DSSR pricer -- defined in its own file rather
than here because its hook methods and state type reference types that load only after this file;
see that file's own docstring for the full reasoning). Both algorithms' public entry points
(`run_aggregate_od_route_column_generation`, `run_joint_routing_assignment_column_generation`)
are thin wrappers around `_run_column_generation`, so both now run through this one shared loop in
production, including Benders' own inner priming solve
(`benders/covering.jl`'s `_solve_fixed_route_covering_by_cg`, which calls the former directly).

Unlike Benders' stateless `hat` `NamedTuple` (fully recomputed each outer iteration), column
generation genuinely needs to carry state across iterations -- which pricing phase it's in,
accumulated log rows, the column pool's next id. Each algorithm defines its own mutable
`AbstractCGState` subtype, built once by `_cg_build_master` and threaded through every later hook
call for that solve.
"""

"""
    AbstractCGState

Base type for the mutable, algorithm-specific state threaded through every hook call for one
`_run_column_generation` solve. Every concrete subtype must have a `cg_iterations::Int` field --
the shared outer loop writes the final iteration count there (`state.cg_iterations = ...`) right
before calling `_cg_finalize_result`, so algorithms that need it in their termination logging
(e.g. `AggregateODRouteCG`) don't need a separate parameter for it.
"""
abstract type AbstractCGState end

# ---------------------------------------------------------------------------
# Dispatched hooks with algorithm-agnostic defaults. Algorithm-specific methods
# override only where their own bookkeeping/logging requires it.
# ---------------------------------------------------------------------------

"""
    _cg_max_outer_iterations(algorithm, solver) -> Int

Outer-loop iteration budget. Default `solver.max_iterations` (its ordinary meaning).
`JointRoutingAssignmentCG` (Phase 5) overrides this to a large safety constant, since its own
`max_iterations`-equivalent budget only counts a subset of outer iterations (early-return-phase
solves), not every outer iteration the shared loop makes -- see that algorithm's own hook
docstrings once added.
"""
_cg_max_outer_iterations(::AbstractColumnGenerationAlgorithm, solver::ColumnGenerationSolver) =
    solver.max_iterations

"""
    _cg_time_budget_exceeded(algorithm, solver, state, t_start) -> Bool

Whether the algorithm's own total-wall-clock budget (if any) has been exceeded. Default `false` --
`AggregateODRouteCG` has no such concept and uses this default unmodified.
"""
_cg_time_budget_exceeded(::AbstractColumnGenerationAlgorithm, ::ColumnGenerationSolver, ::AbstractCGState, ::Float64) =
    false

"""
    _cg_solve_master!(rmp, algorithm, state, iteration) -> (status::Symbol, lp_bound::Union{Nothing,Float64}, primal_feasible::Bool, lp_solve_seconds::Float64)

Re-solves the current restricted master LP. Default: a bare `optimize!` with no extra
bookkeeping. `AggregateODRouteCG` overrides this to also emit its own "no primal solution"
iteration-log row at the exact point of infeasibility (matching
`run_aggregate_od_route_column_generation`'s original inline behavior byte-for-byte).
"""
function _cg_solve_master!(rmp, ::AbstractColumnGenerationAlgorithm, ::AbstractCGState, ::Int)
    m = rmp.model
    lp_start = time()
    optimize!(m)
    lp_solve_seconds = time() - lp_start
    status = termination_status(m)
    primal_feasible = primal_status(m) == MOI.FEASIBLE_POINT
    lp_bound = primal_feasible ? objective_value(m) : nothing
    return status, lp_bound, primal_feasible, lp_solve_seconds
end

"""
    _cg_default_stop_reason(new_is_empty, exhausted) -> Union{Nothing,Symbol}

Shared (non-dispatched) stop-reason resolution for the common "pricing returned nothing" case:
`:optimality_proven` when pricing also reports exhaustion, `:no_columns_not_exhausted` otherwise,
`nothing` (keep going) when new columns were found. Both `AggregateODRouteCG` and
`JointRoutingAssignmentCG` (Phase 5) resolve this sub-case identically; algorithm-specific extra
stop reasons (e.g. PFA's `:no_progress`/`:total_time_limit`) are resolved by the algorithm's own
hook, not here.
"""
_cg_default_stop_reason(new_is_empty::Bool, exhausted::Bool) =
    new_is_empty ? (exhausted ? :optimality_proven : :no_columns_not_exhausted) : nothing

"""
    _run_column_generation(data, formulation, solver, algorithm) -> (final_result::OptResult, metadata::Dict{String,Any}, state)

The generic outer loop -- see this file's module docstring. `formulation`'s type is intentionally
unconstrained here (each algorithm's own hook methods declare the concrete formulation type(s)
they support); this function only orchestrates the hook calls.
"""
function _run_column_generation(
    data::StationSelectionData,
    formulation,
    solver::ColumnGenerationSolver,
    algorithm::AbstractColumnGenerationAlgorithm,
)
    optimizer_env = isnothing(solver.config.optimizer_env) ? Gurobi.Env() : solver.config.optimizer_env
    t_start = time()
    rmp, state = _cg_build_master(data, formulation, algorithm, solver, optimizer_env)

    stop_reason = :max_cg_iters
    lp_bound = NaN
    cg_iterations = 0

    for iteration in 1:_cg_max_outer_iterations(algorithm, solver)
        cg_iterations = iteration
        if _cg_time_budget_exceeded(algorithm, solver, state, t_start)
            stop_reason = :total_time_limit
            break
        end

        _status, iter_lp_bound, primal_feasible, lp_solve_seconds =
            _cg_solve_master!(rmp, algorithm, state, iteration)
        if !primal_feasible
            stop_reason = :no_primal_solution
            break
        end
        lp_bound = iter_lp_bound

        duals = _cg_extract_duals(rmp, algorithm, state)
        _n_added, _exhausted, iter_stop_reason = _cg_price_and_add_columns!(
            data, rmp, algorithm, solver, state, duals, iteration, lp_bound, lp_solve_seconds,
        )

        if !isnothing(iter_stop_reason)
            stop_reason = iter_stop_reason
            break
        end
    end

    state.cg_iterations = cg_iterations
    final_result = _cg_finalize_result(data, rmp, algorithm, formulation, solver, state, stop_reason, lp_bound)
    metadata = Dict{String, Any}(
        "cg_stop_reason" => stop_reason,
        "lp_bound" => lp_bound,
        "cg_iterations" => cg_iterations,
    )
    return final_result, metadata, state
end

# ---------------------------------------------------------------------------
# AggregateODRouteCG hooks. `rmp` is literally the `BuildResult` from
# `build_model(formulation, data; relax_integrality=true)` -- the same object
# `run_aggregate_od_route_column_generation` called `build_result` before this
# extraction; every computation below is that function's original loop body,
# split at hook boundaries, not rewritten.
# ---------------------------------------------------------------------------

"""
    AggregateODRouteCGState <: AbstractCGState

Mutable state for one `AggregateODRouteCG` solve -- the local variables
`run_aggregate_od_route_column_generation`'s original single-function loop closed over, now
threaded explicitly through the generic hooks. `pricing_*`/`use_station_simple`
are the already-resolved (non-`nothing`) values (see `_cg_build_master`);
`pi_by_request` is `nothing` until `_cg_finalize_result` fills it in.
"""
mutable struct AggregateODRouteCGState <: AbstractCGState
    formulation::AnyAggregateODRouteProblem
    build_result::BuildResult
    logger::AggregateODRouteCGLogger
    column_log_path::Union{Nothing, String}
    dual_log_path::Union{Nothing, String}
    column_log_rows::Vector{NamedTuple}
    dual_log_rows::Vector{NamedTuple}
    generated_columns::Vector{AggregateODRouteColumn}
    last_status::Symbol
    columns_before::Int
    pricing_initial_sec::Float64
    pricing_ramp_factor::Float64
    use_station_simple::Bool
    profile_pricing::Bool
    reduced_cost_tol::Float64
    max_new_columns::Int
    n_candidates::Int
    pricing_time_limit_sec::Float64
    ip_time_limit_sec::Float64
    mip_gap::Union{Nothing, Float64}
    silent::Bool
    optimizer_env::Any
    start_time::Float64
    cg_iterations::Int
    pi_by_request::Union{Nothing, Dict{NTuple{3, Int}, Float64}}
end

"""
    _cg_build_master(data, formulation::AnyAggregateODRouteProblem, algorithm::AggregateODRouteCG, solver, optimizer_env)

Builds the LP-relaxed restricted master (`Method=1`, `Presolve=0`, matching the original
function's Gurobi tuning) and its `AggregateODRouteCGState`, resolving `algorithm`'s
`Union{Nothing,_}` sentinel fields against `formulation`/`solver` exactly as
`run_aggregate_od_route_column_generation`'s own kwarg defaults did
(`pricing_initial_sec=pricing_time_limit_sec`, `use_station_simple=model.use_station_simple`).
"""
function _cg_build_master(
    data::StationSelectionData,
    formulation::AnyAggregateODRouteProblem,
    algorithm::AggregateODRouteCG,
    solver::ColumnGenerationSolver,
    optimizer_env,
)
    build_result = build_model(formulation, data; optimizer_env=optimizer_env, relax_integrality=true)
    m = build_result.model
    solver.config.silent && set_silent(m)
    set_optimizer_attribute(m, "Method", 1)
    set_optimizer_attribute(m, "Presolve", 0)

    mapping = build_result.mapping
    logger = _create_aggregate_od_route_cg_logger(verbose=algorithm.verbose, cg_log_path=algorithm.cg_log_path)
    initial_pool_size = length(mapping.columns)
    n_active_pairs = sum(length(mapping.active_jk_s[s]) for s in 1:n_scenarios(data); init=0)
    _aggregate_od_route_log_header!(
        logger, n_active_pairs, initial_pool_size, solver.max_iterations,
        solver.pricing_time_limit_sec, solver.max_columns_per_iteration,
    )

    state = AggregateODRouteCGState(
        formulation,
        build_result,
        logger,
        algorithm.column_log_path,
        algorithm.dual_log_path,
        NamedTuple[],
        NamedTuple[],
        AggregateODRouteColumn[],
        :error,
        0,
        isnothing(algorithm.pricing_initial_sec) ? solver.pricing_time_limit_sec : algorithm.pricing_initial_sec,
        algorithm.pricing_ramp_factor,
        isnothing(algorithm.use_station_simple) ? formulation.use_station_simple : algorithm.use_station_simple,
        algorithm.profile_pricing,
        solver.reduced_cost_tol,
        solver.max_columns_per_iteration,
        solver.n_candidates,
        solver.pricing_time_limit_sec,
        solver.final_ip_time_limit_sec,
        solver.config.mip_gap,
        solver.config.silent,
        optimizer_env,
        time(),
        0,
        nothing,
    )
    return build_result, state
end

"""
    _cg_solve_master!(rmp, algorithm::AggregateODRouteCG, state, iteration)

Same LP re-solve as the generic default, plus the original function's "no primal solution" branch
-- emits that exact iteration-log row (all pricing/dual fields `nothing`, `iteration_seconds ==
lp_solve_seconds`) at the point of infeasibility, since no later hook call will run this iteration
to log it otherwise.
"""
function _cg_solve_master!(
    rmp::BuildResult,
    ::AggregateODRouteCG,
    state::AggregateODRouteCGState,
    iteration::Int,
)
    m = rmp.model
    mapping = rmp.mapping
    columns_before = length(mapping.columns)
    lp_start = time()
    optimize!(m)
    lp_solve_seconds = time() - lp_start
    term_status = termination_status(m)
    last_status = term_status == MOI.OPTIMAL ? :optimal :
        term_status == MOI.INFEASIBLE ? :infeasible :
        term_status == MOI.TIME_LIMIT ? :timeout : :error
    state.last_status = last_status
    state.columns_before = columns_before

    if primal_status(m) != MOI.FEASIBLE_POINT
        _record_aggregate_od_route_cg_iteration!(state.logger, AggregateODRouteCGIterationLog(
            iteration, columns_before, length(mapping.columns), last_status,
            nothing, lp_solve_seconds, nothing, lp_solve_seconds,
            0, 0, 0, nothing, false, :no_primal_solution,
            nothing, nothing, nothing, nothing,
            nothing, nothing, nothing, nothing, nothing, nothing,
            nothing, nothing, nothing, nothing,
        ))
        return last_status, nothing, false, lp_solve_seconds
    end

    return last_status, objective_value(m), true, lp_solve_seconds
end

"""
    _cg_extract_duals(rmp, algorithm::AggregateODRouteCG, state)

Pure dual extraction, no side effects -- `_cg_price_and_add_columns!` (which has `iteration`)
records `dual_log_rows` and dual summary stats.
"""
_cg_extract_duals(rmp::BuildResult, ::AggregateODRouteCG, ::AggregateODRouteCGState) =
    extract_aggregate_od_route_coverage_duals(rmp.model)

"""
    _cg_price_and_add_columns!(data, rmp, algorithm::AggregateODRouteCG, solver, state, duals, iteration, lp_bound, lp_solve_seconds)

The original loop body's pricing/dedup/add/log block (lines ~350-480 of the pre-extraction
`run_aggregate_od_route_column_generation`), unchanged in substance: prices every scenario at a
ramp-scheduled time limit, dedupes new columns by signature (keeping the cheaper `tau` on ties),
sorts/truncates to `max_new_columns`, adds the survivors to the master, and records one iteration
log row either way.
"""
function _cg_price_and_add_columns!(
    data::StationSelectionData,
    rmp::BuildResult,
    ::AggregateODRouteCG,
    solver::ColumnGenerationSolver,
    state::AggregateODRouteCGState,
    duals::AggregateODRouteCoverageDuals,
    iteration::Int,
    lp_bound::Float64,
    lp_solve_seconds::Float64,
)
    mapping = rmp.mapping
    model = state.formulation

    if !isnothing(state.dual_log_path)
        for ((j, k, s), val) in duals.sigma
            push!(state.dual_log_rows, (iteration=iteration, scenario=s, pickup=j, dropoff=k, sigma=val))
        end
    end
    dual_min, dual_max, dual_mean, dual_std = _aggregate_od_route_dual_stats(duals)

    pricing_started = time()
    iter_pricing_sec = min(
        state.pricing_time_limit_sec,
        state.pricing_initial_sec * (state.pricing_ramp_factor ^ (iteration - 1)),
    )

    next_column_id = isempty(mapping.column_ids) ? 1 : maximum(mapping.column_ids) + 1
    all_new_columns = AggregateODRouteColumn[]
    pricing_exhausted = true
    pricing_stats_by_scenario = []
    for s in 1:n_scenarios(data)
        pricing_duals = _scenario_pricing_duals(duals, s)
        pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
        new_columns_s, exhausted_s, stats_s = _aggregate_od_route_price_columns(
            state.use_station_simple,
            pricing_data,
            mapping.columns,
            pricing_duals;
            next_column_id=next_column_id,
            reduced_cost_tol=state.reduced_cost_tol,
            max_new_columns=state.max_new_columns,
            n_candidates=state.n_candidates,
            time_limit=iter_pricing_sec,
            profile=state.profile_pricing,
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
    new_columns = new_columns[1:min(length(new_columns), state.max_new_columns)]
    pricing_seconds = time() - pricing_started
    iteration_seconds = lp_solve_seconds + pricing_seconds
    best_reduced_cost = isempty(new_columns) ? nothing :
        minimum(Float64(get(column.metadata, "reduced_cost", Inf)) for column in new_columns)

    if isempty(new_columns)
        stop_reason = _cg_default_stop_reason(true, pricing_exhausted)
        stats = _merge_pricing_stats(pricing_stats_by_scenario)
        _record_aggregate_od_route_cg_iteration!(state.logger, AggregateODRouteCGIterationLog(
            iteration, state.columns_before, length(mapping.columns), state.last_status,
            lp_bound, lp_solve_seconds, pricing_seconds, iteration_seconds,
            0, 0, 0, best_reduced_cost, pricing_exhausted, stop_reason,
            dual_min, dual_max, dual_mean, dual_std,
            stats.labels_generated, stats.labels_rejected_by_dominance,
            stats.labels_removed_by_dominance, stats.stale_pops,
            stats.max_frontier_size, stats.max_live_labels,
            state.profile_pricing ? stats.t_queue_sec : nothing,
            state.profile_pricing ? stats.t_candidates_sec : nothing,
            state.profile_pricing ? stats.t_extension_sec : nothing,
            state.profile_pricing ? stats.t_dominance_sec : nothing,
        ))
        return 0, pricing_exhausted, stop_reason
    end

    columns_added = 0
    columns_replaced = 0
    for column in new_columns
        _theta, action = add_or_update_aggregate_od_route_column!(rmp, column)
        action == :added && (columns_added += 1)
        action == :replaced && (columns_replaced += 1)
        action in (:added, :replaced) && push!(state.generated_columns, column)
        if !isnothing(state.column_log_path)
            route = get(column.metadata, "route", ())
            push!(state.column_log_rows, (
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
    _record_aggregate_od_route_cg_iteration!(state.logger, AggregateODRouteCGIterationLog(
        iteration, state.columns_before, length(mapping.columns), state.last_status,
        lp_bound, lp_solve_seconds, pricing_seconds, iteration_seconds,
        length(new_columns), columns_added, columns_replaced,
        best_reduced_cost, pricing_exhausted, :continue,
        dual_min, dual_max, dual_mean, dual_std,
        stats.labels_generated, stats.labels_rejected_by_dominance,
        stats.labels_removed_by_dominance, stats.stale_pops,
        stats.max_frontier_size, stats.max_live_labels,
        state.profile_pricing ? stats.t_queue_sec : nothing,
        state.profile_pricing ? stats.t_candidates_sec : nothing,
        state.profile_pricing ? stats.t_extension_sec : nothing,
        state.profile_pricing ? stats.t_dominance_sec : nothing,
    ))
    return length(new_columns), pricing_exhausted, nothing
end

"""
    _cg_finalize_result(data, rmp, algorithm::AggregateODRouteCG, formulation, solver, state, stop_reason, lp_bound) -> OptResult

The original function's post-loop tail: extract `pi_by_request` when the loop proved optimality
on a `RouteCoveringProblem`, flush all three logs, then clone-rebuild the model with
`relax_integrality=false` over the accumulated column pool and solve the final MIP.
"""
function _cg_finalize_result(
    data::StationSelectionData,
    rmp::BuildResult,
    ::AggregateODRouteCG,
    formulation::AnyAggregateODRouteProblem,
    solver::ColumnGenerationSolver,
    state::AggregateODRouteCGState,
    stop_reason::Symbol,
    lp_bound::Float64,
)::OptResult
    m = rmp.model
    mapping = rmp.mapping

    state.pi_by_request = (formulation isa RouteCoveringProblem && stop_reason == :optimality_proven) ?
        _extract_route_covering_pi_by_request(m, formulation, mapping) : nothing

    _flush_aggregate_od_route_cg_log!(state.logger)
    isnothing(state.column_log_path) || _write_aggregate_od_route_cg_log_csv(
        state.column_log_path,
        state.column_log_rows;
        headers=[:iteration, :action, :scenario, :column_id, :n_pairs, :tau, :reduced_cost, :route_length, :route, :pairs],
    )
    isnothing(state.dual_log_path) || _write_aggregate_od_route_cg_log_csv(
        state.dual_log_path,
        state.dual_log_rows;
        headers=[:iteration, :scenario, :pickup, :dropoff, :sigma],
    )
    _record_aggregate_od_route_cg_termination!(
        state.logger,
        AggregateODRouteCGTerminationLog(stop_reason, state.cg_iterations, length(mapping.columns)),
    )

    final_model = _clone_for_final_mip(formulation, copy(mapping.columns))
    final_build_start = time()
    final_build = build_model(final_model, data; optimizer_env=state.optimizer_env)
    final_m = final_build.model
    state.silent && set_silent(final_m)
    set_optimizer_attribute(final_m, "TimeLimit", state.ip_time_limit_sec)
    isnothing(state.mip_gap) || set_optimizer_attribute(final_m, "MIPGap", state.mip_gap)
    final_build_time_sec = time() - final_build_start

    final_solve_start = time()
    optimize!(final_m)
    final_solve_time_sec = time() - final_solve_start
    final_term = termination_status(final_m)
    final_obj = final_term == MOI.OPTIMAL ? objective_value(final_m) : nothing
    final_solution = final_term == MOI.OPTIMAL ?
        (_value_recursive(final_m[:x]), _value_recursive(final_m[:y])) :
        nothing
    final_term == MOI.OPTIMAL && assert_endpoint_chain_near_binary(final_m)
    return OptResult(
        final_term,
        final_obj,
        final_solution,
        time() - state.start_time,
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
end
