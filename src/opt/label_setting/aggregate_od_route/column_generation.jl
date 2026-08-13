"""
The AggregateODRouteProblem column-generation main loop: single-pass column
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
    # Per-request route-covering coverage duals from the LP relaxation that itself proved
    # `cg_stop_reason == :optimality_proven` (nothing when that's not what happened, or when
    # `model` isn't a RouteCoveringProblem -- e.g. the plain joint :cg method, which has no
    # single fixed (j,k) per request to attribute credit to). This is the SAME dual the CG loop's
    # own convergence already certified -- callers deriving a Benders cut from it must NOT
    # re-solve/re-certify from scratch; see `_extract_route_covering_pi_by_request`.
    pi_by_request::Union{Nothing, Dict{NTuple{3, Int}, Float64}}
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

"""
Dispatches to whichever pricer `use_station_simple` selects.
"""
function _aggregate_od_route_price_columns(
    use_station_simple::Bool,
    pricing_data::AggregateODRoutePricingData,
    existing_columns::Vector{AggregateODRouteColumn},
    pricing_duals::AggregateODRoutePricingDuals;
    next_column_id::Int,
    reduced_cost_tol::Float64,
    max_new_columns::Int,
    n_candidates::Int,
    time_limit::Float64,
    profile::Bool=false,
)
    use_station_simple && return aggregate_od_route_pricing_by_station_simple_label_setting(
        pricing_data,
        existing_columns,
        pricing_duals;
        next_column_id=next_column_id,
        reduced_cost_tol=reduced_cost_tol,
        max_new_columns=max_new_columns,
        n_candidates=n_candidates,
        time_limit=time_limit,
        profile=profile,
    )
    return aggregate_od_route_pricing_by_label_setting(
        pricing_data,
        existing_columns,
        pricing_duals;
        next_column_id=next_column_id,
        reduced_cost_tol=reduced_cost_tol,
        max_new_columns=max_new_columns,
        n_candidates=n_candidates,
        time_limit=time_limit,
        profile=profile,
    )
end

function generate_aggregate_od_route_columns(
    master_state::BuildResult,
    duals::AggregateODRouteCoverageDuals,
    data::StationSelectionData,
)
    m = master_state.model
    mapping = master_state.mapping
    model = AggregateODRouteProblem(
        m[:aggregate_od_route_station_budget];
        route_regularization_weight=Float64(m[:aggregate_od_route_route_regularization_weight]),
        repositioning_time=Float64(m[:aggregate_od_route_repositioning_time]),
        max_walking_distance=mapping.max_walking_distance,
        max_wait_time=Float64(m[:aggregate_od_route_max_wait_time]),
        detour_factor=Float64(m[:aggregate_od_route_detour_factor]),
        max_stops=Int(m[:aggregate_od_route_max_stops]),
        max_new_columns=Int(m[:aggregate_od_route_max_new_columns]),
        n_candidates=Int(m[:aggregate_od_route_n_candidates]),
        pricing_time_limit_sec=Float64(m[:aggregate_od_route_pricing_time_limit_sec]),
        reduced_cost_tol=Float64(m[:aggregate_od_route_reduced_cost_tol]),
        relax_integrality=Bool(m[:aggregate_od_route_relax_integrality]),
        use_station_simple=Bool(m[:aggregate_od_route_use_station_simple]),
    )

    next_column_id = isempty(mapping.column_ids) ? 1 : maximum(mapping.column_ids) + 1
    all_columns = AggregateODRouteColumn[]
    for s in 1:n_scenarios(data)
        pricing_duals = _scenario_pricing_duals(duals, s)
        pricing_data = create_aggregate_od_route_pricing_data(model, data, mapping, s, pricing_duals)
        new_columns, _exhausted, _stats = _aggregate_od_route_price_columns(
            model.use_station_simple,
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

function _clone_for_final_mip(model::AggregateODRouteProblem, columns::Vector{AggregateODRouteColumn})
    return AggregateODRouteProblem(
        model.l;
        route_regularization_weight = model.route_regularization_weight,
        walk_cost_weight            = model.walk_cost_weight,
        repositioning_time          = model.repositioning_time,
        max_walking_distance        = model.max_walking_distance,
        max_wait_time               = model.max_wait_time,
        detour_factor               = model.detour_factor,
        max_stops                   = model.max_stops,
        max_new_columns             = model.max_new_columns,
        n_candidates                = model.n_candidates,
        pricing_time_limit_sec      = model.pricing_time_limit_sec,
        reduced_cost_tol            = model.reduced_cost_tol,
        initial_columns             = columns,
        relax_integrality           = false,
        assignment_policy           = model.assignment_policy,
        allow_walk_only             = model.allow_walk_only,
        use_station_simple          = model.use_station_simple,
    )
end

function _clone_for_final_mip(model::RouteCoveringProblem, columns::Vector{AggregateODRouteColumn})
    return RouteCoveringProblem(
        model.l,
        model.open_stations,
        model.fixed_assignments;
        route_regularization_weight = model.route_regularization_weight,
        walk_cost_weight            = model.walk_cost_weight,
        repositioning_time          = model.repositioning_time,
        max_walking_distance        = model.max_walking_distance,
        max_wait_time               = model.max_wait_time,
        detour_factor               = model.detour_factor,
        max_stops                   = model.max_stops,
        max_new_columns             = model.max_new_columns,
        n_candidates                = model.n_candidates,
        pricing_time_limit_sec      = model.pricing_time_limit_sec,
        reduced_cost_tol            = model.reduced_cost_tol,
        initial_columns             = columns,
        relax_integrality           = false,
        assignment_policy           = model.assignment_policy,
        allow_walk_only             = model.allow_walk_only,
        use_station_simple          = model.use_station_simple,
    )
end

"""
    _extract_route_covering_pi_by_request(m, model, mapping) -> Dict{NTuple{3,Int}, Float64}

Per-request route-covering coverage duals, read directly off `m` -- the LP relaxation whose
OWN pricing pass already proved `cg_stop_reason == :optimality_proven` -- with no new solve of
any kind. `model.fixed_assignments` gives each request's single active (j,k) pair; that pair's
coverage-constraint row (`add_aggregate_od_route_coverage_constraints!`, one row per
`(j, k, s, od_idx, pair_idx)`) carries its dual credit.

When two or more requests share the same active `(j, k, s)`, their rows are structurally
duplicates of the exact same `expr - x_od[pair_idx] >= 0` constraint (same `expr`, since the
same routes cover `(j, k)` regardless of which request is asking) -- an LP can split the total
dual credit across duplicate rows arbitrarily depending on which optimal vertex the solver
lands on, so no individual row's dual is meaningful in isolation. This sums the group's total
credit onto one deterministic representative (its `minimum`, for reproducibility) and zeroes
the rest -- still a valid completion, since a single route serving `(j, k)` covers every request
in the group at once, so attributing the full shared credit to one of them and none to the
others doesn't change what's actually being certified.
"""
function _extract_route_covering_pi_by_request(
    m::Model,
    model::RouteCoveringProblem,
    mapping::AggregateODRouteMap,
)::Dict{NTuple{3, Int}, Float64}
    coverage = m[:aggregate_od_route_coverage_constraints]
    by_active_jks = Dict{Tuple{Int, Int, Int}, Vector{NTuple{3, Int}}}()
    pi_by_request = Dict{NTuple{3, Int}, Float64}()
    for (request, pair) in model.fixed_assignments
        if requires_no_vehicle_route(pair)
            pi_by_request[request] = 0.0
            continue
        end
        s, _o, _d = request
        push!(get!(() -> NTuple{3, Int}[], by_active_jks, (pair[1], pair[2], s)), request)
    end

    for ((j, k, s), group) in by_active_jks
        total = 0.0
        for request in group
            _s, o, d = request
            od_idx = findfirst(==((o, d)), mapping.Omega_s[s])
            pair_idx = findfirst(==((j, k)), get_valid_jk_pairs(mapping, o, d))
            con = (isnothing(od_idx) || isnothing(pair_idx)) ? nothing :
                get(coverage, (j, k, s, od_idx, pair_idx), nothing)
            isnothing(con) || (total += dual(con))
        end
        representative = minimum(group)
        for request in group
            pi_by_request[request] = request == representative ? total : 0.0
        end
    end
    return pi_by_request
end

"""
    run_aggregate_od_route_column_generation(model, data; kwargs...) -> AggregateODRouteColumnGenerationResult

Thin wrapper around the generic `_run_column_generation` outer loop
(`pricing/generic_runner.jl`): constructs a `ColumnGenerationSolver(algorithm=AggregateODRouteCG(...))`
from this function's own kwargs, runs the shared loop, and reshapes the resulting
`(OptResult, metadata, state)` triple back into this function's original result struct. Every
kwarg here keeps its original name/default/validation -- only the body now delegates instead of
running the loop inline.
"""
function run_aggregate_od_route_column_generation(
    model::AnyAggregateODRouteProblem,
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
    reduced_cost_tol::Float64=model.reduced_cost_tol,
    pricing_time_limit_sec::Float64=model.pricing_time_limit_sec,
    pricing_initial_sec::Float64=pricing_time_limit_sec,
    pricing_ramp_factor::Float64=1.0,
    use_station_simple::Bool=model.use_station_simple,
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

    resolved_env = isnothing(optimizer_env) ? Gurobi.Env() : optimizer_env
    solver = ColumnGenerationSolver(
        config=SolverConfig(optimizer_env=resolved_env, silent=silent, mip_gap=mip_gap),
        max_iterations=max_cg_iters,
        max_columns_per_iteration=max_new_columns,
        n_candidates=n_candidates,
        reduced_cost_tol=reduced_cost_tol,
        pricing_time_limit_sec=pricing_time_limit_sec,
        final_ip_time_limit_sec=ip_time_limit_sec,
        algorithm=AggregateODRouteCG(
            pricing_initial_sec=pricing_initial_sec,
            pricing_ramp_factor=pricing_ramp_factor,
            use_station_simple=use_station_simple,
            profile_pricing=profile_pricing,
            verbose=verbose,
            cg_log_path=cg_log_path,
            column_log_path=column_log_path,
            dual_log_path=dual_log_path,
        ),
    )

    final_result, metadata, state = _run_column_generation(data, model, solver, solver.algorithm)
    cg_stop_reason = metadata["cg_stop_reason"]::Symbol

    status = final_result.termination_status == MOI.OPTIMAL ? :optimal :
        final_result.termination_status == MOI.INFEASIBLE ? :infeasible :
        final_result.termination_status == MOI.TIME_LIMIT ? :timeout : :error
    cg_stop_reason == :optimality_proven || status != :optimal || (status = :feasible)

    return AggregateODRouteColumnGenerationResult(
        status,
        final_result,
        metadata["lp_bound"]::Float64,
        metadata["cg_iterations"]::Int,
        cg_stop_reason,
        copy(state.build_result.mapping.columns),
        _selected_aggregate_od_route_column_ids(final_result),
        _aggregate_od_route_coverage_summary(final_result),
        copy(state.logger.iteration_rows),
        state.column_log_rows,
        state.dual_log_rows,
        state.pi_by_request,
    )
end

function _run_aggregate_od_route_column_generation_opt(
    instance::StationSelectionData,
    formulation::AnyAggregateODRouteProblem,
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
