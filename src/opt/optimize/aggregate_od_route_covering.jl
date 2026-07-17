"""
Route-covering solve paths for aggregate OD route models.

These helpers adapt the exploration route-covering ideas to this package's
aggregate scenario-OD representation. A positive-demand `(scenario, o, d)` OD
bucket plays the role of a request; station-pair route coverage remains binary.
"""

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
        allow_walk_only=model.allow_walk_only,
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
        allow_walk_only=model.allow_walk_only,
    )
end

function _all_active_aggregate_od_route_pairs(mapping::AggregateODRouteMap)::Vector{Tuple{Int, Int}}
    pairs = Set{Tuple{Int, Int}}()
    for scenario_pairs in values(mapping.active_jk_s)
        union!(pairs, scenario_pairs)
    end
    delete!(pairs, WALK_ONLY_PAIR)
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
    return od_pair_walking_cost(data, o, d, pair)
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

"""
    _first_open_by_cost(data, endpoint, candidates, open_set, side)

Among `candidates` (stations within walking range of a physical `endpoint`,
independent of the other side of any OD pair), the open station with lowest
walking cost, deterministically tie-broken by station id. `nothing` if no
candidate is open.
"""
function _first_open_by_cost(
    data::StationSelectionData,
    endpoint::Int,
    candidates::Vector{Int},
    open_set::Set{Int},
    side::Symbol,
)::Union{Int, Nothing}
    isempty(candidates) && return nothing
    ranked = sort(
        candidates,
        by=j -> (side == :pickup ? get_walking_cost(data, endpoint, j) : get_walking_cost(data, j, endpoint), j),
    )
    idx = findfirst(j -> j in open_set, ranked)
    return isnothing(idx) ? nothing : ranked[idx]
end

"""
    _independent_nearest_open_assignment(data, o, d, max_walking_distance, open_set, allow_walk_only)

Procedural (outside-the-model) counterpart to `:big_m_nearest`'s actual
per-endpoint chain resolution: resolves the pickup and dropoff nearest-open
station *independently* (unlike `_ranked_request_pairs`, which ranks joint
station *pairs* -- correct for `:pair_chain`, but not what `:big_m_nearest`
implements). Returns the resolved `(j,k)` pair, `WALK_ONLY_PAIR` if both
sides resolve to the same station and `allow_walk_only`, or `nothing` if
infeasible (no open candidate on some side, or a same-station collision with
direct walking unavailable -- the latter should already have been rejected
at build time by `_assert_walk_collision_feasible!`).
"""
function _independent_nearest_open_assignment(
    data::StationSelectionData,
    o::Int,
    d::Int,
    max_walking_distance::Float64,
    open_set::Set{Int},
    allow_walk_only::Bool,
)::Union{Tuple{Int, Int}, Nothing}
    pickups = _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup)
    dropoffs = _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff)
    j_star = _first_open_by_cost(data, o, pickups, open_set, :pickup)
    k_star = _first_open_by_cost(data, d, dropoffs, open_set, :dropoff)
    (isnothing(j_star) || isnothing(k_star)) && return nothing
    j_star != k_star && return (j_star, k_star)
    return allow_walk_only ? WALK_ONLY_PAIR : nothing
end

"""
    _fixed_assignments_from_y(data, requests, feasible_pairs, y_hat; style, max_walking_distance, allow_walk_only)

Procedural nearest-open assignment given a fixed binary `y_hat`, used both to
prime `RouteCoveringProblem` CG subproblems and as an independent assertion
oracle (`_assert_x_matches_nearest_open`) against the LP's own resolved `x`.

`style == :pair_chain` ranks each request's
feasible station *pairs* jointly by combined walking cost and picks the
cheapest one with both endpoints open -- correct for
`NearestOpenAggregateODAssignmentPolicy(:pair_chain)`.

`style == :big_m_nearest` or `:endpoint_chain` instead resolves pickup/dropoff
independently per side (`_independent_nearest_open_assignment`). These differ
precisely when both sides' true nearest-open station would coincide, which
`:pair_chain`'s joint ranking instead resolves by falling through to
the next-cheapest *distinct* pair (no direct-walking concept). Requires
`max_walking_distance`.
"""
function _fixed_assignments_from_y(
    data::StationSelectionData,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    y_hat::Vector{Float64};
    style::Symbol=:pair_chain,
    max_walking_distance::Union{Nothing, Float64}=nothing,
    allow_walk_only::Bool=false,
)
    style in (:pair_chain, :big_m_nearest, :endpoint_chain) || throw(ArgumentError("unsupported style $(style)"))
    _is_endpoint_nearest_style(style) && isnothing(max_walking_distance) &&
        throw(ArgumentError("style=$(style) requires max_walking_distance"))
    open_set = Set(_open_station_values(y_hat))
    assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
    infeasible = NTuple{3, Int}[]
    for request in requests
        if _is_endpoint_nearest_style(style)
            _s, o, d = request
            assignment = _independent_nearest_open_assignment(
                data, o, d, max_walking_distance, open_set, allow_walk_only,
            )
            if isnothing(assignment)
                push!(infeasible, request)
            else
                assignments[request] = assignment
            end
        else
            ranked = _ranked_request_pairs(data, request, feasible_pairs[request])
            idx = findfirst(pair -> pair[1] in open_set && pair[2] in open_set, ranked)
            if isnothing(idx)
                push!(infeasible, request)
            else
                assignments[request] = ranked[idx]
            end
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

function _pair_open_cut_satisfied_by_y(
    pairs::Vector{Tuple{Int, Int}},
    open_stations::Set{Int},
)::Bool
    return any(pair -> (pair[1] in open_stations && pair[2] in open_stations), pairs)
end

function _add_endpoint_open_feasibility_cut!(
    master::Model,
    y,
    candidates::Vector{Int},
)::ConstraintRef
    return @constraint(master, sum(y[j] for j in candidates) >= 1.0)
end

function _prior_endpoint_candidates_by_rank(
    data::StationSelectionData,
    endpoint::Int,
    candidates::Vector{Int},
    selected::Int,
    side::Symbol,
)::Vector{Int}
    selected_cost = side == :pickup ?
        get_walking_cost(data, endpoint, selected) :
        get_walking_cost(data, selected, endpoint)
    return [
        j for j in candidates
        if begin
            cost = side == :pickup ? get_walking_cost(data, endpoint, j) : get_walking_cost(data, j, endpoint)
            (cost, j) < (selected_cost, selected)
        end
    ]
end

function _add_endpoint_collision_feasibility_cut!(
    master::Model,
    y,
    data::StationSelectionData,
    request::NTuple{3, Int},
    max_walking_distance::Float64,
    open_set::Set{Int},
)::ConstraintRef
    _s, o, d = request
    pickups = _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup)
    dropoffs = _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff)
    j_star = _first_open_by_cost(data, o, pickups, open_set, :pickup)
    k_star = _first_open_by_cost(data, d, dropoffs, open_set, :dropoff)
    (!isnothing(j_star) && j_star == k_star) || throw(ArgumentError(
        "endpoint collision cut requested for $(request), but resolved endpoints are pickup=$(j_star), dropoff=$(k_star)"
    ))
    prior_pickups = _prior_endpoint_candidates_by_rank(data, o, pickups, j_star, :pickup)
    prior_dropoffs = _prior_endpoint_candidates_by_rank(data, d, dropoffs, k_star, :dropoff)
    return @constraint(
        master,
        y[j_star] <= sum(y[j] for j in prior_pickups; init=0.0) +
                     sum(y[k] for k in prior_dropoffs; init=0.0)
    )
end

"""
    _feasibility_cut_candidate_pairs(data, request, pairs, style, max_walking_distance)

`pairs` (a request's `feasible_pairs` entry) may contain the `WALK_ONLY_PAIR`
sentinel `(0,0)`, which is not a valid `(y[j], y[k])` pair for
`_add_pair_open_feasibility_cut!` (there is no station `0`). Strips it, and
    -- for endpoint nearest-open styles with direct walking available -- replaces it with a
self-pair `(j,j)` for every station `j` common to both endpoints' candidate
sets, since opening any *one* such station alone (not a distinct pair) is
what makes direct walking feasible;
`_add_pair_open_feasibility_cut!`'s `(j,j)` case degrades correctly to
`w >= y[j] - ... ` with `y[j]` binary. A no-op (returns `pairs` unchanged)
    whenever no walk-only entry is present (`:pair_chain`, or an endpoint style
without `allow_walk_only`).
"""
function _feasibility_cut_candidate_pairs(
    data::StationSelectionData,
    request::NTuple{3, Int},
    pairs::Vector{Tuple{Int, Int}},
    style::Symbol,
    max_walking_distance::Float64,
)::Vector{Tuple{Int, Int}}
    any(is_walk_only_pair, pairs) || return pairs
    real_pairs = filter(!is_walk_only_pair, pairs)
    _is_endpoint_nearest_style(style) || return real_pairs
    _s, o, d = request
    pickups = _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup)
    dropoffs = _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff)
    common = intersect(Set(pickups), Set(dropoffs))
    return vcat(real_pairs, [(j, j) for j in common])
end

function _add_endpoint_nearest_feasibility_cuts!(
    master::Model,
    y,
    data::StationSelectionData,
    request::NTuple{3, Int},
    max_walking_distance::Float64,
    open_set::Set{Int},
)::Int
    _s, o, d = request
    cuts_added = 0
    pickups = _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup)
    dropoffs = _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff)

    if !any(j -> j in open_set, pickups)
        _add_endpoint_open_feasibility_cut!(master, y, pickups)
        cuts_added += 1
    end
    if !any(k -> k in open_set, dropoffs)
        _add_endpoint_open_feasibility_cut!(master, y, dropoffs)
        cuts_added += 1
    end
    return cuts_added
end

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
        _s, o, d = request
        pairs = feasible_pairs[request]
        for pair in pairs
            x[(request, pair)] = @variable(master, binary = true)
        end
        @constraint(master, sum(x[(request, pair)] for pair in pairs; init=0.0) == 1.0)
        x_by_pair = Dict(pair => x[(request, pair)] for pair in pairs)
        _add_nearest_open_endpoint_linked_x!(
            master, data, y, o, d, pairs, x_by_pair, max_walking_distance;
            binary=true, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
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
        allow_walk_only=base.allow_walk_only,
    )
end

function _solve_fixed_route_covering_by_cg(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}},
    solver::BendersSolver,
    iteration::Union{Nothing, Int}=nothing,
    open_stations::Union{Nothing, Vector{Int}}=nothing;
    seed_columns::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
)
    inner = solver.inner_solver
    if inner isa DirectSolver
        cfg = inner.config
        optimizer_env = isnothing(cfg.optimizer_env) ? solver.config.optimizer_env : cfg.optimizer_env
        silent = cfg.silent || solver.config.silent
        mip_gap = isnothing(cfg.mip_gap) ? solver.config.mip_gap : cfg.mip_gap
        route_problem = _route_covering_problem_from_assignments(model, assignments, open_stations)
        if !isnothing(seed_columns) && !isempty(seed_columns)
            base_mapping = create_map(route_problem, data)
            combined = _deduplicate_aggregate_od_route_columns(vcat(base_mapping.columns, seed_columns))
            route_problem = _copy_with_initial_columns(route_problem, combined)
        end
        direct_solver = DirectSolver(
            SolverConfig(
                optimizer_env=optimizer_env,
                silent=silent,
                show_counts=cfg.show_counts,
                do_optimize=cfg.do_optimize,
                warm_start=cfg.warm_start,
                check_feasibility=cfg.check_feasibility,
                mip_gap=mip_gap,
                output_dir=cfg.output_dir,
            );
            max_enumerated_routes=inner.max_enumerated_routes,
            max_enumeration_time_sec=inner.max_enumeration_time_sec,
        )
        final_result = run_opt(data, route_problem, direct_solver)
        status = final_result.termination_status == MOI.OPTIMAL ? :optimal :
            final_result.termination_status == MOI.INFEASIBLE ? :infeasible :
            final_result.termination_status == MOI.TIME_LIMIT ? :timeout : :error
        pool = copy(final_result.mapping.columns)
        lp_bound = final_result.objective_value isa Number ? Float64(final_result.objective_value) : NaN
        return AggregateODRouteColumnGenerationResult(
            status,
            final_result,
            lp_bound,
            0,
            :route_enumeration,
            pool,
            _selected_aggregate_od_route_column_ids(final_result),
            _aggregate_od_route_coverage_summary(final_result),
            NamedTuple[],
            NamedTuple[],
            NamedTuple[],
        )
    end
    cfg = inner.config
    optimizer_env = isnothing(cfg.optimizer_env) ? solver.config.optimizer_env : cfg.optimizer_env
    silent = cfg.silent || solver.config.silent
    mip_gap = isnothing(cfg.mip_gap) ? solver.config.mip_gap : cfg.mip_gap
    route_problem = _route_covering_problem_from_assignments(model, assignments, open_stations)
    if !isnothing(seed_columns) && !isempty(seed_columns)
        # Seed this iteration's restricted pool with every column ever
        # discovered across prior BendersY iterations (for any y_hat), not
        # just the singleton defaults for the current y_hat -- see
        # notes/2026-07-14_nearest_open_solver_alignment.md for why a
        # per-iteration-fresh pool makes BendersY's optimality cuts invalid
        # away from the y_hat they were derived at.
        base_mapping = create_map(route_problem, data)
        combined = _deduplicate_aggregate_od_route_columns(vcat(base_mapping.columns, seed_columns))
        route_problem = _copy_with_initial_columns(route_problem, combined)
    end
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
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        for request in requests
            _s, o, d = request
            pairs = feasible_pairs[request]
            for pair in pairs
                x[(request, pair)] = @variable(m, lower_bound = 0.0, upper_bound = 1.0)
            end
            @constraint(m, sum(x[(request, pair)] for pair in pairs; init=0.0) == 1.0)
            x_by_pair = Dict(pair => x[(request, pair)] for pair in pairs)
            _add_nearest_open_endpoint_linked_x!(
                m, data, y, o, d, pairs, x_by_pair, model.max_walking_distance;
                binary=false, allow_walk_only=model.allow_walk_only,
                selector_style=model.assignment_policy.feasibility_cut_style,
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
        @variable(m, 0 <= lambda[1:length(columns), 1:n_scenarios(data)] <= 1)
    cover_cons = Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef}()
    for request in requests
        s, _o, _d = request
        for pair in feasible_pairs[request]
            # Walk-only assignments use no vehicle route, so no route column
            # can (or needs to) cover them -- a coverage row here would wrongly
            # force x[(request, WALK_ONLY_PAIR)] to 0 even when the
            # endpoint-collision constraint (_add_nearest_open_endpoint_linked_x!)
            # forces it to 1, making the LP infeasible.
            is_walk_only_pair(pair) && continue
            covering = [idx for (idx, column) in enumerate(columns) if pair in column.od_pairs]
            cover_cons[(request, pair)] =
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
    _extract_nearest_open_y_subproblem_coverage_duals(cover_cons) -> AggregateODRouteCoverageDuals

`_build_nearest_open_y_subproblem_lp`'s covering constraints are one-per-`(request, pair)`
(each request's own copy of `sum(lambda for covering) >= x[(request, pair)]`), unlike the
main `AggregateODRouteModel` master's one-per-`(j, k, s)` aggregated coverage row. A new
route column serving pair `(j, k)` in scenario `s` would relax *every* `(request, pair)`
constraint sharing that same `(j, k, s)`, so its correct reduced-cost credit is the sum of
those constraints' duals -- exactly mirroring `extract_aggregate_od_route_coverage_duals`'s
own aggregation, just over a different constraint set. Reuses `aggregate_od_route_coverage_sigma`
unmodified since both constraint families are written in the same `sum(...) >= requirement`
direction, so the dual sign convention lines up without adjustment.
"""
function _extract_nearest_open_y_subproblem_coverage_duals(
    cover_cons::Dict{Tuple{NTuple{3, Int}, Tuple{Int, Int}}, ConstraintRef},
)::AggregateODRouteCoverageDuals
    raw = Dict{Any, Float64}()
    sigma = Dict{NTuple{3, Int}, Float64}()
    for ((request, pair), con) in cover_cons
        s, _o, _d = request
        raw_dual = dual(con)
        raw[(request, pair)] = raw_dual
        pair_s = (pair[1], pair[2], s)
        sigma[pair_s] = get(sigma, pair_s, 0.0) + aggregate_od_route_coverage_sigma(raw_dual)
    end
    return AggregateODRouteCoverageDuals(raw, sigma)
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
:optimality_proven`) or `max_reprice_rounds` is hit. Re-solving after adding
repriced columns must preserve the subproblem objective value; a change means
the original restricted LP value was not certified and the routine throws.
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
    max_reprice_rounds::Int=20,
)
    pool = copy(columns)
    v_hat = NaN
    baseline_v_hat = nothing
    max_objective_delta = 0.0
    rho = Dict{Int, Float64}()
    n_new_columns_total = 0
    rounds = 0
    fully_exhausted = true
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
            @warn "BendersY subproblem repricing: pricing hit its time limit before exhausting the search " *
                "while new columns were still being found -- completeness not fully proven this round" round
        @warn "BendersY subproblem repricing found columns beyond the seeded pool -- pool was not complete " *
            "for this subproblem's own dual structure (dual degeneracy or genuine pool gap)" round n_new=length(all_new_columns)
        n_new_columns_total += length(all_new_columns)
        pool = _deduplicate_aggregate_od_route_columns(vcat(pool, all_new_columns))
    end
    return v_hat, rho, pool, n_new_columns_total, rounds, fully_exhausted, max_objective_delta
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
            # Walk-only assignments use no vehicle route, so no route column
            # can (or needs to) cover them — a coverage row here would force
            # x[(request, pair)] to 0 even when the master fixed it to 1.
            is_walk_only_pair(pair) && continue
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

const _BENDERS_ITERATION_LOG_BASE_HEADERS = [
    :iteration,
    :master_status,
    :lower_bound,
    :incumbent_objective,
    :outer_gap,
    :outer_gap_absolute,
    :outer_gap_relative,
    :master_solve_seconds,
    :priming_cg_seconds,
    :subproblem_lp_seconds,
    :cuts_added,
    :feasibility_cuts_added,
    :optimality_cuts_added,
    :selected_assignment_count,
    :generated_column_pool_size,
    :inner_cg_iterations,
]

function _flush_benders_iteration_log!(
    solver::BendersSolver,
    rows::Vector{NamedTuple};
    extra_headers::Vector{Symbol}=Symbol[],
)
    path = _benders_log_path(solver)
    isnothing(path) && return nothing
    _write_aggregate_od_route_cg_log_csv(
        path,
        rows;
        headers=vcat(_BENDERS_ITERATION_LOG_BASE_HEADERS, extra_headers),
    )
    return nothing
end

function _outer_gap(lb::Float64, ub::Float64)
    isfinite(lb) && isfinite(ub) || return nothing
    abs(ub) <= 1e-9 && return abs(ub - lb)
    return abs(ub - lb) / max(1.0, abs(ub))
end

function _outer_gap_absolute(lb::Float64, ub::Float64)
    isfinite(lb) && isfinite(ub) || return nothing
    return ub - lb
end

function _outer_gap_relative(lb::Float64, ub::Float64)
    gap = _outer_gap_absolute(lb, ub)
    isnothing(gap) && return nothing
    abs(ub) <= 1e-9 && return gap
    return gap / max(1.0, abs(ub))
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
    model.assignment_policy.feasibility_cut_style == :pair_chain &&
        assert_no_walk_only_pairs(mapping, "AggregateODRouteModel Benders (BendersY, NearestOpen, :pair_chain)")
    requests, demand, feasible_pairs = _aggregate_od_route_benders_requests(mapping)
    isempty(requests) && throw(ArgumentError("AggregateODRouteModel nearest-open Benders requires positive demand"))
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
    y_core_point = solver.cut_derivation == :standard ? nothing :
        _y_master_core_point(data, model, requests, optimizer_env, cfg.silent)

    master = Model(() -> Gurobi.Optimizer(optimizer_env))
    cfg.silent && set_silent(master)
    @variable(master, y[1:data.n_stations], Bin)
    @variable(master, theta[cut_ids] >= 0.0)
    @constraint(master, sum(y) == model.l)
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    end
    @objective(master, Min, sum(theta[cut_id] for cut_id in cut_ids))

    best_result = nothing
    best_ub = Inf
    feasibility_cuts = 0
    optimality_cuts = 0
    inner_cg_iters = 0
    benders_rows = NamedTuple[]
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

    for iteration in 1:solver.max_iterations
        master_start = time()
        optimize!(master)
        master_solve_seconds = time() - master_start
        primal_status(master) == MOI.FEASIBLE_POINT ||
            throw(ArgumentError("BendersY master failed with status $(termination_status(master))"))
        lower_bound = objective_value(master)
        y_hat = [round(value(y[j])) for j in 1:data.n_stations]
        theta_hat = Dict(cut_id => value(theta[cut_id]) for cut_id in cut_ids)

        assignments, infeasible = _fixed_assignments_from_y(
            data, requests, feasible_pairs, y_hat;
            style=model.assignment_policy.feasibility_cut_style,
            max_walking_distance=model.max_walking_distance,
            allow_walk_only=model.allow_walk_only,
        )
        if !isempty(infeasible)
            feasibility_before = feasibility_cuts
            open_set = Set(_open_station_values(y_hat))
            for request in infeasible
                endpoint_cuts_added = _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style) ?
                    _add_endpoint_nearest_feasibility_cuts!(
                        master, y, data, request, model.max_walking_distance, open_set,
                    ) : 0
                if endpoint_cuts_added > 0
                    feasibility_cuts += endpoint_cuts_added
                else
                    cut_pairs = _feasibility_cut_candidate_pairs(
                        data, request, feasible_pairs[request],
                        model.assignment_policy.feasibility_cut_style, model.max_walking_distance,
                    )
                    if _pair_open_cut_satisfied_by_y(cut_pairs, open_set)
                        # In endpoint nearest-open styles, a request can be infeasible even when
                        # both endpoint sides have an open candidate: the independently
                        # nearest pickup/dropoff endpoints may collide at the same
                        # station while walk-only is disabled. The endpoint-open
                        # cuts and pair-open cut are then already satisfied. Cut
                        # that collision structurally without excluding the whole
                        # station set.
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
                subproblem_ip_seconds=0.0,
                lp_ip_gap=nothing,
                reprice_objective_delta=0.0,
                reprice_columns_found=0,
                reprice_rounds=0,
                cut_derivation=string(solver.cut_derivation),
                mw_fallback_count=0,
                mw_completion_seconds=0.0,
                mw_phi_core=nothing,
            ))
            _flush_benders_iteration_log!(
                solver, benders_rows;
                extra_headers=[
                    :subproblem_ip_seconds, :lp_ip_gap, :reprice_objective_delta, :reprice_columns_found, :reprice_rounds,
                    :cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core,
                ],
            )
            continue
        end

        cg_start = time()
        cg_result = _solve_fixed_route_covering_by_cg(
            data, model, assignments, solver, iteration, _open_station_values(y_hat);
            seed_columns=shared_pool,
        )
        priming_cg_seconds = time() - cg_start
        inner_cg_iters += cg_result.n_cg_iters
        final_result = cg_result.final_result
        if !isnothing(final_result.objective_value) && final_result.objective_value < best_ub
            best_ub = final_result.objective_value
            best_result = final_result
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
                        model,
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
                    model,
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

            # For the restricted-completion cut modes, `v_hat` above is only as good as
            # `shared_pool`'s completeness at this `y_hat` when `reprice_subproblem=false` -- an
            # incomplete pool can only ever inflate `v_hat` (fewer columns can't reduce covering
            # cost), so an inflated `v_hat` can make the `theta_hat < v_hat - tol` gate below
            # believe convergence has already happened, before the cut-derivation code ever runs.
            # `_certified_qbar`'s Section-C CG solve is independent of `shared_pool`/
            # `reprice_subproblem` (always certified exactly from scratch), so tightening `v_hat`
            # with it here closes that gap for these modes without requiring
            # `reprice_subproblem=true`. See notes/2026-07-17_restricted_mw_cut_benders_y.md.
            certified_for_cut = nothing
            qbar_for_cut = nothing
            certification_already_failed = false
            if solver.cut_derivation != :standard
                assignments_for_group = Dict(request => assignments[request] for request in group_requests)
                try
                    certified_for_cut, qbar_for_cut = _certified_qbar(
                        data, model, solver, group_requests, assignments_for_group, _open_station_values(y_hat),
                    )
                    v_hat = min(v_hat, qbar_for_cut)
                catch err
                    certification_already_failed = true
                    @warn "BendersY restricted cut_derivation: certified Q_bar computation failed; " *
                        "falling back to the plain (possibly stale) v_hat for this (iteration, cut_id)" iteration cut_id error = err
                end
            end

            subproblem_lp_seconds += time() - lp_start
            iteration_lp_value += v_hat
            if solver.check_lp_ip_gap
                ip_start = time()
                v_hat_ip = _solve_nearest_open_y_subproblem_ip(
                    data,
                    model,
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
                    master, y, theta, cut_id, data, model, solver,
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
            end
        end
        total_reprice_columns_found += reprice_columns_found
        total_reprice_rounds += reprice_rounds_total
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
        ))
        _flush_benders_iteration_log!(
            solver, benders_rows;
            extra_headers=[
                :subproblem_ip_seconds, :lp_ip_gap, :reprice_objective_delta, :reprice_columns_found, :reprice_rounds,
                :cut_derivation, :mw_fallback_count, :mw_completion_seconds, :mw_phi_core,
            ],
        )

        if cuts_added_this_iteration == 0
            return _opt_result_from_benders(final_result, Dict{String, Any}(
                "solve_method" => "benders",
                "benders_decomposition" => "BendersY",
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
    if _is_endpoint_nearest_style(model.assignment_policy.feasibility_cut_style)
        validate_big_m_nearest_aggregate_od_route!(data, mapping; allow_walk_only=model.allow_walk_only)
    else
        assert_no_walk_only_pairs(mapping, "AggregateODRouteModel Benders (BendersXY, NearestOpen, :pair_chain)")
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
        # No-op unless an endpoint nearest-open style built zp/zd indicators
        # on this master (via _add_nearest_open_endpoint_master_x!).
        assert_endpoint_chain_near_binary(master)

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
            return _opt_result_from_benders(final_result, Dict{String, Any}(
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
            return _opt_result_from_benders(final_result, Dict{String, Any}(
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
