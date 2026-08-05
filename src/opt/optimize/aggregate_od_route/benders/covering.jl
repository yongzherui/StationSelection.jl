"""
Route-covering solve paths for aggregate OD route models: DirectSolver/
ColumnGenerationSolver dispatch, plus infrastructure shared across every
Benders decomposition (`BendersY`/`BendersXY`/`BendersYZ`/`BendersYZH`) --
request/physical-pair grouping, nearest-open assignment resolution, the
fixed-assignment route-covering CG wrapper, and Benders bookkeeping (logging,
gap computation, result wrapping).

These helpers adapt the exploration route-covering ideas to this package's
aggregate scenario-OD representation. A positive-demand `(scenario, o, d)` OD
bucket plays the role of a request; station-pair route coverage remains binary.

Each decomposition's own master/subproblem construction and outer loop lives
in its own file: `y.jl`, `xy.jl`, `yz.jl`, `yzh.jl` (cut-derivation-only
companions: `y_mw_cut.jl` for BendersY's and `yz_mw_cut.jl` for BendersYZ's
zero-completion/Magnanti-Wong cuts; BendersYZH's zero-completion is small
enough to live directly in `yzh.jl`). `dispatch.jl` holds the top-level
`run_opt` dispatch that routes to whichever of those a `BendersSolver`'s
`decomposition` field selects.

`_add_default_endpoint_coverage_constraints!`/`_check_aggregate_od_route_endpoint_feasibility!`
below, together with `allow_same_station=true` always being in effect (`create_map`), make the
subproblem provably always feasible: every physical endpoint any request touches has some open
candidate baked into the master as a hard constraint, so `_fixed_assignments_from_y` can never
place a request in its `infeasible` list -- the `if !isempty(infeasible)` guard in `y.jl`/`yz.jl`'s
outer loops is a correctness assertion (throws if it's ever hit), not reactive cut-derivation. The
reactive feasibility-cut helpers this used to call (`_add_endpoint_nearest_feasibility_cuts!` and
friends) have been removed -- recoverable from git history if a future configuration needs them
restored. See `notes/2026-07-22_endpoint_coverage_feasibility_guarantee.md` for the full argument.
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
        walk_cost_weight=model.walk_cost_weight,
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
        use_station_simple=model.use_station_simple,
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
        walk_cost_weight=model.walk_cost_weight,
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
        use_station_simple=model.use_station_simple,
    )
end

function _all_active_aggregate_od_route_pairs(mapping::AggregateODRouteMap)::Vector{Tuple{Int, Int}}
    pairs = Set{Tuple{Int, Int}}()
    for scenario_pairs in values(mapping.active_jk_s)
        union!(pairs, scenario_pairs)
    end
    filter!(!requires_no_vehicle_route, pairs)
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
    # Same enumerated route universe as the IP solve above, just with lambda/theta (and, under
    # relax_integrality=true, the endpoint-chain z selectors too) relaxed to [0,1] -- gives a
    # genuine LP relaxation of the FULL (enumerated) route set, independent of whatever a separate
    # ColumnGenerationSolver's own pricing-based LP bound would report. Always computed (not
    # opt-in): DirectSolver enumerations are already the expensive part of a call, and leaving
    # `lp_bound`/`"lp_relaxation_objective"` populated with the IP value instead (the old behavior)
    # is actively misleading, not just incomplete. Built and solved directly (not via
    # `_run_opt_impl`) because that helper unconditionally calls
    # `assert_endpoint_chain_near_binary` on any MOI.OPTIMAL solve -- correct for the IP solve
    # above (z genuinely must resolve near-binary there), but wrong here: a real LP relaxation is
    # expected to leave z fractional, and asserting otherwise would throw on exactly the cases
    # this is meant to measure.
    enumerated_lp = _copy_with_initial_columns(formulation, columns; relax_integrality=true)
    lp_build = build_model(enumerated_lp, instance; optimizer_env=cfg.optimizer_env)
    lp_m = lp_build.model
    cfg.silent && set_silent(lp_m)
    optimize!(lp_m)
    result.metadata["lp_relaxation_objective"] = primal_status(lp_m) == MOI.FEASIBLE_POINT ?
        Float64(objective_value(lp_m)) : NaN
    return result
end

function run_opt(
    instance::StationSelectionData,
    formulation::AggregateODRouteModel,
    solver::DirectSolver,
)
    # DirectSolver always means exhaustive route enumeration followed by the
    # monolithic solve.  Free assignment keeps its explicit x assignment
    # variables; only the complete theta route universe is supplied here.
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

"""
    _aggregate_od_route_benders_physical_pairs(mapping) -> (physical_pairs, occurrences, feasible_pairs_by_p)

BendersYZH groups requests by *physical* OD pair `p=(o,d)` (dropping the
scenario component `s`) rather than by `(s,o,d)`, since the whole point of
`h` is to be scenario-compressed: `mapping.valid_jk_pairs` is already keyed by
physical `(o,d)` only (identical across every scenario occurrence), so
`feasible_pairs_by_p[p]` is just `get_valid_jk_pairs(mapping, o, d)` again;
`occurrences[p]` is the list of scenario ids in which `p` has positive
demand, needed both to expand a fixed `h` back into a flat assignments dict
([`_selected_assignments_from_h`](@ref)) and to derive
`occurrence_count[p] = length(occurrences[p])` (the master objective's
per-`h` weight -- see [`_add_nearest_open_master_h!`](@ref)'s caller).
"""
function _aggregate_od_route_benders_physical_pairs(mapping::AggregateODRouteMap)
    physical_pairs = Tuple{Int, Int}[]
    occurrences = Dict{Tuple{Int, Int}, Vector{Int}}()
    feasible_pairs_by_p = Dict{Tuple{Int, Int}, Vector{Tuple{Int, Int}}}()
    for s in sort!(collect(keys(mapping.Q_s)))
        for (o, d) in mapping.Omega_s[s]
            q = get(mapping.Q_s[s], (o, d), 0)
            q > 0 || continue
            p = (o, d)
            if !haskey(occurrences, p)
                push!(physical_pairs, p)
                occurrences[p] = Int[]
                feasible_pairs_by_p[p] = get_valid_jk_pairs(mapping, o, d)
            end
            push!(occurrences[p], s)
        end
    end
    return physical_pairs, occurrences, feasible_pairs_by_p
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

function _assignment_pair_cost(
    data::StationSelectionData, request::NTuple{3, Int}, pair::Tuple{Int, Int}; weight::Float64=1.0,
)
    _s, o, d = request
    return weight * od_pair_walking_cost(data, o, d, pair)
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
    _benders_y_hat_signature(mapping, open_stations) -> String

Deterministic string identity for a master-solved `y_hat`, built from real
station ids (not array indices, which are only stable within one `mapping`)
so it can be compared across iterations and read directly from the iteration
log. Used to tell whether the Benders outer loop's lower-bound master is
revisiting the same first-stage candidate iteration after iteration (theta
alone tightening around a fixed y) or actively jumping between distinct
candidates as cuts accumulate -- see `_BENDERS_ITERATION_LOG_BASE_HEADERS`'s
`y_hat_changed`/`y_hat_repeat_streak` columns.
"""
function _benders_y_hat_signature(mapping::AggregateODRouteMap, open_stations::Vector{Int})::String
    return join(sort([mapping.array_idx_to_station_id[i] for i in open_stations]), ",")
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
    allow_walk_only::Bool;
    allow_same_station::Bool=false,
)::Union{Tuple{Int, Int}, Nothing}
    pickups = _nearest_open_endpoint_candidates(data, o, max_walking_distance, :pickup)
    dropoffs = _nearest_open_endpoint_candidates(data, d, max_walking_distance, :dropoff)
    j_star = _first_open_by_cost(data, o, pickups, open_set, :pickup)
    k_star = _first_open_by_cost(data, d, dropoffs, open_set, :dropoff)
    (isnothing(j_star) || isnothing(k_star)) && return nothing
    j_star != k_star && return (j_star, k_star)
    # Both sides collide on the same station. Prefer WALK_ONLY_PAIR when
    # available (cheaper or equal by the triangle inequality -- direct
    # walk(o,d) <= walk(o,j*)+walk(j*,d) -- and it's what the model's own
    # objective would pick between the two if both were valid x entries), and
    # fall back to the real same-station pair only when it isn't.
    allow_walk_only && return WALK_ONLY_PAIR
    allow_same_station && return (j_star, j_star)
    return nothing
end

"""
    _fixed_assignments_from_y(data, requests, feasible_pairs, y_hat; style, max_walking_distance, allow_walk_only, allow_same_station)

Procedural nearest-open assignment given a fixed binary `y_hat`, used both to
prime `RouteCoveringProblem` CG subproblems and as an independent assertion
oracle (`_assert_x_matches_nearest_open`) against the LP's own resolved `x`.

`allow_same_station` makes a same-station collision resolve to the real pair
`(j*,j*)` instead of `infeasible`. A request can still land in `infeasible`
when `allow_same_station` is on -- no open candidate at all on one side.

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
    allow_same_station::Bool=false,
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
                data, o, d, max_walking_distance, open_set, allow_walk_only;
                allow_same_station=allow_same_station,
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

function _add_endpoint_open_feasibility_cut!(
    master::Model,
    y,
    candidates::Vector{Int},
)::ConstraintRef
    return @constraint(master, sum(y[j] for j in candidates) >= 1.0)
end

"""
    _aggregate_od_route_endpoint_candidate_sets(data, requests, max_walking_distance)
        -> Dict{Tuple{Int, Symbol}, Vector{Int}}

Unique physical `(endpoint, side)` -> nearest-open candidate station set
(`_nearest_open_endpoint_candidates`), deduplicated across every scenario
occurrence of that endpoint in `requests`. `compute_valid_jk_pairs` builds
every request's real `(j,k)` pairs as exactly the off-diagonal (or full, with
`allow_same_station`) Cartesian product of these same independently-computed
per-side sets, regardless of `feasibility_cut_style` -- so "some candidate on
each side must be open" is a necessary condition for any request to have a
servable real pair, whether resolution is `:pair_chain`'s joint ranking or
`:big_m_nearest`/`:endpoint_chain`'s independent per-side selection.
"""
function _aggregate_od_route_endpoint_candidate_sets(
    data::StationSelectionData,
    requests::Vector{NTuple{3, Int}},
    max_walking_distance::Float64,
)::Dict{Tuple{Int, Symbol}, Vector{Int}}
    sets = Dict{Tuple{Int, Symbol}, Vector{Int}}()
    for (_s, o, d) in requests
        for (endpoint, side) in ((o, :pickup), (d, :dropoff))
            key = (endpoint, side)
            haskey(sets, key) && continue
            sets[key] = _nearest_open_endpoint_candidates(data, endpoint, max_walking_distance, side)
        end
    end
    return sets
end

"""
    _add_default_endpoint_coverage_constraints!(master, y, data, model, requests) -> Int

Adds, by default, one `sum(y[j] for j in candidates) >= 1` constraint per
unique physical endpoint touched by `requests` (aggregated across every
scenario, since `y` is scenario-agnostic) -- the simplest necessary condition
for subproblem feasibility, ensuring every request's pickup and dropoff side
has at least one open candidate station. Combined with `allow_same_station=true`
always being in effect (`create_map`), this is also *sufficient*: every
request then always resolves to a real pair (possibly same-station), so
`_fixed_assignments_from_y` can never report a request infeasible and the
reactive feasibility-cut machinery in the outer loop becomes structurally
unreachable, not just less likely. Returns the number of constraints added.
"""
function _add_default_endpoint_coverage_constraints!(
    master::Model,
    y,
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
)::Int
    base = _base_aggregate_od_route_model(model)
    sets = _aggregate_od_route_endpoint_candidate_sets(data, requests, base.max_walking_distance)
    for candidates in values(sets)
        _add_endpoint_open_feasibility_cut!(master, y, candidates)
    end
    return length(sets)
end

"""
    _check_aggregate_od_route_endpoint_feasibility!(data, model, requests, optimizer_env, silent)

Pre-flight feasibility screen run before any Benders master/subproblem
machinery is built: solves the trivial covering-only MILP (`y` binary,
`sum(y) == l`, plus the same endpoint-coverage constraints
`_add_default_endpoint_coverage_constraints!` bakes into the real master, and
nothing else) purely for feasibility. Every real Benders master is a strict
superset of this trivial model's constraints, so if this fails, the real
master can never be feasible either -- fail fast with a targeted diagnostic
instead of letting that surface as a generic "master failed with status ..."
deep inside the outer Benders loop, after `create_map`/CG setup have already
run.
"""
function _check_aggregate_od_route_endpoint_feasibility!(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    optimizer_env,
    silent::Bool,
)::Nothing
    base = _base_aggregate_od_route_model(model)
    m = Model(() -> Gurobi.Optimizer(optimizer_env))
    silent && set_silent(m)
    @variable(m, y[1:data.n_stations], Bin)
    @constraint(m, sum(y) == base.l)
    _add_default_endpoint_coverage_constraints!(m, y, data, model, requests)
    optimize!(m)
    primal_status(m) == MOI.FEASIBLE_POINT || throw(ArgumentError(
        "AggregateODRouteModel Benders pre-flight check failed: no y with sum(y)==$(base.l) can open a " *
        "station within max_walking_distance=$(base.max_walking_distance) of every request's pickup and " *
        "dropoff endpoint -- the full Benders master can never be feasible either. Increase l or " *
        "max_walking_distance."
    ))
    return nothing
end

"""
    _add_nearest_open_master_z!(master, data, y, requests, feasible_pairs, max_walking_distance, allow_walk_only, selector_style)

BendersYZ/BendersYZH master `z`-builder: populates/reuses
`master[:nearest_endpoint_chain_cache]` for every physical endpoint touched
by `requests`, without creating any `x`/`h`. Continuous `[0,1]` (`binary=false`
— see `_add_nearest_open_endpoint_master_x!`'s docstring for why this is
sound given `y` is `Bin`). Naturally deduplicated across scenario-repeats of
the same physical `(o,d)` via `_nearest_open_endpoint_selectors!`'s cache.
"""
function _add_nearest_open_master_z!(
    master::Model,
    data::StationSelectionData,
    y,
    requests::Vector{NTuple{3, Int}},
    feasible_pairs::Dict{NTuple{3, Int}, Vector{Tuple{Int, Int}}},
    max_walking_distance::Float64,
    allow_walk_only::Bool,
    selector_style::Symbol,
)::Nothing
    for request in requests
        _s, o, d = request
        _nearest_open_endpoint_selectors!(
            master, data, y, o, d, feasible_pairs[request], max_walking_distance;
            binary=false, allow_walk_only=allow_walk_only, selector_style=selector_style,
        )
    end
    return nothing
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
        walk_cost_weight=base.walk_cost_weight,
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
        use_station_simple=base.use_station_simple,
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
        final_result.termination_status == MOI.OPTIMAL || throw(ArgumentError(
            "RouteCoveringProblem direct final IP did not solve to optimality; " *
            "status=$(final_result.termination_status)"
        ))
        status = final_result.termination_status == MOI.OPTIMAL ? :optimal :
            final_result.termination_status == MOI.INFEASIBLE ? :infeasible :
            final_result.termination_status == MOI.TIME_LIMIT ? :timeout : :error
        pool = copy(final_result.mapping.columns)
        # Genuine LP relaxation of the full enumerated route set -- always populated by
        # _run_direct_enumerated_aggregate_od_route now, not aliased to the IP objective.
        lp_bound = Float64(final_result.metadata["lp_relaxation_objective"])
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
            nothing,
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
        cg_log_path=(isnothing(iteration) || !solver.log_subiteration_details) ? nothing : _aggregate_od_route_cg_log_path(
            solver,
            "aggregate_od_route_benders_subiter$(iteration)_cg_iterations.csv",
        ),
        column_log_path=(isnothing(iteration) || !solver.log_subiteration_details) ? nothing : _aggregate_od_route_cg_log_path(
            solver,
            "aggregate_od_route_benders_subiter$(iteration)_cg_columns.csv",
        ),
        dual_log_path=(isnothing(iteration) || !solver.log_subiteration_details) ? nothing : _aggregate_od_route_cg_log_path(
            solver,
            "aggregate_od_route_benders_subiter$(iteration)_cg_duals.csv",
        ),
        mip_gap=mip_gap,
        silent=silent,
    )
    cg_result.cg_stop_reason == :optimality_proven ||
        throw(ArgumentError("RouteCoveringProblem CG did not prove pricing exhaustion; stop_reason=$(cg_result.cg_stop_reason)"))
    cg_result.final_result.termination_status == MOI.OPTIMAL || throw(ArgumentError(
        "RouteCoveringProblem CG final IP did not solve to optimality; " *
        "status=$(cg_result.final_result.termination_status)"
    ))
    return cg_result
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
    _price_aggregate_od_route_subproblem_columns(data, model, mapping, pool, duals)

Run one label-pricing pass for every scenario against a Benders route
subproblem's coverage duals. Column IDs remain unique across scenarios and
`pricing_exhausted` is true only when every scenario search was exhausted.
"""
function _price_aggregate_od_route_subproblem_columns(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    mapping::AggregateODRouteMap,
    pool::Vector{AggregateODRouteColumn},
    duals::AggregateODRouteCoverageDuals,
)
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
    return all_new_columns, pricing_exhausted
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

function _finalize_benders_result(
    final_result::OptResult,
    metadata::Dict{String, Any},
    solver::BendersSolver;
    phase1_guided::Bool=false,
)
    gap = get(metadata, "benders_outer_gap_relative", nothing)
    lower_bound = get(metadata, "benders_lower_bound", nothing)
    incumbent = get(metadata, "benders_incumbent_objective", nothing)
    bound_inverted = lower_bound isa Number && incumbent isa Number &&
        isfinite(lower_bound) && isfinite(incumbent) &&
        lower_bound > incumbent + solver.optimality_tol * max(1.0, abs(incumbent))
    # Under `direct_enumeration_guide`'s phase 1, the master's own objective/bound
    # deliberately double-counts routing cost (both `theta` and the exact
    # `theta_direct` term are costed simultaneously -- see `direct_enumeration_guide.jl`),
    # so the master's reported bound is *expected* to exceed the true incumbent whenever
    # `theta`'s cut floor is nonzero. That is by design, not a cut/bound defect, so these
    # two warnings (both driven by the same inflated bound) are suppressed for phase 1 --
    # phase 1's result is never the certified answer (only phase 2's, run without
    # `phase1_guided`, is), and its bound-vs-incumbent relationship isn't meaningful.
    if bound_inverted && !phase1_guided
        @warn(
            "Benders master lower bound exceeds the feasible incumbent; the cut/bound calculation is inconsistent",
            decomposition=get(metadata, "benders_decomposition", "unknown"),
            lower_bound=lower_bound,
            incumbent_objective=incumbent,
            bound_violation=lower_bound - incumbent,
        )
    end
    if gap isa Number && isfinite(gap) && gap > solver.outer_gap_warning_tol && !phase1_guided
        @warn(
            "Benders returned its best feasible incumbent, but the outer optimality gap exceeds the expected tolerance",
            decomposition=get(metadata, "benders_decomposition", "unknown"),
            outer_gap_relative=gap,
            outer_gap_warning_tol=solver.outer_gap_warning_tol,
            lower_bound=lower_bound,
            incumbent_objective=incumbent,
        )
    end
    metadata["benders_outer_gap_warning_tol"] = solver.outer_gap_warning_tol
    metadata["benders_bound_inverted"] = bound_inverted
    metadata["benders_outer_gap_within_warning_tol"] =
        !bound_inverted && gap isa Number && isfinite(gap) && gap <= solver.outer_gap_warning_tol
    return _opt_result_from_benders(final_result, metadata)
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
    :y_hat_signature,
    :y_hat_changed,
    :y_hat_repeat_streak,
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
    return abs(ub - lb)
end

function _outer_gap_relative(lb::Float64, ub::Float64)
    gap = _outer_gap_absolute(lb, ub)
    isnothing(gap) && return nothing
    abs(ub) <= 1e-9 && return gap
    return gap / max(1.0, abs(ub))
end
