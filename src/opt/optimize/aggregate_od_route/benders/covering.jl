"""
Route-covering solve paths for aggregate OD route models: DirectSolver/
ColumnGenerationSolver dispatch, plus infrastructure shared across every
Benders decomposition (`BendersY`/`BendersXY`/`BendersYZ`/`BendersYZH`) --
request/physical-pair grouping, feasibility-cut helpers, nearest-open
assignment resolution, the fixed-assignment route-covering CG wrapper, and
Benders bookkeeping (logging, gap computation, result wrapping).

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
subproblem provably always feasible under the default model configuration
(`allow_walk_only=false`, `unmet_demand_penalty === nothing`) -- the reactive feasibility-cut
helpers in this file (`_add_endpoint_nearest_feasibility_cuts!` and friends) are kept as a
defensive fallback for configurations outside that guarantee, but are no longer reachable in the
default path. See `notes/2026-07-22_endpoint_coverage_feasibility_guarantee.md` for the full
argument and what's deliberately out of scope.
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
        unmet_demand_penalty=model.unmet_demand_penalty,
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
        unmet_demand_penalty=model.unmet_demand_penalty,
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

`allow_same_station` (`AggregateODRouteModel(unmet_demand_penalty=...)`) makes
a same-station collision resolve to the real pair `(j*,j*)` instead of
`infeasible`, mirroring the model's own relaxed `sum(z)<=1`/`sum(x)==u`
constraints. A request can still land in `infeasible` when `allow_same_station`
is on -- no open candidate at all on one side -- and callers under "always
feasible" mode must treat that as "genuinely unserved (`u=0`)", not as
grounds for a feasibility cut.

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

"""
    _endpoint_coverage_applicable(base::AggregateODRouteModel)::Bool

Whether "some open station within `max_walking_distance` of every request
endpoint" is a *necessary* condition for subproblem feasibility, and so safe
to bake into the master as a hard constraint. False only under
`unmet_demand_penalty !== nothing` ("always feasible" mode): an uncovered
endpoint is then a legitimate genuinely-unserved outcome (`u=0`), not an
infeasibility, so forcing coverage would remove that relaxation's whole
point. Applies unconditionally otherwise -- including `allow_walk_only`
models: every physical endpoint gets a real open candidate, so `sum(z)==1`/
`sum(x)==1` stay hard-required everywhere (see `_endpoint_chain_variable!`/
`_endpoint_big_m_variable!`) rather than needing a per-request relaxation;
`WALK_ONLY_PAIR` remains available as a genuinely *cheaper* option once a
station happens to be open nearby, it's just no longer load-bearing for
feasibility.
"""
function _endpoint_coverage_applicable(base::AggregateODRouteModel)::Bool
    return isnothing(base.unmet_demand_penalty)
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
unreachable, not just less likely. No-op (returns 0) when
`!_endpoint_coverage_applicable(base_model)`. Returns the number of
constraints added.
"""
function _add_default_endpoint_coverage_constraints!(
    master::Model,
    y,
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
)::Int
    base = _base_aggregate_od_route_model(model)
    _endpoint_coverage_applicable(base) || return 0
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
run. No-op when `!_endpoint_coverage_applicable(base_model)` (nothing to
check: an uncovered endpoint isn't an infeasibility under that relaxation).
"""
function _check_aggregate_od_route_endpoint_feasibility!(
    data::StationSelectionData,
    model::AnyAggregateODRouteModel,
    requests::Vector{NTuple{3, Int}},
    optimizer_env,
    silent::Bool,
)::Nothing
    base = _base_aggregate_od_route_model(model)
    _endpoint_coverage_applicable(base) || return nothing
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
        "max_walking_distance, or set unmet_demand_penalty for an always-feasible relaxation."
    ))
    return nothing
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
        unmet_demand_penalty=base.unmet_demand_penalty,
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
