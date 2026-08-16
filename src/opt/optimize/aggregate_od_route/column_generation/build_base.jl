"""
`build_model` for `StationSelectionProblem` paired with `AggregateODRouteBaseFormulation`
and `CGSolver` -- the same `y`/`x`/`θ` master `DirectMIPSolver`'s build
(`optimize/aggregate_od_route/direct/build_base.jl`) solves (both share
`_aggregate_od_route_base_master_core!`, `optimize/aggregate_od_route/base_shared.jl`), but
starting from an *empty* column pool and growing it via pricing instead of an up-front
exhaustive enumeration. `DirectMIPSolver`'s build is otherwise untouched by this file: it
still batch-creates `theta` with the closed-form `add_route_variables!`
(`variables/routes.jl`), which stays the right choice for a fully-known column pool. This
build instead routes every `theta` variable -- seed and CG-discovered alike -- through the
single incremental entry point `add_aggregate_od_route_base_column!`
(`constraints/aggregate_od_route/base/route_activation.jl`), matching
`AggregateODRouteJointRoutingAssignmentFormulation`'s own CG style
(`add_joint_routing_assignment_column!`).
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteBaseFormulation,
                solver::CGSolver) -> BuildResult

Free assignment only. Always LP-relaxed (`CGSolver` needs valid simplex duals off this
master). `mapping` starts with an empty column pool (`initial_columns=AggregateODRouteColumn[]`,
overriding `create_aggregate_od_route_map`'s own singleton default) so `route_link` builds
closed-form to `x <= 0` for every row and gets patched up by the seed pass below through the
same incremental path CG pricing uses later -- there is exactly one way `theta` ever enters
this master.

Seed columns default to one route per geographically valid `(j,k)` pair, restricted to the
scenarios that actually use it (`_aggregate_od_route_base_seed_columns`, this file) --
the same "one route per pair, serves every demand group using it" idea as Joint's own
two-stop seed, just built from `_singleton_aggregate_od_route_columns`
(`data/maps/aggregate_od_route_map.jl`, already used as `create_aggregate_od_route_map`'s
default) instead of a separate seeding module. `solver.initial_columns`, when given,
overrides this default entirely -- each entry must already carry `metadata["scenario"]`,
matching what `add_aggregate_od_route_base_column!` expects.
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteBaseFormulation,
        solver::CGSolver,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(
        problem, formulation, data; initial_columns=AggregateODRouteColumn[],
    )
    aggregate_od_route_validate_feasible_coverage(data, mapping)

    seed_columns = something(
        solver.initial_columns,
        _aggregate_od_route_base_seed_columns(data, mapping),
    )
    return _build_aggregate_od_route_base_cg_model(
        data, mapping, problem.k, formulation, true, seed_columns,
    )
end

"""
    _aggregate_od_route_base_seed_columns(data, mapping) -> Vector{AggregateODRouteColumn}

One scenario-tagged column per `(j,k,s)` with `(j,k) in mapping.active_jk_s[s]` -- reuses
`_singleton_aggregate_od_route_columns` (`data/maps/aggregate_od_route_map.jl`) purely for
the geometry/`tau`/finite-routing-cost validation it already does, then fans each resulting
pair out to only the scenarios that actually use it (unlike `DirectMIPSolver`'s batch
`theta`, which creates every `(column,scenario)` pair regardless of relevance -- CG has no
reason to pay for variables `route_link` would never reference).
"""
function _aggregate_od_route_base_seed_columns(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
    )::Vector{AggregateODRouteColumn}
    geometries = _singleton_aggregate_od_route_columns(mapping.active_jk_s, data)
    seed = AggregateODRouteColumn[]
    for geometry in geometries
        j, k = only(geometry.od_pairs)
        for s in 1:n_scenarios(data)
            (j, k) in mapping.active_jk_s[s] || continue
            push!(seed, AggregateODRouteColumn(
                geometry.id, geometry.od_pairs, geometry.tau;
                metadata=Dict{String, Any}("scenario" => s, "initialization" => "singleton"),
            ))
        end
    end
    return seed
end

"""
    _build_aggregate_od_route_base_cg_model(data, mapping, l, formulation,
        relax_integrality, seed_columns) -> BuildResult

Shared master-construction body for `build_model` above (`relax_integrality=true`, seed =
the singleton pool or `solver.initial_columns`) and `_aggregate_od_route_integer_recovery_build`
(`optimize/aggregate_od_route/column_generation/dispatch.jl`;
`relax_integrality=false`, seed = exactly the `(column,scenario)` pairs CG discovered).

`y`/`x`/`x_walk`/coverage/station-linking/route-link/objective all come from the shared
`_aggregate_od_route_base_master_core!` (`optimize/aggregate_od_route/base_shared.jl`), the
same function `DirectMIPSolver`'s build calls -- passed an empty `theta` here (unlike
Direct's fully-populated one), so `route_link` degrades naturally to `x <= 0` for every row
and the objective to just `x`/`x_walk` cost, both as a direct consequence of `theta` being
empty, not a special "CG mode" in that shared code. Every `theta` variable then arrives
through the seed loop at the end, via `add_aggregate_od_route_base_column!`, which patches
`route_link`'s `-1.0` coefficients (and the objective's own coefficient for that column) in
after the fact.
"""
function _build_aggregate_od_route_base_cg_model(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        l::Int,
        formulation::AggregateODRouteBaseFormulation,
        relax_integrality::Bool,
        seed_columns::AbstractVector{AggregateODRouteColumn},
    )::BuildResult
    m = Model(() -> Gurobi.Optimizer())

    # ---- 1. Parameters ----
    n = data.n_stations
    m[:aggregate_od_route_formulation] = formulation
    m[:aggregate_od_route_base_data] = data
    m[:aggregate_od_route_base_l] = l
    m[:aggregate_od_route_base_relax_integrality] = relax_integrality
    m[:aggregate_od_route_base_route_regularization_weight] = formulation.route_regularization_weight
    m[:aggregate_od_route_base_repositioning_time] = formulation.repositioning_time
    m[:aggregate_od_route_base_walk_cost_weight] = formulation.walk_cost_weight
    m[:aggregate_od_route_base_max_wait_time] = formulation.max_wait_time
    m[:aggregate_od_route_base_max_stops] = formulation.max_stops
    m[:aggregate_od_route_base_detour_factor] = formulation.detour_factor
    m[:aggregate_od_route_base_nodes] = collect(1:n)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n, j in 1:n
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) && (travel_cost[(i, j)] = cost)
    end
    m[:aggregate_od_route_base_travel_cost] = travel_cost
    m[:aggregate_od_route_base_column_signatures] = Dict{Any, Int}(
        _aggregate_od_route_column_signature(c) => c.id for c in mapping.columns
    )
    m[:aggregate_od_route_base_columns_by_id] = Dict{Int, AggregateODRouteColumn}(
        c.id => c for c in mapping.columns
    )
    m[:aggregate_od_route_base_next_column_id] = Ref(maximum(mapping.column_ids; init=0) + 1)

    # ---- 2/3/4. Variables/constraints/objective (shared core) ----
    # `theta` starts empty -- `_aggregate_od_route_base_master_core!`'s `route_link` and
    # objective calls both degrade to the "no columns yet" case (`x <= 0`, walk-only cost)
    # as a natural consequence, not a special mode. Every seed/CG-discovered column's own
    # theta variable and objective coefficient are added later by
    # `add_aggregate_od_route_base_column!` via `set_objective_coefficient`.
    m[:aggregate_od_route_base_theta] = Dict{Tuple{Int, Int}, VariableRef}()
    theta = m[:aggregate_od_route_base_theta]
    _y, x, _x_walk, route_link, variable_counts, constraint_counts =
        _aggregate_od_route_base_master_core!(
            m, data, mapping, l, formulation, theta; relax_integrality=relax_integrality,
        )
    m[:aggregate_od_route_base_route_link] = route_link
    m[:aggregate_od_route_base_route_link_keys_by_pair] =
        _aggregate_od_route_base_x_keys_by_pair_scenario(x)

    # ---- 5. Seed columns ----
    n_seeded = 0
    for column in seed_columns
        _theta, action = add_aggregate_od_route_base_column!(m, data, mapping, column)
        action === :added && (n_seeded += 1)
    end

    extra_counts = Dict{String, Int}(
        "demand_groups" => sum(length(mapping.Omega_s[s]) for s in 1:n_scenarios(data); init=0),
        "seed_columns_added" => n_seeded,
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end

"""
    _aggregate_od_route_integer_recovery_build(::AggregateODRouteBaseFormulation,
        build_result, mapping, m) -> BuildResult

`CGSolver` hook real logic (dispatched from
`optimize/aggregate_od_route/column_generation/dispatch.jl`, `recover_integer_solution=true`):
rebuilds the master from scratch via `_build_aggregate_od_route_base_cg_model` as a genuine
MIP, seeded with exactly the `(column_id,s)` activations CG discovered
(`keys(m[:aggregate_od_route_base_theta])`) -- not the original singleton seed. `mapping` is
reused in place: it already holds every geometry (seed and CG-discovered) via
`add_aggregate_od_route_base_column!`'s registration, so no re-derivation is needed, only a
lookup through `m[:aggregate_od_route_base_columns_by_id]`.

Feasibility: the original `build_model` seed pass activates the full singleton pool over
every scenario that uses each pair before CG runs at all, and CG only ever adds activations
(never removes), so `keys(m[:aggregate_od_route_base_theta])` is always a superset of that
original seed -- the same coverage guarantee `aggregate_od_route_validate_feasible_coverage`
+ `_singleton_aggregate_od_route_columns`'s finite-routing-cost check already establishes for
`DirectMIPSolver`'s own build.
"""
function _aggregate_od_route_integer_recovery_build(
        ::AggregateODRouteBaseFormulation,
        build_result::BuildResult,
        mapping::AggregateODRouteMap,
        m::JuMP.Model,
    )::BuildResult
    data = m[:aggregate_od_route_base_data]
    l = Int(m[:aggregate_od_route_base_l])
    formulation = AggregateODRouteBaseFormulation(
        route_regularization_weight=m[:aggregate_od_route_base_route_regularization_weight],
        walk_cost_weight=m[:aggregate_od_route_base_walk_cost_weight],
        repositioning_time=m[:aggregate_od_route_base_repositioning_time],
        max_wait_time=m[:aggregate_od_route_base_max_wait_time],
        detour_factor=m[:aggregate_od_route_base_detour_factor],
        max_stops=m[:aggregate_od_route_base_max_stops],
    )
    columns_by_id = m[:aggregate_od_route_base_columns_by_id]
    discovered = AggregateODRouteColumn[]
    for (column_id, s) in keys(m[:aggregate_od_route_base_theta])
        geometry = columns_by_id[column_id]
        push!(discovered, AggregateODRouteColumn(
            geometry.id, geometry.od_pairs, geometry.tau; metadata=Dict{String, Any}("scenario" => s),
        ))
    end
    return _build_aggregate_od_route_base_cg_model(
        data, mapping, l, formulation, false, discovered,
    )
end
