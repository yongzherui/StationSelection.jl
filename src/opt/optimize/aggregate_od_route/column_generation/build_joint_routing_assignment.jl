"""
`build_model` for `StationSelectionProblem` paired with
`AggregateODRouteJointRoutingAssignmentFormulation` -- the real joint routing+assignment
master: `y` (station), `x_walk` (direct-walk, station-free), and route columns (`θ`)
whose own coefficients directly carry OD assignment. No separate assignment variable `x`
exists here -- routing and assignment are decided by the same variable, which is what
makes this "joint" (see the formulation's own docstring for why the earlier `y`/`x`/`θ`
two-layer model this session briefly built under this same name was not that). No
unserved-demand slack either: `aggregate_od_route_validate_feasible_coverage`
(`data/maps/aggregate_od_route_map.jl`, shared with `AggregateODRouteBaseFormulation`'s
own `build_model`) proves every demand group has a real coverage option before any
variable is built.

Built directly off `AggregateODRouteMap` -- no separate `MasterData`/`Passenger`
structures, matching every other `AggregateODRouteProblem` build path. Demand groups are
keyed by `(s, p)::Tuple{Int,Int}` throughout (`p` the position of `(o,d)` within
`mapping.Omega_s[s]`); `data` and the objective/column-cost
scalars are stashed on `m[...]` (mirrors `m[:aggregate_od_route_route_regularization_weight]`)
so the `CGSolver` hooks -- which only ever see `build_result`/`m`, not `problem` -- can
read them back.

`build_model` itself is a thin entry point over `_build_joint_routing_assignment_model`,
the shared master-construction body: the LP-relaxed CG master and the post-CG
integer-recovery MIP (`integer_recovery_build`, this file, `CGSolver`'s
`recover_integer_solution=true` hook) are otherwise identical constructions -- same
variables/constraints/objective wiring -- differing only in `relax_integrality` and which
column pool seeds `theta`. Building two structurally-identical models by hand in two
places is exactly the drift risk a shared body avoids.
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteJointRoutingAssignmentFormulation,
                solver::CGSolver) -> BuildResult

Free assignment only. Always LP-relaxed (`CGSolver` needs valid simplex duals off this
master).

Seed columns default to every two-stop route (`joint_routing_assignment_two_stop_seed_columns`):
an empty pool is technically feasible via `x_walk` alone whenever direct walking covers
every demand group, but two-stop routes remove the first several CG iterations' phase of
hunting for *any* feasible column per demand group instead of improving routing cost.
`solver.initial_columns`, when given, overrides that default entirely -- e.g. to resume
CG from a previously discovered pool instead of the generic two-stop seed.
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteJointRoutingAssignmentFormulation,
        solver::CGSolver,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)
    aggregate_od_route_validate_feasible_coverage(data, mapping)

    initial_columns = something(
        solver.initial_columns,
        joint_routing_assignment_two_stop_seed_columns(data, mapping),
    )
    return _build_joint_routing_assignment_model(
        data, mapping, problem.k, formulation;
        relax_integrality = true,
        initial_columns = initial_columns,
    )
end

"""
    _build_joint_routing_assignment_model(data, mapping, l, formulation;
        relax_integrality, initial_columns) -> BuildResult

Shared master-construction body -- everything `build_model` above needs for the
LP-relaxed CG master, and everything `integer_recovery_build` below needs to rebuild the
exact same master as a genuine MIP seeded with CG's own discovered column pool instead of
the two-stop seed.

`relax_integrality` threads through to every master variable family: `y`
(`add_station_selection_variables!`), `x_walk`
(`add_walk_variables!`, shared with `AggregateODRouteBaseFormulation`'s own
`build_model`), and `theta` -- the latter not as a
direct argument but stashed as `m[:joint_routing_assignment_relax_integrality]`, since
`theta` variables are created by `add_joint_routing_assignment_column!`, which is also
called later by the fixed-signature `add_columns!` `CGSolver` hook mid-CG and so can't
take the flag as an explicit parameter.

`l` and every formulation scalar are stashed on `m` (mirrors the rest of this function's
`m[:joint_routing_assignment_*]` convention) purely so `integer_recovery_build`, which
only ever sees `build_result`/`m` like every other `CGSolver` hook (not the original
`problem`/`formulation` objects), can read them back to reconstruct an equivalent
`formulation` and rebuild.
"""
function _build_joint_routing_assignment_model(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        l::Int,
        formulation::AggregateODRouteJointRoutingAssignmentFormulation;
        relax_integrality::Bool,
        initial_columns,
    )::BuildResult
    m = Model(() -> Gurobi.Optimizer())

    # ---- 1. Parameters ----
    # Scalars/tables the CGSolver hooks need later: they only ever see `build_result`/`m`,
    # not `problem`/`formulation`, so anything pricing, column-adding, or integer-recovery
    # rebuilding requires has to be stashed here.
    n = data.n_stations
    m[:aggregate_od_route_formulation] = formulation
    m[:joint_routing_assignment_data] = data
    m[:joint_routing_assignment_l] = l
    m[:joint_routing_assignment_relax_integrality] = relax_integrality
    m[:joint_routing_assignment_route_regularization_weight] = formulation.route_regularization_weight
    m[:joint_routing_assignment_repositioning_time] = formulation.repositioning_time
    m[:joint_routing_assignment_walk_cost_weight] = formulation.walk_cost_weight
    m[:joint_routing_assignment_max_wait_time] = formulation.max_wait_time
    m[:joint_routing_assignment_max_stops] = formulation.max_stops
    m[:joint_routing_assignment_detour_factor] = formulation.detour_factor
    m[:joint_routing_assignment_compensated_dominance] = formulation.compensated_dominance
    m[:joint_routing_assignment_pricing_mode] = formulation.pricing_mode
    # Precomputed once here (not per pricing call): pricing needs a dense node list and a
    # full station-to-station routing-cost table, exactly what the discarded MasterData
    # cached -- caching them on `m` avoids re-deriving an O(n^2) table every CG iteration.
    m[:joint_routing_assignment_nodes] = collect(1:n)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n, j in 1:n
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) && (travel_cost[(i, j)] = cost)
    end
    m[:joint_routing_assignment_travel_cost] = travel_cost
    # Empty pool containers: real entries arrive from the seed pass below and (for the
    # LP master) from every later CG iteration (`add_columns!`, routing_and_assignment.jl).
    m[:joint_routing_assignment_theta] = Dict{Int, VariableRef}()
    m[:joint_routing_assignment_columns] = Dict{Int, JointRoutingAssignmentRouteColumn}()
    m[:joint_routing_assignment_column_signatures] = Dict{Any, Int}()

    # ---- 2. Variables ----
    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] =
        add_station_selection_variables!(m, data; relax_integrality = relax_integrality)
    y = m[:y]
    x_walk = add_walk_variables!(m, data, mapping; relax_integrality = relax_integrality)
    variable_counts["x_walk"] = length(x_walk)

    # ---- 3. Constraints ----
    constraint_counts = Dict{String, Int}()
    coverage = add_joint_routing_assignment_coverage_constraints!(m, data, mapping, x_walk)
    pickup_link, dropoff_link = add_joint_routing_assignment_station_linking_constraints!(m, data, mapping, y)
    m[:joint_routing_assignment_coverage] = coverage
    m[:joint_routing_assignment_pickup_link] = pickup_link
    m[:joint_routing_assignment_dropoff_link] = dropoff_link
    constraint_counts["coverage"] = length(coverage)
    constraint_counts["pickup_link"] = length(pickup_link)
    constraint_counts["dropoff_link"] = length(dropoff_link)
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, l; equality = true)

    # ---- 4. Objective ----
    set_joint_routing_assignment_objective!(m, data, mapping, x_walk)

    # ---- 5. Seed initial columns ----
    # Not model construction: this populates the pool through the same
    # `add_joint_routing_assignment_column!` path `add_columns!` uses mid-solve, which is
    # why it has to run after every constraint row and the objective above already exist
    # -- it wires `theta` coefficients into `coverage`/`pickup_link`/`dropoff_link` and
    # patches the objective in place, none of which can target something that isn't built
    # yet.
    n_seeded = 0
    for column in initial_columns
        _theta, action = add_joint_routing_assignment_column!(m, data, mapping, column)
        action === :added && (n_seeded += 1)
    end

    extra_counts = Dict{String, Int}(
        "demand_groups" => sum(length(mapping.Omega_s[s]) for s in 1:n_scenarios(data); init = 0),
        "seed_columns_added" => n_seeded,
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end

"""
    integer_recovery_build(build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model) -> BuildResult

`CGSolver` hook (`opt/solvers/cg_solver.jl`, `recover_integer_solution=true`): rebuilds
the joint routing+assignment master from scratch via `_build_joint_routing_assignment_model`
as a genuine MIP -- `y`/`theta`/`x_walk` all binary -- seeded with exactly the column pool
CG discovered (`m[:joint_routing_assignment_columns]`), not the two-stop seed
`build_model` uses. No further pricing happens against the rebuilt model;
`CGSolver.optimize_model` just re-optimizes it once after this returns (the standard
"restricted master heuristic": feasible and a valid upper bound, but not guaranteed
optimal against the unrestricted column set, since pricing only ever ran against LP
duals).

Feasibility of the rebuild is guaranteed regardless of which columns got discovered:
`x_walk` (always present for every demand group in this formulation) carries no `y`/`z`
linking at all, so "walk everyone directly, activate no routes" is always binary-feasible
for any `y` satisfying the station budget.

Reconstructs an equivalent `formulation` from the scalars `_build_joint_routing_assignment_model`
stashed on `m` at the original build -- this hook only ever sees `build_result`/`mapping`/`m`,
like every other `CGSolver` hook, not the original `problem`/`formulation` objects.
`CGSolver` hook real logic (dispatched from
`optimize/aggregate_od_route/column_generation/dispatch.jl`, which disambiguates from
`AggregateODRouteBaseFormulation`'s own `integer_recovery_build` by formulation type, since
both share `mapping::AggregateODRouteMap`).
"""
function _aggregate_od_route_integer_recovery_build(
    ::AggregateODRouteJointRoutingAssignmentFormulation,
    build_result::BuildResult, mapping::AggregateODRouteMap, m::JuMP.Model,
)::BuildResult
    data = m[:joint_routing_assignment_data]
    l = Int(m[:joint_routing_assignment_l])
    formulation = AggregateODRouteJointRoutingAssignmentFormulation(
        route_regularization_weight = m[:joint_routing_assignment_route_regularization_weight],
        walk_cost_weight = m[:joint_routing_assignment_walk_cost_weight],
        repositioning_time = m[:joint_routing_assignment_repositioning_time],
        max_wait_time = m[:joint_routing_assignment_max_wait_time],
        detour_factor = m[:joint_routing_assignment_detour_factor],
        max_stops = m[:joint_routing_assignment_max_stops],
        compensated_dominance = m[:joint_routing_assignment_compensated_dominance],
        pricing_mode = m[:joint_routing_assignment_pricing_mode],
    )
    initial_columns = collect(values(m[:joint_routing_assignment_columns]))
    return _build_joint_routing_assignment_model(
        data, mapping, l, formulation;
        relax_integrality = false,
        initial_columns = initial_columns,
    )
end
