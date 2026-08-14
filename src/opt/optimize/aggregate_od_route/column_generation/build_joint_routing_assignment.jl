"""
`build_model` for `StationSelectionProblem` paired with
`AggregateODRouteJointRoutingAssignmentFormulation` -- the real joint routing+assignment
master: `y` (station), `v` (unserved-demand-group slack), `x_same` (same-station walk),
and route columns (`θ`) whose own coefficients directly carry OD assignment. No separate
assignment variable `x` exists here -- routing and assignment are decided by the same
variable, which is what makes this "joint" (see the formulation's own docstring for why
the earlier `y`/`x`/`θ` two-layer model this session briefly built under this same name
was not that).

Built directly off `AggregateODRouteMap` -- no separate `MasterData`/`Passenger`
structures, matching every other `AggregateODRouteProblem` build path. Demand groups are
keyed by `(s, p)::Tuple{Int,Int}` throughout (`p` the position of `(o,d)` within
`mapping.Omega_s[s]`); `data` and the objective/column-cost
scalars are stashed on `m[...]` (mirrors `m[:aggregate_od_route_route_regularization_weight]`)
so the `CGSolver` hooks -- which only ever see `build_result`/`m`, not `problem` -- can
read them back.
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteJointRoutingAssignmentFormulation,
                solver::CGSolver) -> BuildResult

Free assignment only.

Always seeds every two-stop route (`joint_routing_assignment_two_stop_seed_columns`)
before returning, for the same reason the discarded model did: an empty pool is feasible
(via `v`'s big-M slack) but leaves the first several CG iterations draining big-M instead
of improving routing cost.

The model is always built LP-relaxed, `y` included: `CGSolver` needs valid simplex duals
off this master, so `y` comes from `add_station_selection_variables!(m, data;
relax_integrality=true)` -- continuous `[0,1]`, not the `Bin` every other
`AggregateODRouteProblem` build path uses.
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteJointRoutingAssignmentFormulation,
        solver::CGSolver,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)

    m = Model(() -> Gurobi.Optimizer())

    variable_counts = Dict{String, Int}()
    # CGSolver needs valid simplex duals off this master, so `y` must be continuous here
    # -- relax_integrality=true, matching add_aggregate_od_route_theta_variables!'s own
    # kwarg for the same reason.
    n = data.n_stations
    variable_counts["station_selection"] = add_station_selection_variables!(m, data; relax_integrality=true)
    y = m[:y]

    v = add_joint_routing_assignment_slack_variables!(m, data, mapping)
    variable_counts["slack"] = length(v)

    m[:joint_routing_assignment_data] = data
    m[:joint_routing_assignment_route_regularization_weight] = formulation.route_regularization_weight
    m[:joint_routing_assignment_repositioning_time] = formulation.repositioning_time
    m[:joint_routing_assignment_walk_cost_weight] = formulation.walk_cost_weight
    m[:joint_routing_assignment_max_wait_time] = formulation.max_wait_time
    m[:joint_routing_assignment_max_stops] = formulation.max_stops
    m[:joint_routing_assignment_detour_factor] = formulation.detour_factor
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
    m[:joint_routing_assignment_theta] = Dict{Int, VariableRef}()
    m[:joint_routing_assignment_columns] = Dict{Int, JointRoutingAssignmentRouteColumn}()
    m[:joint_routing_assignment_column_signatures] = Dict{Any, Int}()

    constraint_counts = Dict{String, Int}()
    coverage = add_joint_routing_assignment_coverage_constraints!(m, data, mapping, v)
    pickup_link, dropoff_link = add_joint_routing_assignment_station_linking_constraints!(m, data, mapping, y)
    m[:joint_routing_assignment_coverage] = coverage
    m[:joint_routing_assignment_pickup_link] = pickup_link
    m[:joint_routing_assignment_dropoff_link] = dropoff_link
    constraint_counts["coverage"] = length(coverage)
    constraint_counts["pickup_link"] = length(pickup_link)
    constraint_counts["dropoff_link"] = length(dropoff_link)
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.l; equality = true)

    x_same = add_joint_routing_assignment_same_station_variables!(m, data, mapping, coverage, pickup_link, dropoff_link)
    variable_counts["x_same"] = length(x_same)

    unserved_penalty = default_joint_routing_assignment_unserved_penalty(problem, formulation, data, mapping)
    set_joint_routing_assignment_objective!(m, data, mapping, v, x_same, unserved_penalty)

    seeds = joint_routing_assignment_two_stop_seed_columns(data, mapping)
    n_seeded = 0
    for column in seeds
        _theta, action = add_joint_routing_assignment_column!(m, data, mapping, column)
        action === :added && (n_seeded += 1)
    end

    extra_counts = Dict{String, Int}(
        "demand_groups" => sum(length(mapping.Omega_s[s]) for s in 1:n_scenarios(data); init = 0),
        "seed_columns_added" => n_seeded,
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}("unserved_penalty" => unserved_penalty))
end
