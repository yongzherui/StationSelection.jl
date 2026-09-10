"""
`build_model` for one scenario's Benders SUBPROBLEM of the joint routing+assignment model.

# Blocks used -- all of them the monolith's own

- `_stash_joint_routing_assignment_cost_parameters!` -- the cost weights and pool
  containers the column machinery reads off `m` (`optimize/aggregate_od_route/joint_shared.jl`)
- `add_station_selection_variables!` -- `y`, relaxed, then pinned per iteration with
  `JuMP.fix`
- `add_walk_variables!(...; scenarios=[s])` -- `x_walk`
- `add_joint_routing_assignment_coverage_constraints!(...; scenarios=[s])`
- `add_joint_routing_assignment_station_linking_constraints!(...; scenarios=[s])`
- `set_joint_routing_assignment_objective!` -- the walking terms
- `add_joint_routing_assignment_column!` -- one call per column, which is what patches the
  `theta` coefficients into the coverage/linking rows and the objective

That list is the monolith's build minus the pure-`y` rows (which are the master's) --
verbatim, not re-derived. The only genuinely new lines below are the scenario filter on
the column pool and the counts.

# Why `y` is a fixed variable and not a right-hand side

Keeping `y` as a variable is what lets
`add_joint_routing_assignment_station_linking_constraints!` be reused with no changes: the
rows stay `theta - y[j] <= 0` exactly as the monolith writes them. Pinning is
`JuMP.fix(y[j], yhat[j]; force=true)` (`force` because `add_station_selection_variables!`
gives the relaxed `y` explicit `[0,1]` bounds), done per iteration in `subproblem.jl`, and
the linking-row duals then read exactly as they do for the CG master --
`extract_joint_routing_assignment_duals`' own sign convention. `y` carries no objective
coefficient, so fixing it changes the feasible region and nothing else.

# Always relaxed

`relax_integrality=true` unconditionally: the cut is the LP dual, so an integral
subproblem would have no usable dual at all. See the subproblem formulation's docstring
for what that means for the answer (`y` binary, second stage continuous -- not
`DirectMIPSolver`'s optimum).
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation,
                solver::BendersSolver; scenario, columns=nothing) -> BuildResult

Standalone entry point for one scenario's subproblem -- for testing it in isolation
against a hand-fixed `y`. The Benders loop does not come through here: `build_master.jl`
calls `_build_joint_routing_assignment_subproblem_model` directly with the master's own
`mapping` and the single shared column pool, because both MUST be shared across the
master and every scenario.

`columns=nothing` enumerates the pool here (filtered to `scenario` by the builder below).
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation,
        solver::BendersSolver;
        scenario::Int,
        columns=nothing,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)
    aggregate_od_route_validate_feasible_coverage(data, mapping)
    resolved_columns = something(
        columns,
        enumerate_joint_routing_assignment_columns(
            problem, formulation, data;
            max_routes = solver.subproblem.max_routes,
            time_limit_sec = solver.subproblem.enumeration_time_limit_sec,
        ),
    )
    return _build_joint_routing_assignment_subproblem_model(
        data, mapping, formulation, scenario, resolved_columns,
    )
end

"""
    _build_joint_routing_assignment_subproblem_model(data, mapping, formulation, scenario, columns)
        -> BuildResult

Build scenario `scenario`'s second stage over `columns` (the whole pool -- entries for
other scenarios are skipped here rather than pre-filtered by the caller, so one shared
pool serves every scenario's build).

**The pool is not filtered by `y`.** Every column for this scenario goes in, and the
linking rows `theta <= y[j]` are what zero out the ones the master did not build. That is
the whole mechanism: filtering the pool to the open stations instead would leave the
linking rows without binding duals, and the "cut" derived from it would be a no-good cut
over station sets, not a Benders cut -- valid-looking, far weaker, and not what the bound
contract assumes.
"""
function _build_joint_routing_assignment_subproblem_model(
        data::StationSelectionData,
        mapping::AggregateODRouteMap,
        formulation::AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation,
        scenario::Int,
        columns;
        pricing_enabled::Bool=false,
        pricing::CGPricingConfig=CGPricingConfig(),
    )::BuildResult
    1 <= scenario <= n_scenarios(data) ||
        throw(ArgumentError("scenario $scenario out of range 1:$(n_scenarios(data))"))
    m = Model(() -> Gurobi.Optimizer())
    set_silent(m)
    scenarios = [scenario]

    # ---- 1. Parameters ----
    _stash_joint_routing_assignment_cost_parameters!(
        m, data, formulation; relax_integrality = true,
    )
    m[:benders_subproblem_scenario] = scenario
    pricing_enabled && _stash_joint_routing_assignment_subproblem_pricing!(
        m, data, formulation, pricing,
    )

    # ---- 2. Variables ----
    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] =
        add_station_selection_variables!(m, data; relax_integrality = true)
    y = m[:y]
    x_walk = add_walk_variables!(
        m, data, mapping; scenarios = scenarios, relax_integrality = true,
    )
    variable_counts["x_walk"] = length(x_walk)

    # ---- 3. Constraints ----
    constraint_counts = Dict{String, Int}()
    coverage = add_joint_routing_assignment_coverage_constraints!(
        m, data, mapping, x_walk; scenarios = scenarios,
    )
    pickup_link, dropoff_link = add_joint_routing_assignment_station_linking_constraints!(
        m, data, mapping, y; scenarios = scenarios,
    )
    m[:joint_routing_assignment_coverage] = coverage
    m[:joint_routing_assignment_pickup_link] = pickup_link
    m[:joint_routing_assignment_dropoff_link] = dropoff_link
    constraint_counts["coverage"] = length(coverage)
    constraint_counts["pickup_link"] = length(pickup_link)
    constraint_counts["dropoff_link"] = length(dropoff_link)
    # No station-limit row and no endpoint-feasibility rows: both are written purely in
    # `y`, which is fixed here, so they are the master's and including them would only
    # risk declaring this model infeasible for a `y` the master had already accepted.

    # ---- 4. Objective ----
    set_joint_routing_assignment_objective!(m, data, mapping, x_walk)

    # ---- 5. Columns ----
    n_seeded = 0
    for column in columns
        Int(column.metadata["scenario"]) == scenario || continue
        _theta, action = add_joint_routing_assignment_column!(m, data, mapping, column)
        action === :added && (n_seeded += 1)
    end

    extra_counts = Dict{String, Int}(
        "scenario" => scenario,
        "demand_groups" => length(mapping.Omega_s[scenario]),
        "columns_added" => n_seeded,
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end

"""
    _stash_joint_routing_assignment_subproblem_pricing!(m, data, formulation, pricing)

The extra model state a subproblem needs to be solved by COLUMN GENERATION rather than over
an enumerated pool. Called only for `oracle = :column_generation`; the enumeration oracle
never prices, and stashing pricing state it cannot use would assert a capability the model
does not have (the same reason `_stash_joint_routing_assignment_cost_parameters!` leaves all
of this out).

Two pieces beyond the obvious node list and travel-cost table:

**`:joint_routing_assignment_pricing_formulation` is a MONOLITH**, derived from this
subproblem formulation, and it is what gets handed to `_run_pricing_round`. Every pricing
hook in `label_setting/joint_routing_assignment/pricing_round.jl` dispatches on
`AggregateODRouteJointRoutingAssignmentFormulation` specifically, so passing the subproblem
type would find no methods. Deriving the monolith from this formulation (rather than
stashing it under `:aggregate_od_route_formulation` and pretending) keeps the model's own
identity honest while giving the pricer the type it dispatches on -- and since the two carry
the identical six encoding fields, the route universe and costs it prices against are
exactly this subproblem's.

**`max_stops` is the FORMULATION's**, with no oracle cap. That is the point of this oracle:
the pricer explores on demand, so there is nothing to bound up front, and the run keeps a
`full_route_universe` optimality scope.
"""
function _stash_joint_routing_assignment_subproblem_pricing!(
        m::Model,
        data::StationSelectionData,
        formulation::AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation,
        pricing::CGPricingConfig,
    )
    n = data.n_stations
    m[:joint_routing_assignment_pricing_formulation] =
        AggregateODRouteJointRoutingAssignmentFormulation(
            route_regularization_weight = formulation.route_regularization_weight,
            walk_cost_weight = formulation.walk_cost_weight,
            repositioning_time = formulation.repositioning_time,
            max_wait_time = formulation.max_wait_time,
            detour_factor = formulation.detour_factor,
            max_stops = formulation.max_stops,
        )
    m[:joint_routing_assignment_pricing_mode] = something(pricing.mode, :exact)
    m[:joint_routing_assignment_compensated_dominance] = pricing.compensated_dominance
    m[:joint_routing_assignment_relaxed_cluster_guide_routes] = pricing.relaxed_cluster_guide_routes
    m[:joint_routing_assignment_aligned_subset_max] = pricing.relaxed_cluster_aligned_subset_max
    m[:joint_routing_assignment_relaxed_cluster_max_count] = pricing.relaxed_cluster_max_count
    m[:joint_routing_assignment_nodes] = collect(1:n)
    travel_cost = Dict{Tuple{Int, Int}, Float64}()
    for i in 1:n, j in 1:n
        i == j && continue
        cost = get_routing_cost(data, i, j)
        isfinite(cost) && (travel_cost[(i, j)] = cost)
    end
    m[:joint_routing_assignment_travel_cost] = travel_cost
    return nothing
end
