"""
`build_model` for the Benders MASTER of the joint routing+assignment model, plus the
convenience entry point that derives a master from the monolith the caller actually asked
to solve.

# Blocks used

Every row here is written in `y` alone, and every one of them is an existing shared block:

- `add_station_selection_variables!` -- `y`, binary
- `add_benders_cut_variables!` -- `Theta`, the cut placeholders (`variables/benders_cuts.jl`)
- `add_station_limit_constraint!(...; equality=true)` -- `sum(y) == k`
- `add_aggregate_od_route_endpoint_feasibility_constraints!` -- reachability rows
- `set_benders_master_objective!` -- `min sum(Theta)`

Nothing else belongs in the master: `y` appears in the monolith's coverage rows not at
all and in its objective not at all, so the rest of the model is
`build_subproblem.jl`'s, and between them the two cover the monolith's rows exactly once
each.
"""

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteJointRoutingAssignmentFormulation,
                solver::BendersSolver) -> BuildResult

Solve the joint routing+assignment model by Benders decomposition. Convenience entry
point: derives the master formulation (at the default `MultiCut(:scenario)`) and forwards.

This is the ordinary call -- `run_opt(problem, AggregateODRouteJointRoutingAssignmentFormulation(...),
BendersSolver(...))`. The user names ONE formulation, because Benders is an algorithm for
the same model `DirectMIPSolver` and `CGSolver` solve, not a different model; the
master/subproblem pair is machinery, derived here so the three can never disagree (see the
family's `shared.jl`). Pass an explicitly constructed
`AggregateODRouteJointRoutingAssignmentMasterFormulation` instead to choose a different
`cut_mode`.
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteJointRoutingAssignmentFormulation,
        solver::BendersSolver,
    )::BuildResult
    return build_model(
        problem,
        AggregateODRouteJointRoutingAssignmentMasterFormulation(formulation),
        solver,
    )
end

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteJointRoutingAssignmentMasterFormulation,
                solver::BendersSolver) -> BuildResult

The real build: the master model, plus -- stashed on it -- the second-stage machinery the
`BendersSolver` hooks will need, since those hooks only ever see `build_result`/`mapping`/`m`.

Stashed on the master model:

- `:benders_subproblem_builds` -- one `BuildResult` per scenario, built ONCE here. The
  joint model's second stage separates exactly by scenario, and rebuilding these per
  iteration would dominate the run; per iteration the loop only re-`fix`es `y` and
  re-optimizes, which lets Gurobi warm-start dual simplex from the previous basis.
- `:benders_subproblem_formulation` -- the derived subproblem formulation, for reporting.
- `:benders_cut_variables` -- the `Theta` map, keyed by cut group (`benders_cut_group`).
- `:benders_cut_signatures` -- the dedup set behind `add_benders_cut!`'s "was this cut
  new" count (see `cuts.jl`). Rounded to 6 decimals, so it is a dedup key ONLY.
- `:benders_cuts` -- `(group, ConstraintRef)` for every cut actually added. This is what an
  auditor must read: a cut rebuilt from its rounded signature is stronger than the real one
  and reports violations the real cut does not have.

**One `mapping`, shared.** The map is created once here and handed to every subproblem
build. It must be the same object: the `(s,p)` demand-group keys and the `valid_jk_pairs`
lists are what the coverage/linking rows and the columns' `assignments` are all written
against, and a master reasoning about one indexing while a subproblem prices another is
not something anything downstream would catch.
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteJointRoutingAssignmentMasterFormulation,
        solver::BendersSolver,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)
    aggregate_od_route_validate_feasible_coverage(data, mapping)

    subproblem_formulation = AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation(
        formulation;
        max_stops = _benders_subproblem_max_stops(formulation, solver.subproblem),
    )

    # The oracle decides where the subproblems' columns come from. `:direct_enumeration`
    # builds the whole universe here, once. `:column_generation` seeds each subproblem with
    # the two-stop routes -- enough to guarantee every coverage row has a term -- and lets
    # pricing grow the pool on demand, so nothing is enumerated at all.
    pricing_oracle = solver.subproblem.oracle === :column_generation
    t_enum = time()
    columns = if pricing_oracle
        joint_routing_assignment_two_stop_seed_columns(data, mapping)
    else
        enumerate_joint_routing_assignment_columns(
            problem, subproblem_formulation, data;
            max_routes = solver.subproblem.max_routes,
            time_limit_sec = solver.subproblem.enumeration_time_limit_sec,
        )
    end
    enumeration_sec = time() - t_enum

    m = Model(() -> Gurobi.Optimizer())

    # ---- 1. Parameters ----
    m[:aggregate_od_route_formulation] = formulation
    m[:joint_routing_assignment_data] = data
    m[:joint_routing_assignment_l] = problem.k
    m[:benders_subproblem_formulation] = subproblem_formulation

    # ---- 2. Variables ----
    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    y = m[:y]
    theta_cuts = add_benders_cut_variables!(m, data, formulation.cut_mode)
    m[:benders_cut_variables] = theta_cuts
    variable_counts["benders_cut_placeholders"] = length(theta_cuts)

    # ---- 3. Constraints ----
    constraint_counts = Dict{String, Int}()
    constraint_counts["station_limit"] =
        add_station_limit_constraint!(m, data, problem.k; equality = true)
    endpoint_feasibility =
        add_aggregate_od_route_endpoint_feasibility_constraints!(m, data, mapping, y)
    constraint_counts["endpoint_feasibility"] = length(endpoint_feasibility)
    # Cuts arrive one iteration at a time; the master starts with none, which is why
    # `Theta`'s lower bound is what keeps iteration 1 bounded.
    constraint_counts["benders_cuts"] = 0
    m[:benders_cut_signatures] = Set{Any}()
    # The cuts actually added, as (cut group, ConstraintRef). Distinct from the signature set
    # above, which is rounded for dedup and therefore unusable for auditing validity -- see
    # `cuts.jl`. Keeping the refs also makes the accumulated cuts inspectable after a solve.
    m[:benders_cuts] = Tuple{Int, ConstraintRef}[]

    # ---- 4. Objective ----
    set_benders_master_objective!(m, theta_cuts)

    # ---- 5. Second stage ----
    subproblem_builds = BuildResult[
        _build_joint_routing_assignment_subproblem_model(
            data, mapping, subproblem_formulation, s, columns;
            pricing_enabled = pricing_oracle,
            pricing = solver.subproblem.pricing,
        )
        for s in 1:n_scenarios(data)
    ]
    m[:benders_subproblem_builds] = subproblem_builds

    extra_counts = Dict{String, Int}(
        "demand_groups" => sum(length(mapping.Omega_s[s]) for s in 1:n_scenarios(data); init = 0),
        "enumerated_columns" => length(columns),
        "subproblem_models" => length(subproblem_builds),
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)

    metadata = Dict{String, Any}(
        "benders_subproblem_oracle" => solver.subproblem.oracle,
        # Under :column_generation this is the SEED size, not a universe -- the pool grows
        # per scenario as pricing runs, so read `benders_cg_pool_final` for the real count.
        "benders_seed_columns" => length(columns),
        "benders_enumerated_columns" => pricing_oracle ? 0 : length(columns),
        "benders_enumeration_sec" => enumeration_sec,
        "benders_subproblem_max_stops" => subproblem_formulation.max_stops,
        "benders_formulation_max_stops" => formulation.max_stops,
        "benders_optimality_scope" =>
            subproblem_formulation.max_stops < formulation.max_stops ?
            "max_stops_restricted" : "full_route_universe",
        "benders_cut_mode" => string(typeof(formulation.cut_mode).name.name),
    )
    return BuildResult(m, mapping, nothing, counts, metadata)
end

"""
    _benders_subproblem_max_stops(formulation, config) -> Union{Nothing, Int}

The `max_stops` the subproblem should be derived at: the oracle's cap, but never *above*
the formulation's own value.

Taking the min rather than the cap outright matters in both directions. A formulation at
`max_stops=3` must not be widened to the oracle's default 4 -- that would enumerate
columns the model does not contain. A formulation at `max_stops=10` (or unbounded) must be
narrowed, because the enumerator is exponential in it; the resulting scope reduction is
reported, not silent (see `BendersSubproblemConfig`).
"""
function _benders_subproblem_max_stops(
        formulation::AggregateODRouteJointRoutingAssignmentMasterFormulation,
        config::BendersSubproblemConfig,
    )::Union{Nothing, Int}
    isnothing(config.max_stops) && return nothing
    return min(config.max_stops, formulation.max_stops)
end
