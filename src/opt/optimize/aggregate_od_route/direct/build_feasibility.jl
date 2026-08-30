"""
`build_model` for `StationSelectionProblem` paired with `AggregateODRouteFeasibilityFormulation`
and `DirectMIPSolver` -- station selection `y` plus `station_limit`
(`Σy == k`) and `endpoint_feasibility` (`add_aggregate_od_route_endpoint_feasibility_constraints!`,
`constraints/endpoint_feasibility.jl`) only. No `x`/`x_walk`/`θ`, no route enumeration, no
routing-cost table, and a constant objective -- this formulation exists purely to answer
"does some `k`-station selection reach every station-dependent demand group" as a single
fast `optimize!` call.
"""

"""
    check_feasibility(problem::StationSelectionProblem,
                       formulation::Union{AggregateODRouteBaseFormulation,
                                           AggregateODRouteJointRoutingAssignmentFormulation},
                       solver::AbstractSolver)

`run_opt`'s generic feasibility-gate hook (`optimize/run_opt.jl`), specialized for the two
live aggregate-OD-route formulations: solves `AggregateODRouteFeasibilityFormulation`
(this file, `y` + `station_limit` + `endpoint_feasibility` only -- no route columns) via
`DirectMIPSolver` and throws `ArgumentError` unless it comes back `OPTIMAL`, regardless of
which solver (`DirectMIPSolver` or `CGSolver`) is about to actually run. `run_opt` calls
this right after `build_model` and before `optimize_model`, so a structurally infeasible
`k` fails immediately instead of only surfacing after expensive route enumeration or
several CG pricing rounds -- and, worse, sometimes not surfacing correctly even then, if
`recover_integer_solution`'s restricted MIP goes infeasible over an incomplete column pool
and gets misread as "this instance is infeasible" -- see
`notes/2026-08-28_study5_dominance_fix_pilot_infeasible_repro.md` for the investigation
that motivated this.

Does not prove the full problem is feasible -- route/capacity/wait-time/detour
constraints can still fail even when this passes -- only that this necessary condition on
`y` alone isn't already broken. `run_opt` also calls this hook on
`AggregateODRouteFeasibilityFormulation` solves themselves (this file's `build_model`
below), but that formulation doesn't match this method's `Union`, so it falls through to
`run_opt`'s generic no-op default instead of recursing.
"""
function check_feasibility(
        problem::StationSelectionProblem,
        formulation::Union{
            AggregateODRouteBaseFormulation,
            AggregateODRouteJointRoutingAssignmentFormulation,
        },
        solver::AbstractSolver,
    )
    result = run_opt(
        problem, AggregateODRouteFeasibilityFormulation(),
        DirectMIPSolver(config=SolverOptions(silent=true)),
    )
    result.termination_status == MOI.OPTIMAL || throw(ArgumentError(
        "aggregate OD route: no size-$(problem.k) station selection can reach every " *
        "demand group that lacks a direct-walk fallback (endpoint feasibility check " *
        "returned $(result.termination_status)) -- the full problem is infeasible " *
        "regardless of route/routing details",
    ))
    return nothing
end

"""
    build_model(problem::StationSelectionProblem,
                formulation::AggregateODRouteFeasibilityFormulation,
                solver::DirectMIPSolver) -> BuildResult
"""
function build_model(
        problem::StationSelectionProblem,
        formulation::AggregateODRouteFeasibilityFormulation,
        solver::DirectMIPSolver,
    )::BuildResult
    data = problem.data
    mapping = create_aggregate_od_route_map(problem, formulation, data)
    aggregate_od_route_validate_feasible_coverage(data, mapping)

    m = Model(() -> Gurobi.Optimizer())

    variable_counts = Dict{String, Int}()
    variable_counts["station_selection"] = add_station_selection_variables!(m, data)
    y = m[:y]

    constraint_counts = Dict{String, Int}()
    constraint_counts["station_limit"] = add_station_limit_constraint!(m, data, problem.k; equality=true)
    endpoint_feasibility = add_aggregate_od_route_endpoint_feasibility_constraints!(m, data, mapping, y)
    m[:aggregate_od_route_endpoint_feasibility] = endpoint_feasibility
    constraint_counts["endpoint_feasibility"] = length(endpoint_feasibility)

    @objective(m, Min, 0.0)

    extra_counts = Dict{String, Int}(
        "demand_groups" => sum(length(mapping.Omega_s[s]) for s in 1:n_scenarios(data); init=0),
    )
    counts = ModelCounts(variable_counts, constraint_counts, extra_counts)
    return BuildResult(m, mapping, nothing, counts, Dict{String, Any}())
end
