"""
The Benders SUBPROBLEM half of the joint routing+assignment model -- one scenario's
second stage, with `y` a fixed parameter rather than a decision. See `shared.jl` for the
family and the derivation rationale.
"""

export AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation

"""
    AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation <: AbstractFormulation

One scenario's Benders subproblem for
`AggregateODRouteJointRoutingAssignmentFormulation`: choose route columns (`theta`) and
direct walks (`x_walk`) to cover that scenario's demand groups at minimum cost, given a
station set `y` fixed by the master.

Carries the family's six shared encoding fields and nothing else -- the scenario it covers
and the `y` it is evaluated at are not fields. Both are deliberate:

- **The scenario is a build argument.** One model per scenario is built once
  (`build_subproblem.jl`), because the second stage separates exactly by scenario, and a
  formulation that named its scenario would make `n_scenarios` distinct formulation
  objects describing one model.
- **`y` is model state, not encoding.** `y` is created as an ordinary (relaxed) station
  variable and pinned per Benders iteration with `JuMP.fix(y[j], yhat[j]; force=true)`.
  Fixing rather than substituting is what lets
  `add_joint_routing_assignment_station_linking_constraints!` be reused VERBATIM -- the
  rows stay `theta - y[j] <= 0`, exactly as the monolith writes them, so there is no
  second code path for a numeric right-hand side to drift from the first. It also makes
  the per-iteration update `n` `fix` calls instead of a `set_normalized_rhs` sweep over
  every linking row, and it hands back the cut's `y` coefficients twice over (aggregated
  linking-row duals, and `reduced_cost(y[j])`), which
  `benders/subproblem.jl` cross-checks against each other.

# Always an LP

The subproblem must be solved as an LP: its dual is where the Benders cut comes from.
`build_subproblem.jl` therefore builds it relaxed unconditionally, and the consequence
travels with the answer -- what a `BendersSolver` run converges to is the optimum of the
**mixed** model (`y` binary, `theta`/`x_walk` continuous), not `DirectMIPSolver`'s
all-binary optimum over the same column pool. Those differ by the formulation's LP-IP gap,
which is not small in this family. The result's
`metadata["benders_second_stage_relaxed"]` records it.

# Construction

    AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation(parent; max_stops=nothing)

`parent` is any family member -- in the normal path the master, which is the only
formulation `build_model(problem, ::MasterFormulation, ::BendersSolver)` has at hand.

`max_stops` narrows the route universe relative to `parent`, for the
`:direct_enumeration` oracle, whose pool is exponential in `max_stops` and is only
tractable at small values (measured: 16,320 columns at `max_stops=4` on a 10-station /
8-pair instance). It is validated to be a RESTRICTION -- passing a value above the
parent's throws rather than silently widening the subproblem past the model the master
believes it is decomposing. When it does narrow, the optimality claim narrows with it and
`metadata["benders_optimality_scope"]` says so; `nothing` (the default) keeps the parent's
own value and leaves the claim full-universe.
"""
struct AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation <: AbstractFormulation
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int

    function AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation(
            parent;
            max_stops::Union{Nothing, Int}=nothing,
        )
        f = _joint_routing_assignment_shared_fields(parent)
        effective_max_stops = something(max_stops, f.max_stops)
        effective_max_stops <= f.max_stops || throw(ArgumentError(
            "Benders subproblem max_stops=$effective_max_stops exceeds the parent " *
            "formulation's $(f.max_stops): a subproblem may only RESTRICT the route " *
            "universe the master is decomposing, never widen it (a wider subproblem " *
            "would price columns the master's model does not contain)",
        ))
        resolved_max_stops = _validate_aggregate_od_route_formulation_fields(
            f.route_regularization_weight, f.walk_cost_weight, f.repositioning_time,
            f.max_wait_time, f.detour_factor, effective_max_stops,
        )
        new(
            f.route_regularization_weight,
            f.walk_cost_weight,
            f.repositioning_time,
            f.max_wait_time,
            f.detour_factor,
            resolved_max_stops,
        )
    end
end
