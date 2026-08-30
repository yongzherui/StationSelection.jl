"""
The bare `y`-only feasibility check for the aggregate-OD-route problem: station
selection plus `add_aggregate_od_route_endpoint_feasibility_constraints!`
(`constraints/endpoint_feasibility.jl`) and nothing else -- no `x`/`x_walk`/`θ`, no
routing, no objective beyond a constant. Answers exactly one question, fast: does *some*
`k`-station selection reach every demand group that has no `WALK_ONLY_PAIR` fallback? A
`DirectMIPSolver` run against this formulation is a cheap necessary-condition check to run
before committing to a full `AggregateODRouteBaseFormulation`/
`AggregateODRouteJointRoutingAssignmentFormulation` solve (enumeration or CG) -- `y`
feasible here does not by itself prove the full problem is feasible (route/capacity/time
constraints could still fail), but `y` infeasible here proves the full problem is
infeasible, immediately, with no pricing or enumeration involved. See
`notes/2026-08-28_study5_dominance_fix_pilot_infeasible_repro.md` for the finding that
motivated this.
"""

export AggregateODRouteFeasibilityFormulation

"""
    AggregateODRouteFeasibilityFormulation <: AbstractFormulation

Dispatch marker, no fields -- unlike `AggregateODRouteBaseFormulation`/
`AggregateODRouteJointRoutingAssignmentFormulation`, this formulation has no
encoding-detail knobs to carry: it never touches routing cost, walk cost weighting,
repositioning time, wait time, detour factor, stop count, or dominance mode, since it
builds no route columns and no walk variables at all. Pairs with `StationSelectionProblem`
(`data`, `k`, `max_walking_distance`) and `DirectMIPSolver` only -- there is no pricing
loop to speak of, so `CGSolver` has nothing to do here.

Not a member of `AnyAggregateODRouteFormulation` (`formulations/aggregate_od_route/
joint_routing_assignment.jl`): that Union exists for formulations sharing the full
route-column encoding-detail field set (`enumerate_aggregate_od_route_columns` and
friends read those fields directly), which this formulation deliberately does not carry.
`create_aggregate_od_route_map` (`data/maps/aggregate_od_route_map.jl`) only ever needs
`_aggregate_od_route_allow_walk_only(formulation)` from its `formulation` argument, so its
parameter type is `AbstractFormulation`, wide enough to admit this formulation too without
joining that Union.
"""
struct AggregateODRouteFeasibilityFormulation <: AbstractFormulation end
