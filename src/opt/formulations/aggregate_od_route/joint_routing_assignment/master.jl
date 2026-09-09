"""
The Benders MASTER half of the joint routing+assignment model -- the pure-first-stage
formulation `BendersSolver` optimizes over. See `shared.jl` for the family and for why
this type derives its encoding fields from a parent rather than accepting them loose.
"""

export AggregateODRouteJointRoutingAssignmentMasterFormulation

"""
    AggregateODRouteJointRoutingAssignmentMasterFormulation <: AbstractFormulation

Benders master for `AggregateODRouteJointRoutingAssignmentFormulation`: the station-build
decision `y` plus the cut placeholders `Theta`, and nothing else.

# What it carries, and why that is all of it

`y` couples the monolith's two halves through exactly two constraint families
(`pickup_link`, `dropoff_link`, `constraints/aggregate_od_route/joint_routing_assignment/linking.jl`),
and it appears in NEITHER the coverage rows nor the objective. So the first stage is
`y` plus the rows written only in `y`:

- `Sum_j y[j] == k`  (`add_station_limit_constraint!`)
- `Sum_{j near pt} y[j] >= 1` for every location with no direct-walk fallback
  (`add_aggregate_od_route_endpoint_feasibility_constraints!`)

and the second stage -- `x_walk`, `theta`, the coverage rows, the linking rows and the
whole objective -- is
`AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation`'s.

**The entire objective is second-stage.** This master has no cost of its own; its
objective is `Sum_s Theta[s]` alone. That is why `add_benders_cut_variables!` gives every
`Theta` a finite lower bound of 0 (valid: walking and route costs are all non-negative) --
without it iteration 1 is unbounded rather than merely uninformative. It also means the
first few masters are genuinely degenerate: any `y` meeting the budget is optimal at
`Theta == 0`, so early incumbents are arbitrary and the lower bound starts at 0.

# Fields

`cut_mode` is the one field that is this formulation's own rather than the family's: it
decides how many `Theta` variables the master carries (`MultiCut(:scenario)` -> one per
scenario, `SingleCut` -> one in total), which is a change to the master's *variable set*
and therefore encoding, not a solver knob. The joint model's second stage separates
exactly by scenario -- every column belongs to one scenario
(`column.metadata["scenario"]`) and every coverage/linking row is keyed by `(s, p)` -- so
`MultiCut(:scenario)` is the natural mode and `SingleCut` is precisely its per-iteration
sum.

The remaining six fields are the family's shared encoding set, copied from the parent (see
`shared.jl`).

# Construction

    AggregateODRouteJointRoutingAssignmentMasterFormulation(parent; cut_mode=MultiCut())

`parent` is any member of the family -- normally the monolith the user actually asked to
solve. There is deliberately no loose-keyword constructor: a master built at one
`detour_factor` against a subproblem built at another produces invalid cuts and a
confidently wrong `OPTIMAL`, so the only way to make one is to derive it.
"""
struct AggregateODRouteJointRoutingAssignmentMasterFormulation <: AbstractFormulation
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    cut_mode::AbstractBendersCutMode

    function AggregateODRouteJointRoutingAssignmentMasterFormulation(
            parent;
            cut_mode::AbstractBendersCutMode=MultiCut(),
        )
        f = _joint_routing_assignment_shared_fields(parent)
        resolved_max_stops = _validate_aggregate_od_route_formulation_fields(
            f.route_regularization_weight, f.walk_cost_weight, f.repositioning_time,
            f.max_wait_time, f.detour_factor, f.max_stops,
        )
        new(
            f.route_regularization_weight,
            f.walk_cost_weight,
            f.repositioning_time,
            f.max_wait_time,
            f.detour_factor,
            resolved_max_stops,
            cut_mode,
        )
    end
end
