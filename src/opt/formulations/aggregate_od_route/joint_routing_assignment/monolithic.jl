"""
Formulation-level encoding for the aggregate-OD-route problem's compact joint
routing+assignment MILP/LP -- the non-Benders-decomposed representation, solved via
column generation (`CGSolver`). See `base.jl` in this directory for the sibling
formulation solved directly against an enumerated column pool (`DirectMIPSolver`), and `master.jl`/`benders_subproblem.jl` in this directory for the two
formulations this one decomposes into under `BendersSolver`. Model construction lives per
(problem family × solver algorithm) under `opt/optimize/` instead -- see
`opt/optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`.
"""

export AggregateODRouteJointRoutingAssignmentFormulation

"""
    AggregateODRouteJointRoutingAssignmentFormulation <: AbstractFormulation

The compact joint routing+assignment MILP/LP, solved via column generation
(`CGSolver`) -- *how* a `StationSelectionProblem` is served, weighted, and staged when
**not** Benders-decomposed. `AggregateODRouteBaseFormulation` (`base.jl`) shares this
exact same field set and structural shape but is solved directly against an
exhaustively enumerated column pool (`DirectMIPSolver`) rather than iteratively priced
-- the two are separate marker types, not one formulation dispatching on solver, so
each can carry its own future structural fields independently. For the decomposed
masters this one decomposes into under `BendersSolver`, see
`AggregateODRouteJointRoutingAssignmentMasterFormulation` (`master.jl`) and
`AggregateODRouteJointRoutingAssignmentBendersSubproblemFormulation`
(`benders_subproblem.jl`), both of which derive their encoding fields from an instance of
this type so the three can never disagree about the route universe or the cost weights.

# Fields
See `AggregateODRouteBaseFormulation`'s docstring for the shared subset:
`route_regularization_weight`, `walk_cost_weight`, `repositioning_time`,
`max_wait_time`, `detour_factor`, and `max_stops`.

# Pricers live on the solver, not here

Every pricer this formulation can be solved with -- `:exact`, `:station_simple`,
`:darp_modified`, `:darp`, and `:relaxed_cluster` -- plus warm-start
phasing and every relaxed-cluster setting are configured on `CGSolver.pricing`
(`CGPricingConfig`, `opt/solvers/cg/pricing_config.jl`), which is also where they are
documented. They are search algorithms, not encoding: runs differing only in pricer solve
the identical model, and three of them are required to reach the identical optimum. This
formulation's `build_model` for `CGSolver` reads that config and resolves the default
(`nothing`) to `:exact`.

**The route universe is a formulation property; which part of it a pricer searches is
not.** `:exact`, `:darp_modified` and `:darp` all search the full revisit-tolerant universe
and are exhaustive-equivalent -- switching among them isolates the search mechanism while
holding the achievable optimum fixed. `:station_simple` searches a strict *subset*
(elementary routes only), so exhausting it proves only that no *elementary* column prices
negative. A `:station_simple` run that exhausts still reports `SOLVE_OPTIMAL` -- the status
keeps its usual meaning, "no improving column remains in the universe searched" -- but the
scope of that claim is narrower, and every such result carries
`metadata["cg_optimality_scope"] == "elementary_routes_only"` (plus
`cg_pricing_universe_restricted == true` and `cg_final_pricing_mode == :station_simple`) so
the restriction travels with the number. Read that key before treating a `:station_simple`
optimum as a full-universe one.

The compatible pricers' `compensated_dominance` toggle lives on `CGPricingConfig`.

No `assignment_policy` field: this
formulation's `build_model` only ever supported free assignment in practice, so free
assignment is simply the only behavior now. No `allow_walk_only` field either -- unlike
`AggregateODRouteBaseFormulation`/`AggregateODRouteBendersYXFormulation`, direct walking
(`WALK_ONLY_PAIR`) is not optional here: it's the only station-free coverage option once
same-station pairs are gone (`compute_valid_jk_pairs` no longer produces `j==k` pairs at
all), so it must always be available for the build-time feasibility guarantee
(`joint_routing_assignment_validate_feasible_coverage`) to hold. See
`_aggregate_od_route_allow_walk_only` (`data/maps/aggregate_od_route_map.jl`) for how
`create_aggregate_od_route_map` resolves this per formulation type instead of reading a
uniform field.
"""
struct AggregateODRouteJointRoutingAssignmentFormulation <: AbstractFormulation
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int

    function AggregateODRouteJointRoutingAssignmentFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
        )
        resolved_max_stops = _validate_aggregate_od_route_formulation_fields(
            route_regularization_weight, walk_cost_weight, repositioning_time,
            max_wait_time, detour_factor, max_stops,
        )
        new(
            Float64(route_regularization_weight),
            Float64(walk_cost_weight),
            Float64(repositioning_time),
            Float64(max_wait_time),
            Float64(detour_factor),
            resolved_max_stops,
        )
    end
end
