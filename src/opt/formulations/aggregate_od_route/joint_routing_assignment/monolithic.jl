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
`pickup_deadline_s`, `ride_time_limit_ratio`, and `max_stops`.

`demand_weighting` (`:count`, the default since 2026-11-03, or `:unit`) sets the demand `Q`
of each OD group `(s,p)`: `:count` is the number of requests on that OD pair in scenario `s`,
`:unit` collapses every group to 1 (the family's only behaviour from 2026-09-15 to
2026-11-03). `Q` multiplies every walking term -- column cost, direct-walk cost, the
pricer's candidate reward and the Benders completions -- and nothing else: a route's travel
is charged once whatever it carries (capacity is unlimited), and coverage stays `>= 1`.

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
`AggregateODRouteBaseFormulation`, direct walking
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
    pickup_deadline_s::Float64
    ride_time_limit_ratio::Float64
    max_stops::Int
    demand_weighting::Symbol
    # Seats per vehicle, counted in requests (a group of Q requests takes Q seats). `nothing`
    # = unlimited, the model as it always was (pricer `:exact`). With a capacity, groups of
    # more than `vehicle_capacity` requests are dropped from the instance (no route could
    # carry them), every route must keep at most `vehicle_capacity` on board on every arc,
    # and CG prices with `:exact_capacitated`. CGSolver only.
    vehicle_capacity::Union{Nothing, Int}

    function AggregateODRouteJointRoutingAssignmentFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            pickup_deadline_s::Number,
            ride_time_limit_ratio::Union{Number, Nothing}=nothing,
            max_stops::Union{Nothing, Int}=nothing,
            demand_weighting::Symbol=:count,
            vehicle_capacity::Union{Nothing, Integer}=nothing,
            detour_factor::Union{Number, Nothing}=nothing,   # old name of ride_time_limit_ratio
        )
        ride_time_limit_ratio = something(_renamed_kwarg(ride_time_limit_ratio, detour_factor,
                                                         :ride_time_limit_ratio, :detour_factor), 1.5)
        _validate_demand_weighting(demand_weighting)
        isnothing(vehicle_capacity) || vehicle_capacity >= 1 || throw(ArgumentError(
            "vehicle_capacity must be nothing (unlimited) or >= 1, got $vehicle_capacity"))
        resolved_max_stops = _validate_aggregate_od_route_formulation_fields(
            route_regularization_weight, walk_cost_weight, repositioning_time,
            pickup_deadline_s, ride_time_limit_ratio, max_stops,
        )
        new(
            Float64(route_regularization_weight),
            Float64(walk_cost_weight),
            Float64(repositioning_time),
            Float64(pickup_deadline_s),
            Float64(ride_time_limit_ratio),
            resolved_max_stops,
            demand_weighting,
            isnothing(vehicle_capacity) ? nothing : Int(vehicle_capacity),
        )
    end
end
