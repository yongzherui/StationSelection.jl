"""
Formulation-level encoding for the aggregate-OD-route problem's compact joint
routing+assignment MILP/LP -- the non-Benders-decomposed representation, solved via
column generation (`CGSolver`). See `base.jl` in this directory for the sibling
formulation solved directly against an enumerated column pool (`DirectMIPSolver`), and
`benders/` for the Benders-decomposed masters. Model construction lives per
(problem family × solver algorithm) under `opt/optimize/` instead -- see
`opt/optimize/aggregate_od_route/column_generation/build_joint_routing_assignment.jl`.
"""

export AggregateODRouteJointRoutingAssignmentFormulation
export AnyAggregateODRouteFormulation

"""
    AggregateODRouteJointRoutingAssignmentFormulation <: AbstractFormulation

The compact joint routing+assignment MILP/LP, solved via column generation
(`CGSolver`) -- *how* a `StationSelectionProblem` is served, weighted, and staged when
**not** Benders-decomposed. `AggregateODRouteBaseFormulation` (`base.jl`) shares this
exact same field set and structural shape but is solved directly against an
exhaustively enumerated column pool (`DirectMIPSolver`) rather than iteratively priced
-- the two are separate marker types, not one formulation dispatching on solver, so
each can carry its own future structural fields independently. For the decomposed
masters, see `AggregateODRouteBendersYFormulation`/`XY`/`YZ`/`YZH` in `benders/`.

# Fields
See `AggregateODRouteBaseFormulation`'s docstring for the shared subset:
`route_regularization_weight`, `walk_cost_weight`, `repositioning_time`,
`max_wait_time`, `detour_factor`, `max_stops`, `compensated_dominance` (same
toggle, same default, applying here to `JointRoutingAssignmentSearchContext`'s
dominance test instead -- `label_setting/joint_routing_assignment/exact/dominate.jl`).

`pricing_mode::Symbol` (`:exact`, `:darp_modified`, or `:darp`, default
`:exact`) picks which label-setting pricer `_pricing_build_scenario_context`
(`label_setting/joint_routing_assignment/pricing_round.jl`) builds -- three
pricers that are all, run to exhaustion, required to reach the *same*
optimum, differing only in search mechanism and cost:
- `:exact` -- `JointRoutingAssignmentSearchContext` (`exact/context.jl`), which
  credits each passenger their single *best* certified `(j,k)` regardless of
  visitation order, via a reward-layer running-max trick.
- `:darp_modified` -- `JointRoutingAssignmentDarpModifiedSearchContext`
  (`darp_modified/context.jl`), which computes the same optimum by
  branching the search on whether to commit each passenger's candidate as
  it's reached or leave it open for a possibly-better one later, tracking
  only *which passengers* are committed (compensated dominance, an
  upper-bound weight per passenger).
- `:darp` -- `JointRoutingAssignmentDarpSearchContext` (`darp/context.jl`), a
  literal onboard-bitset DARP-style pricer: boarding commits to a specific
  `(j,k)` pair (not deferred like `:darp_modified`'s), tracked with plain
  (uncompensated) dominance over the full triple identity plus an explicit
  onboard/liability resource, and a ride-limit violation is hard
  infeasibility (the whole label is discarded) rather than a soft miss.
  Closest to textbook DARP label-setting, and the most expensive of the
  three.
See `darp_modified/types.jl`'s and `darp/types.jl`'s module docstrings for
the exhaustive-equivalence invariant each is required to satisfy and the
mechanism that makes it hold. `compensated_dominance` (below) applies to
`:exact` and `:darp_modified` only -- `:darp` has no such field, since a
compensated version would be unsound for its triple-indexed served set (see
`darp/types.jl`). Switching `pricing_mode` alone isolates the
search-mechanism difference, holding the achievable optimum fixed.
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
    compensated_dominance::Bool
    pricing_mode::Symbol

    function AggregateODRouteJointRoutingAssignmentFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            compensated_dominance::Bool=true,
            pricing_mode::Symbol=:exact,
        )
        resolved_max_stops = _validate_aggregate_od_route_formulation_fields(
            route_regularization_weight, walk_cost_weight, repositioning_time,
            max_wait_time, detour_factor, max_stops,
        )
        pricing_mode in (:exact, :darp_modified, :darp) || throw(ArgumentError(
            "pricing_mode must be :exact, :darp_modified, or :darp, got $(repr(pricing_mode))",
        ))
        new(
            Float64(route_regularization_weight),
            Float64(walk_cost_weight),
            Float64(repositioning_time),
            Float64(max_wait_time),
            Float64(detour_factor),
            resolved_max_stops,
            compensated_dominance,
            pricing_mode,
        )
    end
end

"""
    AnyAggregateODRouteFormulation

Every `StationSelectionProblem`-paired aggregate-OD-route formulation that carries the
identical encoding-detail field set (see `AggregateODRouteBaseFormulation`'s docstring),
so shared-engine functions (`create_aggregate_od_route_map`,
`enumerate_aggregate_od_route_columns`) dispatch on this rather than repeating themselves
per formulation. Note `AggregateODRouteJointRoutingAssignmentFormulation` itself does NOT
carry an `allow_walk_only` field (see its own docstring) despite matching this Union's
field set otherwise -- `create_aggregate_od_route_map` resolves that one field via
`_aggregate_od_route_allow_walk_only` instead of direct field access. Mirrors
`AnyAggregateODRouteProblem` (`opt/problems/route_covering.jl`) for the same reason.
`AggregateODRouteBendersYXFormulation` (`benders/yx.jl`) shares this field set too --
its subproblem reuses `AggregateODRouteBaseFormulation`'s own map/enumeration code
verbatim.
"""
const AnyAggregateODRouteFormulation = Union{
    AggregateODRouteBaseFormulation,
    AggregateODRouteJointRoutingAssignmentFormulation,
    AggregateODRouteBendersYXFormulation,
}
