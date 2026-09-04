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

`pricing_mode::Symbol` (`:exact`, `:station_simple`, `:darp_modified`, or `:darp`,
default `:exact`) picks which label-setting pricer `_pricing_build_scenario_context`
(`label_setting/joint_routing_assignment/pricing_round.jl`) builds.

**Three of the four are exhaustive-equivalent; `:station_simple` is not.** `:exact`,
`:darp_modified` and `:darp` all search the full revisit-tolerant route universe and are
required to reach the *same* optimum when run to exhaustion, differing only in search
mechanism and cost -- switching among them isolates the mechanism while holding the
achievable optimum fixed. `:station_simple` searches a strict *subset* of that universe
(elementary routes only), so exhausting it proves only that no *elementary* column prices
negative. A `:station_simple` run that exhausts still reports `SOLVE_OPTIMAL` -- the status
keeps its usual meaning, "no improving column remains in the universe searched" -- but the
scope of that claim is narrower, and every such result carries
`metadata["cg_optimality_scope"] == "elementary_routes_only"` (plus
`cg_pricing_universe_restricted == true` and `cg_final_pricing_mode == :station_simple`) so
the restriction travels with the number. Read that key before treating a
`:station_simple` optimum as a full-universe one. To certify against the full universe
while still getting the elementary pricer's speed, use `CGSolver`'s
`warm_start_pricing_mode`, which harvests columns cheaply in the elementary universe and
then hands off to the formulation's own pricer, whose exhaustion is what certifies.
- `:exact` -- `JointRoutingAssignmentSearchContext` (`exact/context.jl`), which
  credits each passenger their single *best* certified `(j,k)` regardless of
  visitation order, via a reward-layer running-max trick.
- `:station_simple` -- `JointRoutingAssignmentStationSimpleSearchContext`
  (`station_simple/context.jl`), the elementary-route restriction: consumes the
  identical `pricing_data` as `:exact` and differs only in label type and dominance
  rule (`U_a ⊆ U_b` over visited stations), so a route may not revisit a station.
  Cheaper than `:exact` because the elementarity resource strengthens dominance, but
  it forfeits any column whose value depends on a revisit -- e.g. serving `o→d` and
  `d→o` on one vehicle needs `o→d→o`. See
  `test_joint_routing_assignment_station_simple_pricing.jl`'s "restriction is visible
  where a revisit strictly helps" for the minimal case.
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
`darp/types.jl`), and to `:station_simple`, which reuses `:exact`'s `pricing_data`
verbatim.
`relaxed_cluster_count::Union{Nothing, Int}` (default `nothing`) sizes the station
partition the *relaxed-cluster certification pricer* runs on
(`label_setting/joint_routing_assignment/relaxed_cluster/`). It is a formulation field, not
a solver one, because the partition is computed **once at build time** and stashed on the
model (`m[:joint_routing_assignment_station_clustering]`): the cells must be identical
across every CG iteration of a run for the cluster count to be a meaningful swept
parameter. Setting it costs one k-medoids pass at build time and nothing else -- the
relaxation only ever runs when `CGSolver.certification_pricing_mode = :relaxed_cluster`
asks for it. Note `:relaxed_cluster` is deliberately **not** a `pricing_mode` value: that
pricer searches a relaxed cluster graph, so its routes are not real routes and cannot
become columns; it only answers "can an improving column still exist", and a
`no` from it is a full-route-universe optimality certificate. See
`relaxed_cluster/types.jl` for the bound's proof.

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
    relaxed_cluster_count::Union{Nothing, Int}
    relaxed_cluster_guide_routes::Int
    relaxed_cluster_guide_time_limit_sec::Float64

    function AggregateODRouteJointRoutingAssignmentFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            compensated_dominance::Bool=true,
            pricing_mode::Symbol=:exact,
            relaxed_cluster_count::Union{Nothing, Int}=nothing,
            relaxed_cluster_guide_routes::Int=5,
            relaxed_cluster_guide_time_limit_sec::Number=10.0,
        )
        resolved_max_stops = _validate_aggregate_od_route_formulation_fields(
            route_regularization_weight, walk_cost_weight, repositioning_time,
            max_wait_time, detour_factor, max_stops,
        )
        pricing_mode in (:exact, :station_simple, :darp_modified, :darp, :relaxed_cluster_guided) ||
            throw(ArgumentError(
                "pricing_mode must be :exact, :station_simple, :darp_modified, :darp, or " *
                ":relaxed_cluster_guided, got $(repr(pricing_mode)). Note :relaxed_cluster " *
                "(the pure relaxation) is NOT a pricing_mode: it produces no columns, and is " *
                "enabled via CGSolver's certification_pricing_mode instead",
            ))
        isnothing(relaxed_cluster_count) || relaxed_cluster_count >= 1 || throw(ArgumentError(
            "relaxed_cluster_count must be >= 1 (or nothing), got $(relaxed_cluster_count)",
        ))
        pricing_mode !== :relaxed_cluster_guided || !isnothing(relaxed_cluster_count) ||
            throw(ArgumentError(
                "pricing_mode=:relaxed_cluster_guided needs a station partition to guide it -- " *
                "set relaxed_cluster_count = K",
            ))
        relaxed_cluster_guide_routes >= 1 || throw(ArgumentError(
            "relaxed_cluster_guide_routes must be >= 1, got $(relaxed_cluster_guide_routes)",
        ))
        relaxed_cluster_guide_time_limit_sec > 0 || throw(ArgumentError(
            "relaxed_cluster_guide_time_limit_sec must be positive, got " *
            "$(relaxed_cluster_guide_time_limit_sec)",
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
            relaxed_cluster_count,
            relaxed_cluster_guide_routes,
            Float64(relaxed_cluster_guide_time_limit_sec),
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
