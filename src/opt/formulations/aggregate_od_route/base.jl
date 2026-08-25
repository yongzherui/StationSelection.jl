"""
The bare structural encoding of `AggregateODRouteBaseFormulation`'s MILP:
station-selection `y`, decoupled assignment `x[s,p,j,k]`, and route columns `θ` linked to
`x` via `add_aggregate_od_route_base_route_linking_constraints!` -- unlike the sibling
`AggregateODRouteJointRoutingAssignmentFormulation`, where route columns carry OD
assignment directly and there is no separate `x` (see that formulation's own docstring
for what "joint" means there). The two share the same encoding-detail field set
(`route_regularization_weight`, `walk_cost_weight`, etc. -- see
`_validate_aggregate_od_route_formulation_fields`) and are separate marker types only
because they pair with different solvers -- this one with `DirectMIPSolver`, against an
exhaustively enumerated column pool built once at `build_model` time (`y`/`x`/`θ`
integral, no iterative pricing loop), the other with `CGSolver`.
"""

export AggregateODRouteBaseFormulation

"""
    _validate_aggregate_od_route_formulation_fields(
        route_regularization_weight, walk_cost_weight, repositioning_time,
        max_wait_time, detour_factor, max_stops,
    ) -> resolved_max_stops::Int

Shared cross-field validation for `AggregateODRouteBaseFormulation`'s and
`AggregateODRouteJointRoutingAssignmentFormulation`'s constructors -- both carry the
exact same encoding-detail field set (see this file's own module docstring for why
they're still separate types), so this keeps their validation and defaults from
drifting apart.
"""
function _validate_aggregate_od_route_formulation_fields(
        route_regularization_weight::Number,
        walk_cost_weight::Number,
        repositioning_time::Number,
        max_wait_time::Number,
        detour_factor::Number,
        max_stops::Union{Nothing, Int},
    )::Int
    route_regularization_weight >= 0 ||
        throw(ArgumentError("route_regularization_weight must be non-negative"))
    walk_cost_weight >= 0 ||
        throw(ArgumentError("walk_cost_weight must be non-negative"))
    repositioning_time >= 0 ||
        throw(ArgumentError("repositioning_time must be non-negative"))
    max_wait_time >= 0 ||
        throw(ArgumentError("max_wait_time must be non-negative"))
    detour_factor >= 1.0 ||
        throw(ArgumentError("detour_factor must be at least 1.0"))
    resolved_max_stops = isnothing(max_stops) ? typemax(Int) : max_stops
    resolved_max_stops >= 2 || throw(ArgumentError("max_stops must be at least 2"))
    return resolved_max_stops
end

"""
    AggregateODRouteBaseFormulation <: AbstractFormulation

Encoding-detail knobs for the compact `y`/`x`/`θ` MILP's variable structure -- *how* a
`StationSelectionProblem` is served, weighted, and staged when solved directly
(`DirectMIPSolver`) against an exhaustively enumerated column pool. Pairs with
`StationSelectionProblem`, which carries only `data`,
`l`, and `max_walking_distance` -- everything else that used to live on
`AggregateODRouteProblem` (still used by the not-yet-migrated Benders/`RouteCoveringProblem`
paths) belongs here instead, since it's an encoding choice, not a business decision.

# Fields
- `route_regularization_weight`: μ, multiplying each aggregate OD route column cost
- `walk_cost_weight`: multiplies the walking-cost term everywhere it enters the objective
- `repositioning_time`: ρ, added to every aggregate OD route column travel/service cost
- `max_wait_time`: maximum passenger wait time
- `detour_factor`: maximum allowed in-vehicle detour ratio
- `max_stops`: maximum stops per route
- `compensated_dominance`: whether the CG pricer's label-setting dominance test uses the
  compensated reward-diff rule (`rc_a + w(A_a \\ A_b) <= rc_b`) or the older, weaker plain
  subset rule (`A_a subseteq A_b`); a toggle because compensated trades away column
  diversity per search for speed, and which side wins for column generation overall is an
  end-to-end question, not a pricing-speed one (see `RouteCoveringSearchContext`,
  `label_setting/route_covering/exact/exact.jl`). Only affects `CGSolver`'s pricing loop --
  `DirectMIPSolver`'s exhaustive enumeration (`enumerate_aggregate_od_route_columns`) never
  performs dominance at all, so this field is inert there.

No `assignment_policy` field: this formulation's `build_model` only ever supported free
assignment in practice, so free assignment is simply the only behavior now. No
`allow_walk_only` field either -- direct walking (`WALK_ONLY_PAIR`, surfaced here as
`x_walk[(s,p)]`, `add_walk_variables!`) is mandatory, not
configurable, mirroring `AggregateODRouteJointRoutingAssignmentFormulation` (see its own
docstring for why). See `_aggregate_od_route_allow_walk_only`
(`data/maps/aggregate_od_route_map.jl`) for how `create_aggregate_od_route_map` resolves
this per formulation type instead of reading a uniform field.
"""
struct AggregateODRouteBaseFormulation <: AbstractFormulation
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    compensated_dominance::Bool

    function AggregateODRouteBaseFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            compensated_dominance::Bool=true,
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
            compensated_dominance,
        )
    end
end
