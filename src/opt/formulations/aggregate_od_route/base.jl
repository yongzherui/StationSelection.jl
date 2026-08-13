"""
The bare structural encoding of the aggregate-OD-route problem's compact joint
routing+assignment MILP: station-selection `y`, unserved-demand slack `v`,
same-station assignment `x_same`, and route columns `θ` whose own coefficients
directly carry OD assignment (no separate assignment variable `x` -- see
`AggregateODRouteJointRoutingAssignmentFormulation`'s own docstring for why that's
what makes this "joint"). Exactly the same structural shape as that sibling
formulation; the two are separate marker types only because they pair with
different solvers -- this one with `DirectMIPSolver`, against an exhaustively
enumerated column pool built once at `build_model` time (`y`/`θ` integral, no
iterative pricing loop), the other with `CGSolver`.
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

Encoding-detail knobs for the compact joint routing+assignment MILP's variable
structure (`y`, `v`, `x_same`, `θ`) -- *how* a `StationSelectionProblem` is served,
weighted, and staged when solved directly (`DirectMIPSolver`) against an exhaustively
enumerated column pool. Pairs with `StationSelectionProblem`, which carries only `data`,
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
- `allow_walk_only`: if true, an OD pair may be assigned a station-free "walk directly"
  option whenever the direct walk is within `2 * problem.max_walking_distance`.

No `assignment_policy` field: this formulation's `build_model` only ever supported free
assignment in practice, so free assignment is simply the only behavior now.
"""
struct AggregateODRouteBaseFormulation <: AbstractFormulation
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    allow_walk_only::Bool

    function AggregateODRouteBaseFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            allow_walk_only::Bool=false,
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
            allow_walk_only,
        )
    end
end
