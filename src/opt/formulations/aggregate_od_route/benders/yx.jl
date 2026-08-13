"""
Benders master expressed over first-stage station-selection variables `y` only; the
subproblem resolves free assignment `x` and route activation `theta` jointly for fixed
`y`, exactly as `AggregateODRouteBaseFormulation` already does -- no deterministic
nearest-station assignment resolver, unlike the historical `BendersY`/`XY`/`YZ`/`YZH`
decompositions archived under `optimize/aggregate_od_route/benders/archive/` (which
were built around a `NearestOpenAggregateODAssignmentPolicy` concept the rest of this
formulation layer no longer has). Named `YX` (master carries `Y`, subproblem carries free
`X`) to avoid colliding with the historical, differently-shaped `AggregateODRouteBendersXYFormulation`
in `xy.jl` (master carries `y` *and* `x` together there).

Cut derivation is the standard LP-duality subgradient cut only -- no zero-completion or
Magnanti-Wong variant (those were also nearest-open-specific in the archived design).
"""

export AggregateODRouteBendersYXFormulation

"""
    AggregateODRouteBendersYXFormulation <: AbstractFormulation

# Fields
Shares `route_regularization_weight`, `walk_cost_weight`, `repositioning_time`,
`max_wait_time`, `detour_factor`, `max_stops`, `allow_walk_only` with
`AggregateODRouteBaseFormulation` (see its docstring) -- the subproblem reuses that
formulation's own coverage/linking/objective code verbatim, just with `y` fixed instead
of free. `cut_mode` controls how many `theta` cut-placeholder variables the master
carries (see `AbstractBendersCutMode`).
"""
struct AggregateODRouteBendersYXFormulation <: AbstractFormulation
    route_regularization_weight::Float64
    walk_cost_weight::Float64
    repositioning_time::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    allow_walk_only::Bool
    cut_mode::AbstractBendersCutMode

    function AggregateODRouteBendersYXFormulation(;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            allow_walk_only::Bool=false,
            cut_mode::AbstractBendersCutMode=MultiCut(),
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
            cut_mode,
        )
    end
end
