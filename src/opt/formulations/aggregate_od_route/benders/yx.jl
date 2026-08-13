"""
Benders master expressed over first-stage station-selection variables `y` only; the
subproblem resolves free assignment `x` and route activation `theta` jointly for fixed
`y` -- no deterministic nearest-station assignment resolver, unlike the historical
`BendersY`/`XY`/`YZ`/`YZH` decompositions (built around a
`NearestOpenAggregateODAssignmentPolicy` concept the rest of this formulation layer no
longer has; removed, see git history). Named `YX` (master carries `Y`, subproblem
carries free `X`) to avoid colliding with the differently-shaped
`AggregateODRouteBendersXYFormulation` in `xy.jl` (master carries `y` *and* `x`
together there).

A first working version of this exact shape (subproblem = `AggregateODRouteBaseFormulation`'s
own coverage/linking/objective code with `y` fixed, standard LP-duality cuts, no CG) was
built and verified exact against `DirectMIPSolver` -- see git history
("AggregateODRouteBendersYXFormulation") -- then deliberately removed: it needed the
subproblem's route pool to be exhaustively enumerated up front to be correct, which
defeats Benders' actual purpose (avoiding exactly that). The next attempt should solve
the subproblem via column generation instead, which likely means fixing `x` too (i.e.
this decomposition's real shape converges with `xy.jl`'s: solve a `RouteCoveringProblem`,
`theta`-only, given `x=1` from the master) rather than keeping `x` free here -- worth
resolving before reimplementing whether `YX` stays a distinct decomposition from `XY` or
this file's struct gets retired in favor of it.
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
