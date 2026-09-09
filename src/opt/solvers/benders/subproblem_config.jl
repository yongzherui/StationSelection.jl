"""
`BendersSubproblemConfig` -- how `BendersSolver` obtains and solves the second stage.

Parallel to `CGPricingConfig` (`opt/solvers/cg/pricing_config.jl`) and for the same
reason: "where do the route columns come from" is a *search algorithm*, not part of the
model, so two runs differing only in it solve the identical model. Keeping it on the
solver rather than the formulation is what lets an oracle sweep vary one solver and reuse
one formulation.
"""

export BendersSubproblemConfig

"""
    BendersSubproblemConfig(; oracle=:direct_enumeration, max_stops=4,
                              max_routes=200_000, enumeration_time_limit_sec=180.0,
                              time_limit_sec=nothing)

# `oracle`

Where the subproblem's route columns come from. Only `:direct_enumeration` today: the
whole column universe is enumerated once at build time
(`enumerate_joint_routing_assignment_columns`) and every subproblem LP is solved against
the complete pool.

That is deliberately the *wrong* long-run answer -- enumerating the universe up front is
the cost Benders exists to avoid, and the same objection retired the pre-split
`AggregateODRouteBendersYXFormulation` (see `formulations/aggregate_od_route/benders/yx.jl`).
It is the right *first* answer: with the pool complete and fixed, the subproblem is an
honest LP whose duals are exact, so the loop, the cut algebra and the hook wiring can be
verified against a reference optimum before a column-generation oracle
(`:column_generation`, the intended second value) introduces the question of whether an
inexact subproblem's duals still give valid cuts.

# `max_stops` -- the enumeration cap, and a real restriction

`enumerate_joint_routing_assignment_columns` is exponential in `max_stops` on two axes at
once (the physical-route DFS, then the per-route cartesian product over multi-certified
passengers), and throws rather than truncating when it blows past `max_routes`. `4` is
the largest value measured tractable on the reference instances (16,320 columns at a
10-station / 8-pair instance), so it is the default here rather than the formulation's own
value.

**When it is below the formulation's own `max_stops`, the run's optimality claim narrows
with it** -- the answer is optimal over routes of at most this many stops, and a longer
route can beat it. The build records that as
`metadata["benders_optimality_scope"] = "max_stops_restricted"` (versus
`"full_route_universe"` when no narrowing happened), alongside
`benders_subproblem_max_stops` and `benders_formulation_max_stops`, in exactly the spirit
of `CGSolver`'s `cg_optimality_scope`. Read it before pooling a `BendersSolver` objective
with a `DirectMIPSolver` one. `nothing` disables the cap and enumerates the formulation's
own universe.

# The rest

`max_routes`/`enumeration_time_limit_sec` are the enumerator's own guard rails, passed
straight through. `time_limit_sec` bounds each individual subproblem LP solve
(`nothing` = no limit); a subproblem that hits it returns a non-optimal status, which
`solve_subproblem` treats as a hard error rather than a weak cut, since a truncated LP's
duals are not a valid underestimator.
"""
struct BendersSubproblemConfig
    oracle::Symbol
    max_stops::Union{Nothing, Int}
    max_routes::Int
    enumeration_time_limit_sec::Float64
    time_limit_sec::Union{Nothing, Float64}

    function BendersSubproblemConfig(;
            oracle::Symbol=:direct_enumeration,
            max_stops::Union{Nothing, Int}=4,
            max_routes::Int=200_000,
            enumeration_time_limit_sec::Number=180.0,
            time_limit_sec::Union{Nothing, Number}=nothing,
        )
        oracle === :direct_enumeration || throw(ArgumentError(
            "unsupported Benders subproblem oracle $(repr(oracle)); only " *
            ":direct_enumeration is implemented (:column_generation is the intended " *
            "next one -- see this type's docstring)",
        ))
        isnothing(max_stops) || max_stops >= 2 ||
            throw(ArgumentError("max_stops must be at least 2"))
        max_routes > 0 || throw(ArgumentError("max_routes must be positive"))
        enumeration_time_limit_sec > 0 ||
            throw(ArgumentError("enumeration_time_limit_sec must be positive"))
        isnothing(time_limit_sec) || time_limit_sec > 0 ||
            throw(ArgumentError("time_limit_sec must be positive"))
        new(
            oracle, max_stops, max_routes, Float64(enumeration_time_limit_sec),
            isnothing(time_limit_sec) ? nothing : Float64(time_limit_sec),
        )
    end
end
