"""
Formulation-level (solver-independent) counterpart to the `BendersY` decomposition marker
in `optimize/aggregate_od_route/solver_types.jl` -- captures *what the Benders master looks
like* (which variables it carries, how its optimality cuts are derived, whether walking
cost is lifted into it, whether it's guided by an exact-enumeration term). That is
structural, not a solver run-time knob, so it belongs on an `AbstractFormulation`, not
`BendersSolver`.

`BendersSolver` still carries its own copies of the overlapping fields (`cut_mode`,
`cut_derivation`, `lifted_walking_objective`, `route_regularization_weight_schedule`,
`direct_enumeration_guide` and friends) for now, and every existing Benders
build_model/run_opt call site still reads those solver fields, not this formulation type --
migrating them over is a follow-up, not done here.
"""

export AggregateODRouteBendersYFormulation

"""
    AggregateODRouteBendersYFormulation <: AbstractFormulation

Benders master expressed over first-stage station-selection variables `y` only; the
subproblem resolves nearest-open assignment and routing for fixed `y`. Structural
counterpart to the `BendersY` decomposition marker -- see `BendersY`'s and `BendersSolver`'s
docstrings in `solver_types.jl` for the semantics of each field.
"""
struct AggregateODRouteBendersYFormulation <: AbstractFormulation
    cut_mode::AbstractBendersCutMode
    cut_derivation::Symbol
    lifted_walking_objective::Bool
    route_regularization_weight_schedule::Union{Nothing, Vector{Float64}}
    direct_enumeration_guide::Bool
    direct_enumeration_max_routes::Int
    direct_enumeration_time_limit_sec::Float64
    direct_enumeration_max_stops::Union{Nothing, Int}
    direct_enumeration_relax_integrality::Bool

    function AggregateODRouteBendersYFormulation(;
            cut_mode::AbstractBendersCutMode=MultiCut(),
            cut_derivation::Symbol=:zero_completion,
            lifted_walking_objective::Bool=true,
            route_regularization_weight_schedule::Union{AbstractVector{<:Number}, Nothing}=nothing,
            direct_enumeration_guide::Bool=false,
            direct_enumeration_max_routes::Int=10_000,
            direct_enumeration_time_limit_sec::Number=30.0,
            direct_enumeration_max_stops::Union{Int, Nothing}=nothing,
            direct_enumeration_relax_integrality::Bool=false,
        )
        cut_derivation in (:standard, :zero_completion, :restricted_mw_fixed_pi) || throw(ArgumentError(
            "cut_derivation must be :standard, :zero_completion, or :restricted_mw_fixed_pi"
        ))
        direct_enumeration_guide && !lifted_walking_objective && throw(ArgumentError(
            "direct_enumeration_guide requires lifted_walking_objective=true"
        ))
        direct_enumeration_relax_integrality && !direct_enumeration_guide && throw(ArgumentError(
            "direct_enumeration_relax_integrality requires direct_enumeration_guide=true"
        ))
        direct_enumeration_max_routes > 0 ||
            throw(ArgumentError("direct_enumeration_max_routes must be positive"))
        direct_enumeration_time_limit_sec > 0 ||
            throw(ArgumentError("direct_enumeration_time_limit_sec must be positive"))
        isnothing(direct_enumeration_max_stops) || direct_enumeration_max_stops > 0 ||
            throw(ArgumentError("direct_enumeration_max_stops must be positive"))
        resolved_schedule = isnothing(route_regularization_weight_schedule) ?
            nothing : Float64.(route_regularization_weight_schedule)
        if !isnothing(resolved_schedule)
            lifted_walking_objective || throw(ArgumentError(
                "route_regularization_weight_schedule requires lifted_walking_objective=true"
            ))
            !isempty(resolved_schedule) ||
                throw(ArgumentError("route_regularization_weight_schedule must not be empty"))
            all(resolved_schedule .> 0) ||
                throw(ArgumentError("route_regularization_weight_schedule entries must all be positive"))
            all(resolved_schedule[i] < resolved_schedule[i + 1] for i in 1:(length(resolved_schedule) - 1)) ||
                throw(ArgumentError("route_regularization_weight_schedule must be strictly increasing"))
        end
        new(
            cut_mode, cut_derivation, lifted_walking_objective, resolved_schedule,
            direct_enumeration_guide, direct_enumeration_max_routes,
            Float64(direct_enumeration_time_limit_sec), direct_enumeration_max_stops,
            direct_enumeration_relax_integrality,
        )
    end
end
