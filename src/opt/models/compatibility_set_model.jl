"""
CompatibilitySetModel - restricted master problem for compatibility-set columns.

This model selects stations, assigns scenario OD demand to feasible station OD
pairs, activates those OD pairs, and covers activated OD pairs with a restricted
pool of compatibility-set columns.
"""

export CompatibilitySetModel
export CompatibilitySetAssignmentModel
export AnyCompatibilitySetModel
export CompatibilityColumn

struct CompatibilityColumn
    id::Int
    od_pairs::Vector{Tuple{Int, Int}}
    tau::Float64
    metadata::Dict{String, Any}

    function CompatibilityColumn(
            id::Int,
            od_pairs::AbstractVector{<:Tuple{Int, Int}},
            tau::Number;
            metadata::Dict{String, Any}=Dict{String, Any}()
        )
        id > 0 || throw(ArgumentError("column id must be positive"))
        isempty(od_pairs) && throw(ArgumentError("compatibility column must cover at least one OD pair"))
        tau >= 0 || throw(ArgumentError("tau must be non-negative"))
        unique_pairs = unique(Tuple{Int, Int}.(od_pairs))
        new(id, unique_pairs, Float64(tau), metadata)
    end
end

"""
    CompatibilitySetModel <: AbstractODModel

Column-generation-ready restricted master problem.

# Fields
- `l`: number of stations selected in the first stage
- `route_regularization_weight`: μ, multiplying each compatibility column cost
- `repositioning_time`: ρ, added to every compatibility column travel/service cost
- `max_walking_distance`: walking feasibility radius used to build Δ(o), Δ(d)
- `initial_columns`: optional restricted compatibility-set pool. If omitted,
  singleton columns `{(j,k)}` are created for every feasible station OD pair.
- `relax_integrality`: if true, build the LP relaxation used by column generation
"""
struct CompatibilitySetModel <: AbstractODModel
    l::Int
    route_regularization_weight::Float64
    repositioning_time::Float64
    max_walking_distance::Float64
    max_wait_time::Float64
    detour_factor::Float64
    max_stops::Int
    max_visits_per_node::Int
    max_new_columns::Int
    n_candidates::Int
    pricing_time_limit_sec::Float64
    reduced_cost_tol::Float64
    initial_columns::Union{Nothing, Vector{CompatibilityColumn}}
    relax_integrality::Bool

    function CompatibilitySetModel(
            l::Int;
            route_regularization_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_walking_distance::Number=300,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            max_visits_per_node::Int=2,
            max_new_columns::Int=20,
            n_candidates::Int=max_new_columns,
            pricing_time_limit_sec::Number=30.0,
            reduced_cost_tol::Number=1e-6,
            initial_columns::Union{Nothing, Vector{CompatibilityColumn}}=nothing,
            relax_integrality::Bool=false
        )
        l > 0 || throw(ArgumentError("l must be positive"))
        route_regularization_weight >= 0 ||
            throw(ArgumentError("route_regularization_weight must be non-negative"))
        repositioning_time >= 0 ||
            throw(ArgumentError("repositioning_time must be non-negative"))
        max_walking_distance >= 0 ||
            throw(ArgumentError("max_walking_distance must be non-negative"))
        max_wait_time >= 0 ||
            throw(ArgumentError("max_wait_time must be non-negative"))
        detour_factor >= 1.0 ||
            throw(ArgumentError("detour_factor must be at least 1.0"))
        resolved_max_stops = isnothing(max_stops) ? typemax(Int) : max_stops
        resolved_max_stops >= 2 ||
            throw(ArgumentError("max_stops must be at least 2"))
        max_visits_per_node >= 1 ||
            throw(ArgumentError("max_visits_per_node must be positive"))
        max_new_columns > 0 ||
            throw(ArgumentError("max_new_columns must be positive"))
        n_candidates >= max_new_columns ||
            throw(ArgumentError("n_candidates must be >= max_new_columns"))
        pricing_time_limit_sec > 0 ||
            throw(ArgumentError("pricing_time_limit_sec must be positive"))
        reduced_cost_tol >= 0 ||
            throw(ArgumentError("reduced_cost_tol must be non-negative"))
        new(
            l,
            Float64(route_regularization_weight),
            Float64(repositioning_time),
            Float64(max_walking_distance),
            Float64(max_wait_time),
            Float64(detour_factor),
            resolved_max_stops,
            max_visits_per_node,
            max_new_columns,
            n_candidates,
            Float64(pricing_time_limit_sec),
            Float64(reduced_cost_tol),
            initial_columns,
            relax_integrality,
        )
    end
end

"""
    CompatibilitySetAssignmentModel

Identical to CompatibilitySetModel in all fields and column-generation parameters,
but uses equality constraints (==) for coverage instead of inequalities (>=).
Duals from the LP can be negative, and the pricer filters out pairs with non-positive
duals so they are invisible to the label-setting algorithm.
"""
struct CompatibilitySetAssignmentModel <: AbstractODModel
    base::CompatibilitySetModel

    CompatibilitySetAssignmentModel(l::Int; kwargs...) =
        new(CompatibilitySetModel(l; kwargs...))
end

Base.getproperty(m::CompatibilitySetAssignmentModel, name::Symbol) =
    name === :base ? getfield(m, :base) : getproperty(getfield(m, :base), name)

const AnyCompatibilitySetModel = Union{CompatibilitySetModel, CompatibilitySetAssignmentModel}
