"""
AggregateODRouteModel - restricted master problem for aggregate OD route columns.

This model selects stations, assigns scenario OD demand to feasible station OD
pairs, activates those OD pairs, and covers activated OD pairs with a restricted
pool of aggregate OD route columns.
"""

export AggregateODRouteModel
export RouteCoveringProblem
export AnyAggregateODRouteModel
export AggregateODRouteColumn
export AbstractAggregateODAssignmentPolicy
export FreeAggregateODAssignmentPolicy
export NearestOpenAggregateODAssignmentPolicy

struct AggregateODRouteColumn
    id::Int
    od_pairs::Vector{Tuple{Int, Int}}
    tau::Float64
    metadata::Dict{String, Any}

    function AggregateODRouteColumn(
            id::Int,
            od_pairs::AbstractVector{<:Tuple{Int, Int}},
            tau::Number;
            metadata::Dict{String, Any}=Dict{String, Any}()
        )
        id > 0 || throw(ArgumentError("column id must be positive"))
        isempty(od_pairs) && throw(ArgumentError("aggregate OD route column must cover at least one OD pair"))
        tau >= 0 || throw(ArgumentError("tau must be non-negative"))
        unique_pairs = unique(Tuple{Int, Int}.(od_pairs))
        new(id, unique_pairs, Float64(tau), metadata)
    end
end

abstract type AbstractAggregateODAssignmentPolicy end

struct FreeAggregateODAssignmentPolicy <: AbstractAggregateODAssignmentPolicy end

struct NearestOpenAggregateODAssignmentPolicy <: AbstractAggregateODAssignmentPolicy
    feasibility_cut_style::Symbol

    function NearestOpenAggregateODAssignmentPolicy(
        feasibility_cut_style::Symbol=:big_m_nearest,
    )
        feasibility_cut_style in (:big_m_nearest, :endpoint_chain, :pair_chain) ||
            throw(ArgumentError("feasibility_cut_style must be :big_m_nearest, :endpoint_chain, or :pair_chain"))
        new(feasibility_cut_style)
    end
end

"""
    AggregateODRouteModel <: AbstractODModel

Column-generation-ready restricted master problem.

# Fields
- `l`: number of stations selected in the first stage
- `route_regularization_weight`: μ, multiplying each aggregate OD route column cost
- `walk_cost_weight`: multiplies the walking-cost term (`od_pair_walking_cost`) everywhere
  it enters an objective or a Benders subproblem/dual-completion LP for this model, so it
  scales consistently with the true objective and doesn't corrupt reduced costs/cuts.
  Defaults to 1.0 (walking cost enters at its raw data-derived value, matching prior
  behavior exactly).
- `repositioning_time`: ρ, added to every aggregate OD route column travel/service cost
- `max_walking_distance`: walking feasibility radius used to build Δ(o), Δ(d)
- `initial_columns`: optional restricted aggregate OD route pool. If omitted,
  singleton columns `{(j,k)}` are created for every feasible station OD pair.
- `relax_integrality`: if true, build the LP relaxation used by column generation
- `assignment_policy`: controls aggregate OD assignment semantics.
- `allow_walk_only`: if true, an OD pair may be assigned a station-free "walk
  directly" option (no vehicle route) whenever the direct walk is within
  `2 * max_walking_distance`. Off by default for backward compatibility.
  Supported with the default free-assignment policy and with the independent
  endpoint nearest-open policies (`:big_m_nearest` and `:endpoint_chain`),
  including the NearestOpen Benders paths. `NearestOpenAggregateODAssignmentPolicy(:pair_chain)`
  still rejects walk-only assignments because it ranks station pairs jointly
  and has no station-free endpoint-collision representation.
- `unmet_demand_penalty`: if set (non-`nothing`), turns on "always feasible" mode --
  same-station assignment (`x_{j,j}=1`, needs no vehicle route, see
  [`is_same_station_pair`](@ref)) becomes a valid resolution even with
  `allow_walk_only=false`, and every OD's assignment constraint relaxes from
  `sum(x) == 1` to `sum(x) == u` with a binary service indicator `u`, penalized
  `unmet_demand_penalty` per unit left unserved (`1-u`) in the objective. `nothing`
  (default) preserves existing behavior exactly -- the assignment/coverage
  constraints and the shared endpoint-selector's `sum(z)==1` are unconditional
  hard constraints, so a station budget `l` too small for some request's
  candidates makes the model outright infeasible, same as before this field
  existed.
"""
struct AggregateODRouteModel <: AbstractODModel
    l::Int
    route_regularization_weight::Float64
    walk_cost_weight::Float64
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
    initial_columns::Union{Nothing, Vector{AggregateODRouteColumn}}
    relax_integrality::Bool
    assignment_policy::AbstractAggregateODAssignmentPolicy
    allow_walk_only::Bool
    unmet_demand_penalty::Union{Nothing, Float64}

    function AggregateODRouteModel(
            l::Int;
            route_regularization_weight::Number=1.0,
            walk_cost_weight::Number=1.0,
            repositioning_time::Number=20.0,
            max_walking_distance::Number=300,
            max_wait_time::Number=Inf,
            detour_factor::Number=1.5,
            max_stops::Union{Nothing, Int}=nothing,
            max_visits_per_node::Int=typemax(Int),
            max_new_columns::Int=20,
            n_candidates::Int=max_new_columns,
            pricing_time_limit_sec::Number=30.0,
            reduced_cost_tol::Number=1e-6,
            initial_columns::Union{Nothing, Vector{AggregateODRouteColumn}}=nothing,
            relax_integrality::Bool=false,
            assignment_policy::AbstractAggregateODAssignmentPolicy=FreeAggregateODAssignmentPolicy(),
            allow_walk_only::Bool=false,
            unmet_demand_penalty::Union{Nothing, Number}=nothing,
        )
        l > 0 || throw(ArgumentError("l must be positive"))
        route_regularization_weight >= 0 ||
            throw(ArgumentError("route_regularization_weight must be non-negative"))
        walk_cost_weight >= 0 ||
            throw(ArgumentError("walk_cost_weight must be non-negative"))
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
        isnothing(unmet_demand_penalty) || unmet_demand_penalty >= 0 ||
            throw(ArgumentError("unmet_demand_penalty must be non-negative"))
        new(
            l,
            Float64(route_regularization_weight),
            Float64(walk_cost_weight),
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
            assignment_policy,
            allow_walk_only,
            isnothing(unmet_demand_penalty) ? nothing : Float64(unmet_demand_penalty),
        )
    end
end

"""
    RouteCoveringProblem

Fixed-station, fixed-assignment aggregate OD route-covering problem. The
assignment map keys are `(scenario, origin_station, destination_station)` and
values are the assigned `(pickup_station, dropoff_station)` pair.
"""
struct RouteCoveringProblem <: AbstractODModel
    base::AggregateODRouteModel
    open_stations::Vector{Int}
    fixed_assignments::Dict{NTuple{3, Int}, Tuple{Int, Int}}

    function RouteCoveringProblem(
            l::Int,
            open_stations::AbstractVector{<:Integer},
            fixed_assignments::AbstractDict{<:Tuple{Int, Int, Int}, <:Tuple{Int, Int}};
            kwargs...
        )
        unique_open = sort!(unique(Int.(open_stations)))
        length(unique_open) == l ||
            throw(ArgumentError("open_stations must contain exactly l unique stations"))
        assignments = Dict{NTuple{3, Int}, Tuple{Int, Int}}()
        for (key, pair) in fixed_assignments
            assignments[(Int(key[1]), Int(key[2]), Int(key[3]))] = (Int(pair[1]), Int(pair[2]))
        end
        new(AggregateODRouteModel(l; kwargs...), unique_open, assignments)
    end
end

Base.getproperty(m::RouteCoveringProblem, name::Symbol) =
    name in (:base, :open_stations, :fixed_assignments) ?
        getfield(m, name) :
        getproperty(getfield(m, :base), name)

const AnyAggregateODRouteModel = Union{
    AggregateODRouteModel,
    RouteCoveringProblem,
}
