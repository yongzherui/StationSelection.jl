"""
TwoStageRouteModel — two-stage stochastic model with pre-generated vehicle routes.

Extends the OD assignment approach by lifting station-pair assignments to explicit
vehicle routes, adding a capacity constraint that links demand to route activations,
and penalising total activated route travel time.
"""

export TwoStageRouteModel

"""
    TwoStageRouteModel <: AbstractODModel

Two-stage stochastic station selection with route-based capacity constraints.

OD pairs are time-indexed within each scenario (time window `time_window_sec`).
Routes are pre-generated sequences of VBS stops; θ^r_s activates route r in scenario s.

# Fields
- `k::Int`: Stations to activate per scenario (second stage)
- `l::Int`: Stations to build (first stage)
- `route_regularization_weight::Float64`: μ — penalty per unit route travel time activated
- `vehicle_capacity::Int`: Passengers per route leg (C)
- `time_window_sec::Int`: Groups requests into discrete time windows within each scenario
- `max_route_travel_time::Union{Float64,Nothing}`: Upper bound on route travel time (filter passed to generate_routes)
- `max_intermediate_stops::Int`: 0 = direct only; 1 = allow one intermediate stop
- `use_walking_distance_limit::Bool`: Restrict valid (j,k) pairs by walking distance
- `max_walking_distance::Union{Float64,Nothing}`: Walking limit (required when limit is enabled)
- `max_wait_time::Union{Float64,Nothing}`: Enables cross-window temporal BFS when set.
  Max seconds after `t_id * time_window_sec` that the vehicle can arrive at a pickup.
  When `nothing` (default), uses the standard scenario-independent route pool.

# Formulation (standard mode, `max_wait_time = nothing`)

**Objective:**
    min Σ_s [ Σ_{(o,d,t)∈Ω_s} Σ_{(j,k)} q_{odts} (d_{oj} + d_{kd}) x_{odtjks}
            + μ Σ_r τ^r θ^r_s ]

**Capacity constraint (per j,k,t,s):**
    Σ_{(o,d)∈Ω_{s,t}} q_{odts} x_{odtjks}  ≤  Σ_{r: (j,k)∈r} C · θ^r_s

# Formulation (temporal mode, `max_wait_time` set)

Routes are generated per-scenario via cross-window BFS. θ^r_s activates route r
(scenario-specific index). The capacity constraint becomes a covering constraint:

    Σ_{r: (j,k,t)∈r} α^r_{t,jk} · θ^r_s  ≥  Σ_{(o,d)∈Ω_{s,t}} q_{odts} x_{odtjks}

where α^r_{t,jk} is the actual passengers route r carries on leg (j,k) in window t.

Standard two-stage constraints: station_limit, activation_limit, activation_linking,
assignment_coverage, assignment_to_active (tight: x ≤ z[j,s], x ≤ z[k,s]).
"""
struct TwoStageRouteModel <: AbstractODModel
    k::Int
    l::Int
    route_regularization_weight::Float64
    vehicle_capacity::Int
    time_window_sec::Int
    max_route_travel_time::Union{Float64, Nothing}
    max_intermediate_stops::Int
    use_walking_distance_limit::Bool
    max_walking_distance::Union{Float64, Nothing}
    max_detour_time::Union{Float64, Nothing}
    max_detour_ratio::Union{Float64, Nothing}
    max_wait_time::Union{Float64, Nothing}

    function TwoStageRouteModel(
            k::Int,
            l::Int;
            route_regularization_weight::Number = 1.0,
            vehicle_capacity::Int = 4,
            time_window_sec::Int = 120,
            max_route_travel_time::Union{Number, Nothing} = nothing,
            max_intermediate_stops::Int = 0,
            use_walking_distance_limit::Bool = false,
            max_walking_distance::Union{Number, Nothing} = nothing,
            max_detour_time::Union{Number, Nothing} = nothing,
            max_detour_ratio::Union{Number, Nothing} = nothing,
            max_wait_time::Union{Number, Nothing} = nothing
        )
        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        route_regularization_weight >= 0 || throw(ArgumentError("route_regularization_weight must be non-negative"))
        vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
        time_window_sec > 0 || throw(ArgumentError("time_window_sec must be positive"))
        max_intermediate_stops >= 0 || throw(ArgumentError("max_intermediate_stops must be non-negative"))

        mdt  = isnothing(max_detour_time)  ? nothing : Float64(max_detour_time)
        mdr  = isnothing(max_detour_ratio) ? nothing : Float64(max_detour_ratio)
        mrtt = isnothing(max_route_travel_time) ? nothing : Float64(max_route_travel_time)
        mwt  = isnothing(max_wait_time)    ? nothing : Float64(max_wait_time)

        if !isnothing(mwt)
            mwt >= 0.0 || throw(ArgumentError("max_wait_time must be non-negative"))
        end

        if use_walking_distance_limit
            isnothing(max_walking_distance) && throw(ArgumentError(
                "max_walking_distance must be provided when use_walking_distance_limit=true"
            ))
            max_walking_distance >= 0 || throw(ArgumentError("max_walking_distance must be non-negative"))
            new(k, l, Float64(route_regularization_weight), vehicle_capacity,
                time_window_sec, mrtt, max_intermediate_stops,
                true, Float64(max_walking_distance), mdt, mdr, mwt)
        else
            new(k, l, Float64(route_regularization_weight), vehicle_capacity,
                time_window_sec, mrtt, max_intermediate_stops,
                false, nothing, mdt, mdr, mwt)
        end
    end
end
