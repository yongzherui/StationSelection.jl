"""
RouteFleetLimitModel — fleet-bounded route model with per-passenger delay cost.

Extends RouteVehicleCapacityModel with:
  - Fleet-size constraint: Σ_r θ^r_{ts} ≤ F  ∀ t, s
  - Per-passenger delay cost in objective, penalised by μ:
      μ · d^r_{jk} · α^r_{jkts}
  - Unmet demand variable v_{jkts} ∈ ℤ₊; route-linking becomes equality:
      Σ x = v + Σ_r α
  - Unmet demand penalty: λ · Σ v_{jkts}

All other structure (y, z, x, α, θ, vehicle-capacity segment constraint) is
identical to RouteVehicleCapacityModel.
"""

export RouteFleetLimitModel


"""
    RouteFleetLimitModel <: AbstractODModel

Two-stage stochastic station selection with fleet-size constraint and
per-passenger delay cost in the objective (April 1, 2026 formulation).

# Key differences from RouteVehicleCapacityModel
- `fleet_size`: maximum routes active per time bucket per scenario (Σ_r θ^r_{ts} ≤ F).
- Per-passenger delay is penalised by `route_regularization_weight` (μ).
- `unmet_demand_penalty` (λ): penalty per unserved passenger-leg.
- Route-linking constraint is now equality with slack variable v_{jkts}.

# Fields
- `k::Int`: Stations to activate per scenario (second stage)
- `l::Int`: Stations to build (first stage)
- `fleet_size::Int`: F — max routes active per (time bucket, scenario)
- `route_regularization_weight::Float64`: μ — penalty on route activation and route-served delay
- `repositioning_time::Float64`: ρ — constant repositioning overhead (seconds)
- `unmet_demand_penalty::Float64`: λ — penalty per unmet passenger-leg
- `vehicle_capacity::Int`: Cap_r — vehicle capacity for segment constraints
- `max_route_travel_time::Union{Float64,Nothing}`: route filter upper bound
- `max_walking_distance::Float64`: prunes valid (j,k) pairs
- `max_detour_time::Float64`: max extra in-vehicle seconds vs direct trip
- `max_detour_ratio::Float64`: max ratio `in_vehicle/direct - 1`
- `time_window_sec::Int`: width of time bucket (seconds); default 3600
- `max_stations_visited::Int`: max stops per generated route
- `routes_file::Union{String,Nothing}`: load routes from CSV instead of DFS
"""
struct RouteFleetLimitModel <: AbstractODModel
    k::Int
    l::Int
    fleet_size::Int
    route_regularization_weight::Float64
    repositioning_time::Float64
    unmet_demand_penalty::Float64
    vehicle_capacity::Int
    max_route_travel_time::Union{Float64, Nothing}
    max_walking_distance::Float64
    max_detour_time::Float64
    max_detour_ratio::Float64
    time_window_sec::Int
    max_stations_visited::Int
    routes_file::Union{String, Nothing}

    function RouteFleetLimitModel(
            k::Int,
            l::Int;
            fleet_size::Int,
            route_regularization_weight::Number = 1.0,
            repositioning_time::Number = 20.0,
            unmet_demand_penalty::Number = 10000.0,
            vehicle_capacity::Int = 18,
            max_route_travel_time::Union{Number, Nothing} = nothing,
            max_walking_distance::Number = 300,
            max_detour_time::Number = 1200,
            max_detour_ratio::Number = 2.0,
            time_window_sec::Int = 3600,
            max_stations_visited::Int = typemax(Int),
            routes_file::Union{String, Nothing} = nothing
        )

        k > 0 || throw(ArgumentError("k must be positive"))
        l >= k || throw(ArgumentError("l must be >= k"))
        fleet_size > 0 || throw(ArgumentError("fleet_size must be positive"))
        route_regularization_weight >= 0 ||
            throw(ArgumentError("route_regularization_weight must be non-negative"))
        unmet_demand_penalty >= 0 ||
            throw(ArgumentError("unmet_demand_penalty must be non-negative"))
        vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
        time_window_sec > 0 || throw(ArgumentError("time_window_sec must be positive"))
        max_stations_visited >= 1 ||
            throw(ArgumentError("max_stations_visited must be >= 1"))
        !isnothing(routes_file) && !isfile(routes_file) &&
            throw(ArgumentError("routes_file not found: $routes_file"))

        mdt  = Float64(max_detour_time)
        mdr  = Float64(max_detour_ratio)
        mrtt = isnothing(max_route_travel_time) ? nothing : Float64(max_route_travel_time)

        mdt >= 0.0 || throw(ArgumentError("max_detour_time must be non-negative"))
        mdr >= 0.0 || throw(ArgumentError("max_detour_ratio must be non-negative"))
        max_walking_distance >= 0 ||
            throw(ArgumentError("max_walking_distance must be non-negative"))

        new(k, l, fleet_size,
            Float64(route_regularization_weight), Float64(repositioning_time),
            Float64(unmet_demand_penalty),
            vehicle_capacity, mrtt,
            Float64(max_walking_distance), mdt, mdr,
            time_window_sec, max_stations_visited, routes_file)
    end
end
