"""
AlphaRouteModel — route-based station selection with α as a fixed parameter.

Routes are loaded from CSV. Alpha capacity values per (route, pickup, dropoff) leg are
also loaded from CSV and treated as fixed parameters in the covering constraint.

The single covering constraint:

    Σ_{od: (j,k) valid} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

where α^r_{jk} is the fixed Float64 parameter from `alpha_profile_file`.

There is no constraint (iii) (no per-segment vehicle capacity constraint).
Everything else (y, z, x variables; station/activation/assignment constraints;
objective structure) is the same as RouteVehicleCapacityModel.
"""

export AlphaRouteModel


"""
    AlphaRouteModel <: AbstractODModel

Two-stage stochastic station selection where route alpha values are fixed parameters
(not decision variables). Routes and their alpha profiles are supplied via CSV files.

Covering constraint:
    Σ_{od using (j,k)} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

No constraint (iii).  θ^r_{ts} ∈ Z+ remains a decision variable.

# Fields
- `k::Int`: Stations to activate per scenario
- `l::Int`: Stations to build
- `route_regularization_weight::Float64`: μ — penalty per unit route travel time deployed
- `repositioning_time::Float64`: ρ — repositioning overhead (seconds) added per deployment
- `routes_file::String`: Path to `routes_input.csv` (route_id, station_ids, travel_time)
- `alpha_profile_file::String`: Path to `alpha_profile.csv` (route_id, pickup_id, dropoff_id, value)
- `max_walking_distance::Float64`: Walking distance limit for (j,k) pair pruning
- `max_detour_time::Float64`: Max extra in-vehicle seconds vs direct trip (for detour_feasible_legs)
- `max_detour_ratio::Float64`: Max ratio `in_vehicle/direct - 1`
- `time_window_sec::Int`: Width of time bucket t (seconds)
- `use_lazy_constraints::Bool`: If true, the capacity constraint is submitted lazily
"""
struct AlphaRouteModel <: AbstractODModel
    k                           :: Int
    l                           :: Int
    route_regularization_weight :: Float64
    repositioning_time          :: Float64
    routes_file                 :: String
    alpha_profile_file          :: String
    max_walking_distance        :: Float64
    max_detour_time             :: Float64
    max_detour_ratio            :: Float64
    time_window_sec             :: Int
    use_lazy_constraints        :: Bool

    function AlphaRouteModel(
            k::Int,
            l::Int;
            route_regularization_weight :: Number = 1.0,
            repositioning_time          :: Number = 20.0,
            routes_file                 :: String,
            alpha_profile_file          :: String,
            max_walking_distance        :: Number = 300,
            max_detour_time             :: Number = 1200,
            max_detour_ratio            :: Number = 2.0,
            time_window_sec             :: Int    = 3600,
            use_lazy_constraints        :: Bool   = false
        )

        k > 0                           || throw(ArgumentError("k must be positive"))
        l >= k                          || throw(ArgumentError("l must be >= k"))
        route_regularization_weight >= 0 ||
            throw(ArgumentError("route_regularization_weight must be non-negative"))
        isfile(routes_file)             || throw(ArgumentError("routes_file not found: $routes_file"))
        isfile(alpha_profile_file)      ||
            throw(ArgumentError("alpha_profile_file not found: $alpha_profile_file"))
        time_window_sec > 0             || throw(ArgumentError("time_window_sec must be positive"))

        mdt = Float64(max_detour_time)
        mdr = Float64(max_detour_ratio)
        mdt >= 0.0 || throw(ArgumentError("max_detour_time must be non-negative"))
        mdr >= 0.0 || throw(ArgumentError("max_detour_ratio must be non-negative"))
        Float64(max_walking_distance) >= 0 ||
            throw(ArgumentError("max_walking_distance must be non-negative"))

        new(k, l,
            Float64(route_regularization_weight),
            Float64(repositioning_time),
            routes_file,
            alpha_profile_file,
            Float64(max_walking_distance),
            mdt, mdr,
            time_window_sec,
            use_lazy_constraints)
    end
end
