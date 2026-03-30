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
(not decision variables).

Routes can be supplied in two ways, controlled by `generate_routes`:
- `generate_routes = false` (default): load from CSV files (`routes_file`, `alpha_profile_file`)
- `generate_routes = true`: generate via DFS from demand data using balanced alpha formula

Covering constraint:
    Σ_{od using (j,k)} x[s][t][od][pair]  ≤  Σ_r α^r_{jk} · θ^r_{ts}   ∀ j,k,t,s

No constraint (iii).  θ^r_{ts} ∈ Z+ remains a decision variable.

# Fields
- `k::Int`: Stations to activate per scenario
- `l::Int`: Stations to build
- `route_regularization_weight::Float64`: μ — penalty per unit route travel time deployed
- `repositioning_time::Float64`: ρ — repositioning overhead (seconds) added per deployment
- `generate_routes::Bool`: If true, generate routes via DFS; if false, load from CSV files
- `routes_file::Union{String,Nothing}`: Path to `routes_input.csv` (required when `generate_routes=false`)
- `alpha_profile_file::Union{String,Nothing}`: Path to `alpha_profile.csv` (required when `generate_routes=false`)
- `max_route_length::Int`: Max stops per route for DFS generation (default 3; ignored when `generate_routes=false`)
- `max_walking_distance::Float64`: Walking distance limit for (j,k) pair pruning
- `max_detour_time::Float64`: Max extra in-vehicle seconds vs direct trip (for detour_feasible_legs)
- `max_detour_ratio::Float64`: Max ratio `in_vehicle/direct - 1`
- `time_window_sec::Int`: Width of time bucket t (seconds)
- `use_lazy_constraints::Bool`: If true, the capacity constraint is submitted lazily
- `vehicle_capacity::Int`: Vehicle capacity C (default 18)
"""
struct AlphaRouteModel <: AbstractODModel
    k                           :: Int
    l                           :: Int
    route_regularization_weight :: Float64
    repositioning_time          :: Float64
    generate_routes             :: Bool
    routes_file                 :: Union{String, Nothing}
    alpha_profile_file          :: Union{String, Nothing}
    max_route_length            :: Int
    max_walking_distance        :: Float64
    max_detour_time             :: Float64
    max_detour_ratio            :: Float64
    time_window_sec             :: Int
    use_lazy_constraints        :: Bool
    vehicle_capacity            :: Int

    function AlphaRouteModel(
            k::Int,
            l::Int;
            route_regularization_weight :: Number               = 1.0,
            repositioning_time          :: Number               = 20.0,
            generate_routes             :: Bool                 = false,
            routes_file                 :: Union{String,Nothing} = nothing,
            alpha_profile_file          :: Union{String,Nothing} = nothing,
            max_route_length            :: Int                  = 3,
            max_walking_distance        :: Number               = 300,
            max_detour_time             :: Number               = 1200,
            max_detour_ratio            :: Number               = 2.0,
            time_window_sec             :: Int                  = 3600,
            use_lazy_constraints        :: Bool                 = false,
            vehicle_capacity            :: Int                  = 18
        )

        k > 0                           || throw(ArgumentError("k must be positive"))
        l >= k                          || throw(ArgumentError("l must be >= k"))
        route_regularization_weight >= 0 ||
            throw(ArgumentError("route_regularization_weight must be non-negative"))
        time_window_sec > 0             || throw(ArgumentError("time_window_sec must be positive"))
        vehicle_capacity > 0            || throw(ArgumentError("vehicle_capacity must be positive"))
        max_route_length > 0            || throw(ArgumentError("max_route_length must be positive"))

        if !generate_routes
            isnothing(routes_file) &&
                throw(ArgumentError("routes_file is required when generate_routes=false"))
            isnothing(alpha_profile_file) &&
                throw(ArgumentError("alpha_profile_file is required when generate_routes=false"))
            isfile(routes_file) ||
                throw(ArgumentError("routes_file not found: $routes_file"))
            isfile(alpha_profile_file) ||
                throw(ArgumentError("alpha_profile_file not found: $alpha_profile_file"))
        end

        mdt = Float64(max_detour_time)
        mdr = Float64(max_detour_ratio)
        mdt >= 0.0 || throw(ArgumentError("max_detour_time must be non-negative"))
        mdr >= 0.0 || throw(ArgumentError("max_detour_ratio must be non-negative"))
        Float64(max_walking_distance) >= 0 ||
            throw(ArgumentError("max_walking_distance must be non-negative"))

        new(k, l,
            Float64(route_regularization_weight),
            Float64(repositioning_time),
            generate_routes,
            routes_file,
            alpha_profile_file,
            max_route_length,
            Float64(max_walking_distance),
            mdt, mdr,
            time_window_sec,
            use_lazy_constraints,
            vehicle_capacity)
    end
end
