"""
OD mapping for RouteFleetLimitModel.

Wraps a VehicleCapacityODMap (inner) and adds:
  - fleet_size::Int — maximum routes active per (time bucket, scenario)
  - delay_coeff — precomputed per-passenger delay coefficients d^r_{jk}
    keyed by (s, t_id, r_idx, j_idx, k_idx)

Field access for all VehicleCapacityODMap fields is delegated to `inner`
via Base.getproperty, so existing helper functions (e.g. `_time_ids`,
`_time_od_pairs`, `get_valid_jk_pairs`) can be called with `mapping.inner`
directly.
"""

export FleetLimitODMap
export create_fleet_limit_od_map


"""
    FleetLimitODMap <: AbstractClusteringMap

Data mapping for RouteFleetLimitModel.  Delegates all route/OD data to an
inner VehicleCapacityODMap; adds fleet_size and precomputed delay coefficients.

# Fields
- `inner::VehicleCapacityODMap`: full OD/route data (shared with RouteVehicleCapacityModel)
- `fleet_size::Int`: F — max active routes per (t, s)
- `delay_coeff::Dict{NTuple{5,Int},Float64}`: d^r_{jk} keyed by (s, t_id, r_idx, j_idx, k_idx)
"""
struct FleetLimitODMap <: AbstractClusteringMap
    inner        :: VehicleCapacityODMap
    fleet_size   :: Int
    delay_coeff  :: Dict{NTuple{5, Int}, Float64}
end


# Delegate field access so existing utilities work via mapping.inner
has_walking_distance_limit(mapping::FleetLimitODMap) = true

function get_valid_jk_pairs(mapping::FleetLimitODMap, o::Int, d::Int)
    return get_valid_jk_pairs(mapping.inner, o, d)
end


"""
    create_fleet_limit_od_map(model::RouteFleetLimitModel, data) -> FleetLimitODMap

Build the OD mapping for RouteFleetLimitModel:
1. Delegate route/OD construction to `create_vehicle_capacity_od_map` via a
   proxy RouteVehicleCapacityModel with matching parameters.
2. Precompute per-passenger delay coefficients for every (s, t_id, r_idx, j_idx, k_idx).
"""
function create_fleet_limit_od_map(
    model :: RouteFleetLimitModel,
    data  :: StationSelectionData
)::FleetLimitODMap

    # Build inner map by reusing VehicleCapacityODMap creation logic
    proxy = RouteVehicleCapacityModel(
        model.k, model.l;
        route_regularization_weight = model.route_regularization_weight,
        repositioning_time          = model.repositioning_time,
        vehicle_capacity            = model.vehicle_capacity,
        max_route_travel_time       = model.max_route_travel_time,
        max_walking_distance        = model.max_walking_distance,
        max_detour_time             = model.max_detour_time,
        max_detour_ratio            = model.max_detour_ratio,
        time_window_sec             = model.time_window_sec,
        max_stations_visited        = model.max_stations_visited,
        routes_file                 = model.routes_file,
    )
    inner = create_vehicle_capacity_od_map(proxy, data)

    # Precompute delay coefficients d^r_{jk} for every detour-feasible leg
    println("  Precomputing per-passenger delay coefficients...")
    flush(stdout)
    delay_coeff = _compute_delay_coefficients(inner, data)
    println("  → $(length(delay_coeff)) (s,t,r,j,k) delay entries")
    flush(stdout)

    return FleetLimitODMap(inner, model.fleet_size, delay_coeff)
end


"""
    _compute_delay_coefficients(mapping, data) -> Dict{NTuple{5,Int}, Float64}

For each (s, t_id, r_idx) route and each detour-feasible leg (j_idx, k_idx),
compute:

    d^r_{jk} = (in-vehicle time from j to k along route r) − direct(j, k)

Returns a dict keyed by (s, t_id, r_idx, j_idx, k_idx) with non-negative values.
Entries where the direct time is unavailable or delay ≤ 0 are omitted.
"""
function _compute_delay_coefficients(
    mapping :: VehicleCapacityODMap,
    data    :: StationSelectionData
)::Dict{NTuple{5, Int}, Float64}

    delay_coeff = Dict{NTuple{5, Int}, Float64}()
    S = length(mapping.scenarios)

    for s in 1:S
        for (t_id, routes_t) in mapping.routes_s[s]
            for (r_idx, route) in enumerate(routes_t)
                n = length(route.station_indices)
                n < 2 && continue

                # Precompute per-leg segment costs along the route
                seg = Vector{Float64}(undef, n - 1)
                for i in 1:(n - 1)
                    seg[i] = get_routing_cost(
                        data,
                        route.station_indices[i],
                        route.station_indices[i + 1]
                    )
                end

                for (j_idx, k_idx) in route.detour_feasible_legs
                    pos_j = findfirst(==(j_idx), route.station_indices)
                    pos_k = findfirst(==(k_idx), route.station_indices)
                    (isnothing(pos_j) || isnothing(pos_k)) && continue
                    pos_j >= pos_k && continue

                    in_vehicle = sum(seg[i] for i in pos_j:(pos_k - 1))
                    direct     = get_routing_cost(data, j_idx, k_idx)
                    delay      = in_vehicle - direct
                    delay > 0 || continue   # no delay — skip (zero contribution)

                    delay_coeff[(s, t_id, r_idx, j_idx, k_idx)] = delay
                end
            end
        end
    end

    return delay_coeff
end
