"""
Route and alpha generation for AlphaRouteModel without simulation data.

Reuses `generate_simple_routes` (the same DFS algorithm as RouteVehicleCapacityModel)
and derives balanced alpha values purely from route structure.

## Alpha formula

For a route with station sequence [s₁, …, sₙ] and detour-feasible legs:
- Segment l covers the arc sₗ → sₗ₊₁  (l = 1 … n-1)
- n_classes[l] = number of feasible legs (j,k) whose passengers occupy segment l
  (i.e., pos_j ≤ l < pos_k, 1-based positions)
- α_{r,j,k} = floor(C / max{ n_classes[l] : l spanned by (j,k) })

Direct 2-stop routes [j,k] always have n_classes = 1, so α = C.

Example — 3-stop route [A,B,C], all legs feasible, C=18:
  Seg 1 (A→B): spanned by (A,B) and (A,C)  → n_classes[1] = 2
  Seg 2 (B→C): spanned by (A,C) and (B,C)  → n_classes[2] = 2
  α(A,B) = floor(18/2) = 9
  α(A,C) = floor(18/2) = 9   (max of seg1=2, seg2=2)
  α(B,C) = floor(18/2) = 9
  Check: 9+9 = 18 ≤ C on both segments ✓
"""

export derive_balanced_alpha, generate_routes_and_alpha


"""
    derive_balanced_alpha(routes, vehicle_capacity) -> Dict{NTuple{3,Int}, Float64}

Compute balanced alpha values for a set of routes purely from route structure.

For each route, for each detour-feasible leg (j,k):
  α = floor(C / max n_classes[l]) where n_classes[l] = number of feasible legs spanning l.

Returns a dict keyed (route_id, pickup_sid, dropoff_sid) → alpha value.
"""
function derive_balanced_alpha(
    routes           :: Vector{RouteData},
    vehicle_capacity :: Int
) :: Dict{NTuple{3, Int}, Float64}

    alpha = Dict{NTuple{3, Int}, Float64}()

    for route in routes
        sids  = route.station_ids
        n     = length(sids)
        legs  = route.detour_feasible_legs

        isempty(legs) && continue

        if n == 2
            # Direct route: only one leg, one segment, n_classes = 1
            j_id, k_id = sids[1], sids[2]
            alpha[(route.id, j_id, k_id)] = Float64(vehicle_capacity)
            continue
        end

        # 1-based position of each station in the route
        pos = Dict{Int, Int}(sid => i for (i, sid) in enumerate(sids))

        # For each segment l ∈ 1..n-1, count how many feasible legs span it
        n_classes = zeros(Int, n - 1)
        for (j_id, k_id) in legs
            pj = get(pos, j_id, 0)
            pk = get(pos, k_id, 0)
            (pj == 0 || pk == 0 || pj >= pk) && continue
            for l in pj:(pk - 1)
                n_classes[l] += 1
            end
        end

        # Alpha for each feasible leg
        for (j_id, k_id) in legs
            pj = get(pos, j_id, 0)
            pk = get(pos, k_id, 0)
            (pj == 0 || pk == 0 || pj >= pk) && continue
            max_n = maximum(n_classes[l] for l in pj:(pk - 1))
            max_n == 0 && continue
            alpha[(route.id, j_id, k_id)] = Float64(floor(Int, vehicle_capacity / max_n))
        end
    end

    return alpha
end


"""
    generate_routes_and_alpha(
        data, valid_jk_pairs_global, array_idx_to_station_id;
        vehicle_capacity, max_route_length, max_detour_time, max_detour_ratio
    ) -> (Vector{RouteData}, Dict{NTuple{3,Int}, Float64})

Generate routes via DFS and derive balanced alpha values from route structure.

`valid_jk_pairs_global` should be the union of all valid (j_idx, k_idx) pairs across all
scenarios and time buckets, expressed as array indices into `array_idx_to_station_id`.
"""
function generate_routes_and_alpha(
    data                    :: StationSelectionData,
    valid_jk_pairs_global   :: Set{Tuple{Int, Int}},
    array_idx_to_station_id :: Vector{Int};
    vehicle_capacity  :: Int     = 18,
    max_route_length  :: Int     = 3,
    max_detour_time   :: Float64 = Inf,
    max_detour_ratio  :: Float64 = Inf
) :: Tuple{Vector{RouteData}, Dict{NTuple{3, Int}, Float64}}

    println("  Generating routes via DFS (max_route_length=$max_route_length)...")
    flush(stdout)

    routes = generate_simple_routes(
        valid_jk_pairs_global,
        array_idx_to_station_id,
        data;
        max_route_length = max_route_length,
        max_detour_time  = max_detour_time,
        max_detour_ratio = max_detour_ratio
    )

    n_direct   = count(r -> length(r.station_ids) == 2, routes)
    n_multileg = length(routes) - n_direct
    println("  Generated $(length(routes)) routes: $n_direct direct, $n_multileg multi-leg")
    flush(stdout)

    println("  Deriving balanced alpha values (C=$vehicle_capacity)...")
    flush(stdout)

    alpha = derive_balanced_alpha(routes, vehicle_capacity)
    println("  Derived $(length(alpha)) alpha entries")
    flush(stdout)

    return routes, alpha
end
