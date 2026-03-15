"""
Route generation for TwoStageRouteModel.

Enumerates feasible vehicle routes (direct and multi-stop) from station data.
"""

export generate_routes

"""
    generate_routes(
        data::StationSelectionData;
        vehicle_capacity::Int = 4,
        max_route_travel_time::Union{Float64, Nothing} = nothing,
        max_intermediate_stops::Int = 0
    ) -> Vector{RouteData}

Enumerate all feasible routes from the candidate stations.

**Direct routes** (always generated): For each ordered pair (j_id, k_id) with j ≠ k,
create a route if `c_{jk} ≤ max_route_travel_time` (or no limit).
- `station_ids = [j_id, k_id]`, `travel_time = c_{jk}`
- `od_capacity = {(j_id, k_id) => C}`

**One-stop routes** (`max_intermediate_stops ≥ 1`): For each ordered triple
(j_id, l_id, k_id) all distinct, create a route if `c_{jl} + c_{lk} ≤ max_route_travel_time`.
- `station_ids = [j_id, l_id, k_id]`, `travel_time = c_{jl} + c_{lk}`
- `od_capacity = {(j_id, l_id) => C, (j_id, k_id) => C, (l_id, k_id) => C}`

Requires routing costs (`has_routing_costs(data)` must be true).
"""
function generate_routes(
    data::StationSelectionData;
    vehicle_capacity::Int = 4,
    max_route_travel_time::Union{Float64, Nothing} = nothing,
    max_intermediate_stops::Int = 0
)::Vector{RouteData}
    has_routing_costs(data) || error(
        "generate_routes requires routing costs (data.routing_costs must not be nothing)"
    )
    vehicle_capacity > 0 || throw(ArgumentError("vehicle_capacity must be positive"))
    max_intermediate_stops >= 0 || throw(ArgumentError("max_intermediate_stops must be non-negative"))

    C = vehicle_capacity
    routes = RouteData[]
    route_id = 0
    sids = Vector{Int}(data.stations.id)

    # Direct routes: all (j_id, k_id) pairs with j ≠ k
    for j_id in sids
        for k_id in sids
            j_id == k_id && continue
            c_jk = get_routing_cost(data, j_id, k_id)
            !isnothing(max_route_travel_time) && c_jk > max_route_travel_time && continue
            route_id += 1
            push!(routes, RouteData(
                route_id,
                [j_id, k_id],
                c_jk,
                Dict{Tuple{Int, Int}, Int}((j_id, k_id) => C)
            ))
        end
    end

    # One-stop routes
    if max_intermediate_stops >= 1
        for j_id in sids
            for l_id in sids
                j_id == l_id && continue
                c_jl = get_routing_cost(data, j_id, l_id)
                for k_id in sids
                    k_id == j_id && continue
                    k_id == l_id && continue
                    c_lk = get_routing_cost(data, l_id, k_id)
                    tt = c_jl + c_lk
                    !isnothing(max_route_travel_time) && tt > max_route_travel_time && continue
                    route_id += 1
                    push!(routes, RouteData(
                        route_id,
                        [j_id, l_id, k_id],
                        tt,
                        Dict{Tuple{Int, Int}, Int}(
                            (j_id, l_id) => C,
                            (j_id, k_id) => C,
                            (l_id, k_id) => C
                        )
                    ))
                end
            end
        end
    end

    return routes
end
