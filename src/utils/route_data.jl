"""
Route data structure for TwoStageRouteWithTimeModel.
"""

export RouteData

"""
    RouteData

Represents a pre-generated vehicle route (ordered sequence of VBS stops).

# Fields
- `id::Int`: 1-based index in the routes vector
- `station_ids::Vector{Int}`: Ordered station ID sequence
- `travel_time::Float64`: τ^r = sum of routing costs between consecutive stops
- `od_capacity::Dict{Tuple{Int,Int},Int}`: (pickup_id, dropoff_id) → capacity C

For a direct route [j, k]:
    od_capacity = {(j, k) => C}

For a one-stop route [j, l, k]:
    od_capacity = {(j, l) => C, (j, k) => C, (l, k) => C}
"""
struct RouteData
    id::Int
    station_ids::Vector{Int}
    travel_time::Float64
    od_capacity::Dict{Tuple{Int, Int}, Int}
end
