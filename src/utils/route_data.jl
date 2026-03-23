export RouteData

"""
    RouteData

Represents a pre-generated vehicle route (ordered sequence of VBS stops).

# Fields
- `id::Int`: 1-based index in the routes vector
- `station_ids::Vector{Int}`: Ordered station ID sequence
- `travel_time::Float64`: τ^r = sum of routing costs between consecutive stops
- `detour_feasible_legs::Vector{Tuple{Int,Int}}`: (pickup_id, dropoff_id) pairs whose
  in-vehicle detour satisfies the detour constraints. Only these legs are eligible for
  `alpha_r_jkts` variable creation in the MILP.

For a direct route [j, k]:
    detour_feasible_legs = [(j, k)]

For a one-stop route [j, l, k] where all legs are feasible:
    detour_feasible_legs = [(j, l), (j, k), (l, k)]
"""
struct RouteData
    id::Int
    station_ids::Vector{Int}
    travel_time::Float64
    detour_feasible_legs::Vector{Tuple{Int, Int}}
end
