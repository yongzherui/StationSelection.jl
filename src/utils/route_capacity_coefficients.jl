"""
Route capacity coefficients for RouteAlphaCapacityModel and RouteVehicleCapacityModel.
"""

export compute_alpha_r_jkt
export compute_beta_r_jkl


"""
    compute_alpha_r_jkt(route, j_idx, k_idx, t_id) -> Int

Placeholder: α^r_{jkt} — capacity of route r for demand class (j, k, t).

`j_idx` and `k_idx` are station array indices (1-based).
`t_id` is the time-bucket index as computed by `compute_time_to_od_count_mapping`.

TODO: implement via route service profile subproblem.
"""
function compute_alpha_r_jkt(
    route  :: NonTimedRouteData,
    j_idx  :: Int,
    k_idx  :: Int,
    t_id   :: Int
)::Int
    error("compute_alpha_r_jkt: not yet implemented")
end


"""
    compute_beta_r_jkl(route, j_idx, k_idx, l, array_idx_to_station_id) -> Bool

β^r_{jkl} — whether demand class (j, k) occupies segment l on route r.

Segment l is the arc from `route.station_ids[l]` to `route.station_ids[l+1]`
(1-based, so l ∈ 1 : length(station_ids) - 1).

Returns true iff position(j_id) ≤ l < position(k_id) in route.station_ids,
i.e. passengers boarding at j and alighting at k occupy segment l.
"""
function compute_beta_r_jkl(
    route                   :: RouteData,
    j_idx                   :: Int,
    k_idx                   :: Int,
    l                       :: Int,
    array_idx_to_station_id :: Vector{Int}
)::Bool
    j_id  = array_idx_to_station_id[j_idx]
    k_id  = array_idx_to_station_id[k_idx]
    sids  = route.station_ids
    pos_j = findfirst(==(j_id), sids)
    pos_k = findfirst(==(k_id), sids)
    (pos_j === nothing || pos_k === nothing) && return false
    return pos_j <= l < pos_k
end
