"""
Route activation variables for RouteVehicleCapacityModel (new formulation).

α^r_{jkts}, θ^r_{ts} ∈ Z+ stored as sparse Dicts keyed by NTuples.
"""

export add_alpha_r_jkts_variables!
export add_theta_r_ts_variables!
export compute_beta_r_jkl

# ─────────────────────────────────────────────────────────────────────────────
# VehicleCapacityODMap (RouteVehicleCapacityModel — new formulation)
# ─────────────────────────────────────────────────────────────────────────────

"""
    compute_beta_r_jkl(route, j_idx, k_idx, l, array_idx_to_station_id) -> Bool

Return true if leg (j_idx → k_idx) occupies segment l of route r.

Segment l is the arc from `route.station_ids[l]` to `route.station_ids[l+1]`.
A passenger riding from j to k occupies segment l iff `pos_j ≤ l < pos_k`,
where pos_j and pos_k are the 1-based positions of j and k in the station sequence.
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
    pos_j === nothing && return false
    pos_k === nothing && return false
    return pos_j <= l < pos_k
end


"""
Check whether route serves leg (j_idx → k_idx): j_id and k_id must both appear in
route.station_ids with j before k.
"""
function _route_serves_jk(
    route                   :: RouteData,
    j_idx                   :: Int,
    k_idx                   :: Int,
    array_idx_to_station_id :: Vector{Int}
)::Bool
    j_id  = array_idx_to_station_id[j_idx]
    k_id  = array_idx_to_station_id[k_idx]
    sids  = route.station_ids
    pos_j = findfirst(==(j_id), sids)
    pos_k = findfirst(==(k_id), sids)
    pos_j !== nothing && pos_k !== nothing && pos_j < pos_k &&
        (j_id, k_id) ∈ route.detour_feasible_legs
end


"""
    add_alpha_r_jkts_variables!(m, data, mapping::VehicleCapacityODMap) -> Int

Add integer route-serving variables α^r_{jkts} ∈ Z+ for RouteVehicleCapacityModel.

`α^r_{jkts}` represents the amount of class-(j,k) demand served by route r in
time window t of scenario s.

Created for each (s, r_idx, j_idx, k_idx, t_id) where:
- Route r serves leg (j→k): both j_id and k_id appear in route.station_ids with j before k
- At least one OD pair in Omega_s_t[s][t_id] has (j_idx, k_idx) as a valid leg

Stored as `m[:alpha_r_jkts]::Dict{NTuple{5,Int}, VariableRef}` keyed
`(s, r_idx, j_idx, k_idx, t_id)`.

Also stores a secondary index `m[:alpha_r_jkts_by_srt]::Dict{NTuple{3,Int}, Vector{Tuple{Int,Int}}}`
mapping `(s, r_idx, t_id) → [(j_idx, k_idx), ...]` for efficient constraint building.

Returns the number of variables added.
"""
function add_alpha_r_jkts_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::VehicleCapacityODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    alpha_r_jkts        = Dict{NTuple{5,Int}, VariableRef}()
    alpha_r_jkts_by_srt = Dict{NTuple{3,Int}, Vector{Tuple{Int,Int}}}()

    for s in 1:S
        # Collect active (j_idx, k_idx) pairs per time bucket from valid OD demand
        jk_by_t = Dict{Int, Set{Tuple{Int,Int}}}()
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(get!(jk_by_t, t_id, Set{Tuple{Int,Int}}()), (j, k))
                end
            end
        end

        for (t_id, jk_set_t) in jk_by_t
            routes_t = get(mapping.routes_s[s], t_id, RouteData[])
            for (r_idx, route) in enumerate(routes_t)
                for (j_idx, k_idx) in jk_set_t
                    _route_serves_jk(route, j_idx, k_idx, mapping.array_idx_to_station_id) || continue

                    key5 = (s, r_idx, j_idx, k_idx, t_id)
                    alpha_r_jkts[key5] = @variable(m, integer = true, lower_bound = 0)

                    srt_key = (s, r_idx, t_id)
                    push!(get!(alpha_r_jkts_by_srt, srt_key, Tuple{Int,Int}[]),
                          (j_idx, k_idx))
                end
            end
        end
    end

    m[:alpha_r_jkts]        = alpha_r_jkts
    m[:alpha_r_jkts_by_srt] = alpha_r_jkts_by_srt
    return JuMP.num_variables(m) - before
end


"""
    add_theta_r_ts_variables!(m, data, mapping::VehicleCapacityODMap) -> Int

Add integer route-deployment variables θ^r_{ts} ∈ Z+ for RouteVehicleCapacityModel.

`θ^r_{ts}` counts how many times route r is deployed in time window t of scenario s.
Created for each (s, t_id, r_idx) where at least one α^r_{jkts} variable exists.

Stored as `m[:theta_r_ts]::Dict{NTuple{3,Int}, VariableRef}` keyed `(s, t_id, r_idx)`.

Returns the number of variables added.
"""
function add_theta_r_ts_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::VehicleCapacityODMap
)::Int
    before = JuMP.num_variables(m)

    # Find all (s, t_id, r_idx) combinations that have any alpha_r_jkts variable
    srt_with_alpha = Set{NTuple{3,Int}}()
    for (s, r_idx, j_idx, k_idx, t_id) in keys(m[:alpha_r_jkts])
        push!(srt_with_alpha, (s, t_id, r_idx))
    end

    theta_r_ts = Dict{NTuple{3,Int}, VariableRef}()
    for (s, t_id, r_idx) in srt_with_alpha
        theta_r_ts[(s, t_id, r_idx)] = @variable(m, integer = true, lower_bound = 0)
    end

    m[:theta_r_ts] = theta_r_ts
    return JuMP.num_variables(m) - before
end
