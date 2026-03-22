"""
Route activation variables.

θ_s[s][r] ∈ {0,1} for TwoStageRouteWithTimeModel and RouteAlphaCapacityModel,
stored as `m[:theta_s]` (Vector{Vector{VariableRef}}).

d_{jkts}, α^r_{jkts}, θ^r_{ts} ∈ Z+ for RouteVehicleCapacityModel (new formulation),
stored as sparse Dicts keyed by NTuples.
"""

export add_route_theta_variables!
export add_d_jkts_variables!
export add_alpha_r_jkts_variables!
export add_theta_ts_variables!

"""
    add_route_theta_variables!(m, data, mapping::TwoStageRouteODMap) -> Int

Add binary route activation variables θ_s[s][r] for all scenarios.

Stored as `m[:theta_s]`, a `Vector{Vector{VariableRef}}` where `m[:theta_s][s][r]`
is the variable for route r of scenario s.

Returns the number of variables added.
"""
function add_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::TwoStageRouteODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    theta_s = [VariableRef[] for _ in 1:S]
    for s in 1:S
        n_r = length(mapping.routes_s[s])
        for _ in 1:n_r
            push!(theta_s[s], @variable(m, binary = true))
        end
    end
    m[:theta_s] = theta_s

    return JuMP.num_variables(m) - before
end


"""
    add_route_theta_variables!(m, data, mapping::RouteODMap) -> Int

Add binary route activation variables θ_s[s][r] for RouteAlphaCapacityModel.

Stored as `m[:theta_s]`, a `Vector{Vector{VariableRef}}` where `m[:theta_s][s][r]`
is the variable for route r of scenario s.
"""
function add_route_theta_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::RouteODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    theta_s = [VariableRef[] for _ in 1:S]
    for s in 1:S
        n_r = length(mapping.routes_s[s])
        for _ in 1:n_r
            push!(theta_s[s], @variable(m, binary = true))
        end
    end
    m[:theta_s] = theta_s

    return JuMP.num_variables(m) - before
end


# ─────────────────────────────────────────────────────────────────────────────
# VehicleCapacityODMap (RouteVehicleCapacityModel — new formulation)
# ─────────────────────────────────────────────────────────────────────────────

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
    pos_j !== nothing && pos_k !== nothing && pos_j < pos_k
end


"""
    add_d_jkts_variables!(m, data, mapping::VehicleCapacityODMap) -> Int

Add integer induced-demand variables d_{jkts} ∈ Z+ for RouteVehicleCapacityModel.

`d_{jkts}` represents total demand of class (j, k) routed through VBS leg (j→k)
in time window t of scenario s.

Created for each unique (s, j_idx, k_idx, t_id) where at least one OD pair in
`Omega_s_t[s][t_id]` has (j_idx, k_idx) as a valid leg and positive demand.

Stored as `m[:d_jkts]::Dict{NTuple{4,Int}, VariableRef}` keyed `(s, j_idx, k_idx, t_id)`.

Returns the number of variables added.
"""
function add_d_jkts_variables!(
    m::Model,
    data::StationSelectionData,
    mapping::VehicleCapacityODMap
)::Int
    before = JuMP.num_variables(m)
    S = n_scenarios(data)

    d_jkts = Dict{NTuple{4,Int}, VariableRef}()

    for s in 1:S
        for (t_id, od_pairs) in mapping.Omega_s_t[s]
            od_demand_t = mapping.Q_s_t[s][t_id]
            # Collect (j,k) pairs that have positive demand in this bucket
            jk_with_demand = Set{Tuple{Int,Int}}()
            for (o, d) in od_pairs
                get(od_demand_t, (o, d), 0) == 0 && continue
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_with_demand, (j, k))
                end
            end

            for (j_idx, k_idx) in jk_with_demand
                key = (s, j_idx, k_idx, t_id)
                d_jkts[key] = @variable(m, integer = true, lower_bound = 0)
            end
        end
    end

    m[:d_jkts] = d_jkts
    return JuMP.num_variables(m) - before
end


"""
    add_alpha_r_jkts_variables!(m, data, mapping::VehicleCapacityODMap) -> Int

Add integer route-serving variables α^r_{jkts} ∈ Z+ for RouteVehicleCapacityModel.

`α^r_{jkts}` represents the amount of class-(j,k) demand served by route r in
time window t of scenario s.

Created for each (s, r_idx, j_idx, k_idx, t_id) where:
- Route r serves leg (j→k): both j_id and k_id appear in route.station_ids with j before k
- A d_jkts variable exists for (s, j_idx, k_idx, t_id)

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

    d_jkts = m[:d_jkts]

    # Group d_jkts keys by scenario for efficient inner loop
    d_keys_by_s = Dict{Int, Vector{NTuple{3,Int}}}()  # s → [(j_idx, k_idx, t_id), ...]
    for (s, j_idx, k_idx, t_id) in keys(d_jkts)
        push!(get!(d_keys_by_s, s, NTuple{3,Int}[]), (j_idx, k_idx, t_id))
    end

    alpha_r_jkts     = Dict{NTuple{5,Int}, VariableRef}()
    alpha_r_jkts_by_srt = Dict{NTuple{3,Int}, Vector{Tuple{Int,Int}}}()

    for s in 1:S
        jkt_list = get(d_keys_by_s, s, NTuple{3,Int}[])
        isempty(jkt_list) && continue

        # Group jkt_list by t_id so we can look up per-time-bucket routes efficiently
        jk_by_t = Dict{Int, Vector{Tuple{Int,Int}}}()
        for (j_idx, k_idx, t_id) in jkt_list
            push!(get!(jk_by_t, t_id, Tuple{Int,Int}[]), (j_idx, k_idx))
        end

        for (t_id, jk_list_t) in jk_by_t
            routes_t = get(mapping.routes_s[s], t_id, RouteData[])
            for (r_idx, route) in enumerate(routes_t)
                for (j_idx, k_idx) in jk_list_t
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
    add_theta_ts_variables!(m, data, mapping::VehicleCapacityODMap) -> Int

Add integer route-deployment variables θ^r_{ts} ∈ Z+ for RouteVehicleCapacityModel.

`θ^r_{ts}` counts how many times route r is deployed in time window t of scenario s.
Created for each (s, t_id, r_idx) where at least one α^r_{jkts} variable exists.

Stored as `m[:theta_ts]::Dict{NTuple{3,Int}, VariableRef}` keyed `(s, t_id, r_idx)`.

Returns the number of variables added.
"""
function add_theta_ts_variables!(
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

    theta_ts = Dict{NTuple{3,Int}, VariableRef}()
    for (s, t_id, r_idx) in srt_with_alpha
        theta_ts[(s, t_id, r_idx)] = @variable(m, integer = true, lower_bound = 0)
    end

    m[:theta_ts] = theta_ts
    return JuMP.num_variables(m) - before
end
