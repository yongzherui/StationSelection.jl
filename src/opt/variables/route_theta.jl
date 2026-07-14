"""
Route activation variables for ExactDARPRouteModel.

θ^r_{ts} ∈ Z+ stored as a sparse Dict keyed by scenario, time bucket, and route.
"""

export add_theta_r_ts_variables!
export compute_beta_r_jkl

# ─────────────────────────────────────────────────────────────────────────────
# Shared route helpers
# ─────────────────────────────────────────────────────────────────────────────

"""
    compute_beta_r_jkl(route, j_idx, k_idx, l) -> Bool

Return true if leg (j_idx → k_idx) occupies segment l of route r.

Segment l is the arc from `route.station_indices[l]` to `route.station_indices[l+1]`.
A passenger riding from j to k occupies segment l iff `pos_j ≤ l < pos_k`,
where pos_j and pos_k are the 1-based positions of j and k in the station sequence.
"""
function compute_beta_r_jkl(
    route                   :: RouteData,
    j_idx                   :: Int,
    k_idx                   :: Int,
    l                       :: Int
)::Bool
    station_indices = route.station_indices
    pos_j = findfirst(==(j_idx), station_indices)
    pos_k = findfirst(==(k_idx), station_indices)
    pos_j === nothing && return false
    pos_k === nothing && return false
    return pos_j <= l < pos_k
end


"""
Check whether route serves leg (j_idx → k_idx): both station indices must appear in
route.station_indices with j before k.
"""
function _route_serves_jk(
    route                   :: RouteData,
    j_idx                   :: Int,
    k_idx                   :: Int
)::Bool
    station_indices = route.station_indices
    pos_j = findfirst(==(j_idx), station_indices)
    pos_k = findfirst(==(k_idx), station_indices)
    pos_j !== nothing && pos_k !== nothing && pos_j < pos_k &&
        (j_idx, k_idx) ∈ route.detour_feasible_legs
end


# ─────────────────────────────────────────────────────────────────────────────
# ExactDARPRouteODMap (ExactDARPRouteModel — fixed alpha parameters)
# ─────────────────────────────────────────────────────────────────────────────

"""
Emit warnings for (j,k,t,s) combinations that have demand but no positive alpha coverage.
These legs will have no capacity constraint and may be freely assigned without route coverage.
"""
function _arm_warn_uncovered_jk(
    data             :: StationSelectionData,
    mapping          :: ExactDARPRouteODMap,
    arm_alpha_params :: Dict{NTuple{5, Int}, Float64}
)
    S = n_scenarios(data)
    # Build set of (s, t_id, j_idx, k_idx) that have at least one alpha entry
    covered = Set{NTuple{4, Int}}()
    for (s, t_id, r_idx, j_idx, k_idx) in keys(arm_alpha_params)
        push!(covered, (s, t_id, j_idx, k_idx))
    end

    n_uncovered = 0
    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            for (o, d) in od_pairs
                get(mapping.Q_s_t[s][t_id], (o, d), 0) > 0 || continue
                for (j_idx, k_idx) in get_valid_jk_pairs(mapping, o, d)
                    (s, t_id, j_idx, k_idx) ∈ covered && continue
                    if n_uncovered < 5
                        @warn "ExactDARPRouteModel: no alpha coverage for (s=$s, t_id=$t_id, pickup_idx=$j_idx, dropoff_idx=$k_idx) — capacity constraint skipped for this leg"
                    end
                    n_uncovered += 1
                end
            end
        end
    end
    if n_uncovered > 5
        @warn "ExactDARPRouteModel: $n_uncovered total (j,k,t,s) legs have no alpha coverage (first 5 shown)"
    end
end


"""
    add_theta_r_ts_variables!(m, data, mapping::ExactDARPRouteODMap) -> Int

Integer route-deployment variables θ^r_{ts} ∈ Z+ for ExactDARPRouteModel.

Created for each (s, t_id, r_idx) where the route serves at least one valid (j,k) pair
in the bucket AND has a positive alpha value for that leg in `mapping.alpha_profile`.

Stored as `m[:theta_r_ts]::Dict{NTuple{3,Int}, VariableRef}` keyed `(s, t_id, r_idx)`.
Also stores `m[:arm_alpha_params]::Dict{NTuple{5,Int}, Float64}` keyed
`(s, t_id, r_idx, j_idx, k_idx)` for use in constraint building.

Returns the number of variables added.
"""
function add_theta_r_ts_variables!(
    m       :: Model,
    data    :: StationSelectionData,
    mapping :: ExactDARPRouteODMap
)::Int
    before        = JuMP.num_variables(m)
    S             = n_scenarios(data)
    alpha_profile = mapping.alpha_profile

    # Precompute: for each (s, t_id, r_idx, j_idx, k_idx), the fixed alpha param
    # Only entries with alpha > 0 are stored.
    arm_alpha_params = Dict{NTuple{5, Int}, Float64}()
    srt_with_alpha   = Set{NTuple{3, Int}}()

    for s in 1:S
        for t_id in _time_ids(mapping, s)
            od_pairs = _time_od_pairs(mapping, s, t_id)
            # Collect valid (j_idx, k_idx) pairs for this bucket
            jk_set = Set{Tuple{Int, Int}}()
            for (o, d) in od_pairs
                for (j, k) in get_valid_jk_pairs(mapping, o, d)
                    push!(jk_set, (j, k))
                end
            end
            isempty(jk_set) && continue

            routes_t = get(mapping.routes_s[s], t_id, RouteData[])
            for (r_idx, route) in enumerate(routes_t)
                for (j_idx, k_idx) in jk_set
                    alpha_val = get(alpha_profile, (route.id, j_idx, k_idx), 0.0)
                    alpha_val > 0 || continue
                    arm_alpha_params[(s, t_id, r_idx, j_idx, k_idx)] = alpha_val
                    push!(srt_with_alpha, (s, t_id, r_idx))
                end
            end
        end
    end

    theta_r_ts = Dict{NTuple{3, Int}, VariableRef}()
    for (s, t_id, r_idx) in srt_with_alpha
        theta_r_ts[(s, t_id, r_idx)] = @variable(m, integer = true, lower_bound = 0)
    end

    m[:theta_r_ts]       = theta_r_ts
    m[:arm_alpha_params] = arm_alpha_params

    n_theta = JuMP.num_variables(m) - before
    @info "add_theta_r_ts_variables!: done" n_alpha_params=length(arm_alpha_params) n_theta=n_theta

    # Warn about (j,k,t,s) pairs with demand but zero total alpha coverage
    _arm_warn_uncovered_jk(data, mapping, arm_alpha_params)

    return n_theta
end
