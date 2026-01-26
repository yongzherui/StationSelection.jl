"""
Shared constraint creation functions for station selection models.

These functions add constraints to JuMP models. They are designed
to be composable - models can pick and choose which constraint sets they need.
"""

using JuMP

export add_assignment_constraints!
export add_station_limit_constraint!
export add_scenario_activation_limit_constraints!
export add_assignment_to_active_constraints!
export add_activation_linking_constraints!
export add_assignment_to_flow_constraints!
export add_assignment_to_same_source_detour_constraints!
export add_assignment_to_same_dest_detour_constraints!

# ============================================================================
# Assignment Constraints
# ============================================================================

"""
    add_assignment_constraints!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Each OD request must be assigned to exactly one station pair.
    Σⱼₖ x[s][t][od][j,k] = 1  ∀(o,d,t) ∈ Ω, s
"""
function add_assignment_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    n = data.n_stations
    S = n_scenarios(data)
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                @constraint(m, sum(x[s][time_id][od]) == 1)
            end
        end
    end

    return nothing
end


# ============================================================================
# Station Selection/Activation Limits
# ============================================================================

"""
    add_station_limit_constraint!(m::Model, data::StationSelectionData, limit::Int;
                                  equality::Bool=true)

Limit total number of stations selected.
    Σⱼ y[j] = limit  (if equality)
    Σⱼ y[j] ≤ limit  (otherwise)
"""
function add_station_limit_constraint!(
    m::Model,
    data::StationSelectionData,
    limit::Int;
    equality::Bool=true
)
    y = m[:y]
    if equality
        @constraint(m, station_limit, sum(y) == limit)
    else
        @constraint(m, station_limit, sum(y) <= limit)
    end
    return nothing
end

"""
    add_scenario_activation_limit_constraints!(m::Model, data::StationSelectionData, k::Int)

Limit active stations per scenario.
    Σⱼ z[j,s] = k  ∀s
"""
function add_scenario_activation_limit_constraints!(
    m::Model,
    data::StationSelectionData,
    k::Int
)
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    @constraint(m, activation_limit[s=1:S], sum(z[j,s] for j in 1:n) == k)
    return nothing
end

# ============================================================================
# Linking Constraints
# ============================================================================

"""
    add_activation_linking_constraints!(m::Model, data::StationSelectionData)

Active stations must be built.
    z[j,s] ≤ y[j]  ∀j,s
"""
function add_activation_linking_constraints!(m::Model, data::StationSelectionData)
    n = data.n_stations
    S = n_scenarios(data)
    y = m[:y]
    z = m[:z]
    @constraint(m, link_zy[j=1:n, s=1:S], z[j,s] <= y[j])
    return nothing
end

"""
    add_assignment_to_active_constraints!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Assignment requires both stations to be active.
    2 * x[s][t][od][j,k] ≤ z[j,s] + z[k,s]  ∀(o,d,t) ∈ Ω, j, k, s
"""
function add_assignment_to_active_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    n = data.n_stations
    S = n_scenarios(data)
    z = m[:z]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                for j in 1:n, k in 1:n
                    @constraint(m, 2 * x[s][time_id][od][j, k] <= z[j, s] + z[k, s])
                end
            end
        end
    end

    return nothing
end

# ============================================================================
# Flow Constraints
# ============================================================================

"""
    add_assignment_to_flow_constraints!(m::Model, data::StationSelectionData, mapping::PoolingScenarioOriginDestTimeMap)

Assignment implies flow on that edge.
    x[s][t][od][j,k] ≤ f[s][t][j,k]  ∀(o,d,t) ∈ Ω, j, k, s
"""
function add_assignment_to_flow_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap
    )
    n = data.n_stations
    S = n_scenarios(data)
    f = m[:f]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            for od in od_vector
                @constraint(m, [j in 1:n, k in 1:n], x[s][time_id][od][j, k] <= f[s][time_id][j, k])
            end
        end
    end

    return nothing
end

# ============================================================================
# Detour Constraints
# ============================================================================

"""
    add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}}
    )

Same-source pooling constraints for triplet (j, k, l):
- Pools trip (j→l) with trip (k→l) - both end at l
- Vehicle path: j → k → l

For each (j,k,l) ∈ Ξ, if pooling is enabled (u=1), we need assignments:
    x_{od,t,jk,s} ≥ u_{t,idx,s}   (assignment on j→k edge)
    x_{od,t,jl,s} ≥ u_{t,idx,s}   (assignment on j→l edge)

Where idx is the index of (j,k,l) in Xi_same_source.

Note: These constraints link pooling decisions to assignment variables.
If u[s][t][idx] = 1, then there must exist OD pairs assigned to edges (j,k) and (j,l).
"""
function add_assignment_to_same_source_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}}
    )
    S = n_scenarios(data)
    u = m[:u]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # For each triplet (j, k, l) in Xi_same_source
            for (idx, (j, k, l)) in enumerate(Xi_same_source)
                # Constraint 1: x_{od,t,jk,s} >= u_{t,idx,s}
                # Sum over all OD pairs that could use edge (j,k)
                # If u=1, at least one OD must be assigned to (j,k)

                j = mapping.station_id_to_array_idx[j]
                k = mapping.station_id_to_array_idx[k]
                l = mapping.station_id_to_array_idx[l]

                @constraint(m,
                    sum(x[s][time_id][od][j, k] for od in od_vector) >= u[s][time_id][idx]
                )

                # Constraint 2: x_{od,t,jl,s} >= u_{t,idx,s}
                # If u=1, at least one OD must be assigned to (j,l)
                @constraint(m,
                    sum(x[s][time_id][od][j, l] for od in od_vector) >= u[s][time_id][idx]
                )
            end
        end
    end

    return nothing
end

"""
    add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )

Same-destination pooling constraints for quadruplet (j, k, l, t'):
- Pools trip (j→l) with trip (j→k) - both start at j
- Vehicle picks up both at j, drops first at k, continues to l
- The second leg (k→l) occurs at time t + t'

For each (j,k,l,t') ∈ Ξ, if pooling is enabled (v=1), we need assignments:
    x_{od,t,jl,s} ≥ v_{t,idx,s}       (assignment on j→l edge at time t)
    x_{od,t+t',kl,s} ≥ v_{t,idx,s}    (assignment on k→l edge at time t+t')

Where idx is the index of (j,k,l,t') in Xi_same_dest.
"""
function add_assignment_to_same_dest_detour_constraints!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}}
    )
    S = n_scenarios(data)
    v = m[:v]
    x = m[:x]

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # For each quadruplet (j, k, l, time_delta) in Xi_same_dest
            for (idx, (j, k, l, time_delta)) in enumerate(Xi_same_dest)
                future_time_id = time_id + time_delta

                # Only add constraints if future_time_id exists in the mapping
                # If future time doesn't exist, pooling cannot happen so skip entirely
                if !haskey(mapping.Omega_s_t[s], future_time_id)
                    continue
                end

                j = mapping.station_id_to_array_idx[j]
                k = mapping.station_id_to_array_idx[k]
                l = mapping.station_id_to_array_idx[l]

                # Constraint 1: x_{od,t,jl,s} >= v_{t,idx,s}
                # If v=1, at least one OD must be assigned to (j,l) at time t
                @constraint(m,
                    sum(x[s][time_id][od][j, l] for od in od_vector) >= v[s][time_id][idx]
                )

                # Constraint 2: x_{od,t+t',kl,s} >= v_{t,idx,s}
                # If v=1, at least one OD must be assigned to (k,l) at time t+t'
                future_od_vector = mapping.Omega_s_t[s][future_time_id]
                @constraint(m,
                    sum(x[s][future_time_id][od][k, l] for od in future_od_vector) >= v[s][time_id][idx]
                )
            end
        end
    end

    return nothing
end
