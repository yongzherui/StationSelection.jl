"""
Objective function for TwoStageSingleDetourModel.

The objective minimizes:
- Assignment costs: walking distances + routing costs for each OD assignment, weighted by demand
- Routing costs via flow variables
- Minus pooling savings from same-origin and same-destination detours

Objective:
    min Σ_s [ Σ_{(o,d,t)∈Ω} Σ_{j,k} q_{od,s,t} (d^origin_{oj} + d^dest_{dk} + c_{jk}) x_{od,t,jk,s}
            + γ (Σ_{j,k,t} c_{jk} f_{t,jk,s}
                 - Σ_{(j,k,l)∈Ξ,t} r_{jl,kl} · u_{t,idx,s}
                 - Σ_{(j,k,l,t')∈Ξ,t} r_{jl,jk} · v_{t,idx,s}) ]

Where:
- q_{od,s,t} = demand count (number of requests) for OD pair (o,d) in scenario s at time t
- r_{jl,kl} = c_{jl} - c_{kl} (savings from same-origin pooling)
- r_{jl,jk} = c_{jl} - c_{jk} (savings from same-dest pooling)
"""

using JuMP

export set_two_stage_single_detour_objective!


"""
    set_two_stage_single_detour_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    )

Set the minimization objective for TwoStageSingleDetourModel.

# Arguments
- `m::Model`: JuMP model with variables x, f, u, v already added
- `data::StationSelectionData`: Problem data with walking_costs and routing_costs
- `mapping::PoolingScenarioOriginDestTimeMap`: Scenario/time to OD mapping
- `Xi_same_source::Vector{Tuple{Int,Int,Int}}`: Same-source detour triplets (j,k,l)
- `Xi_same_dest::Vector{Tuple{Int,Int,Int,Int}}`: Same-dest detour quadruplets (j,k,l,t')
- `routing_weight::Float64`: Weight γ for routing/pooling terms (default 1.0)

# Objective Components
1. **Assignment costs**: For each OD assignment x[s][t][od][j,k]:
   - Weighted by q_{od,s,t} (demand count for that OD pair at that time in that scenario)
   - Origin walking: d^origin_{o,j} (walking from origin o to pickup station j)
   - Destination walking: d^dest_{d,k} (walking from dropoff station k to destination d)
   - Routing: c_{jk} (vehicle routing cost from j to k)

2. **Flow routing costs**: γ · Σ c_{jk} · f[s][t][j,k]

3. **Same-origin pooling savings**: -γ · Σ r_{jl,kl} · u[s][t][idx]
   where r_{jl,kl} = c_{jl} - c_{kl}

4. **Same-dest pooling savings**: -γ · Σ r_{jl,jk} · v[s][t][idx]
   where r_{jl,jk} = c_{jl} - c_{jk}

# Note
The demand count q_{od,s,t} is obtained from mapping.Q_s_t[s][t][(o,d)].
"""
function set_two_stage_single_detour_objective!(
        m::Model,
        data::StationSelectionData,
        mapping::PoolingScenarioOriginDestTimeMap,
        Xi_same_source::Vector{Tuple{Int, Int, Int}},
        Xi_same_dest::Vector{Tuple{Int, Int, Int, Int}};
        routing_weight::Float64=1.0
    )

    n = data.n_stations
    S = n_scenarios(data)

    x = m[:x]
    f = m[:f]
    u = m[:u]
    v = m[:v]

    # Precompute pooling savings for same-source triplets
    # r_{jl,kl} = c_{jl} - c_{kl}
    r_same_source = Float64[]
    for (j, k, l) in Xi_same_source
        c_jl = get_routing_cost(data, j, l)
        c_kl = get_routing_cost(data, k, l)
        push!(r_same_source, c_jl - c_kl)
    end

    # Precompute pooling savings for same-dest quadruplets
    # r_{jl,jk} = c_{jl} - c_{jk}
    r_same_dest = Float64[]
    for (j, k, l, _) in Xi_same_dest
        c_jl = get_routing_cost(data, j, l)
        c_jk = get_routing_cost(data, j, k)
        push!(r_same_dest, c_jl - c_jk)
    end

    # Build objective expression
    obj_expr = AffExpr(0.0)

    for s in 1:S
        for (time_id, od_vector) in mapping.Omega_s_t[s]
            # 1. Assignment costs: walking + routing for each OD, weighted by demand
            for (o, d) in od_vector
                # Get demand count q_{od,s,t} for this OD pair at this time in this scenario
                q_od_s_t = mapping.Q_s_t[s][time_id][(o, d)]

                for j in 1:n, k in 1:n
                    # Get station IDs
                    j_id = mapping.array_idx_to_station_id[j]
                    k_id = mapping.array_idx_to_station_id[k]

                    # Walking cost from origin o to pickup station j
                    d_origin_oj = get_walking_cost(data, o, j_id)

                    # Walking cost from dropoff station k to destination d
                    d_dest_dk = get_walking_cost(data, k_id, d)

                    # Routing cost from j to k
                    c_jk = get_routing_cost(data, j_id, k_id)

                    # Total assignment cost weighted by demand count
                    assignment_cost = q_od_s_t * (d_origin_oj + d_dest_dk + c_jk)

                    add_to_expression!(obj_expr, assignment_cost, x[s][time_id][(o, d)][j, k])
                end
            end

            # 2. Flow routing costs: γ · c_{jk} · f[s][t][j,k]
            for j in 1:n, k in 1:n
                j_id = mapping.array_idx_to_station_id[j]
                k_id = mapping.array_idx_to_station_id[k]
                c_jk = get_routing_cost(data, j_id, k_id)

                add_to_expression!(obj_expr, routing_weight * c_jk, f[s][time_id][j, k])
            end

            # 3. Same-origin pooling savings: -γ · r_{jl,kl} · u[s][t][idx]
            for (idx, r) in enumerate(r_same_source)
                if r > 0  # Only add if there's actual savings
                    add_to_expression!(obj_expr, -routing_weight * r, u[s][time_id][idx])
                end
            end

            # 4. Same-dest pooling savings: -γ · r_{jl,jk} · v[s][t][idx]
            for (idx, r) in enumerate(r_same_dest)
                if r > 0  # Only add if there's actual savings
                    add_to_expression!(obj_expr, -routing_weight * r, v[s][time_id][idx])
                end
            end
        end
    end

    @objective(m, Min, obj_expr)

    return nothing
end
