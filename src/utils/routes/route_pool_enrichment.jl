export enrich_alpha_profiles!

using JuMP

struct AlphaPressureDiagnostics
    max_binding_ratio :: Dict{Tuple{Int,Int}, Float64}
    pressure_score    :: Dict{Tuple{Int,Int}, Float64}
    max_assigned      :: Dict{Tuple{Int,Int,Int}, Float64}
end

function compute_alpha_pressure_diagnostics(
    result::OptResult,
    config::AlphaEnrichmentConfig,
)::AlphaPressureDiagnostics
    mapping    = result.mapping::AlphaRouteODMap
    theta_r_ts = get(result.model.obj_dict, :theta_r_ts, Dict{NTuple{3,Int}, VariableRef}())
    arm_alpha  = get(result.model.obj_dict, :arm_alpha_params, Dict{NTuple{5,Int}, Float64}())
    x_vars     = result.model[:x]

    max_binding_ratio = Dict{Tuple{Int,Int}, Float64}()
    max_assigned      = Dict{Tuple{Int,Int,Int}, Float64}()

    S = length(mapping.scenarios)
    for s in 1:S
        for t_id in _time_ids(mapping, s)
            routes_t  = get(get(mapping.routes_s, s, Dict{Int,Vector{RouteData}}()), t_id, RouteData[])
            x_t       = get(get(x_vars, s, Dict{Int,Dict{Int,Vector{VariableRef}}}()), t_id, Dict{Int,Vector{VariableRef}}())
            od_pairs  = _time_od_pairs(mapping, s, t_id)

            provided = Dict{Tuple{Int,Int}, Float64}()
            assigned = Dict{Tuple{Int,Int}, Float64}()

            for (r_idx, route) in enumerate(routes_t)
                theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
                theta_var === nothing && continue
                theta_val = JuMP.value(theta_var)
                theta_val > 0 || continue
                for (j_idx, k_idx) in route.detour_feasible_legs
                    alpha_val = get(arm_alpha, (s, t_id, r_idx, j_idx, k_idx), 0.0)
                    provided[(j_idx, k_idx)] = get(provided, (j_idx, k_idx), 0.0) + alpha_val * theta_val
                end
            end

            for (od_idx, (o, d)) in enumerate(od_pairs)
                x_od = get(x_t, od_idx, VariableRef[])
                isempty(x_od) && continue
                valid_pairs = get_valid_jk_pairs(mapping, o, d)
                for (pair_idx, (j_idx, k_idx)) in enumerate(valid_pairs)
                    pair_idx > length(x_od) && break
                    val = JuMP.value(x_od[pair_idx])
                    val > 0 || continue
                    assigned[(j_idx, k_idx)] = get(assigned, (j_idx, k_idx), 0.0) + val
                end
            end

            for ((j_idx, k_idx), prov) in provided
                prov > 0 || continue
                asgn  = get(assigned, (j_idx, k_idx), 0.0)
                ratio = asgn / prov
                key   = (j_idx, k_idx)
                max_binding_ratio[key] = max(get(max_binding_ratio, key, 0.0), ratio)
                t_key = (j_idx, k_idx, t_id)
                max_assigned[t_key] = max(get(max_assigned, t_key, 0.0), asgn)
            end
        end
    end

    pt  = config.pressure_threshold
    bt  = config.binding_threshold
    dbt = bt - pt
    pressure_score = Dict{Tuple{Int,Int}, Float64}(
        jk => clamp((ratio - pt) / dbt, 0.0, 1.0)
        for (jk, ratio) in max_binding_ratio
    )

    return AlphaPressureDiagnostics(max_binding_ratio, pressure_score, max_assigned)
end

function _route_sequence_profile_count(
    bucket_state::RoutePoolState,
    station_indices::AbstractVector{Int},
)::Int
    count = 0
    for route in values(bucket_state.routes_by_id)
        route.id ∉ bucket_state.removed_route_ids || continue
        route.station_indices == station_indices && (count += 1)
    end
    return count
end

function _existing_alpha_for_route(
    bucket_state::RoutePoolState,
    route_id::Int,
    jk_legs::AbstractVector{Tuple{Int,Int}},
)::Dict{Tuple{Int,Int}, Float64}
    result = Dict{Tuple{Int,Int}, Float64}()
    for (j_idx, k_idx) in jk_legs
        result[(j_idx, k_idx)] = get(bucket_state.alpha_profile, (route_id, j_idx, k_idx), 0.0)
    end
    return result
end

function _all_profiles_for_sequence(
    bucket_state::RoutePoolState,
    station_indices::AbstractVector{Int},
)::Vector{Dict{Tuple{Int,Int}, Float64}}
    profiles = Dict{Tuple{Int,Int}, Float64}[]
    for (route_id, route) in bucket_state.routes_by_id
        route_id ∈ bucket_state.removed_route_ids && continue
        route.station_indices == station_indices || continue
        profile = Dict{Tuple{Int,Int}, Float64}()
        for (j_idx, k_idx) in route.detour_feasible_legs
            profile[(j_idx, k_idx)] = get(bucket_state.alpha_profile, (route_id, j_idx, k_idx), 0.0)
        end
        push!(profiles, profile)
    end
    return profiles
end

function _build_leg_segments(route::RouteData)::Dict{Tuple{Int,Int}, Vector{Int}}
    stations = route.station_indices
    m = length(stations)
    leg_segments = Dict{Tuple{Int,Int}, Vector{Int}}()
    for (j_idx, k_idx) in route.detour_feasible_legs
        pickup_pos  = findfirst(==(j_idx), stations)
        isnothing(pickup_pos) && continue
        dropoff_pos = nothing
        for pos in (pickup_pos + 1):m
            if stations[pos] == k_idx
                dropoff_pos = pos
                break
            end
        end
        isnothing(dropoff_pos) && continue
        leg_segments[(j_idx, k_idx)] = collect(pickup_pos:(dropoff_pos - 1))
    end
    return leg_segments
end

function _build_enriched_alpha(
    route::RouteData,
    existing_alpha::Dict{Tuple{Int,Int}, Float64},
    pressure_score::Dict{Tuple{Int,Int}, Float64},
    vehicle_capacity::Int,
    config::AlphaEnrichmentConfig,
)::Union{Nothing, Dict{Tuple{Int,Int}, Float64}}
    isempty(route.detour_feasible_legs) && return nothing

    leg_segments = _build_leg_segments(route)
    isempty(leg_segments) && return nothing

    legs    = collect(keys(leg_segments))
    n_segs  = length(route.station_indices) - 1
    residual = fill(float(vehicle_capacity), n_segs)

    weight = Dict{Tuple{Int,Int}, Float64}(
        leg => (get(existing_alpha, leg, 0.0) + 1.0) *
               (1.0 + config.alpha_scale_factor * get(pressure_score, leg, 0.0))
        for leg in legs
    )

    alpha   = Dict{Tuple{Int,Int}, Float64}(leg => 0.0 for leg in legs)
    blocked = Set{Tuple{Int,Int}}()

    while length(blocked) < length(legs)
        best_leg = nothing
        best_score = -Inf
        for leg in legs
            leg ∈ blocked && continue
            score = weight[leg] / (1.0 + alpha[leg])
            if score > best_score
                best_score = score
                best_leg = leg
            end
        end
        isnothing(best_leg) && break

        segs = leg_segments[best_leg]
        if all(residual[seg] >= 1.0 for seg in segs)
            alpha[best_leg] += 1.0
            for seg in segs
                residual[seg] -= 1.0
            end
        else
            push!(blocked, best_leg)
        end
    end

    any(get(pressure_score, leg, 0.0) > 0.0 && alpha[leg] > get(existing_alpha, leg, 0.0)
        for leg in legs) || return nothing

    return alpha
end

function _profile_is_valid(
    new_alpha::Dict{Tuple{Int,Int}, Float64},
    existing_profiles::Vector{Dict{Tuple{Int,Int}, Float64}},
    vehicle_capacity::Int,
    route::RouteData,
    min_diff::Int,
)::Bool
    all(v >= 0.0 && isinteger(v) for v in values(new_alpha)) || return false
    any(v > 0.0 for v in values(new_alpha)) || return false

    leg_segments = _build_leg_segments(route)
    n_segs = length(route.station_indices) - 1
    seg_load = zeros(Float64, n_segs)
    for (leg, alpha_val) in new_alpha
        segs = get(leg_segments, leg, Int[])
        for seg in segs
            seg_load[seg] += alpha_val
        end
    end
    all(load <= vehicle_capacity for load in seg_load) || return false

    for existing in existing_profiles
        existing == new_alpha && return false
        all_legs = union(Set(keys(new_alpha)), Set(keys(existing)))
        l1 = sum(abs(get(new_alpha, leg, 0.0) - get(existing, leg, 0.0)) for leg in all_legs)
        l1 < min_diff && return false
    end

    return true
end

function enrich_alpha_profiles!(
    global_state::AlphaRouteBucketPoolsState,
    result::OptResult,
    config::AlphaEnrichmentConfig,
    vehicle_capacity::Int,
)
    config.enabled || return (
        skipped=true,
        added=0,
        n_pressured_legs=0,
        n_binding_legs=0,
        n_buckets_with_pressure=0,
        n_candidate_routes=0,
    )

    mapping = result.mapping
    isa(mapping, AlphaRouteODMap) ||
        return (
            skipped=true,
            added=0,
            n_pressured_legs=0,
            n_binding_legs=0,
            n_buckets_with_pressure=0,
            n_candidate_routes=0,
        )

    @debug "enrich_alpha_profiles!: starting" pressure_threshold=config.pressure_threshold binding_threshold=config.binding_threshold alpha_scale_factor=config.alpha_scale_factor max_new_profiles=config.max_new_profiles_per_iteration max_profiles_per_sequence=config.max_profiles_per_route_sequence min_profile_diff=config.min_profile_difference

    diagnostics = compute_alpha_pressure_diagnostics(result, config)

    n_pressured_pre = count(v >= config.pressure_threshold for v in values(diagnostics.max_binding_ratio))
    n_binding_pre   = count(v >= config.binding_threshold  for v in values(diagnostics.max_binding_ratio))

    any(v >= config.pressure_threshold for v in values(diagnostics.max_binding_ratio)) ||
        begin
            @debug "enrich_alpha_profiles!: skipped (no pressured legs)" n_jk_pairs_checked=length(diagnostics.max_binding_ratio)
            return (
                skipped=true,
                added=0,
                n_pressured_legs=0,
                n_binding_legs=n_binding_pre,
                n_buckets_with_pressure=0,
                n_candidate_routes=0,
            )
        end

    n_pressured = n_pressured_pre
    n_binding   = n_binding_pre
    @debug "enrich_alpha_profiles!: diagnostics" n_jk_pairs_checked=length(diagnostics.max_binding_ratio) n_pressured_legs=n_pressured n_binding_legs=n_binding

    theta_r_ts = get(result.model.obj_dict, :theta_r_ts, Dict{NTuple{3,Int}, VariableRef}())
    total_added = 0
    buckets_with_pressure = 0
    candidate_routes = 0

    for bucket_key in _sorted_bucket_route_pool_keys(global_state)
        total_added >= config.max_new_profiles_per_iteration && break

        bucket_state = global_state.bucket_states[bucket_key]
        s, t_id = bucket_key
        routes_t = get(get(mapping.routes_s, s, Dict{Int,Vector{RouteData}}()), t_id, RouteData[])
        isempty(routes_t) && continue

        selected_routes = RouteData[]
        for (r_idx, route) in enumerate(routes_t)
            theta_var = get(theta_r_ts, (s, t_id, r_idx), nothing)
            theta_var === nothing && continue
            JuMP.value(theta_var) > 0 || continue
            isempty(route.detour_feasible_legs) && continue
            any(get(diagnostics.pressure_score, leg, 0.0) > 0.0
                for leg in route.detour_feasible_legs) || continue
            push!(selected_routes, route)
        end

        isempty(selected_routes) && continue
        buckets_with_pressure += 1

        sort!(selected_routes, by=r -> -maximum(
            get(diagnostics.pressure_score, leg, 0.0) for leg in r.detour_feasible_legs
        ))
        candidates = selected_routes[1:min(config.max_candidate_routes_for_enrichment, length(selected_routes))]
        candidate_routes += length(candidates)

        for route in candidates
            total_added >= config.max_new_profiles_per_iteration && break

            n_existing = _route_sequence_profile_count(bucket_state, route.station_indices)
            n_existing >= config.max_profiles_per_route_sequence && continue

            existing_alpha = _existing_alpha_for_route(bucket_state, route.id, route.detour_feasible_legs)
            new_alpha = _build_enriched_alpha(
                route, existing_alpha, diagnostics.pressure_score, vehicle_capacity, config,
            )
            isnothing(new_alpha) && continue

            existing_profiles = _all_profiles_for_sequence(bucket_state, route.station_indices)
            _profile_is_valid(new_alpha, existing_profiles, vehicle_capacity, route,
                              config.min_profile_difference) || continue

            temp_route = RouteData(0, route.station_indices, route.travel_time, route.detour_feasible_legs)
            source_alpha = Dict{NTuple{3,Int}, Float64}(
                (0, j, k) => v for ((j, k), v) in new_alpha
            )
            _, was_inserted = _insert_route_variant!(global_state, bucket_state, temp_route, source_alpha, :alpha_enriched)
            was_inserted && (total_added += 1)
        end
    end

    @debug "enrich_alpha_profiles!: done" added=total_added n_pressured_legs=n_pressured n_binding_legs=n_binding n_buckets_with_pressure=buckets_with_pressure n_candidate_routes=candidate_routes
    return (
        skipped=false,
        added=total_added,
        n_pressured_legs=n_pressured,
        n_binding_legs=n_binding,
        n_buckets_with_pressure=buckets_with_pressure,
        n_candidate_routes=candidate_routes,
    )
end
