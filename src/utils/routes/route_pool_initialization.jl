export initialize_route_pool
export export_route_pool_state

function _route_alpha_signature(route::RouteData, alpha_profile::Dict{NTuple{3, Int}, Float64})::String
    legs = String[]
    for (rid, j_idx, k_idx) in sort!(collect(keys(alpha_profile)); by=x -> (x[2], x[3], x[1]))
        rid == route.id || continue
        push!(legs, "$(j_idx)>$(k_idx)=$(repr(alpha_profile[(rid, j_idx, k_idx)]))")
    end
    return join(route.station_indices, "|") * "#" * join(legs, ";")
end

function _empty_route_pool_state()
    return RoutePoolState(
        Dict{Int, RouteData}(),
        Dict{NTuple{3, Int}, Float64}(),
        Dict{String, Int}(),
        Dict{Int, Set{Symbol}}(),
        Set{Int}(),
        Set{Int}(),
        Set{Int}(),
        1,
        0,
    )
end

function _copy_route_with_id(route::RouteData, route_id::Int)::RouteData
    return RouteData(route_id, copy(route.station_indices), route.travel_time, copy(route.detour_feasible_legs))
end

function _insert_route_variant!(
    state::RoutePoolState,
    route::RouteData,
    source_alpha::Dict{NTuple{3, Int}, Float64},
    provenance::Symbol;
    protect_direct::Bool=false
)::Tuple{Int, Bool}
    signature = _route_alpha_signature(route, source_alpha)
    if haskey(state.signature_to_route_id, signature)
        route_id = state.signature_to_route_id[signature]
        push!(get!(state.provenance_by_route_id, route_id, Set{Symbol}()), provenance)
        if protect_direct
            push!(state.protected_route_ids, route_id)
            push!(state.direct_seed_route_ids, route_id)
        end
        for (rid, j_idx, k_idx) in keys(source_alpha)
            rid == route.id || continue
            new_key = (route_id, j_idx, k_idx)
            state.alpha_profile[new_key] = max(
                get(state.alpha_profile, new_key, 0.0),
                source_alpha[(rid, j_idx, k_idx)]
            )
        end
        return route_id, false
    end

    route_id = state.next_route_id
    state.next_route_id += 1
    state.routes_by_id[route_id] = _copy_route_with_id(route, route_id)
    state.signature_to_route_id[signature] = route_id
    state.provenance_by_route_id[route_id] = Set([provenance])
    if protect_direct
        push!(state.protected_route_ids, route_id)
        push!(state.direct_seed_route_ids, route_id)
    end

    for (rid, j_idx, k_idx) in keys(source_alpha)
        rid == route.id || continue
        state.alpha_profile[(route_id, j_idx, k_idx)] = source_alpha[(rid, j_idx, k_idx)]
    end

    return route_id, true
end

function _all_od_pairs(data::StationSelectionData)
    all_od_pairs = Set{Tuple{Int, Int}}()
    for scenario in data.scenarios
        union!(all_od_pairs, keys(compute_scenario_od_count(scenario)))
    end
    return all_od_pairs
end

function _all_valid_jk_pairs(data::StationSelectionData, max_walking_distance::Float64)
    valid_pairs = compute_valid_jk_pairs(_all_od_pairs(data), data, max_walking_distance)
    jk_global = Set{Tuple{Int, Int}}()
    for pairs in values(valid_pairs)
        union!(jk_global, pairs)
    end
    return jk_global
end

function _build_direct_route_variants(
    data::StationSelectionData,
    jk_global::Set{Tuple{Int, Int}},
    vehicle_capacity::Int
)::Tuple{Vector{RouteData}, Dict{NTuple{3, Int}, Float64}}
    routes = RouteData[]
    alpha = Dict{NTuple{3, Int}, Float64}()
    next_id = 1
    for (j_idx, k_idx) in sort!(collect(jk_global))
        tt = get_routing_cost(data, j_idx, k_idx)
        route = RouteData(next_id, [j_idx, k_idx], tt, [(j_idx, k_idx)])
        push!(routes, route)
        alpha[(next_id, j_idx, k_idx)] = Float64(vehicle_capacity)
        next_id += 1
    end
    return routes, alpha
end

function _merge_route_variants!(
    state::RoutePoolState,
    routes::Vector{RouteData},
    alpha_profile::Dict{NTuple{3, Int}, Float64},
    provenance::Symbol;
    protect_direct::Bool=false
)::Int
    added = 0
    for route in routes
        _, inserted = _insert_route_variant!(
            state,
            route,
            alpha_profile,
            provenance;
            protect_direct=protect_direct && length(route.station_indices) == 2
        )
        added += inserted ? 1 : 0
    end
    return added
end

function initialize_route_pool(
    init_spec::RoutePoolInitSpec,
    data::StationSelectionData;
    vehicle_capacity::Int,
    max_walking_distance::Float64,
    max_detour_time::Float64,
    max_detour_ratio::Float64,
    stop_dwell_time::Float64,
    initial_generated_max_route_length::Union{Int, Nothing}=nothing
)::RoutePoolState
    state = _empty_route_pool_state()
    jk_global = _all_valid_jk_pairs(data, max_walking_distance)

    direct_routes, direct_alpha = _build_direct_route_variants(data, jk_global, vehicle_capacity)
    _merge_route_variants!(state, direct_routes, direct_alpha, :direct_seed; protect_direct=true)

    if init_spec.mode in (:file, :combined)
        rio = load_routes_and_alpha(
            init_spec.routes_file,
            init_spec.alpha_profile_file,
            data;
            max_detour_time=max_detour_time,
            max_detour_ratio=max_detour_ratio
        )
        _merge_route_variants!(state, rio.routes, rio.alpha_profile, :file)
    end

    if init_spec.mode in (:generated, :combined)
        !isnothing(initial_generated_max_route_length) ||
            throw(ArgumentError("initial_generated_max_route_length is required for generated/combined route-pool initialization"))
        routes, alpha = generate_routes_and_alpha(
            data,
            jk_global;
            vehicle_capacity=vehicle_capacity,
            max_route_length=initial_generated_max_route_length,
            max_detour_time=max_detour_time,
            max_detour_ratio=max_detour_ratio,
            stop_dwell_time=stop_dwell_time
        )
        _merge_route_variants!(state, routes, alpha, :generated)
        state.current_generated_max_route_length = max(
            state.current_generated_max_route_length,
            initial_generated_max_route_length
        )
    end

    return state
end

function export_route_pool_state(
    state::RoutePoolState,
    output_dir::String;
    array_idx_to_station_id::Union{Vector{Int}, Nothing}=nothing
)
    mkpath(output_dir)

    routes_df = DataFrame(
        route_id = Int[],
        station_ids = String[],
        travel_time = Float64[],
        provenance = String[],
    )
    for route_id in sort!(collect(keys(state.routes_by_id)))
        route = state.routes_by_id[route_id]
        station_ids = isnothing(array_idx_to_station_id) ?
            route.station_indices :
            [array_idx_to_station_id[idx] for idx in route.station_indices]
        push!(routes_df, (
            route_id,
            join(station_ids, "|"),
            route.travel_time,
            join(sort!(string.(collect(get(state.provenance_by_route_id, route_id, Set{Symbol}())))), "|"),
        ))
    end
    CSV.write(joinpath(output_dir, "routes_input.csv"), routes_df)

    alpha_df = DataFrame(
        route_id = Int[],
        pickup_id = Int[],
        dropoff_id = Int[],
        value = Float64[],
    )
    for (route_id, pickup_id, dropoff_id) in sort!(collect(keys(state.alpha_profile)))
        exported_pickup = isnothing(array_idx_to_station_id) ? pickup_id : array_idx_to_station_id[pickup_id]
        exported_dropoff = isnothing(array_idx_to_station_id) ? dropoff_id : array_idx_to_station_id[dropoff_id]
        push!(alpha_df, (route_id, exported_pickup, exported_dropoff, state.alpha_profile[(route_id, pickup_id, dropoff_id)]))
    end
    CSV.write(joinpath(output_dir, "alpha_profile.csv"), alpha_df)

    summary = Dict(
        "n_routes" => length(state.routes_by_id),
        "n_alpha_entries" => length(state.alpha_profile),
        "current_generated_max_route_length" => state.current_generated_max_route_length,
    )
    open(joinpath(output_dir, "route_pool_summary.json"), "w") do io
        JSON.print(io, summary, 4)
    end
end
