export export_route_pool_state
export export_alpha_route_bucket_pools_state

function export_route_pool_state(
    state::RoutePoolState,
    output_dir::String;
    array_idx_to_station_id::Union{Vector{Int}, Nothing}=nothing
)
    mkpath(output_dir)

    routes_df = DataFrame(route_id=Int[], station_ids=String[], travel_time=Float64[], provenance=String[])
    for route_id in sort!(collect(keys(state.routes_by_id)))
        route = state.routes_by_id[route_id]
        station_ids = isnothing(array_idx_to_station_id) ? route.station_indices :
            [array_idx_to_station_id[idx] for idx in route.station_indices]
        push!(routes_df, (
            route_id,
            join(station_ids, "|"),
            route.travel_time,
            join(sort!(string.(collect(get(state.provenance_by_route_id, route_id, Set{Symbol}())))), "|"),
        ))
    end
    CSV.write(joinpath(output_dir, "routes_input.csv"), routes_df)

    alpha_df = DataFrame(route_id=Int[], pickup_id=Int[], dropoff_id=Int[], value=Float64[])
    for (route_id, pickup_id, dropoff_id) in sort!(collect(keys(state.alpha_profile)))
        exported_pickup  = isnothing(array_idx_to_station_id) ? pickup_id  : array_idx_to_station_id[pickup_id]
        exported_dropoff = isnothing(array_idx_to_station_id) ? dropoff_id : array_idx_to_station_id[dropoff_id]
        push!(alpha_df, (route_id, exported_pickup, exported_dropoff, state.alpha_profile[(route_id, pickup_id, dropoff_id)]))
    end
    CSV.write(joinpath(output_dir, "alpha_profile.csv"), alpha_df)

    summary = Dict(
        "scenario_idx" => state.scenario_idx,
        "time_id" => state.time_id,
        "n_routes" => length(state.routes_by_id),
        "n_alpha_entries" => length(state.alpha_profile),
        "n_valid_jk_pairs" => length(state.valid_jk_pairs),
        "x_candidate_count" => state.x_candidate_count,
        "current_generated_max_route_length" => state.current_generated_max_route_length,
    )
    open(joinpath(output_dir, "route_pool_summary.json"), "w") do io
        JSON.print(io, summary, 4)
    end
end

function export_alpha_route_bucket_pools_state(
    global_state::AlphaRouteBucketPoolsState,
    output_dir::String;
    array_idx_to_station_id::Union{Vector{Int}, Nothing}=nothing
)
    mkpath(output_dir)

    routes_df = DataFrame(
        scenario_idx=Int[], time_id=Int[], route_id=Int[],
        station_ids=String[], travel_time=Float64[], provenance=String[],
    )
    alpha_df = DataFrame(
        scenario_idx=Int[], time_id=Int[], route_id=Int[],
        pickup_id=Int[], dropoff_id=Int[], value=Float64[],
    )
    bucket_summary_df = DataFrame(
        scenario_idx=Int[], time_id=Int[], n_routes=Int[],
        n_alpha_entries=Int[], n_valid_jk_pairs=Int[],
        x_candidate_count=Int[], current_generated_max_route_length=Int[],
    )

    for (s, t_id) in _sorted_bucket_route_pool_keys(global_state)
        bucket_state = global_state.bucket_states[(s, t_id)]
        for route_id in sort!(collect(keys(bucket_state.routes_by_id)))
            route = bucket_state.routes_by_id[route_id]
            station_ids = isnothing(array_idx_to_station_id) ? route.station_indices :
                [array_idx_to_station_id[idx] for idx in route.station_indices]
            push!(routes_df, (
                s, t_id, route_id, join(station_ids, "|"), route.travel_time,
                join(sort!(string.(collect(get(bucket_state.provenance_by_route_id, route_id, Set{Symbol}())))), "|"),
            ))
        end
        for (route_id, pickup_id, dropoff_id) in sort!(collect(keys(bucket_state.alpha_profile)))
            exported_pickup  = isnothing(array_idx_to_station_id) ? pickup_id  : array_idx_to_station_id[pickup_id]
            exported_dropoff = isnothing(array_idx_to_station_id) ? dropoff_id : array_idx_to_station_id[dropoff_id]
            push!(alpha_df, (s, t_id, route_id, exported_pickup, exported_dropoff, bucket_state.alpha_profile[(route_id, pickup_id, dropoff_id)]))
        end
        push!(bucket_summary_df, (
            s, t_id,
            length(bucket_state.routes_by_id),
            length(bucket_state.alpha_profile),
            length(bucket_state.valid_jk_pairs),
            bucket_state.x_candidate_count,
            bucket_state.current_generated_max_route_length,
        ))
    end

    CSV.write(joinpath(output_dir, "routes_input.csv"), routes_df)
    CSV.write(joinpath(output_dir, "alpha_profile.csv"), alpha_df)
    CSV.write(joinpath(output_dir, "bucket_summary.csv"), bucket_summary_df)

    n_routes_total = sum(length(b.routes_by_id) for b in values(global_state.bucket_states))
    n_alpha_total  = sum(length(b.alpha_profile) for b in values(global_state.bucket_states))
    open(joinpath(output_dir, "route_pool_summary.json"), "w") do io
        JSON.print(io, Dict(
            "n_buckets" => length(global_state.bucket_states),
            "n_routes_total" => n_routes_total,
            "n_alpha_entries_total" => n_alpha_total,
        ), 4)
    end
    return nothing
end
