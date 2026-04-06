"""
CSV loading of pre-built routes and alpha capacity profiles for RouteVehicleCapacityModel.

Input file formats (cross-compatible with existing variable exports):

  routes_input.csv:  route_id, station_ids (pipe-sep), travel_time
  alpha_profile.csv: route_id, pickup_id, dropoff_id, value

`routes_input.csv` can be derived from `theta_r_ts.csv` by deduplicating on station_ids:

    theta = CSV.read("theta_r_ts.csv", DataFrame)
    routes = unique(theta[:, [:station_ids, :travel_time]])
    insertcols!(routes, 1, :route_id => 1:nrow(routes))
    CSV.write("routes_input.csv", routes[:, [:route_id, :station_ids, :travel_time]])

`alpha_profile.csv` can be derived from `alpha_r_jkts.csv` by joining with the route
deduplication map and aggregating (max) per (route_id, pickup_id, dropoff_id):

    alpha = CSV.read("alpha_r_jkts.csv", DataFrame)
    # join on route station_ids to obtain route_id for each alpha row, then:
    profile = combine(groupby(merged, [:route_id, :pickup_id, :dropoff_id]),
                      :value => maximum => :value)
    CSV.write("alpha_profile.csv", profile)
"""

export RouteIOData, load_routes_and_alpha


"""
    RouteIOData

Parsed routes and alpha capacity profile loaded from CSV files.

# Fields
- `routes`: Vector of `RouteData` objects; each `RouteData.id` equals the CSV `route_id`.
- `alpha_profile`: Dict mapping `(route_id, pickup_idx, dropoff_idx)` → Float64
  alpha value for use as a warm-start hint. CSV station IDs are converted at load time.
"""
struct RouteIOData
    routes        :: Vector{RouteData}
    alpha_profile :: Dict{NTuple{3, Int}, Float64}   # (route_id, pickup_idx, dropoff_idx)
end


"""
    load_routes_and_alpha(routes_file, alpha_profile_file, data; ...) -> RouteIOData

Parse `routes_input.csv` and, optionally, `alpha_profile.csv`.

Each route's `detour_feasible_legs` is recomputed using the same logic as
`generate_simple_routes`: a leg (j→k) is feasible iff the in-vehicle detour
satisfies both `max_detour_time` and `max_detour_ratio`.

Routes are skipped (with a warning) if:
- they reference station IDs not present in `data`
- they have fewer than 2 stops

Alpha profile rows whose `route_id` has no matching route are skipped silently.
If duplicate keys appear in the alpha profile the maximum value is kept.
"""
function load_routes_and_alpha(
    routes_file        :: String,
    alpha_profile_file :: Union{String, Nothing},
    data               :: StationSelectionData;
    max_detour_time    :: Float64 = Inf,
    max_detour_ratio   :: Float64 = Inf
) :: RouteIOData

    # ── Load routes ───────────────────────────────────────────────────────────
    routes_df = CSV.read(routes_file, DataFrame)
    routes         = RouteData[]
    valid_route_ids = Set{Int}()

    for row in eachrow(routes_df)
        route_id = Int(row.route_id)

        # Parse pipe-separated station IDs (strip whitespace around each token),
        # then convert to compact station indices for internal use.
        station_ids = [parse(Int, strip(s)) for s in split(string(row.station_ids), '|')]

        if length(station_ids) < 2
            @warn "Route $route_id has fewer than 2 stops — skipping"
            continue
        end
        bad = filter(sid -> !haskey(data.station_id_to_array_idx, sid), station_ids)
        if !isempty(bad)
            @warn "Route $route_id references unknown station IDs $bad — skipping"
            continue
        end
        station_indices = [data.station_id_to_array_idx[sid] for sid in station_ids]

        travel_time = Float64(row.travel_time)

        # Compute detour_feasible_legs: same logic as generate_simple_routes DFS recorder
        m   = length(station_indices)
        seg = Vector{Float64}(undef, m - 1)
        for i in 1:(m - 1)
            seg[i] = get_routing_cost(data, station_indices[i], station_indices[i + 1])
        end
        feasible_legs = Tuple{Int, Int}[]
        for i in 1:m
            cum = 0.0
            for j in (i + 1):m
                cum   += seg[j - 1]
                direct = get_routing_cost(data, station_indices[i], station_indices[j])
                if (cum - direct <= max_detour_time) &&
                   (direct == 0.0 || cum / direct <= 1.0 + max_detour_ratio)
                    push!(feasible_legs, (station_indices[i], station_indices[j]))
                end
            end
        end

        push!(routes, RouteData(route_id, station_indices, travel_time, feasible_legs))
        push!(valid_route_ids, route_id)
    end

    n_skipped_routes = nrow(routes_df) - length(routes)
    if n_skipped_routes > 0
        println("  Loaded $(length(routes)) routes from CSV ($n_skipped_routes skipped)")
    else
        println("  Loaded $(length(routes)) routes from CSV")
    end
    flush(stdout)

    # ── Load alpha profile ────────────────────────────────────────────────────
    alpha_profile = Dict{NTuple{3, Int}, Float64}()

    if !isnothing(alpha_profile_file)
        alpha_df        = CSV.read(alpha_profile_file, DataFrame)
        n_loaded        = 0
        n_skipped_alpha = 0

        for row in eachrow(alpha_df)
            rid = Int(row.route_id)
            if rid ∉ valid_route_ids
                n_skipped_alpha += 1
                continue
            end
            pickup_id = Int(row.pickup_id)
            dropoff_id = Int(row.dropoff_id)
            haskey(data.station_id_to_array_idx, pickup_id) || continue
            haskey(data.station_id_to_array_idx, dropoff_id) || continue
            key = (rid, data.station_id_to_array_idx[pickup_id], data.station_id_to_array_idx[dropoff_id])
            # Take maximum if the same key appears more than once
            alpha_profile[key] = max(get(alpha_profile, key, 0.0), Float64(row.value))
            n_loaded += 1
        end

        if n_skipped_alpha > 0
            println("  Loaded $n_loaded alpha profile entries ($n_skipped_alpha skipped — route_id not in routes_input)")
        else
            println("  Loaded $n_loaded alpha profile entries")
        end
        flush(stdout)
    end

    return RouteIOData(routes, alpha_profile)
end
