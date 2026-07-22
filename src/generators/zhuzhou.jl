"""
    ZhuzhouStation

A real station from the Zhuzhou microtransit network, annotated with total
historical request volume.
"""
struct ZhuzhouStation
    id::Int
    name::String
    lon::Float64
    lat::Float64
    total_count::Int
end

"""
    ZhuzhouInstance

Real-world test case drawn from Zhuzhou station, segment, and order CSV files.
`active_pairs` may either be sampled distinct OD pairs or the request sequence
observed inside a time window.
"""
struct ZhuzhouInstance
    stations::Vector{ZhuzhouStation}
    travel_time::Dict{Tuple{Int,Int},Float64}
    active_pairs::Vector{Tuple{Int,Int}}
    endpoint_overlap::Float64
    seed::Int
end

function _parse_zhuzhou_datetime(value::AbstractString)::DateTime
    stripped = strip(value)
    isempty(stripped) && throw(ArgumentError("empty timestamp value"))

    formats = (
        dateformat"yyyy-mm-dd HH:MM:SS",
        dateformat"yyyy-mm-ddTHH:MM:SS",
        dateformat"yyyy/mm/dd HH:MM:SS",
        dateformat"yyyy-mm-dd HH:MM",
        dateformat"yyyy/mm/dd HH:MM",
        dateformat"yyyy-mm-dd",
        dateformat"yyyy/mm/dd",
    )
    for format in formats
        try
            return DateTime(stripped, format)
        catch
        end
    end
    return DateTime(stripped)
end

_as_zhuzhou_datetime(value::DateTime) = value
_as_zhuzhou_datetime(value::Date) = DateTime(value)
_as_zhuzhou_datetime(value::AbstractString) = _parse_zhuzhou_datetime(value)

function _column_index(header::AbstractVector{<:AbstractString}, candidates; default::Union{Nothing,Int} = nothing)
    normalized = Dict(lowercase(strip(name)) => i for (i, name) in enumerate(header))
    for candidate in candidates
        key = lowercase(strip(String(candidate)))
        haskey(normalized, key) && return normalized[key]
    end
    isnothing(default) || return default
    throw(ArgumentError("could not find any order.csv column named $(collect(candidates))"))
end

function _load_zhuzhou_stations(data_dir::AbstractString)::Vector{ZhuzhouStation}
    path = joinpath(data_dir, "station_request_counts.csv")
    isfile(path) || throw(ArgumentError("missing Zhuzhou station file: $path"))

    stations = ZhuzhouStation[]
    open(path) do io
        readline(io)
        for line in eachline(io)
            isempty(strip(line)) && continue
            parts = split(line, ',')
            length(parts) >= 6 || throw(ArgumentError("expected at least 6 columns in $path"))
            pickup = parse(Int, parts[5])
            dropoff = parse(Int, parts[6])
            push!(
                stations,
                ZhuzhouStation(
                    parse(Int, parts[1]),
                    parts[2],
                    parse(Float64, parts[3]),
                    parse(Float64, parts[4]),
                    pickup + dropoff,
                ),
            )
        end
    end
    return stations
end

function _load_zhuzhou_travel_times(
    data_dir::AbstractString,
    station_set::Set{Int},
)::Dict{Tuple{Int,Int},Float64}
    path = joinpath(data_dir, "segment.csv")
    isfile(path) || throw(ArgumentError("missing Zhuzhou segment file: $path"))

    travel_time = Dict{Tuple{Int,Int},Float64}()
    open(path) do io
        readline(io)
        for line in eachline(io)
            isempty(strip(line)) && continue
            parts = split(line, ',')
            length(parts) >= 5 || throw(ArgumentError("expected at least 5 columns in $path"))
            from = parse(Int, parts[2])
            to = parse(Int, parts[3])
            if from in station_set && to in station_set
                travel_time[(from, to)] = parse(Float64, parts[5])
            end
        end
    end
    return travel_time
end

function _load_zhuzhou_valid_pairs(
    data_dir::AbstractString,
    station_set::Set{Int},
)::Set{Tuple{Int,Int}}
    path = joinpath(data_dir, "order.csv")
    isfile(path) || throw(ArgumentError("missing Zhuzhou order file: $path"))

    valid_pairs = Set{Tuple{Int,Int}}()
    open(path) do io
        readline(io)
        for line in eachline(io)
            isempty(strip(line)) && continue
            parts = split(line, ',')
            length(parts) >= 6 || throw(ArgumentError("expected at least 6 columns in $path"))
            origin = parse(Int, parts[5])
            destination = parse(Int, parts[6])
            origin == destination && continue
            origin in station_set || continue
            destination in station_set || continue
            push!(valid_pairs, (origin, destination))
        end
    end
    return valid_pairs
end

function _load_zhuzhou_pairs_in_window(
    data_dir::AbstractString,
    station_set::Set{Int},
    start_time::DateTime,
    end_time::DateTime;
    timestamp_col = nothing,
    origin_col = nothing,
    destination_col = nothing,
    unique_pairs::Bool = false,
    max_pairs::Union{Nothing,Int} = nothing,
)::Vector{Tuple{Int,Int}}
    start_time <= end_time || throw(ArgumentError("start_time must be <= end_time"))
    isnothing(max_pairs) || max_pairs > 0 || throw(ArgumentError("max_pairs must be positive when provided"))

    path = joinpath(data_dir, "order.csv")
    isfile(path) || throw(ArgumentError("missing Zhuzhou order file: $path"))

    pairs = Tuple{Int,Int}[]
    seen = Set{Tuple{Int,Int}}()
    open(path) do io
        header = split(readline(io), ',')
        ts_idx = isnothing(timestamp_col) ?
            _column_index(
                header,
                (
                    "timestamp",
                    "time",
                    "datetime",
                    "date_time",
                    "order_time",
                    "request_time",
                    "created_at",
                    "create_time",
                    "departure_time",
                    "pickup_time",
                ),
            ) :
            (timestamp_col isa Integer ? timestamp_col : _column_index(header, (timestamp_col,)))
        origin_idx = isnothing(origin_col) ?
            _column_index(header, ("origin", "origin_station", "origin_station_id", "pickup_station_id"), default = 5) :
            (origin_col isa Integer ? origin_col : _column_index(header, (origin_col,)))
        destination_idx = isnothing(destination_col) ?
            _column_index(header, ("destination", "destination_station", "destination_station_id", "dropoff_station_id"), default = 6) :
            (destination_col isa Integer ? destination_col : _column_index(header, (destination_col,)))

        for line in eachline(io)
            isempty(strip(line)) && continue
            parts = split(line, ',')
            maximum((ts_idx, origin_idx, destination_idx)) <= length(parts) ||
                throw(ArgumentError("order row has fewer columns than required in $path"))

            timestamp = _parse_zhuzhou_datetime(parts[ts_idx])
            start_time <= timestamp <= end_time || continue

            origin = parse(Int, parts[origin_idx])
            destination = parse(Int, parts[destination_idx])
            origin == destination && continue
            origin in station_set || continue
            destination in station_set || continue

            pair = (origin, destination)
            if unique_pairs
                pair in seen && continue
                push!(seen, pair)
            end
            push!(pairs, pair)
            !isnothing(max_pairs) && length(pairs) >= max_pairs && break
        end
    end

    return pairs
end

"""
    generate_zhuzhou_instance(data_dir, n_stations, n_pairs; endpoint_overlap = 2.0, seed = 42)

Select the top `n_stations` by historical request volume and sample `n_pairs`
historical OD pairs using Zipf-weighted endpoint sampling.
"""
function generate_zhuzhou_instance(
    data_dir::AbstractString,
    n_stations::Int,
    n_pairs::Int;
    endpoint_overlap::Float64 = 2.0,
    seed::Int = 42,
)::ZhuzhouInstance
    n_stations > 0 || throw(ArgumentError("n_stations must be positive"))
    n_pairs > 0 || throw(ArgumentError("n_pairs must be positive"))

    all_stations = _load_zhuzhou_stations(data_dir)
    sort!(all_stations, by = station -> station.total_count, rev = true)
    n_stations <= length(all_stations) || throw(ArgumentError(
        "n_stations=$n_stations exceeds available stations ($(length(all_stations)))",
    ))

    selected = all_stations[1:n_stations]
    station_set = Set(station.id for station in selected)
    station_ids = [station.id for station in selected]
    travel_time = _load_zhuzhou_travel_times(data_dir, station_set)
    valid_pairs = _load_zhuzhou_valid_pairs(data_dir, station_set)

    rng = Random.MersenneTwister(seed)
    cumulative_weights = cumsum(_zipf_weights(n_stations, endpoint_overlap))
    active_pairs = Tuple{Int,Int}[]
    seen_pairs = Set{Tuple{Int,Int}}()

    max_misses = max(10_000, n_pairs * 500)
    misses = 0
    while length(active_pairs) < n_pairs && misses < max_misses
        origin = station_ids[_weighted_sample(rng, cumulative_weights)]
        destination = station_ids[_weighted_sample(rng, cumulative_weights)]
        origin == destination && (misses += 1; continue)

        pair = (origin, destination)
        pair in seen_pairs && (misses += 1; continue)
        pair in valid_pairs || (misses += 1; continue)

        push!(seen_pairs, pair)
        push!(active_pairs, pair)
        misses = 0
    end

    if length(active_pairs) < n_pairs
        @warn "Could only assemble $(length(active_pairs)) valid Zhuzhou OD pairs" n_stations endpoint_overlap requested = n_pairs
    end

    return ZhuzhouInstance(selected, travel_time, active_pairs, endpoint_overlap, seed)
end

"""
    generate_zhuzhou_instance(data_dir, n_stations, start_time, end_time; ...)

Select the top `n_stations` by historical request volume and use orders whose
timestamp falls in `[start_time, end_time]` as the request list. By default,
repeated OD pairs are preserved because they represent separate requests.
"""
function generate_zhuzhou_instance(
    data_dir::AbstractString,
    n_stations::Int,
    start_time,
    end_time;
    timestamp_col = nothing,
    origin_col = nothing,
    destination_col = nothing,
    unique_pairs::Bool = false,
    max_pairs::Union{Nothing,Int} = nothing,
)::ZhuzhouInstance
    n_stations > 0 || throw(ArgumentError("n_stations must be positive"))

    all_stations = _load_zhuzhou_stations(data_dir)
    sort!(all_stations, by = station -> station.total_count, rev = true)
    n_stations <= length(all_stations) || throw(ArgumentError(
        "n_stations=$n_stations exceeds available stations ($(length(all_stations)))",
    ))

    selected = all_stations[1:n_stations]
    station_set = Set(station.id for station in selected)
    travel_time = _load_zhuzhou_travel_times(data_dir, station_set)
    active_pairs = _load_zhuzhou_pairs_in_window(
        data_dir,
        station_set,
        _as_zhuzhou_datetime(start_time),
        _as_zhuzhou_datetime(end_time);
        timestamp_col = timestamp_col,
        origin_col = origin_col,
        destination_col = destination_col,
        unique_pairs = unique_pairs,
        max_pairs = max_pairs,
    )
    isempty(active_pairs) && throw(ArgumentError(
        "no Zhuzhou orders found in the requested time window for the selected stations",
    ))

    return ZhuzhouInstance(selected, travel_time, active_pairs, NaN, 0)
end

function _haversine_m(lat1::Float64, lon1::Float64, lat2::Float64, lon2::Float64)::Float64
    radius_m = 6_371_000.0
    phi1 = lat1 * pi / 180
    phi2 = lat2 * pi / 180
    delta_phi = (lat2 - lat1) * pi / 180
    delta_lambda = (lon2 - lon1) * pi / 180
    a = sin(delta_phi / 2)^2 + cos(phi1) * cos(phi2) * sin(delta_lambda / 2)^2
    return 2 * radius_m * asin(sqrt(a))
end

function _zhuzhou_station_frame(instance::ZhuzhouInstance)::DataFrame
    return DataFrame(
        id = [station.id for station in instance.stations],
        lon = [station.lon for station in instance.stations],
        lat = [station.lat for station in instance.stations],
        name = [station.name for station in instance.stations],
        total_count = [station.total_count for station in instance.stations],
    )
end

function _zhuzhou_request_frame(
    instance::ZhuzhouInstance;
    request_time::DateTime = DateTime(2026, 1, 1),
)::DataFrame
    return DataFrame(
        id = collect(1:length(instance.active_pairs)),
        origin_station_id = [origin for (origin, _) in instance.active_pairs],
        destination_station_id = [destination for (_, destination) in instance.active_pairs],
        request_time = fill(request_time, length(instance.active_pairs)),
    )
end

function _zhuzhou_walking_costs(
    instance::ZhuzhouInstance;
    max_walking_distance::Float64,
    walking_speed::Float64,
    walking_cost_scale::Float64,
)::Dict{Tuple{Int,Int},Float64}
    station_by_id = Dict(station.id => station for station in instance.stations)
    nodes = collect(keys(station_by_id))
    costs = Dict{Tuple{Int,Int},Float64}()
    for u in nodes, v in nodes
        from_station = station_by_id[u]
        to_station = station_by_id[v]
        walk_seconds = _haversine_m(
            from_station.lat,
            from_station.lon,
            to_station.lat,
            to_station.lon,
        ) / walking_speed
        if walk_seconds <= max_walking_distance + 1e-9
            costs[(u, v)] = walking_cost_scale * walk_seconds
        end
    end
    return costs
end

"""
    create_zhuzhou_problem_data(instance; max_walking_distance, ...)

Convert a Zhuzhou instance into `StationSelectionData`. Walking feasibility is
measured in seconds, using haversine distance divided by `walking_speed`.
"""
function create_zhuzhou_problem_data(
    instance::ZhuzhouInstance;
    max_walking_distance::Float64,
    walking_speed::Float64 = 1.4,
    walking_cost_scale::Float64 = 1.0,
    routing_cost_scale::Float64 = 1.0,
    request_time::DateTime = DateTime(2026, 1, 1),
)::StationSelectionData
    max_walking_distance >= 0.0 || throw(ArgumentError("max_walking_distance must be nonnegative"))
    walking_speed > 0.0 || throw(ArgumentError("walking_speed must be positive"))
    walking_cost_scale >= 0.0 || throw(ArgumentError("walking_cost_scale must be nonnegative"))
    routing_cost_scale >= 0.0 || throw(ArgumentError("routing_cost_scale must be nonnegative"))

    routing_costs = Dict(key => routing_cost_scale * value for (key, value) in instance.travel_time)
    walking_costs = _zhuzhou_walking_costs(
        instance;
        max_walking_distance = max_walking_distance,
        walking_speed = walking_speed,
        walking_cost_scale = walking_cost_scale,
    )

    return create_station_selection_data(
        _zhuzhou_station_frame(instance),
        _zhuzhou_request_frame(instance; request_time = request_time),
        walking_costs;
        routing_costs = routing_costs,
    )
end

create_zhuzhou_station_selection_data(instance::ZhuzhouInstance; kwargs...) =
    create_zhuzhou_problem_data(instance; kwargs...)

function print_zhuzhou_summary(instance::ZhuzhouInstance)
    endpoint_freq = Dict{Int,Int}()
    for (origin, destination) in instance.active_pairs
        endpoint_freq[origin] = get(endpoint_freq, origin, 0) + 1
        endpoint_freq[destination] = get(endpoint_freq, destination, 0) + 1
    end

    @printf(
        "Zhuzhou instance n_stations=%d n_pairs=%d endpoint_overlap=%.2f seed=%d\n",
        length(instance.stations),
        length(instance.active_pairs),
        instance.endpoint_overlap,
        instance.seed,
    )
    for (rank, station) in enumerate(instance.stations)
        endpoint_count = get(endpoint_freq, station.id, 0)
        @printf(
            "  %2d. id=%-6d count=%-8d endpoints=%-4d %s\n",
            rank,
            station.id,
            station.total_count,
            endpoint_count,
            station.name,
        )
    end
end
