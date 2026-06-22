"""
    scripts/generate_zhuzhou_instance.jl

Generate Zhuzhou StationSelectionData on the fly from the base data files.
Include this file after `using StationSelection` to get:

  generate_zhuzhou_data(data_dir, n_stations, n_pairs; endpoint_overlap, seed)
      -> (StationSelectionData, NamedTuple)

  print_zhuzhou_data_summary(data, meta)

The generator mirrors the logic in CompatibilityCovering.jl/src/generators/zhuzhou_instance.jl
but produces StationSelectionData instead of a CompatibilityCovering NetworkPricingData.

Data files expected in data_dir:
  station_request_counts.csv  — station_id, station_name, station_lon, station_lat, pickup_count, dropoff_count
  segment.csv                 — id, from_station, to_station, seg_dist, seg_time
  order.csv                   — ..., origin_station_id, destination_station_id, ...
"""

using DataFrames, Dates, Random, Printf

# ── private loaders ───────────────────────────────────────────────────────────

function _zz_load_station_counts(data_dir::AbstractString)
    path = joinpath(data_dir, "station_request_counts.csv")
    rows = NamedTuple{(:id, :name, :lon, :lat, :total_count), Tuple{Int,String,Float64,Float64,Int}}[]
    open(path) do io
        readline(io)  # header: station_id,station_name,station_lon,station_lat,pickup_count,dropoff_count
        for line in eachline(io)
            parts = split(line, ',')
            length(parts) >= 6 || continue
            id      = parse(Int,     strip(parts[1]))
            name    =                strip(parts[2])
            lon     = parse(Float64, strip(parts[3]))
            lat     = parse(Float64, strip(parts[4]))
            pickup  = parse(Int,     strip(parts[5]))
            dropoff = parse(Int,     strip(parts[6]))
            push!(rows, (id=id, name=name, lon=lon, lat=lat, total_count=pickup + dropoff))
        end
    end
    sort!(rows, by=r -> r.total_count, rev=true)
    return rows
end

function _zz_load_valid_pairs(
    data_dir    :: AbstractString,
    station_set :: Set{Int},
)::Set{Tuple{Int,Int}}
    path  = joinpath(data_dir, "order.csv")
    valid = Set{Tuple{Int,Int}}()
    open(path) do io
        readline(io)  # header: order_id,region_id,pax_num,order_time,origin_station_id,destination_station_id,...
        for line in eachline(io)
            parts = split(line, ',')
            length(parts) >= 6 || continue
            origin = tryparse(Int, strip(parts[5]))
            dest   = tryparse(Int, strip(parts[6]))
            (isnothing(origin) || isnothing(dest)) && continue
            origin == dest           && continue
            !(origin in station_set) && continue
            !(dest   in station_set) && continue
            push!(valid, (origin, dest))
        end
    end
    return valid
end

_zz_zipf_weights(n::Int, s::Float64) = [1.0 / k^s for k in 1:n]

# ── main generator ────────────────────────────────────────────────────────────

"""
    generate_zhuzhou_data(data_dir, n_stations, n_pairs;
                          n_scenarios=1, endpoint_overlap=2.0, seed=42)
    -> (StationSelectionData, NamedTuple)

Build a Zhuzhou StationSelectionData:
  - `n_stations`:     top-N stations by total request volume (pickup + dropoff count)
  - `n_pairs`:        distinct (o,d) demand pairs per scenario
  - `n_scenarios`:    number of independent demand scenarios (each sampled separately)
  - `endpoint_overlap`: Zipf exponent (higher → demand concentrated on popular stations)

Stations are converted from BD-09 to WGS-84.
Walking costs:  Haversine distance / 1.4 m/s (seconds).
Routing costs:  Floyd-Warshall on segment travel times (seconds).
Only (o,d) pairs that appear in real order history are accepted.

Each scenario is sampled independently using seed + (s-1)*1000 as its RNG seed,
and placed in a 1-hour time window starting at 8am + (s-1) hours.

Returns (data, meta) where meta contains summary fields for logging/printing.
"""
function generate_zhuzhou_data(
    data_dir         :: AbstractString,
    n_stations       :: Int,
    n_pairs          :: Int;
    n_scenarios      :: Int     = 1,
    endpoint_overlap :: Float64 = 2.0,
    seed             :: Int     = 42,
)::Tuple{StationSelectionData, NamedTuple}
    n_stations   > 0 || throw(ArgumentError("n_stations must be positive"))
    n_pairs      > 0 || throw(ArgumentError("n_pairs must be positive"))
    n_scenarios  > 0 || throw(ArgumentError("n_scenarios must be positive"))
    endpoint_overlap >= 0 || throw(ArgumentError("endpoint_overlap must be non-negative"))

    # 1. Top-N stations by total request volume
    all_rows = _zz_load_station_counts(data_dir)
    n_stations <= length(all_rows) || throw(ArgumentError(
        "n_stations=$n_stations exceeds available stations ($(length(all_rows)))"
    ))
    top_rows    = all_rows[1:n_stations]
    station_set = Set{Int}(r.id for r in top_rows)
    station_ids = [r.id for r in top_rows]  # rank 1 = most popular

    # 2. BD-09 → WGS-84 coordinate conversion (same as read_candidate_stations)
    wgs_coords = [bd09_to_wgs84(r.lon, r.lat) for r in top_rows]
    stations   = DataFrame(
        id  = [r.id  for r in top_rows],
        lon = [c[1]  for c in wgs_coords],
        lat = [c[2]  for c in wgs_coords],
    )

    # 3. Walking costs: Haversine distance (metres) / 1.4 m/s → seconds
    walking_costs = compute_station_pairwise_costs(stations)

    # 4. Routing costs: Floyd-Warshall over segment travel times → seconds
    segment_file  = joinpath(data_dir, "segment.csv")
    routing_costs = read_routing_costs_from_segments(segment_file, stations)

    # 5. Valid OD pairs drawn from real order history
    valid_pairs = _zz_load_valid_pairs(data_dir, station_set)

    weights = _zz_zipf_weights(n_stations, endpoint_overlap)
    cumw    = cumsum(weights)
    max_misses = max(10_000, n_pairs * 500)

    # 6. Sample independently per scenario; each gets its own RNG seed
    all_requests   = DataFrame[]
    pairs_per_scenario = Int[]
    request_id     = 1

    for s in 1:n_scenarios
        rng    = Random.MersenneTwister(seed + (s - 1) * 1000)
        active = Tuple{Int,Int}[]
        seen   = Set{Tuple{Int,Int}}()
        misses = 0

        while length(active) < n_pairs && misses < max_misses
            oi = searchsortedfirst(cumw, rand(rng) * cumw[end])
            di = searchsortedfirst(cumw, rand(rng) * cumw[end])
            oi == di && (misses += 1; continue)
            pair = (station_ids[oi], station_ids[di])
            pair in seen       && (misses += 1; continue)
            pair ∉ valid_pairs && (misses += 1; continue)
            push!(seen, pair)
            push!(active, pair)
            misses = 0
        end

        n_actual = length(active)
        if n_actual < n_pairs
            @warn "Scenario $s: could only assemble $n_actual / $n_pairs valid OD pairs " *
                  "(n_stations=$n_stations, endpoint_overlap=$endpoint_overlap, seed=$(seed + (s-1)*1000))"
        end
        push!(pairs_per_scenario, n_actual)

        # Place requests in a 1-hour window starting at 8am + (s-1) hours
        t0 = DateTime(2024, 1, 1, 7 + s, 0, 0)
        df = DataFrame(
            id                     = request_id:(request_id + n_actual - 1),
            origin_station_id      = [p[1] for p in active],
            destination_station_id = [p[2] for p in active],
            request_time           = [t0 + Second(i) for i in 1:n_actual],
        )
        push!(all_requests, df)
        request_id += n_actual
    end

    requests = vcat(all_requests...)

    # 7. Build time window strings for scenario splitting
    _fmt_hour(h::Int) = lpad(h, 2, '0')
    scenario_windows = n_scenarios == 1 ? nothing : Tuple{String,String}[
        (
            "2024-01-01 $(_fmt_hour(7+s)):00:00",
            "2024-01-01 $(_fmt_hour(8+s)):00:00",
        )
        for s in 1:n_scenarios
    ]

    data = create_station_selection_data(
        stations, requests, walking_costs;
        routing_costs=routing_costs,
        scenarios=scenario_windows,
    )

    meta = (
        n_stations_requested = n_stations,
        n_stations_actual    = nrow(stations),
        n_scenarios_actual   = StationSelection.n_scenarios(data),
        n_pairs_requested    = n_pairs,
        pairs_per_scenario   = pairs_per_scenario,
        endpoint_overlap     = endpoint_overlap,
        seed                 = seed,
        station_ids          = station_ids,
        station_names        = [r.name for r in top_rows],
        station_counts       = [r.total_count for r in top_rows],
    )
    return data, meta
end

# ── diagnostic summary ────────────────────────────────────────────────────────

function print_zhuzhou_data_summary(data::StationSelectionData, meta::NamedTuple)
    @printf("Zhuzhou instance  n_stations=%d  n_scenarios=%d  endpoint_overlap=%.2f  seed=%d\n",
            meta.n_stations_actual, meta.n_scenarios_actual, meta.endpoint_overlap, meta.seed)
    for (s, n_actual) in enumerate(meta.pairs_per_scenario)
        if n_actual < meta.n_pairs_requested
            @printf("  WARNING: scenario %d assembled only %d / %d pairs\n",
                    s, n_actual, meta.n_pairs_requested)
        end
    end
    @printf("  Pairs per scenario: %s  (requested %d each)\n",
            join(meta.pairs_per_scenario, ", "), meta.n_pairs_requested)

    rout = data.routing_costs
    if !isnothing(rout)
        n = data.n_stations
        finite_t = [rout[i,j] for i in 1:n, j in 1:n if i != j && isfinite(rout[i,j])]
        if !isempty(finite_t)
            @printf("  Routing times    : min=%.1fs  mean=%.1fs  max=%.1fs\n",
                    minimum(finite_t), sum(finite_t)/length(finite_t), maximum(finite_t))
        end
    end

    walk = data.walking_costs
    n    = data.n_stations
    finite_w = [walk[i,j] for i in 1:n, j in 1:n if i != j && isfinite(walk[i,j])]
    if !isempty(finite_w)
        @printf("  Walking times    : min=%.1fs  mean=%.1fs  max=%.1fs\n",
                minimum(finite_w), sum(finite_w)/length(finite_w), maximum(finite_w))
    end

    println()
    println("  Stations (ranked by request volume):")
    for (rank, (id, name, count)) in enumerate(
            zip(meta.station_ids, meta.station_names, meta.station_counts))
        @printf("    %2d.  id=%-4d  count=%-5d  %s\n", rank, id, count, name)
    end
    println()
end
