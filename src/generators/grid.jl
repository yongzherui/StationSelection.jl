using Random

"""
    GridStation

A station on a rectangular grid. Station ids use row-major order:
`id = (row - 1) * nx + col`.
"""
struct GridStation
    id::Int
    row::Int
    col::Int
end

"""
    GridInstance

Synthetic station-and-demand data for aggregate OD route experiments.
"""
struct GridInstance
    nx::Int
    ny::Int
    stations::Vector{GridStation}
    dist::Matrix{Float64}
    active_pairs::Vector{Tuple{Int,Int}}
    endpoint_overlap::Float64
    seed::Int
end

grid_station_id(row::Int, col::Int, nx::Int)::Int = (row - 1) * nx + col

grid_manhattan_dist(s1::GridStation, s2::GridStation)::Float64 =
    Float64(abs(s1.row - s2.row) + abs(s1.col - s2.col))

function _zipf_weights(n::Int, exponent::Float64)::Vector{Float64}
    exponent >= 0.0 || throw(ArgumentError("endpoint_overlap must be nonnegative"))
    return [1.0 / k^exponent for k in 1:n]
end

function _weighted_sample(rng::AbstractRNG, cumulative_weights::Vector{Float64})::Int
    return searchsortedfirst(cumulative_weights, rand(rng) * cumulative_weights[end])
end

"""
    generate_grid_instance(nx, ny, n_pairs; endpoint_overlap = 2.0, seed = 42)

Generate a grid instance with `n_pairs` distinct active OD pairs. Origins and
destinations are sampled from a randomly permuted Zipf ranking, so larger
`endpoint_overlap` values create more shared endpoints.
"""
function generate_grid_instance(
    nx::Int,
    ny::Int,
    n_pairs::Int;
    endpoint_overlap::Float64 = 2.0,
    seed::Int = 42,
)::GridInstance
    nx > 0 || throw(ArgumentError("nx must be positive"))
    ny > 0 || throw(ArgumentError("ny must be positive"))
    n_pairs > 0 || throw(ArgumentError("n_pairs must be positive"))

    n_stations = nx * ny
    max_distinct = n_stations * (n_stations - 1)
    n_pairs <= max_distinct || throw(ArgumentError(
        "n_pairs=$n_pairs exceeds the $max_distinct distinct OD pairs on a $(ny)x$(nx) grid",
    ))

    rng = Random.MersenneTwister(seed)
    stations = [GridStation(grid_station_id(r, c, nx), r, c) for r in 1:ny for c in 1:nx]

    dist = Matrix{Float64}(undef, n_stations, n_stations)
    for i in 1:n_stations, j in 1:n_stations
        dist[i, j] = grid_manhattan_dist(stations[i], stations[j])
    end

    rank_to_station = randperm(rng, n_stations)
    cumulative_weights = cumsum(_zipf_weights(n_stations, endpoint_overlap))
    active_pairs = Tuple{Int,Int}[]
    seen_pairs = Set{Tuple{Int,Int}}()

    while length(active_pairs) < n_pairs
        origin = rank_to_station[_weighted_sample(rng, cumulative_weights)]
        destination = rank_to_station[_weighted_sample(rng, cumulative_weights)]
        origin == destination && continue
        pair = (origin, destination)
        pair in seen_pairs && continue
        push!(seen_pairs, pair)
        push!(active_pairs, pair)
    end

    return GridInstance(nx, ny, stations, dist, active_pairs, endpoint_overlap, seed)
end

function grid_travel_cost_dict(instance::GridInstance)
    nodes = [station.id for station in instance.stations]
    travel_cost = Dict{Tuple{Int,Int},Float64}()
    for u in nodes, v in nodes
        travel_cost[(u, v)] = instance.dist[u, v]
    end
    return nodes, travel_cost
end

function _grid_station_frame(instance::GridInstance)::DataFrame
    return DataFrame(
        id = [station.id for station in instance.stations],
        lon = Float64[station.col for station in instance.stations],
        lat = Float64[station.row for station in instance.stations],
    )
end

function _grid_request_frame(
    instance::GridInstance;
    request_time::DateTime = DateTime(2026, 1, 1),
)::DataFrame
    return DataFrame(
        id = collect(1:length(instance.active_pairs)),
        origin_station_id = [origin for (origin, _) in instance.active_pairs],
        destination_station_id = [destination for (_, destination) in instance.active_pairs],
        request_time = fill(request_time, length(instance.active_pairs)),
    )
end

function _grid_walking_costs(
    instance::GridInstance;
    max_walking_distance::Float64,
    walking_cost_scale::Float64,
)::Dict{Tuple{Int,Int},Float64}
    costs = Dict{Tuple{Int,Int},Float64}()
    nodes = [station.id for station in instance.stations]
    for u in nodes, v in nodes
        dist = instance.dist[u, v]
        if dist <= max_walking_distance + 1e-9
            costs[(u, v)] = walking_cost_scale * dist
        end
    end
    return costs
end

"""
    create_grid_problem_data(instance; max_walking_distance, ...)

Convert a grid instance into `StationSelectionData`. Walking feasibility uses
Manhattan distance on the grid; routing costs are the full grid Manhattan costs.
"""
function create_grid_problem_data(
    instance::GridInstance;
    max_walking_distance::Float64,
    walking_cost_scale::Float64 = 1.0,
    routing_cost_scale::Float64 = 1.0,
    request_time::DateTime = DateTime(2026, 1, 1),
)::StationSelectionData
    max_walking_distance >= 0.0 || throw(ArgumentError("max_walking_distance must be nonnegative"))
    walking_cost_scale >= 0.0 || throw(ArgumentError("walking_cost_scale must be nonnegative"))
    routing_cost_scale >= 0.0 || throw(ArgumentError("routing_cost_scale must be nonnegative"))

    _, travel_cost = grid_travel_cost_dict(instance)
    routing_costs = Dict(key => routing_cost_scale * value for (key, value) in travel_cost)
    walking_costs = _grid_walking_costs(
        instance;
        max_walking_distance = max_walking_distance,
        walking_cost_scale = walking_cost_scale,
    )

    return create_station_selection_data(
        _grid_station_frame(instance),
        _grid_request_frame(instance; request_time = request_time),
        walking_costs;
        routing_costs = routing_costs,
    )
end

create_grid_station_selection_data(instance::GridInstance; kwargs...) =
    create_grid_problem_data(instance; kwargs...)

function print_grid_summary(instance::GridInstance)
    endpoint_freq = Dict{Int,Int}()
    for (origin, destination) in instance.active_pairs
        endpoint_freq[origin] = get(endpoint_freq, origin, 0) + 1
        endpoint_freq[destination] = get(endpoint_freq, destination, 0) + 1
    end

    @printf(
        "Grid instance n_stations=%d n_pairs=%d endpoint_overlap=%.2f seed=%d\n",
        length(instance.stations),
        length(instance.active_pairs),
        instance.endpoint_overlap,
        instance.seed,
    )
    for row in 1:instance.ny
        for col in 1:instance.nx
            sid = grid_station_id(row, col, instance.nx)
            count = get(endpoint_freq, sid, 0)
            label = count > 0 ? "$(sid)[$(count)]" : string(sid)
            print(rpad(label, 8))
        end
        println()
    end
end
