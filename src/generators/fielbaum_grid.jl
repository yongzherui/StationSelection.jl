using Dates
using Printf
using Random

# =============================================================================
# Fielbaum et al. (2021) toy grid network
# =============================================================================
#
# Fielbaum, A., Bai, X., & Alonso-Mora, J. (2021). "On-demand ridesharing with
# optimized pick-up and drop-off walking locations." Transportation Research
# Part C 126, 103061. Section 3.1 ("Results over a toy network") and Fig. 4.
#
# The PDF is NOT in this repository. It sits in the sibling checkout at
# `MicroTransitSimulator.jl/docs/references/fielbaum-2021-pudo-walking.pdf`,
# itself untracked (copyrighted article); that directory's README says how to
# re-fetch it from the DOI. Section 3.1 is pages 13-16, Fig. 4 is on page 13.
#
# The paper's verbatim network specification (Section 3.1, first paragraph):
#
#   "The base graph we use is as follows: a 10x10 grid, in which streets can be
#    slow (20 km/h), mid-speed (30 km/h), or fast (40 km/h). Mid-speed streets
#    are unidirectional, and the others are bidirectional. All the arcs are
#    assumed to have the same length (0.15 km), so walking times are the same
#    everywhere; walking speed is 5 [km/h]."
#
# The *which street is which* part of that specification lives only in Fig. 4
# (green = fast, grey = mid-speed, red = slow; grey arcs carry a single
# arrowhead giving their direction). `FIELBAUM_BASE_ROW_STREETS` and
# `FIELBAUM_BASE_COL_STREETS` below transcribe that figure. Each street is
# homogeneous along its whole length in the figure, so one entry per row and one
# per column fully determines the network.
#
# HOW that transcription was made, and how to re-verify it, is recorded in
# `notes/2026-09-14_fielbaum_grid_figure_transcription.md` -- read it before
# changing either street vector, because the paper's text cannot settle them.
# That note also lists the three places the paper is silent and this generator
# had to interpret (the non-uniform spacing profile, concentrated-demand
# collisions, and grid sizes other than 10x10).
#
# Why this generator exists separately from `generators/grid.jl`: the plain grid
# generator gives every arc the same cost, so a station's attractiveness depends
# only on where it sits. Here the *speed* varies, which is what makes staying on
# a fast street worth a detour -- the effect Fielbaum et al. use to motivate
# coordinating vehicles and walking passengers. Costs are therefore emitted in
# MINUTES, not metres: in metres the speed hierarchy is invisible.

"""
    FielbaumStreet

One street of the Fielbaum et al. (2021) toy grid: its speed class and, for the
unidirectional mid-speed streets, which way it runs.

# Fields
- `class::Symbol`: `:slow` (20 km/h), `:mid` (30 km/h) or `:fast` (40 km/h).
- `direction::Int`: `0` bidirectional, `+1` one-way in the direction of
  increasing node index, `-1` one-way in the direction of decreasing node index.
  For a row street the index is the column, so `+1` is eastbound (rightwards in
  Fig. 4); for a column street the index is the row, so `+1` is southbound
  (downwards in Fig. 4, since row 1 is the top row).

The paper requires `class == :mid` to be exactly the unidirectional case;
`fielbaum_validate_streets` enforces that.
"""
struct FielbaumStreet
    class::Symbol
    direction::Int
end

"""
Speed of each street class in km/h, from Fielbaum et al. (2021) Section 3.1.
"""
const FIELBAUM_SPEEDS_KMH = (slow = 20.0, mid = 30.0, fast = 40.0)

"""Walking speed in km/h (Fielbaum et al. 2021, Section 3.1)."""
const FIELBAUM_WALKING_SPEED_KMH = 5.0

"""Length of every arc of the base grid in metres (0.15 km in the paper)."""
const FIELBAUM_LINK_LENGTH_M = 150.0

"""Node spacing at the centre of the `:non_uniform` network variant, in metres."""
const FIELBAUM_NON_UNIFORM_CENTER_SPACING_M = 75.0

"""Node spacing at the edges of the `:non_uniform` network variant, in metres."""
const FIELBAUM_NON_UNIFORM_EDGE_SPACING_M = 300.0

"""
Maximum walking time per leg, minutes (the paper's `Ω_a`).

The one paper bound this package can act on: pass it as
`StationSelectionProblem.max_walking_distance` *and* as
`create_fielbaum_grid_problem_data`'s `max_walking_time` so the two agree on
which pairs are walkable. It is a time, not a distance -- see that function's
docstring for why this instance measures everything in minutes.
"""
const FIELBAUM_MAX_WALKING_TIME_MIN = 12.0

"""
Maximum waiting time, minutes (the paper's `Ω_w`).

Transcribed for callers; **nothing in StationSelection.jl reads it.** It is a
dispatch-time bound, so its consumer is MicroTransitSimulator.jl's dynamic PUDO
path, not any formulation here.
"""
const FIELBAUM_MAX_WAITING_TIME_MIN = 5.0

"""
Maximum delay, minutes (the paper's `Ω_d`).

Transcribed for callers; **nothing in StationSelection.jl reads it.** Like
[`FIELBAUM_MAX_WAITING_TIME_MIN`](@ref) it is a dispatch-time bound and belongs
to the simulator.
"""
const FIELBAUM_MAX_DELAY_MIN = 10.0

"""
Unit cost of walking relative to one minute of in-vehicle time: the paper sets
`p_a = p_w = 2 p_v`, so walking a minute hurts twice as much as riding a minute.

Transcribed for callers; **nothing in StationSelection.jl reads it.** Pass it
yourself as `walk_cost_weight` to the `AggregateODRoute*` formulations when both
cost matrices come from this generator -- the formulations default to their own
weight and will not pick this up.
"""
const FIELBAUM_WALK_COST_WEIGHT = 2.0

"""
The ten horizontal streets of Fig. 4, transcribed top row first.

Row 1 is the top row of the figure. `+1` is eastbound. Reading the figure:
mid-speed one-ways alternate direction down the grid (rows 1 and 8 eastbound,
rows 4 and 10 westbound), the three fast streets are rows 2, 6 and 9, and the
three slow streets are rows 3, 5 and 7.
"""
const FIELBAUM_BASE_ROW_STREETS = FielbaumStreet[
    FielbaumStreet(:mid, +1),   # row 1  -- one-way eastbound
    FielbaumStreet(:fast, 0),   # row 2
    FielbaumStreet(:slow, 0),   # row 3
    FielbaumStreet(:mid, -1),   # row 4  -- one-way westbound
    FielbaumStreet(:slow, 0),   # row 5
    FielbaumStreet(:fast, 0),   # row 6
    FielbaumStreet(:slow, 0),   # row 7
    FielbaumStreet(:mid, +1),   # row 8  -- one-way eastbound
    FielbaumStreet(:fast, 0),   # row 9
    FielbaumStreet(:mid, -1),   # row 10 -- one-way westbound
]

"""
The ten vertical streets of Fig. 4, transcribed left column first.

Column 1 is the left column. `+1` is southbound (towards row 10), `-1`
northbound. Reading the figure: the fast streets are columns 3, 6 and 9, the
slow streets are columns 1, 8 and 10, and the mid-speed one-ways are columns 2
and 5 (northbound) and 4 and 7 (southbound).
"""
const FIELBAUM_BASE_COL_STREETS = FielbaumStreet[
    FielbaumStreet(:slow, 0),   # col 1
    FielbaumStreet(:mid, -1),   # col 2  -- one-way northbound
    FielbaumStreet(:fast, 0),   # col 3
    FielbaumStreet(:mid, +1),   # col 4  -- one-way southbound
    FielbaumStreet(:mid, -1),   # col 5  -- one-way northbound
    FielbaumStreet(:fast, 0),   # col 6
    FielbaumStreet(:mid, +1),   # col 7  -- one-way southbound
    FielbaumStreet(:slow, 0),   # col 8
    FielbaumStreet(:fast, 0),   # col 9
    FielbaumStreet(:slow, 0),   # col 10
]

"""
    FielbaumGridInstance

The Fielbaum et al. (2021) toy grid, with its arc-level speed structure kept
intact alongside the all-pairs matrices derived from it.

# Fields
- `ny`, `nx`: grid size (10 x 10 in the paper).
- `stations::Vector{GridStation}`: nodes, ids in row-major order (row 1 = top).
- `node_x`, `node_y`: planar coordinates in metres, one entry per station index.
  `node_x` grows eastwards (with the column index) and `node_y` grows
  northwards, so row 1 carries the *largest* `node_y` and a `lon`/`lat` scatter
  of the stations comes out the same way up as Fig. 4.
- `row_streets`, `col_streets`: the per-street speed class and direction.
- `arc_length`: directed single-arc length in metres, keyed by station id pair.
- `arc_time`: directed single-arc drive time in minutes, keyed by station id pair.
  Only adjacent pairs that the one-way rules permit appear in either dict.
- `drive_time`: all-pairs shortest **directed** drive time in minutes, indexed by
  station id. Asymmetric wherever a one-way street forces a longer return leg.
- `walk_distance`: all-pairs walking distance in metres. Pedestrians use the same
  streets but ignore the one-way rules, so this is symmetric.
- `walk_time`: `walk_distance` at `FIELBAUM_WALKING_SPEED_KMH`, in minutes.
- `active_pairs`, `request_times`, `passengers`: the demand stream, one entry per
  request, aligned across the three vectors.
- `layout`: `:uniform` or `:non_uniform`.
- `demand`: `:uniform` or `:concentrated`.
- `seed`: RNG seed used for the demand draw. Geometry does not depend on it.
"""
struct FielbaumGridInstance
    ny::Int
    nx::Int
    stations::Vector{GridStation}
    node_x::Vector{Float64}
    node_y::Vector{Float64}
    row_streets::Vector{FielbaumStreet}
    col_streets::Vector{FielbaumStreet}
    arc_length::Dict{Tuple{Int,Int},Float64}
    arc_time::Dict{Tuple{Int,Int},Float64}
    drive_time::Matrix{Float64}
    walk_distance::Matrix{Float64}
    walk_time::Matrix{Float64}
    active_pairs::Vector{Tuple{Int,Int}}
    request_times::Vector{DateTime}
    passengers::Vector{Int}
    layout::Symbol
    demand::Symbol
    seed::Int
end

"""
    fielbaum_speed_kmh(class::Symbol) -> Float64

Speed of a street class, in km/h. Throws on anything but `:slow`/`:mid`/`:fast`.
"""
function fielbaum_speed_kmh(class::Symbol)::Float64
    class === :slow && return FIELBAUM_SPEEDS_KMH.slow
    class === :mid && return FIELBAUM_SPEEDS_KMH.mid
    class === :fast && return FIELBAUM_SPEEDS_KMH.fast
    throw(ArgumentError("unknown street class $(class); expected :slow, :mid or :fast"))
end

"""
    fielbaum_validate_streets(streets, label)

Check a street vector against the paper's rule that the mid-speed streets are
exactly the unidirectional ones.
"""
function fielbaum_validate_streets(streets::AbstractVector{FielbaumStreet}, label::AbstractString)
    for (i, street) in enumerate(streets)
        fielbaum_speed_kmh(street.class)
        street.direction in (-1, 0, 1) ||
            throw(ArgumentError("$label street $i has direction $(street.direction); expected -1, 0 or 1"))
        if street.class === :mid
            street.direction == 0 &&
                throw(ArgumentError("$label street $i is :mid but bidirectional; the paper's mid-speed streets are one-way"))
        elseif street.direction != 0
            throw(ArgumentError("$label street $i is :$(street.class) but one-way; the paper's slow and fast streets are bidirectional"))
        end
    end
    return streets
end

# Travel time in minutes for `length_m` metres at `speed_kmh`.
_fielbaum_minutes(length_m::Float64, speed_kmh::Float64)::Float64 = 60.0 * length_m / (1000.0 * speed_kmh)

"""
    fielbaum_node_spacings(n, layout; uniform_spacing, center_spacing, edge_spacing)

The `n - 1` gaps between `n` consecutive nodes along one axis, in metres.

`:uniform` returns `uniform_spacing` everywhere (the paper's base grid, 150 m).
`:non_uniform` reproduces the paper's second network, where "the distance
between consecutive nodes increases from 75[m] at the center of the network to
300[m] at the edges": gaps are interpolated linearly in their distance from the
middle gap, symmetrically about the centre. The paper gives only the two
endpoint values, so the linear rule between them is this package's reading of
it; `center_spacing` and `edge_spacing` are exposed so it can be changed.
"""
function fielbaum_node_spacings(
    n::Int,
    layout::Symbol;
    uniform_spacing::Float64 = FIELBAUM_LINK_LENGTH_M,
    center_spacing::Float64 = FIELBAUM_NON_UNIFORM_CENTER_SPACING_M,
    edge_spacing::Float64 = FIELBAUM_NON_UNIFORM_EDGE_SPACING_M,
)::Vector{Float64}
    n >= 2 || throw(ArgumentError("need at least 2 nodes per axis, got $n"))
    n_gaps = n - 1

    if layout === :uniform
        uniform_spacing > 0.0 || throw(ArgumentError("uniform_spacing must be positive"))
        return fill(uniform_spacing, n_gaps)
    elseif layout === :non_uniform
        center_spacing > 0.0 || throw(ArgumentError("center_spacing must be positive"))
        edge_spacing > 0.0 || throw(ArgumentError("edge_spacing must be positive"))
        middle = (n_gaps + 1) / 2
        half_span = max(middle - 1.0, 1.0)
        return [
            center_spacing + (edge_spacing - center_spacing) * (abs(i - middle) / half_span)
            for i in 1:n_gaps
        ]
    end
    throw(ArgumentError("unknown layout $(layout); expected :uniform or :non_uniform"))
end

# Directed arcs of the grid: (from_id, to_id) => length in metres.
function _fielbaum_arc_lengths(
    ny::Int,
    nx::Int,
    row_streets::Vector{FielbaumStreet},
    col_streets::Vector{FielbaumStreet},
    row_spacings::Vector{Float64},
    col_spacings::Vector{Float64},
)::Dict{Tuple{Int,Int},Float64}
    arcs = Dict{Tuple{Int,Int},Float64}()

    add_arc!(from, to, len) = (arcs[(from, to)] = len)
    function add_street_arc!(from, to, len, direction)
        # `from`/`to` are already in increasing-index order, so `direction == +1`
        # keeps the forward arc only and `-1` keeps the reverse arc only.
        direction >= 0 && add_arc!(from, to, len)
        direction <= 0 && add_arc!(to, from, len)
    end

    for row in 1:ny, col in 1:(nx - 1)
        from = grid_station_id(row, col, nx)
        to = grid_station_id(row, col + 1, nx)
        add_street_arc!(from, to, col_spacings[col], row_streets[row].direction)
    end
    for col in 1:nx, row in 1:(ny - 1)
        from = grid_station_id(row, col, nx)
        to = grid_station_id(row + 1, col, nx)
        add_street_arc!(from, to, row_spacings[row], col_streets[col].direction)
    end

    return arcs
end

# Speed class of the street carrying the arc between two adjacent nodes.
function _fielbaum_arc_class(
    from::GridStation,
    to::GridStation,
    row_streets::Vector{FielbaumStreet},
    col_streets::Vector{FielbaumStreet},
)::Symbol
    return from.row == to.row ? row_streets[from.row].class : col_streets[from.col].class
end

# All-pairs shortest path on a dense directed matrix. n is 100 for the paper's
# grid, so Floyd-Warshall's O(n^3) is immaterial; it stays exact and dependency
# free for the larger grids the kwargs allow.
function _fielbaum_floyd_warshall!(dist::Matrix{Float64})::Matrix{Float64}
    n = size(dist, 1)
    for k in 1:n, i in 1:n
        dik = dist[i, k]
        isfinite(dik) || continue
        for j in 1:n
            candidate = dik + dist[k, j]
            if candidate < dist[i, j]
                dist[i, j] = candidate
            end
        end
    end
    return dist
end

# Move one step diagonally towards the grid centre, as the paper's "concentrated
# demand" scenario does.
function _fielbaum_shift_towards_center(station::GridStation, ny::Int, nx::Int)::Int
    center_row = (1 + ny) / 2
    center_col = (1 + nx) / 2
    row = station.row + Int(sign(center_row - station.row))
    col = station.col + Int(sign(center_col - station.col))
    return grid_station_id(row, col, nx)
end

function _fielbaum_draw_demand(
    rng::AbstractRNG,
    stations::Vector{GridStation},
    ny::Int,
    nx::Int,
    n_requests::Int,
    demand::Symbol,
    concentration_probability::Float64,
)::Vector{Tuple{Int,Int}}
    n_stations = length(stations)
    pairs = Vector{Tuple{Int,Int}}(undef, n_requests)

    for r in 1:n_requests
        origin = rand(rng, 1:n_stations)
        destination = rand(rng, 1:n_stations)
        while destination == origin
            destination = rand(rng, 1:n_stations)
        end

        if demand === :concentrated
            shifted_origin = rand(rng) < concentration_probability ?
                _fielbaum_shift_towards_center(stations[origin], ny, nx) : origin
            shifted_destination = rand(rng) < concentration_probability ?
                _fielbaum_shift_towards_center(stations[destination], ny, nx) : destination
            # A shift that collapses the request onto a single node would make it
            # meaningless, so such a draw keeps its original endpoints.
            if shifted_origin != shifted_destination
                origin, destination = shifted_origin, shifted_destination
            end
        end

        pairs[r] = (origin, destination)
    end

    return pairs
end

function _fielbaum_draw_passengers(rng::AbstractRNG, n_requests::Int)::Vector{Int}
    # "each of them has only one passenger with probability 0.8, and two or three
    #  passengers with probability 0.1 each"
    return [begin
        u = rand(rng)
        u < 0.8 ? 1 : (u < 0.9 ? 2 : 3)
    end for _ in 1:n_requests]
end

"""
    generate_fielbaum_grid_instance(; kwargs...) -> FielbaumGridInstance

Build the toy grid of Fielbaum et al. (2021), Section 3.1 / Fig. 4: a 10x10 grid
whose streets are slow (20 km/h, bidirectional), mid-speed (30 km/h,
unidirectional) or fast (40 km/h, bidirectional), all arcs 150 m long, walking at
5 km/h. The per-street classes and one-way directions are transcribed from Fig. 4
into [`FIELBAUM_BASE_ROW_STREETS`](@ref) and [`FIELBAUM_BASE_COL_STREETS`](@ref).

# Keyword arguments
- `ny`, `nx = 10`: grid size. Sizes other than 10 tile the paper's street pattern
  cyclically, which is an extension, not something the paper specifies.
- `row_streets`, `col_streets`: override the street layout outright. Defaults tile
  the Fig. 4 vectors to `ny`/`nx`.
- `layout = :uniform`: `:uniform` for the paper's base grid (every gap 150 m) or
  `:non_uniform` for its "non-uniform network" scenario (75 m at the centre
  widening to 300 m at the edges). See [`fielbaum_node_spacings`](@ref).
- `demand = :uniform`: `:uniform` draws origins and destinations uniformly over
  the nodes; `:concentrated` reproduces the paper's "concentrated demand"
  scenario by moving each endpoint one step diagonally towards the centre with
  probability `concentration_probability`.
- `n_requests`: defaults to the paper's stream, `requests_per_arrival` requests
  every `arrival_interval_seconds` for `horizon_minutes` (2 every 15 s for one
  hour = 480 requests).
- `start_time = DateTime(2026, 1, 1, 8)`: timestamp of the first arrival batch.
- `link_length_m`, `center_spacing_m`, `edge_spacing_m`, `walking_speed_kmh`:
  the geometry constants, exposed for sensitivity runs.
- `seed = 42`: seeds the demand draw only. The network is deterministic.

The returned instance keeps `arc_time` (per-arc, so the speed hierarchy stays
inspectable) next to `drive_time` (all-pairs shortest path, **asymmetric**
because of the one-way streets) and `walk_time` (symmetric: pedestrians use the
same streets and ignore the one-way rules). All times are in minutes.
"""
function generate_fielbaum_grid_instance(;
    ny::Int = 10,
    nx::Int = 10,
    row_streets::Union{Nothing,AbstractVector{FielbaumStreet}} = nothing,
    col_streets::Union{Nothing,AbstractVector{FielbaumStreet}} = nothing,
    layout::Symbol = :uniform,
    demand::Symbol = :uniform,
    n_requests::Union{Nothing,Int} = nothing,
    requests_per_arrival::Int = 2,
    arrival_interval_seconds::Int = 15,
    horizon_minutes::Int = 60,
    start_time::DateTime = DateTime(2026, 1, 1, 8),
    link_length_m::Float64 = FIELBAUM_LINK_LENGTH_M,
    center_spacing_m::Float64 = FIELBAUM_NON_UNIFORM_CENTER_SPACING_M,
    edge_spacing_m::Float64 = FIELBAUM_NON_UNIFORM_EDGE_SPACING_M,
    walking_speed_kmh::Float64 = FIELBAUM_WALKING_SPEED_KMH,
    concentration_probability::Float64 = 0.5,
    seed::Int = 42,
)::FielbaumGridInstance
    ny >= 2 || throw(ArgumentError("ny must be at least 2, got $ny"))
    nx >= 2 || throw(ArgumentError("nx must be at least 2, got $nx"))
    walking_speed_kmh > 0.0 || throw(ArgumentError("walking_speed_kmh must be positive"))
    demand in (:uniform, :concentrated) ||
        throw(ArgumentError("unknown demand $(demand); expected :uniform or :concentrated"))
    0.0 <= concentration_probability <= 1.0 ||
        throw(ArgumentError("concentration_probability must lie in [0, 1]"))

    rows = isnothing(row_streets) ?
        [FIELBAUM_BASE_ROW_STREETS[mod1(r, length(FIELBAUM_BASE_ROW_STREETS))] for r in 1:ny] :
        collect(FielbaumStreet, row_streets)
    cols = isnothing(col_streets) ?
        [FIELBAUM_BASE_COL_STREETS[mod1(c, length(FIELBAUM_BASE_COL_STREETS))] for c in 1:nx] :
        collect(FielbaumStreet, col_streets)
    length(rows) == ny || throw(ArgumentError("row_streets has $(length(rows)) entries, expected ny=$ny"))
    length(cols) == nx || throw(ArgumentError("col_streets has $(length(cols)) entries, expected nx=$nx"))
    fielbaum_validate_streets(rows, "row")
    fielbaum_validate_streets(cols, "column")

    row_spacings = fielbaum_node_spacings(
        ny, layout;
        uniform_spacing = link_length_m,
        center_spacing = center_spacing_m,
        edge_spacing = edge_spacing_m,
    )
    col_spacings = fielbaum_node_spacings(
        nx, layout;
        uniform_spacing = link_length_m,
        center_spacing = center_spacing_m,
        edge_spacing = edge_spacing_m,
    )

    stations = [GridStation(grid_station_id(r, c, nx), r, c) for r in 1:ny for c in 1:nx]
    n_stations = length(stations)

    # Row 1 is the top row of Fig. 4, so it gets the largest northing.
    x_of_col = [0.0; cumsum(col_spacings)]
    depth_of_row = [0.0; cumsum(row_spacings)]
    total_depth = depth_of_row[end]
    node_x = [x_of_col[station.col] for station in stations]
    node_y = [total_depth - depth_of_row[station.row] for station in stations]

    arc_length = _fielbaum_arc_lengths(ny, nx, rows, cols, row_spacings, col_spacings)
    arc_time = Dict{Tuple{Int,Int},Float64}()
    drive_time = fill(Inf, n_stations, n_stations)
    for i in 1:n_stations
        drive_time[i, i] = 0.0
    end
    for ((from, to), len) in arc_length
        speed = fielbaum_speed_kmh(_fielbaum_arc_class(stations[from], stations[to], rows, cols))
        minutes = _fielbaum_minutes(len, speed)
        arc_time[(from, to)] = minutes
        drive_time[from, to] = min(drive_time[from, to], minutes)
    end
    _fielbaum_floyd_warshall!(drive_time)

    # The grid is a full lattice, so the shortest undirected walk between two
    # nodes is any monotone staircase between them and its length is the
    # rectilinear distance between their coordinates.
    walk_distance = Matrix{Float64}(undef, n_stations, n_stations)
    walk_time = Matrix{Float64}(undef, n_stations, n_stations)
    for i in 1:n_stations, j in 1:n_stations
        metres = abs(node_x[i] - node_x[j]) + abs(node_y[i] - node_y[j])
        walk_distance[i, j] = metres
        walk_time[i, j] = _fielbaum_minutes(metres, walking_speed_kmh)
    end

    arrival_interval_seconds > 0 ||
        throw(ArgumentError("arrival_interval_seconds must be positive"))
    requests_per_arrival > 0 || throw(ArgumentError("requests_per_arrival must be positive"))
    n_batches = max(1, (horizon_minutes * 60) ÷ arrival_interval_seconds)
    total_requests = isnothing(n_requests) ? requests_per_arrival * n_batches : n_requests
    total_requests > 0 || throw(ArgumentError("n_requests must be positive"))

    rng = Random.MersenneTwister(seed)
    active_pairs = _fielbaum_draw_demand(
        rng, stations, ny, nx, total_requests, demand, concentration_probability,
    )
    passengers = _fielbaum_draw_passengers(rng, total_requests)
    request_times = [
        start_time + Second(arrival_interval_seconds * ((r - 1) ÷ requests_per_arrival))
        for r in 1:total_requests
    ]

    return FielbaumGridInstance(
        ny,
        nx,
        stations,
        node_x,
        node_y,
        rows,
        cols,
        arc_length,
        arc_time,
        drive_time,
        walk_distance,
        walk_time,
        active_pairs,
        request_times,
        passengers,
        layout,
        demand,
        seed,
    )
end

"""
    fielbaum_grid_travel_cost_dict(instance) -> (nodes, travel_cost)

All-pairs directed drive times in minutes, keyed by station id pair -- the
`grid_travel_cost_dict` counterpart for this generator.
"""
function fielbaum_grid_travel_cost_dict(instance::FielbaumGridInstance)
    nodes = [station.id for station in instance.stations]
    travel_cost = Dict{Tuple{Int,Int},Float64}()
    for u in nodes, v in nodes
        travel_cost[(u, v)] = instance.drive_time[u, v]
    end
    return nodes, travel_cost
end

function _fielbaum_station_frame(instance::FielbaumGridInstance)::DataFrame
    return DataFrame(
        id = [station.id for station in instance.stations],
        lon = copy(instance.node_x),
        lat = copy(instance.node_y),
        row = [station.row for station in instance.stations],
        col = [station.col for station in instance.stations],
    )
end

function _fielbaum_request_frame(instance::FielbaumGridInstance)::DataFrame
    return DataFrame(
        id = collect(1:length(instance.active_pairs)),
        origin_station_id = [instance.stations[origin].id for (origin, _) in instance.active_pairs],
        destination_station_id = [instance.stations[dest].id for (_, dest) in instance.active_pairs],
        request_time = copy(instance.request_times),
        passengers = copy(instance.passengers),
    )
end

function _fielbaum_walking_costs(
    instance::FielbaumGridInstance;
    max_walking_time::Float64,
    walking_cost_scale::Float64,
)::Dict{Tuple{Int,Int},Float64}
    costs = Dict{Tuple{Int,Int},Float64}()
    for from in instance.stations, to in instance.stations
        minutes = instance.walk_time[from.id, to.id]
        if minutes <= max_walking_time + 1e-9
            costs[(from.id, to.id)] = walking_cost_scale * minutes
        end
    end
    return costs
end

"""
    create_fielbaum_grid_problem_data(instance; kwargs...) -> StationSelectionData

Turn a [`FielbaumGridInstance`](@ref) into `StationSelectionData`.

**Both cost matrices are in minutes**, which is the whole point of this grid:
every arc is the same length, so only travel *time* distinguishes a fast street
from a slow one. That makes the walking limit a walking *time* limit, so
`StationSelectionProblem`'s `max_walking_distance` must be given in minutes too
-- pass `FIELBAUM_MAX_WALKING_TIME_MIN` (the paper's `Ω_a = 12` min) to match the
paper, and keep `max_walking_time` here in step with it so the two agree on which
pairs are walkable.

Routing costs are **asymmetric**: a one-way mid-speed street can make the return
leg strictly longer. `StationSelectionData` indexes routing costs as
`[from, to]`, so this survives the conversion.

# Keyword arguments
- `max_walking_time = FIELBAUM_MAX_WALKING_TIME_MIN`: pairs further apart than
  this on foot are left out of the walking dict, so they read back as `Inf`.
- `walking_cost_scale = 1.0`, `routing_cost_scale = 1.0`: multipliers on the two
  matrices. Leave both at 1 to keep minutes; the paper's relative weighting of
  walking against riding (`p_a = 2 p_v`) belongs in the formulation's
  `walk_cost_weight`, for which [`FIELBAUM_WALK_COST_WEIGHT`](@ref) is the value.
- `scenarios`: optional `(start, end)` time-window strings passed straight
  through to `create_station_selection_data`.
"""
function create_fielbaum_grid_problem_data(
    instance::FielbaumGridInstance;
    max_walking_time::Float64 = FIELBAUM_MAX_WALKING_TIME_MIN,
    walking_cost_scale::Float64 = 1.0,
    routing_cost_scale::Float64 = 1.0,
    scenarios::Union{Vector{Tuple{String,String}},Nothing} = nothing,
)::StationSelectionData
    max_walking_time >= 0.0 || throw(ArgumentError("max_walking_time must be nonnegative"))
    walking_cost_scale >= 0.0 || throw(ArgumentError("walking_cost_scale must be nonnegative"))
    routing_cost_scale >= 0.0 || throw(ArgumentError("routing_cost_scale must be nonnegative"))

    _, travel_cost = fielbaum_grid_travel_cost_dict(instance)
    routing_costs = Dict(key => routing_cost_scale * value for (key, value) in travel_cost)
    walking_costs = _fielbaum_walking_costs(
        instance;
        max_walking_time = max_walking_time,
        walking_cost_scale = walking_cost_scale,
    )

    return create_station_selection_data(
        _fielbaum_station_frame(instance),
        _fielbaum_request_frame(instance),
        walking_costs;
        routing_costs = routing_costs,
        scenarios = scenarios,
    )
end

create_fielbaum_grid_station_selection_data(instance::FielbaumGridInstance; kwargs...) =
    create_fielbaum_grid_problem_data(instance; kwargs...)

_fielbaum_row_glyph(street::FielbaumStreet)::String =
    street.class === :fast ? "=F=" :
    street.class === :slow ? "-S-" :
    street.direction > 0 ? "-M>" : "<M-"

_fielbaum_col_glyph(street::FielbaumStreet)::String =
    street.class === :fast ? "F" :
    street.class === :slow ? "S" :
    street.direction > 0 ? "v" : "^"

"""
    print_fielbaum_grid_street_map(instance)

Render the street layout as ASCII so it can be checked against Fig. 4 of the
paper. `F` is fast, `S` slow, `M`/`^`/`v`/`>`/`<` mid-speed with its one-way
direction; row 1 prints at the top, as in the figure.
"""
function print_fielbaum_grid_street_map(instance::FielbaumGridInstance)
    for row in 1:instance.ny
        horizontal = _fielbaum_row_glyph(instance.row_streets[row])
        println("o" * repeat(horizontal * "o", instance.nx - 1))
        if row < instance.ny
            println(join((_fielbaum_col_glyph(street) for street in instance.col_streets), "   "))
        end
    end
    return nothing
end

"""
    print_fielbaum_grid_summary(instance)

Print the instance's size, layout, speed mix, demand stream and the asymmetry
that the one-way streets induce in the all-pairs drive times, then the street map.
"""
function print_fielbaum_grid_summary(instance::FielbaumGridInstance)
    n_stations = length(instance.stations)
    class_counts = Dict(:slow => 0, :mid => 0, :fast => 0)
    for street in Iterators.flatten((instance.row_streets, instance.col_streets))
        class_counts[street.class] += 1
    end

    finite = [
        instance.drive_time[i, j]
        for i in 1:n_stations for j in 1:n_stations
        if i != j && isfinite(instance.drive_time[i, j])
    ]
    asymmetry = maximum(
        abs(instance.drive_time[i, j] - instance.drive_time[j, i])
        for i in 1:n_stations for j in 1:n_stations
    )

    @printf(
        "Fielbaum grid %dx%d layout=%s demand=%s seed=%d\n",
        instance.ny, instance.nx, instance.layout, instance.demand, instance.seed,
    )
    @printf(
        "  streets: %d fast (%.0f km/h), %d mid one-way (%.0f km/h), %d slow (%.0f km/h); walking %.0f km/h\n",
        class_counts[:fast], FIELBAUM_SPEEDS_KMH.fast,
        class_counts[:mid], FIELBAUM_SPEEDS_KMH.mid,
        class_counts[:slow], FIELBAUM_SPEEDS_KMH.slow,
        FIELBAUM_WALKING_SPEED_KMH,
    )
    @printf(
        "  arcs: %d directed; drive time min/mean/max %.2f/%.2f/%.2f min; max |t(i,j)-t(j,i)| %.2f min\n",
        length(instance.arc_time), minimum(finite), sum(finite) / length(finite), maximum(finite), asymmetry,
    )
    @printf(
        "  walk time max %.2f min; requests %d over %s..%s; passengers total %d\n",
        maximum(instance.walk_time),
        length(instance.active_pairs),
        string(first(instance.request_times)),
        string(last(instance.request_times)),
        sum(instance.passengers),
    )
    print_fielbaum_grid_street_map(instance)
    return nothing
end
