using Dates
using Printf
using Random

# =============================================================================
# Nested-square "spiderweb" gadget with CONTINUOUS passenger endpoints
# =============================================================================
#
# A road network in the unit square: K nested axis-aligned square rings centred
# at (0.5, 0.5), each ring's four corners joined around the ring, and the
# corresponding corners of consecutive rings joined along four diagonal spokes
# that meet at the centre. Every ring corner is a candidate station; the centre
# is always a road node and optionally a candidate too.
#
# Purpose: vary how many ring levels a passenger can walk to, and see whether a
# model that prices each passenger's direct ride (ClusteringTwoStageOD, the
# "ivt" arm) and one that prices pooled routes (AggregateODRouteJoint under
# CG) pick different ring levels. See
# `experiments/2026-09-26_spiderweb_gadget/README.md`.
#
# ---------------------------------------------------------------------------
# How continuous passengers reach the optimiser
# ---------------------------------------------------------------------------
# Requests in `StationSelectionData` name an origin and a destination STATION,
# and walking costs are a station-by-station matrix. A passenger here stands at
# an arbitrary point, not at a station, and snapping that point to its nearest
# station would change exactly the thing under study: which stations lie within
# `R_walk`, and how far each is.
#
# So each passenger endpoint becomes its own node in `data.stations`, with
# `candidate = false`:
#
#   * id `SPIDERWEB_ORIGIN_ID_OFFSET + p` for passenger p's origin and
#     `SPIDERWEB_DESTINATION_ID_OFFSET + p` for its destination -- one node per
#     endpoint, never shared, so no two passengers are merged into one OD group
#     (the AggregateODRoute family collapses repeated OD pairs to unit demand);
#   * walking cost from the endpoint to every candidate station is the exact
#     Euclidean distance from the continuous point;
#   * no walking cost between two demand nodes (not even to itself) and no
#     routing cost to or from one, so both read back as `Inf`;
#   * `candidate = false` keeps it from being built, offered as a pickup/dropoff,
#     or visited by a route (`candidate_station_mask`, data/core/struct.jl).
#
# The walking limit is then just `StationSelectionProblem.max_walking_distance`,
# compared against those exact point-to-station distances, so the feasible
# pickup set of passenger p is O_p(R) = {j : ||o_p - s_j|| <= R} with no
# rounding anywhere.
#
# ---------------------------------------------------------------------------
# Units
# ---------------------------------------------------------------------------
# Coordinates live in [0,1]^2. Walking cost = `walk_cost_per_unit` x Euclidean
# distance; routing cost = `routing_cost_scale` x road travel time, where an
# edge takes length / speed. With the defaults (all 1) both are in unit-square
# lengths, so `max_walking_distance` is R_walk in the same units. The relative
# price of walking against riding belongs in the formulation weights
# (`in_vehicle_time_weight`; `walk_cost_weight` / `route_regularization_weight`),
# not here. Unit-free costs mean any time-valued formulation field
# (`repositioning_time`, `max_wait_time`) must be given in the same units --
# the owner's Zhuzhou values (20 s, 300 s) are meaningless on this instance.

const SPIDERWEB_QUADRANTS = (:SW, :SE, :NE, :NW)
const SPIDERWEB_DEFAULT_HALF_WIDTHS = [0.50, 0.375, 0.25, 0.125]
const SPIDERWEB_ORIGIN_ID_OFFSET = 1000
const SPIDERWEB_DESTINATION_ID_OFFSET = 2000
const SPIDERWEB_CENTER = (0.5, 0.5)

# Unit vector from the centre towards each quadrant's corner.
const _SPIDERWEB_QUADRANT_SIGN = Dict(
    :SW => (-1.0, -1.0), :SE => (1.0, -1.0), :NE => (1.0, 1.0), :NW => (-1.0, 1.0),
)

"""
    SpiderwebStation

A node of the spiderweb road network.

# Fields
- `id::Int`: station ID (1-based, stable across instances with the same rings)
- `label::String`: e.g. `"SW1"` (outermost ring is level 1), `"C"` for the centre
- `x::Float64`, `y::Float64`: coordinates in the unit square
- `quadrant::Symbol`: `:SW`, `:SE`, `:NE`, `:NW`, or `:C` for the centre
- `ring::Int`: ring level, 1 = outermost, `K` = innermost ring, `K + 1` = centre
- `radial::Float64`: Euclidean distance from (0.5, 0.5)
- `candidate::Bool`: may be built; `false` only for a centre that is a road node only
"""
struct SpiderwebStation
    id::Int
    label::String
    x::Float64
    y::Float64
    quadrant::Symbol
    ring::Int
    radial::Float64
    candidate::Bool
end

"""
    SpiderwebEdge

An undirected road edge between two `SpiderwebStation` ids. `kind` is `:ring`
(circumferential, along one square) or `:spoke` (radial, along a diagonal).
`time = length / speed` for that kind.
"""
struct SpiderwebEdge
    u::Int
    v::Int
    kind::Symbol
    length::Float64
    time::Float64
end

"""
    ContinuousRequest

One passenger with continuous origin `(ox, oy)` and destination `(dx, dy)` in the
unit square. `group` is a free-text tag from the demand generator (e.g.
`"SW->NE"`), carried for plotting and never read by the optimiser.
"""
struct ContinuousRequest
    id::Int
    ox::Float64
    oy::Float64
    dx::Float64
    dy::Float64
    group::String
end

"""
    SpiderwebNetwork

The road network: every node (candidate or not), every edge, and all-pairs
shortest-path road time `road_time[i, j]` indexed by position in `nodes`.
"""
struct SpiderwebNetwork
    half_widths::Vector{Float64}
    include_center::Bool
    ring_speed::Float64
    spoke_speed::Float64
    nodes::Vector{SpiderwebStation}
    edges::Vector{SpiderwebEdge}
    road_time::Matrix{Float64}
end

"""
    SpiderwebInstance

A `SpiderwebNetwork` plus a set of `ContinuousRequest`s.
"""
struct SpiderwebInstance
    network::SpiderwebNetwork
    requests::Vector{ContinuousRequest}
end

"""Candidate stations of the network, in id order."""
spiderweb_candidates(net::SpiderwebNetwork) = [s for s in net.nodes if s.candidate]
spiderweb_candidates(inst::SpiderwebInstance) = spiderweb_candidates(inst.network)

"""
    generate_spiderweb_network(; half_widths, include_center=false,
                                 ring_speed=1.0, spoke_speed=1.0) -> SpiderwebNetwork

Build the nested-square network. `half_widths` must be strictly decreasing and
in `(0, 0.5]`; ring `l` has corners at `0.5 ± half_widths[l]`. Station ids run
ring by ring from the outside in, SW, SE, NE, NW within a ring, then the centre.

The centre is always a road node, because the four spokes meet there; it is a
candidate station only when `include_center = true`.

The construction is symmetric under 90-degree rotation about the centre by
design. Asymmetry, if wanted, belongs in the demand.
"""
function generate_spiderweb_network(;
    half_widths::AbstractVector{<:Real} = SPIDERWEB_DEFAULT_HALF_WIDTHS,
    include_center::Bool = false,
    ring_speed::Real = 1.0,
    spoke_speed::Real = 1.0,
)::SpiderwebNetwork
    r = Float64.(collect(half_widths))
    isempty(r) && throw(ArgumentError("half_widths must be non-empty"))
    all(0 .< r .<= 0.5) || throw(ArgumentError("half_widths must lie in (0, 0.5]"))
    all(diff(r) .< 0) || throw(ArgumentError("half_widths must be strictly decreasing"))
    ring_speed > 0 && spoke_speed > 0 || throw(ArgumentError("speeds must be positive"))

    K = length(r)
    cx, cy = SPIDERWEB_CENTER
    nodes = SpiderwebStation[]
    id_of = Dict{Tuple{Symbol, Int}, Int}()
    for l in 1:K, q in SPIDERWEB_QUADRANTS
        sx, sy = _SPIDERWEB_QUADRANT_SIGN[q]
        id = length(nodes) + 1
        push!(nodes, SpiderwebStation(
            id, "$(q)$(l)", cx + sx * r[l], cy + sy * r[l], q, l, sqrt(2.0) * r[l], true,
        ))
        id_of[(q, l)] = id
    end
    center_id = length(nodes) + 1
    push!(nodes, SpiderwebStation(center_id, "C", cx, cy, :C, K + 1, 0.0, include_center))

    edges = SpiderwebEdge[]
    ring_order = (:SW, :SE, :NE, :NW)
    for l in 1:K, i in 1:4
        u = id_of[(ring_order[i], l)]
        v = id_of[(ring_order[mod1(i + 1, 4)], l)]
        len = 2.0 * r[l]
        push!(edges, SpiderwebEdge(u, v, :ring, len, len / ring_speed))
    end
    for q in SPIDERWEB_QUADRANTS
        for l in 1:(K - 1)
            u, v = id_of[(q, l)], id_of[(q, l + 1)]
            len = sqrt(2.0) * (r[l] - r[l + 1])
            push!(edges, SpiderwebEdge(u, v, :spoke, len, len / spoke_speed))
        end
        u = id_of[(q, K)]
        len = sqrt(2.0) * r[K]
        push!(edges, SpiderwebEdge(u, center_id, :spoke, len, len / spoke_speed))
    end

    n = length(nodes)
    dist = fill(Inf, n, n)
    for i in 1:n
        dist[i, i] = 0.0
    end
    for e in edges
        dist[e.u, e.v] = min(dist[e.u, e.v], e.time)
        dist[e.v, e.u] = min(dist[e.v, e.u], e.time)
    end
    for k in 1:n, i in 1:n, j in 1:n
        alt = dist[i, k] + dist[k, j]
        alt < dist[i, j] && (dist[i, j] = alt)
    end

    return SpiderwebNetwork(r, include_center, Float64(ring_speed), Float64(spoke_speed),
                            nodes, edges, dist)
end

# -----------------------------------------------------------------------------
# Demand generators
# -----------------------------------------------------------------------------

"""
    spiderweb_point_quadrant(x, y) -> Symbol

Quadrant of a continuous point relative to the centre. Points on a dividing
line go to the south / west side.
"""
spiderweb_point_quadrant(x::Real, y::Real)::Symbol =
    y <= 0.5 ? (x <= 0.5 ? :SW : :SE) : (x <= 0.5 ? :NW : :NE)

"""
    spiderweb_quadrant_box(q; lo=0.05, hi=0.35) -> (xlo, xhi, ylo, yhi)

The sampling box for quadrant `q`: `[lo, hi]^2` for SW, mirrored for the
others. The default keeps endpoints off the outer ring corners and off the
dividing lines, so no passenger stands exactly at a station.
"""
function spiderweb_quadrant_box(q::Symbol; lo::Real = 0.05, hi::Real = 0.35)
    0 <= lo < hi <= 0.5 || throw(ArgumentError("need 0 <= lo < hi <= 0.5"))
    sx, sy = _SPIDERWEB_QUADRANT_SIGN[q]
    xs = sx < 0 ? (lo, hi) : (1 - hi, 1 - lo)
    ys = sy < 0 ? (lo, hi) : (1 - hi, 1 - lo)
    return (Float64(xs[1]), Float64(xs[2]), Float64(ys[1]), Float64(ys[2]))
end

_uniform_in(rng, a, b) = a + (b - a) * rand(rng)

function _sample_box(rng::AbstractRNG, box)
    xlo, xhi, ylo, yhi = box
    return (_uniform_in(rng, xlo, xhi), _uniform_in(rng, ylo, yhi))
end

"""Named quadrant-to-quadrant flow sets for `sample_spiderweb_requests`."""
const SPIDERWEB_FLOW_PATTERNS = Dict{Symbol, Vector{Tuple{Symbol, Symbol}}}(
    # A. each quadrant to the opposite one
    :opposite => [(:SW, :NE), (:NE, :SW), (:NW, :SE), (:SE, :NW)],
    # B. each to a neighbouring quadrant
    :adjacent => [(:SW, :SE), (:SW, :NW), (:NE, :NW), (:NE, :SE)],
    # C. both of the above
    :mixed => [(:SW, :NE), (:SW, :SE), (:NW, :SE), (:NE, :NW),
               (:NE, :SW), (:SE, :NW), (:SW, :NW), (:NE, :SE)],
    # D. two diagonal families whose straight lines cross at the centre
    :crossing => [(:SW, :NE), (:SE, :NW)],
)

"""
    sample_spiderweb_requests(rng, pattern; n, lo=0.05, hi=0.35) -> Vector{ContinuousRequest}
    sample_spiderweb_requests(rng, flows::Vector{Tuple{Symbol,Symbol}}; n, lo, hi)

Draw `n` passengers with continuous endpoints.

`pattern` is a key of `SPIDERWEB_FLOW_PATTERNS` (`:opposite`, `:adjacent`,
`:mixed`, `:crossing`) -- passengers cycle through its flows in order, so the
flow mix is exact rather than sampled -- or one of

- `:uniform`: origin and destination uniform on the unit square (F);
- `:clustered`: each endpoint drawn around one of `n_clusters` random centres
  with standard deviation `cluster_sd`, clipped to the square (F).

For the quadrant flows each endpoint is uniform in `spiderweb_quadrant_box`.
For slightly asymmetric demand (E), pass the result through
`perturb_spiderweb_requests`.
"""
function sample_spiderweb_requests(
    rng::AbstractRNG,
    flows::AbstractVector{<:Tuple{Symbol, Symbol}};
    n::Int,
    lo::Real = 0.05,
    hi::Real = 0.35,
)::Vector{ContinuousRequest}
    n > 0 || throw(ArgumentError("n must be positive"))
    isempty(flows) && throw(ArgumentError("flows must be non-empty"))
    reqs = ContinuousRequest[]
    for p in 1:n
        qo, qd = flows[mod1(p, length(flows))]
        ox, oy = _sample_box(rng, spiderweb_quadrant_box(qo; lo = lo, hi = hi))
        dx, dy = _sample_box(rng, spiderweb_quadrant_box(qd; lo = lo, hi = hi))
        push!(reqs, ContinuousRequest(p, ox, oy, dx, dy, "$(qo)->$(qd)"))
    end
    return reqs
end

function sample_spiderweb_requests(
    rng::AbstractRNG,
    pattern::Symbol;
    n::Int,
    lo::Real = 0.05,
    hi::Real = 0.35,
    n_clusters::Int = 3,
    cluster_sd::Real = 0.07,
)::Vector{ContinuousRequest}
    haskey(SPIDERWEB_FLOW_PATTERNS, pattern) &&
        return sample_spiderweb_requests(rng, SPIDERWEB_FLOW_PATTERNS[pattern]; n = n, lo = lo, hi = hi)
    if pattern === :uniform
        return [ContinuousRequest(p, rand(rng), rand(rng), rand(rng), rand(rng), "uniform")
                for p in 1:n]
    elseif pattern === :clustered
        centres = [(rand(rng), rand(rng)) for _ in 1:n_clusters]
        draw() = begin
            c = centres[rand(rng, 1:n_clusters)]
            (clamp(c[1] + cluster_sd * randn(rng), 0.0, 1.0),
             clamp(c[2] + cluster_sd * randn(rng), 0.0, 1.0))
        end
        reqs = ContinuousRequest[]
        for p in 1:n
            (ox, oy), (dx, dy) = draw(), draw()
            push!(reqs, ContinuousRequest(p, ox, oy, dx, dy, "clustered"))
        end
        return reqs
    end
    throw(ArgumentError("unknown spiderweb demand pattern $(pattern)"))
end

"""
    perturb_spiderweb_requests(rng, reqs; n_moved=1, sd=0.02) -> Vector{ContinuousRequest}

Slightly asymmetric demand (E): move `n_moved` randomly chosen endpoints by a
Gaussian step of standard deviation `sd`, clipped to the unit square. Enough to
break ties between rotated copies of a solution without changing which quadrant
anything is in, in expectation.
"""
function perturb_spiderweb_requests(
    rng::AbstractRNG,
    reqs::AbstractVector{ContinuousRequest};
    n_moved::Int = 1,
    sd::Real = 0.02,
)::Vector{ContinuousRequest}
    out = collect(reqs)
    endpoints = [(p, side) for p in eachindex(out) for side in (:origin, :destination)]
    for (p, side) in endpoints[randperm(rng, length(endpoints))[1:min(n_moved, end)]]
        r = out[p]
        if side === :origin
            out[p] = ContinuousRequest(r.id, clamp(r.ox + sd * randn(rng), 0.0, 1.0),
                                       clamp(r.oy + sd * randn(rng), 0.0, 1.0), r.dx, r.dy, r.group)
        else
            out[p] = ContinuousRequest(r.id, r.ox, r.oy, clamp(r.dx + sd * randn(rng), 0.0, 1.0),
                                       clamp(r.dy + sd * randn(rng), 0.0, 1.0), r.group)
        end
    end
    return out
end

"""
    spiderweb_segments_cross(a, b) -> Bool

Whether the straight O-D segments of two requests properly intersect. A visual
proxy for "these two trips could share a vehicle through the middle"; used to
check that a `:crossing` draw actually crosses.
"""
function spiderweb_segments_cross(a::ContinuousRequest, b::ContinuousRequest)::Bool
    orient(px, py, qx, qy, rx, ry) = sign((qx - px) * (ry - py) - (qy - py) * (rx - px))
    o1 = orient(a.ox, a.oy, a.dx, a.dy, b.ox, b.oy)
    o2 = orient(a.ox, a.oy, a.dx, a.dy, b.dx, b.dy)
    o3 = orient(b.ox, b.oy, b.dx, b.dy, a.ox, a.oy)
    o4 = orient(b.ox, b.oy, b.dx, b.dy, a.dx, a.dy)
    return o1 * o2 < 0 && o3 * o4 < 0
end

# -----------------------------------------------------------------------------
# Walking geometry on continuous endpoints
# -----------------------------------------------------------------------------

"""
    spiderweb_walk_matrices(inst) -> (walk_o, walk_d)

`walk_o[p, c]` = Euclidean distance from passenger p's origin to the c-th
candidate station (`spiderweb_candidates` order); `walk_d` likewise for the
destination. Unit-square lengths, before any cost scaling.
"""
function spiderweb_walk_matrices(inst::SpiderwebInstance)
    cands = spiderweb_candidates(inst)
    P, C = length(inst.requests), length(cands)
    walk_o = Matrix{Float64}(undef, P, C)
    walk_d = Matrix{Float64}(undef, P, C)
    for (p, r) in enumerate(inst.requests), (c, s) in enumerate(cands)
        walk_o[p, c] = hypot(r.ox - s.x, r.oy - s.y)
        walk_d[p, c] = hypot(r.dx - s.x, r.dy - s.y)
    end
    return walk_o, walk_d
end

"""
    spiderweb_walk_breakpoints(inst) -> Vector{Float64}

Every distinct endpoint-to-candidate distance, sorted. The feasible sets
O_p(R), D_p(R) are constant between consecutive breakpoints and change only
when R crosses one, so solving just after each breakpoint visits every distinct
access configuration.
"""
function spiderweb_walk_breakpoints(inst::SpiderwebInstance)::Vector{Float64}
    walk_o, walk_d = spiderweb_walk_matrices(inst)
    return sort!(unique!(vcat(vec(walk_o), vec(walk_d))))
end

"""
    spiderweb_access_sets(inst, R) -> (O, D)

`O[p]` / `D[p]`: candidate-station IDs within walking distance `R` of passenger
p's origin / destination. `<=`, matching `compute_valid_jk_pairs`.
"""
function spiderweb_access_sets(inst::SpiderwebInstance, R::Real)
    cands = spiderweb_candidates(inst)
    walk_o, walk_d = spiderweb_walk_matrices(inst)
    P = length(inst.requests)
    O = [[cands[c].id for c in eachindex(cands) if walk_o[p, c] <= R] for p in 1:P]
    D = [[cands[c].id for c in eachindex(cands) if walk_d[p, c] <= R] for p in 1:P]
    return O, D
end

# -----------------------------------------------------------------------------
# Conversion to StationSelectionData
# -----------------------------------------------------------------------------

spiderweb_origin_id(p::Int) = SPIDERWEB_ORIGIN_ID_OFFSET + p
spiderweb_destination_id(p::Int) = SPIDERWEB_DESTINATION_ID_OFFSET + p

"""
    create_spiderweb_problem_data(inst; walk_cost_per_unit=1.0, routing_cost_scale=1.0,
                                  request_time=DateTime(2026, 1, 1, 8)) -> StationSelectionData

Turn a `SpiderwebInstance` into `StationSelectionData`, one scenario, one
request per passenger. See this file's header for how the continuous endpoints
are carried (non-candidate demand nodes with exact point-to-station walking
costs) and for the units.

`data.stations` carries, besides `id`/`lon`/`lat` (= x, y): `candidate`,
`label`, `kind` (`"station"`, `"origin"`, `"destination"`), `quadrant`, `ring`.
Non-candidate road nodes (a centre that is not a candidate) are left out
entirely: nobody walks to them and routing costs already pass through them.
"""
function create_spiderweb_problem_data(
    inst::SpiderwebInstance;
    walk_cost_per_unit::Real = 1.0,
    routing_cost_scale::Real = 1.0,
    request_time::DateTime = DateTime(2026, 1, 1, 8),
)::StationSelectionData
    walk_cost_per_unit > 0 || throw(ArgumentError("walk_cost_per_unit must be positive"))
    routing_cost_scale > 0 || throw(ArgumentError("routing_cost_scale must be positive"))
    P = length(inst.requests)
    P < SPIDERWEB_DESTINATION_ID_OFFSET - SPIDERWEB_ORIGIN_ID_OFFSET ||
        throw(ArgumentError("too many requests for the demand-node id scheme"))

    net = inst.network
    cands = spiderweb_candidates(net)
    ids = Int[]; xs = Float64[]; ys = Float64[]; is_cand = Bool[]
    labels = String[]; kinds = String[]; quads = String[]; rings = Int[]
    for s in cands
        push!(ids, s.id); push!(xs, s.x); push!(ys, s.y); push!(is_cand, true)
        push!(labels, s.label); push!(kinds, "station"); push!(quads, String(s.quadrant))
        push!(rings, s.ring)
    end
    for (p, r) in enumerate(inst.requests)
        for (kind, id, x, y) in (("origin", spiderweb_origin_id(p), r.ox, r.oy),
                                 ("destination", spiderweb_destination_id(p), r.dx, r.dy))
            push!(ids, id); push!(xs, x); push!(ys, y); push!(is_cand, false)
            push!(labels, "$(kind == "origin" ? "o" : "d")$(p)"); push!(kinds, kind)
            push!(quads, String(spiderweb_point_quadrant(x, y))); push!(rings, 0)
        end
    end
    stations = DataFrame(id = ids, lon = xs, lat = ys, candidate = is_cand, label = labels,
                         kind = kinds, quadrant = quads, ring = rings)

    walking = Dict{Tuple{Int, Int}, Float64}()
    w = Float64(walk_cost_per_unit)
    # candidate <-> candidate (not used by the OD formulations; kept so that
    # station-level formulations see a complete matrix among real stations)
    for a in cands, b in cands
        walking[(a.id, b.id)] = w * hypot(a.x - b.x, a.y - b.y)
    end
    # demand point <-> candidate, both directions (the walking matrix must be
    # symmetric: `_assert_symmetric_walking_costs`). Demand point <-> demand
    # point is deliberately absent, i.e. Inf.
    for (p, r) in enumerate(inst.requests), s in cands
        for (id, x, y) in ((spiderweb_origin_id(p), r.ox, r.oy),
                           (spiderweb_destination_id(p), r.dx, r.dy))
            c = w * hypot(x - s.x, y - s.y)
            walking[(id, s.id)] = c
            walking[(s.id, id)] = c
        end
    end

    routing = Dict{Tuple{Int, Int}, Float64}()
    rs = Float64(routing_cost_scale)
    for a in cands, b in cands
        routing[(a.id, b.id)] = rs * net.road_time[a.id, b.id]
    end

    requests = DataFrame(
        id = collect(1:P),
        origin_station_id = [spiderweb_origin_id(p) for p in 1:P],
        destination_station_id = [spiderweb_destination_id(p) for p in 1:P],
        request_time = fill(request_time, P),
        group = [r.group for r in inst.requests],
    )
    return create_station_selection_data(stations, requests, walking; routing_costs = routing)
end

# -----------------------------------------------------------------------------
# Spatial statistics of a station set
# -----------------------------------------------------------------------------

"""
    spiderweb_station_set_metrics(net, station_ids) -> NamedTuple

Compactness statistics of a set of candidate stations:
`span_x`, `span_y`, `bbox_area`, `diameter` (Euclidean), `road_diameter`,
`radial_mean`, `radial_range` (in ring LEVELS, max - min), `n_rings`,
`n_quadrants`. The centre counts as its own quadrant `:C` and as ring K+1.
"""
function spiderweb_station_set_metrics(net::SpiderwebNetwork, station_ids)
    S = [net.nodes[id] for id in station_ids]
    isempty(S) && throw(ArgumentError("empty station set"))
    xs = [s.x for s in S]; ys = [s.y for s in S]
    span_x = maximum(xs) - minimum(xs)
    span_y = maximum(ys) - minimum(ys)
    diameter = maximum(hypot(a.x - b.x, a.y - b.y) for a in S, b in S)
    road_diameter = maximum(net.road_time[a.id, b.id] for a in S, b in S)
    rings = [s.ring for s in S]
    return (
        span_x = span_x, span_y = span_y, bbox_area = span_x * span_y,
        diameter = diameter, road_diameter = road_diameter,
        radial_mean = sum(s.radial for s in S) / length(S),
        radial_range = maximum(rings) - minimum(rings),
        n_rings = length(unique(rings)),
        n_quadrants = length(unique(s.quadrant for s in S)),
    )
end

"""Human-readable labels for a set of station ids, sorted by ring then quadrant."""
function spiderweb_labels(net::SpiderwebNetwork, station_ids)
    S = sort([net.nodes[id] for id in station_ids]; by = s -> (s.ring, s.id))
    return [s.label for s in S]
end

function print_spiderweb_summary(inst::SpiderwebInstance)
    net = inst.network
    K = length(net.half_widths)
    @printf("Spiderweb: %d rings (half-widths %s), centre %s, ring speed %.3g, spoke speed %.3g\n",
            K, string(net.half_widths), net.include_center ? "candidate" : "road node only",
            net.ring_speed, net.spoke_speed)
    @printf("  %d candidate stations, %d edges, %d passengers\n",
            length(spiderweb_candidates(net)), length(net.edges), length(inst.requests))
    for r in inst.requests
        @printf("  p%-2d %-8s o=(%.3f, %.3f) d=(%.3f, %.3f)\n", r.id, r.group, r.ox, r.oy, r.dx, r.dy)
    end
end
