"""
Hub-and-disk benchmark (2026-10-22): one hub station A and a walkable disk B of candidate stations,
with symmetric demand A -> B and B -> A. The question: how many stations should B get?

Geometry (metres, disk centre at the origin):
- Hub A at (-hub_distance_m, 0): station id 1, a candidate. All of its demand stands exactly at A.
- Disk B of radius `radius_m`: candidate stations on concentric rings (`ring_radii_frac` x radius, with
  `ring_counts` stations each, alternate rings rotated half a step). The default is
  1 + 8 + 16 = 25 stations, ids 2..26.
- Every B rider endpoint is uniform in the disk and stands at a NON-candidate node (`candidate = false`).
  With `demand_grid_m = nothing` each endpoint is its own node (id `HUB_DISK_DEMAND_ID_OFFSET + p`): exact walking,
  and no two riders share an OD pair (the AggregateODRoute family collapses repeated OD pairs to unit demand).
  With `demand_grid_m = g` endpoints snap to the nearest point of a g-metre square lattice clipped to the disk
  (id `HUB_DISK_DEMAND_ID_OFFSET + lattice index`, only points some rider uses are written): the node count stays
  bounded on long instances (the simulator re-reads segment.csv every run), at a snapping error <= g / sqrt(2).

Two properties are asserted, not assumed:
- B is walkable: the largest station-station and point-station distance inside B (at most 2R) is within
  `max_walk_m`, so every station set that keeps A covers every rider and only k is varied.
- A is not: the nearest B station to A is beyond `max_walk_m`, so A is built in every feasible set
  (report k_B = k - 1) and no rider walks between the hub and the disk.

Demand: Poisson arrivals at `lambda_per_hour` in EACH direction, uniform over the window, so the
two flows give the fleet a natural repositioning trip in each direction.

Driving: straight line x `circuity` at `vehicle_speed_mps`, a complete graph over every node (hub, stations,
demand points); demand-point rows are needed for direct drive distances and G2. Coordinates go to
lon/lat through the test-case reference point (`tc_to_lon` / `tc_to_lat`).

Writes `station.csv` (extra columns `candidate`, `role`, `x_m`, `y_m`), `segment.csv`, `order.csv` and
`metadata.json`: the layout `03_run_selection.jl` and `run_simulation_from_files` read.
"""

const HUB_DISK_HUB_ID = 1
const HUB_DISK_DEMAND_ID_OFFSET = 1000
const HUB_DISK_DEFAULT_RING_RADII_FRAC = [0.0, 0.45, 0.9]
const HUB_DISK_DEFAULT_RING_COUNTS = [1, 8, 16]

"""
    hub_disk_stations(; radius_m, hub_distance_m, ring_radii_frac, ring_counts) -> Vector{NamedTuple}

Hub then disk stations, as `(id, name, x_m, y_m, ring, role)`. Ring 0 is the hub.
"""
function hub_disk_stations(;
    radius_m::Real,
    hub_distance_m::Real,
    ring_radii_frac::AbstractVector{<:Real} = HUB_DISK_DEFAULT_RING_RADII_FRAC,
    ring_counts::AbstractVector{<:Integer} = HUB_DISK_DEFAULT_RING_COUNTS,
)
    length(ring_radii_frac) == length(ring_counts) ||
        throw(ArgumentError("ring_radii_frac and ring_counts must have the same length"))
    all(0 .<= ring_radii_frac .<= 1) || throw(ArgumentError("ring_radii_frac must lie in [0, 1]"))
    stations = [(id = HUB_DISK_HUB_ID, name = "A", x_m = -Float64(hub_distance_m), y_m = 0.0,
                 ring = 0, role = "hub")]
    for (l, (f, n)) in enumerate(zip(ring_radii_frac, ring_counts))
        r = f * radius_m
        (r == 0 && n != 1) && throw(ArgumentError("a ring of radius 0 must hold exactly one station"))
        offset = iseven(l) ? 0.5 : 0.0
        for i in 1:n
            θ = 2π * (i - 1 + offset) / n
            push!(stations, (id = length(stations) + 1, name = "B$(l)_$(i)",
                             x_m = r * cos(θ), y_m = r * sin(θ), ring = l, role = "disk"))
        end
    end
    return stations
end

"""
    sample_hub_disk_orders(rng; lambda_per_hour, hours, radius_m, window_start) -> Vector{NamedTuple}

Both flows, merged and sorted by time: `(time, direction, x_m, y_m)` with `direction` `:AB` (the point is
the destination) or `:BA` (the point is the origin). Points uniform in the disk (r = R sqrt(u)).
"""
function sample_hub_disk_orders(
    rng::AbstractRNG;
    lambda_per_hour::Real,
    hours::Real,
    radius_m::Real,
    window_start::DateTime,
)
    # One Poisson draw per hour: `tc_poisson_draw` multiplies uniforms down to exp(-lam), which underflows to 0 once
    # lam > ~745, and the count then saturates there (24 h at 40/h drew 717 instead of ~960).
    rows = NamedTuple[]
    for dir in (:AB, :BA), h in 0:(ceil(Int, hours) - 1)
        span = min(1.0, hours - h)
        n = tc_poisson_draw(lambda_per_hour * span, rng)
        for ts in tc_rand_timestamps(n, rng, window_start + Second(3600 * h), round(Int, span * 3600))
            r, θ = radius_m * sqrt(rand(rng)), 2π * rand(rng)
            push!(rows, (time = ts, direction = dir, x_m = r * cos(θ), y_m = r * sin(θ)))
        end
    end
    return sort!(rows; by = r -> r.time)
end

"""
    generate_hub_disk_instance(outdir; radius_m=400, hub_distance_m=3000, lambda_per_hour=20, hours=3,
        seed=1, max_walk_m=840, vehicle_speed_mps=8, circuity=1, ring_radii_frac, ring_counts,
        window_start=DateTime(2026, 1, 1, 8), demand_grid_m=nothing) -> NamedTuple

Write one instance to `outdir` (created if needed) and return its metadata. `max_walk_m` is the walking
limit both stages will use (h_a x walking speed; 600 s x 1.4 m/s = 840 m), checked against the geometry.
"""
function generate_hub_disk_instance(
    outdir::AbstractString;
    radius_m::Real = 400.0,
    hub_distance_m::Real = 3000.0,
    lambda_per_hour::Real = 20.0,
    hours::Real = 3,
    seed::Integer = 1,
    max_walk_m::Real = 840.0,
    vehicle_speed_mps::Real = 8.0,
    circuity::Real = 1.0,
    ring_radii_frac::AbstractVector{<:Real} = HUB_DISK_DEFAULT_RING_RADII_FRAC,
    ring_counts::AbstractVector{<:Integer} = HUB_DISK_DEFAULT_RING_COUNTS,
    window_start::DateTime = DateTime(2026, 1, 1, 8),
    demand_grid_m::Union{Nothing,Real} = nothing,
)
    circuity >= 1 || throw(ArgumentError("circuity must be >= 1"))
    stations = hub_disk_stations(; radius_m, hub_distance_m, ring_radii_frac, ring_counts)
    disk = [s for s in stations if s.role == "disk"]
    d(a, b) = hypot(a.x_m - b.x_m, a.y_m - b.y_m)

    # B walkable: a point anywhere in the disk to any disk station is at most R + max ring radius.
    far = radius_m + maximum(hypot(s.x_m, s.y_m) for s in disk)
    far <= max_walk_m || throw(ArgumentError(
        "disk not walkable: point-to-station distance up to $(round(far)) m > max_walk_m $(max_walk_m) m"))
    near_hub = minimum(d(stations[1], s) for s in disk)
    near_hub > max_walk_m || throw(ArgumentError(
        "hub walkable from B: nearest disk station $(round(near_hub)) m <= max_walk_m $(max_walk_m) m"))

    rng = MersenneTwister(seed)
    draws = sample_hub_disk_orders(rng; lambda_per_hour, hours, radius_m, window_start)
    if demand_grid_m === nothing
        points = [(id = HUB_DISK_DEMAND_ID_OFFSET + p, name = "p$(p)", x_m = r.x_m, y_m = r.y_m,
                   ring = -1, role = r.direction == :AB ? "dest_point" : "origin_point")
                  for (p, r) in enumerate(draws)]
        node_of = [pt.id for pt in points]
    else
        g = Float64(demand_grid_m); m = ceil(Int, radius_m / g)
        lattice = [(i * g, j * g) for i in -m:m for j in -m:m if hypot(i * g, j * g) <= radius_m]
        lat_id(i) = HUB_DISK_DEMAND_ID_OFFSET + i
        node_of = [lat_id(argmin([hypot(r.x_m - x, r.y_m - y) for (x, y) in lattice])) for r in draws]
        used = sort!(unique(node_of))
        points = [(id = u, name = "g$(u - HUB_DISK_DEMAND_ID_OFFSET)", x_m = lattice[u - HUB_DISK_DEMAND_ID_OFFSET][1],
                   y_m = lattice[u - HUB_DISK_DEMAND_ID_OFFSET][2], ring = -1, role = "demand_point") for u in used]
    end
    nodes = [stations; points]

    mkpath(outdir)
    station_df = DataFrame(
        station_id = [n.id for n in nodes], station_name = [n.name for n in nodes],
        station_lon = [tc_to_lon(n.x_m / 1000) for n in nodes],
        station_lat = [tc_to_lat(n.y_m / 1000) for n in nodes],
        candidate = [n.role in ("hub", "disk") for n in nodes], role = [n.role for n in nodes],
        ring = [n.ring for n in nodes], x_m = [round(n.x_m; digits = 2) for n in nodes],
        y_m = [round(n.y_m; digits = 2) for n in nodes],
    )
    CSV.write(joinpath(outdir, "station.csv"), station_df)

    seg_rows = NamedTuple[]
    for a in nodes, b in nodes
        a.id == b.id && continue
        dist = circuity * d(a, b)
        push!(seg_rows, (id = length(seg_rows) + 1, from_station = a.id, to_station = b.id,
                         seg_dist = round(dist; digits = 2), seg_time = round(dist / vehicle_speed_mps; digits = 3)))
    end
    CSV.write(joinpath(outdir, "segment.csv"), DataFrame(seg_rows))

    order_rows = NamedTuple[]; order_id = Ref(1)
    for (p, r) in enumerate(draws)
        pid = node_of[p]
        o, dst = r.direction == :AB ? (HUB_DISK_HUB_ID, pid) : (pid, HUB_DISK_HUB_ID)
        tc_push_order!(order_rows, order_id, o, dst, r.time)
    end
    CSV.write(joinpath(outdir, "order.csv"), DataFrame(order_rows))

    meta = (
        benchmark = "hub_disk", seed = seed, radius_m = Float64(radius_m), hub_distance_m = Float64(hub_distance_m),
        lambda_per_hour_each_direction = Float64(lambda_per_hour), hours = hours,
        window_start = string(window_start), window_end = string(window_start + Second(round(Int, hours * 3600))),
        max_walk_m = Float64(max_walk_m), vehicle_speed_mps = Float64(vehicle_speed_mps), circuity = Float64(circuity),
        ring_radii_frac = collect(Float64.(ring_radii_frac)), ring_counts = collect(Int.(ring_counts)),
        n_disk_stations = length(disk), disk_station_ids = [s.id for s in disk],
        n_AB = count(r -> r.direction == :AB, draws), n_BA = count(r -> r.direction == :BA, draws),
        max_point_to_station_m = far, nearest_disk_station_to_hub_m = near_hub,
        demand_grid_m = demand_grid_m === nothing ? nothing : Float64(demand_grid_m), n_demand_nodes = length(points),
    )
    open(joinpath(outdir, "metadata.json"), "w") do f
        JSON.print(f, meta, 1)
    end
    return meta
end
